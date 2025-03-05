use super::*;

use core::future::poll_fn;
use core::pin::pin;
use core::sync::atomic::{fence, Ordering};
use core::task::Poll;

use embassy_futures::select::{select, Either};
use embassy_hal_internal::drop::OnDrop;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::{Mutex, MutexGuard};

use crate::pac::i2c::vals::{Dir, Oamsk};

type BlockMutex<T> = Mutex<CriticalSectionRawMutex, T>;

/// I2C Address
///
/// Currently only 7-bit supported
#[derive(Copy, Clone, Debug)]
pub struct TargetAddress(pub u8);

#[cfg(feature = "defmt")]
impl defmt::Format for TargetAddress {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "TargetAddress({:#02x})", self.0)
    }
}

pub struct I2cMulti<'d> {
    i2c: Mutex<CriticalSectionRawMutex, I2c<'d, Async>>,
}

impl<'d> I2cMulti<'d> {
    pub fn new(i2c: I2c<'d, Async>) -> Self {
        Self { i2c: Mutex::new(i2c) }
    }

    /// Create separate Target/Controller instances
    pub fn split(
        &mut self,
        own_addr1: TargetAddress,
    ) -> Result<(I2cController<'_, 'd>, I2cTarget<'_, 'd>), Error> {
        let c = I2cController { multi: self };
        let t = I2cTarget::new(self, own_addr1)?;
        Ok((c, t))
    }
}

pub enum Command {
    Read,
    Write { len: usize },
    WriteRead(usize),
}

pub struct CommandGuard<'s, 'd> {
    pub command: Command,
    i2c_guard: Option<MutexGuard<'s, CriticalSectionRawMutex, I2c<'d, Async>>>,
}

impl<'s, 'd> CommandGuard<'s, 'd> {
    pub async fn respond_to_read(&mut self, buffer: &[u8]) -> Result<usize, Error> {
        let Some(ref mut i2c) = self.i2c_guard else {
            trace!("Not a Read command");
            // TODO error type
            return Err(Error::Bus);
        };
        I2cTarget::respond_to_read(i2c, buffer).await
    }
}

pub struct I2cTarget<'s, 'd> {
    multi: &'s I2cMulti<'d>,
    own_addr1: TargetAddress,
    // Only used for drop
    drop_regs: &'static crate::pac::i2c::I2c,
}
impl<'s, 'd> I2cTarget<'s, 'd> {
    // Always called with an unlocked `multi.i2c`, or will panic.
    fn new(multi: &'s I2cMulti<'d>, own_addr1: TargetAddress) -> Result<Self, Error> {
        if (own_addr1.0 & !0x7f) != 0 {
            return Err(Error::BadAddress);
        }

        let i2c = multi.i2c.try_lock().expect("Called unlocked from .split()");
        let r = i2c.info.regs;

        // Own Address
        // TODO: why doesn't oa1 work?
        r.oar2().modify(|w| {
            w.set_oa2en(false);
        });
        r.oar2().modify(|w| {
            w.set_oa2(own_addr1.0 as u8);
            w.set_oa2msk(Oamsk::NO_MASK);
            w.set_oa2en(true);
        });

        Ok(Self {
            multi,
            own_addr1,
            drop_regs: &i2c.info.regs,
        })
    }

    pub async fn regtrace(&self, name: &str) {
        let i2c = self.multi.i2c.lock().await;
        trace!("regs {}: isr {:08x} cr1 {:08x} cr2 {:08x}",
            name,
            i2c.info.regs.isr().read().0,
            i2c.info.regs.cr1().read().0,
            i2c.info.regs.cr2().read().0,
        );
    }

    /// `buffer` must not be empty.
    pub async fn listen(&mut self, buffer: &mut [u8]) -> Result<CommandGuard<'s, 'd>, Error> {
        if buffer.is_empty() {
            debug!("i2c empty buffer");
            return Err(Error::Bus);
        }

        loop {
            // Wait until addressed. Returns a MutexGuard
            let mut i2c = poll_fn(|cx| {
                // Get the lock, in case a Controller is finishing a transaction
                let lock = pin!(self.multi.i2c.lock());
                let Poll::Ready(i2c) = lock.poll(cx) else {
                    trace!("target {:02x} lock nope", self.own_addr1);
                    return Poll::Pending;
                };

                trace!("target {:02x} lock nope", self.own_addr1);
                i2c.state.target_waker.register(cx.waker());
                // Wake when addressed
                i2c.info.regs.cr1().modify(|w| {
                    w.set_addrie(true);
                });

                if i2c.info.regs.isr().read().addr() {
                    // Keep the i2c MutexGuard once addressed.
                    // trace!("target ready");
                    trace!("target addressed {:02x}", self.own_addr1);
                    return Poll::Ready(i2c);
                }

                // Clear any previous stop flag.
                // TODO is this necessary?
                // i2c.info.regs.icr().modify(|w| {
                //     w.set_stopcf(true);
                // });
                // trace!("target pending");
                trace!("target {:02x} locked, addr pending", self.own_addr1);
                Poll::Pending
            })
            .await;

            let r = i2c.info.regs;
            // trace!("target cr1 {:08x} cr2 {:08x}", r.cr1().read().0, r.cr2().read().0);

            if r.isr().read().dir() == Dir::READ {
                // Caller will respond
                let cg = CommandGuard {
                    command: Command::Read,
                    i2c_guard: Some(i2c),
                };
                return Ok(cg);
            }

            // Receive Dir::WRITE data
            match select(
                Self::wait_error(&i2c.info.regs, &i2c.state),
                Self::receive_write(&mut i2c, buffer),
            )
            .await
            {
                // Error, continue
                Either::First(e) => {
                    error!("Listen {} error {}", self.own_addr1, e);
                    // TODO should this return the error to the caller?
                    // Maybe for BusError, probably not for Arbitration?
                }
                // Complete listen event
                Either::Second(res) => {
                    trace!("Listen {} done", self.own_addr1);
                    return res;
                }
            }
        }
    }

    async fn receive_write(i2c: &mut I2c<'d, Async>, buf: &mut [u8]) -> Result<CommandGuard<'s, 'd>, Error> {
        let r = i2c.info.regs;

        // Clean up on cancellation.
        let _on_drop = OnDrop::new(|| {
            r.cr1().modify(|w| {
                w.set_rxdmaen(false);
                w.set_stopie(false);
            });
        });

        trace!(
            "target receive_write {} top isr {:08x} cr1 {:08x} cr2 {:08x}",
            line!(),
            r.isr().read().0,
            r.cr1().read().0,
            r.cr2().read().0,
        );

        // Received address goes in first byte
        let (dest_byte, dmabuf) = buf.split_first_mut().expect("buf is non-empty");
        *dest_byte = r.isr().read().addcode() << 1;

        // Provide receive buffer to DMA
        let total_buf = dmabuf.len();
        r.cr1().modify(|w| {
            w.set_rxdmaen(true);
        });
        let rxptr = r.rxdr().as_ptr() as *mut u8;
        let mut dma_transfer = unsafe { i2c.rx_dma.as_mut().unwrap().read(rxptr, dmabuf, Default::default()) };

        r.icr().write(|w| {
            w.set_stopcf(true);
            w.set_nackcf(true);
        });

        // Ack the address, must be done after DMA setup.
        r.icr().write(|w| w.set_addrcf(true));

        trace!("target isr {} {:08x}", line!(), r.isr().read().0);

        // Wait for the buffer to fill, or stop condition
        let stop = poll_fn(|cx| {
            i2c.state.target_waker.register(cx.waker());
            r.cr1().modify(|w| {
                w.set_stopie(true);
            });

            let isr = r.isr().read();
            if isr.stopf() {
                // stop condition from controller
                trace!("stop");
                r.icr().write(|w| {
                    w.set_stopcf(true);
                });
                return Poll::Ready(());
            }
            Poll::Pending
        });

        match select(stop, &mut dma_transfer).await {
            Either::First(()) => {
                // Stop receive. Determine byte count filled.
                // Need to stop the transfer for
                // get_remaining_transfer() to be reliable.
                dma_transfer.request_stop();
                while dma_transfer.is_running() {}
                fence(Ordering::SeqCst);
                let rem = dma_transfer.get_remaining_transfers() as usize;
                trace!("rem {} buf {} diff {}", rem, total_buf, total_buf - rem);
                // +1 for initial address byte
                let len = 1 + total_buf.checked_sub(rem).expect("remaining <= total");

                let c = CommandGuard {
                    command: Command::Write { len },
                    i2c_guard: None,
                };
                return Ok(c);
            }
            Either::Second(()) => {
                trace!("dma finished");
                // DMA finished
            }
        }

        trace!("target after poll isr {} {:08x}", line!(), r.isr().read().0);
        // No more buffer space, done with DMA.
        r.cr1().modify(|w| {
            w.set_rxdmaen(false);
        });
        drop(dma_transfer);
        // Keep nacking until the controller stops, then return Overrun.
        Self::nack_receive(i2c).await;
        debug!("target overrun finished");
        Err(Error::Overrun)
    }

    async fn nack_receive(i2c: &mut I2c<'d, Async>) {
        let r = i2c.info.regs;

        let discard = async {
            let mut one_byte = [0u8];
            loop {
                r.cr2().modify(|w| {
                    w.set_nack(true);
                });
                let rxptr = r.rxdr().as_ptr() as *mut u8;
                let dma_transfer = unsafe {
                    i2c.rx_dma
                        .as_mut()
                        .unwrap()
                        .read(rxptr, &mut one_byte, Default::default())
                };
                dma_transfer.await;
            }
        };

        let stop = poll_fn(|cx| {
            i2c.state.target_waker.register(cx.waker());
            r.cr1().modify(|w| {
                w.set_stopie(true);
                w.set_addrie(true);
            });
            let isr = r.isr().read();
            if isr.stopf() {
                return Poll::Ready(());
            }
            if isr.addr() {
                // TODO: also set addrcf if addressed? or can we nack an address?
                trace!("addr during nack");
                // return Poll::Ready(());
            }
            Poll::Pending
        });

        let _ = select(stop, discard).await;
    }

    #[expect(unused_variables)] // TODO
    async fn respond_to_read(i2c: &mut I2c<'d, Async>, buf: &[u8]) -> Result<usize, Error> {
        todo!()
    }

    async fn wait_error(regs: &crate::pac::i2c::I2c, state: &State) -> Error {
        poll_fn(|cx| {
            state.target_waker.register(cx.waker());
            regs.cr1().modify(|w| w.set_errie(true));
            let isr = regs.isr().read();
            if isr.arlo() {
                Poll::Ready(Error::Arbitration)
            } else if isr.berr() {
                Poll::Ready(Error::Bus)
            } else {
                Poll::Pending
            }

            // timeouten isn't set
            // OVR is only when NOSTRETCH=1
        })
        .await
    }
}

impl<'s, 'd> Drop for I2cTarget<'s, 'd> {
    fn drop(&mut self) {
        // Disable address interrupt
        self.drop_regs.oar1().modify(|w| {
            w.set_oa1en(false);
        });
        // Clear address flag to end possible clock stretching
        self.drop_regs.icr().write(|w| {
            w.set_addrcf(true);
        });
    }
}

pub struct I2cController<'s, 'd> {
    multi: &'s I2cMulti<'d>,
}

impl<'s, 'd> I2cController<'s, 'd> {
    pub async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Error> {
        trace!("controller lock wait {:02x}", address);
        let mut i2c = self.multi.i2c.lock().await;
        trace!("controller got {:02x}", address);
        let r = i2c.write(address, write).await;
        trace!("controller drop {:02x}", address);
        r
    }

    pub async fn regtrace(&self, name: &str) {
        let i2c = self.multi.i2c.lock().await;
        trace!("regs {}: isr {:08x} cr1 {:08x} cr2 {:08x}",
            name,
            i2c.info.regs.isr().read().0,
            i2c.info.regs.cr1().read().0,
            i2c.info.regs.cr2().read().0,
        );
    }

}

impl<'s, 'd> embedded_hal_1::i2c::ErrorType for I2cController<'s, 'd> {
    type Error = Error;
}

impl<'s, 'd> embedded_hal_async::i2c::I2c for I2cController<'s, 'd> {

    async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        self.write(address, write).await.map_err(|e| {
            warn!("ctrl {:02x} write error {}", address, e);
            e
        })
    }

    #[expect(unused_variables)] // TODO
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal_1::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        todo!()
    }
}
