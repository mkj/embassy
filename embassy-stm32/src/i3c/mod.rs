#![macro_use]

use core::marker::PhantomData;

use embassy_hal_common::{into_ref, PeripheralRef};
use embassy_sync::waitqueue::AtomicWaker;

use crate::dma::{NoDma, Transfer};
use crate::interrupt::typelevel::Interrupt;
use crate::{interrupt, Peripheral};
use crate::time::Hertz;
use crate::pac::i3c;
use crate::pac;
use crate::peripherals;
use crate::gpio::sealed::AFType;

pub(crate) mod sealed {
    use super::*;
    pub trait Instance: crate::rcc::RccPeripheral {
        fn regs() -> crate::pac::i3c::I3c;
        fn state() -> &'static State;
    }
}

pub trait Instance: sealed::Instance + 'static {
    type Interrupt: interrupt::typelevel::Interrupt;
}

pin_trait!(SclPin, Instance);
pin_trait!(SdaPin, Instance);
dma_trait!(RxDma, Instance);
dma_trait!(TxDma, Instance);

foreach_interrupt!(
    ($inst:ident, i3c, $block:ident, EV, $irq:ident) => {
        impl sealed::Instance for peripherals::$inst {
            fn regs() -> crate::pac::i3c::I3c {
                crate::pac::$inst
            }

            fn state() -> &'static State {
                static STATE: State = State::new();
                &STATE
            }
        }

        impl Instance for peripherals::$inst {
            type Interrupt = crate::interrupt::typelevel::$irq;
        }
    };
);

/// Interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        info!("int, ev {:08x}",
            T::regs().evr().read().0,
            );
        // let regs = T::regs();
        // let sr = regs.sr().read();

        // if sr.tcr() || sr.tc() {
        //     T::state().waker.wake();
        // }
        // // The flag can only be cleared by writting to nbytes, we won't do that here, so disable
        // // the interrupt
        // critical_section::with(|_| {
        //     regs.cr1().modify(|w| w.set_tcie(false));
        // });
    }
}

pub struct State {
    waker: AtomicWaker,
}

impl State {
    pub(crate) const fn new() -> Self {
        Self {
            waker: AtomicWaker::new(),
        }
    }
}


pub struct I3cTarget<'d, T: Instance, TXDMA = NoDma, RXDMA = NoDma> {
    _peri: PeripheralRef<'d, T>,
    tx_dma: PeripheralRef<'d, TXDMA>,
    #[allow(dead_code)]
    rx_dma: PeripheralRef<'d, RXDMA>,
}

#[derive(Default)]
pub struct TargetConfig {
    // Device Characteristics Register
    dcr: u8,
    // 4 bit MIPI Instance ID, as part of the PID
    instance_id: Option<u8>,
}

impl<'d, T: Instance, TXDMA, RXDMA> I3cTarget<'d, T, TXDMA, RXDMA> {
    pub fn new(
        peri: impl Peripheral<P = T> + 'd,
        scl: impl Peripheral<P = impl SclPin<T>> + 'd,
        sda: impl Peripheral<P = impl SdaPin<T>> + 'd,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        tx_dma: impl Peripheral<P = TXDMA> + 'd,
        rx_dma: impl Peripheral<P = RXDMA> + 'd,
        config: TargetConfig,
    ) -> Self {
        into_ref!(peri, scl, sda, tx_dma, rx_dma);

        debug!("pac enr {:08x}", pac::RCC.apb1lenr().read().0);

        // RCC state
        T::enable();
        T::reset();

        debug!("pac after enr {:08x}", pac::RCC.apb1lenr().read().0);


        debug!("af scl {} sda {}", scl.af_num(), sda.af_num());
        scl.set_as_af(
            scl.af_num(),
            AFType::OutputOpenDrain,
        );

        sda.set_as_af(
            sda.af_num(),
            AFType::OutputOpenDrain,
        );

        // ref RM0492 35.7.3 Target initialization

        // reset has cfgr.crinit == 0, target role

        // Number of kernel clock cycles to set a time unit of 1 µs
        let aval = (T::frequency() / 1_000_000u32).0 as u8;
        debug!("aval {}", aval);
        T::regs().timingr1().write(|reg| {
            reg.set_aval(aval);
        });

        T::regs().devr0().modify(|reg| {
            // TODO MCTP specific?
            reg.set_ibien(true);
            reg.set_cren(true);
            reg.set_hjen(true);
        });

        T::regs().bcr().modify(|reg| {
            // ibi has mandatory data byte payload
            // TODO MCTP specific?
            reg.set_bcr2(true);
        });

        T::regs().dcr().modify(|reg| {
            reg.set_dcr(config.dcr.into())
        });

        T::regs().maxrlr().modify(|reg| {
            // IBI payload data maximum size 1
            // TODO MCTP specific?
            reg.set_ibip(0b001.into());
            // TODO
            reg.set_mrl(100);
        });

        T::regs().maxwlr().modify(|reg| {
            // TODO
            reg.set_mwl(100);
        });

        T::regs().getcapr().modify(|reg| {
            // Pending read notification
            // TODO MCTP specific?
            reg.set_cappend(true);
        });

        if let Some(inst) = config.instance_id {
            T::regs().epidr().modify(|reg| {
                reg.set_mipiid(inst);
            });
        }

        T::regs().ier().modify(|reg| {
            reg.set_daupdie(true);
            reg.set_errie(true);
        });

        // enable it
        T::regs().cfgr().modify(|reg| {
            reg.set_en(true);
        });

        debug!("ccipr4 {:08x}", pac::RCC.ccipr4().read().0);
        debug!("ccipr4 i3c {:08x}", pac::RCC.ccipr4().read().i3c1sel() as u8);

        debug!("cfgr {:08x}", T::regs().cfgr().read().0);

        Self {
            _peri: peri,
            tx_dma,
            rx_dma,
        }
    }

    pub fn hotjoin(&mut self) {
        info!("hotjoin evr {:08x} devr0 {:08x} hjen {} sr {:08x} ser {:08x}  cr {:08x}",
            T::regs().evr().read().0,
            T::regs().devr0().read().0,
            T::regs().devr0().read().hjen(),
            T::regs().sr().read().0,
            T::regs().ser().read().0,
            T::regs().cr().read().0,
            );
        T::regs().cr().modify(|reg| {
            reg.set_mtype(0b1000.into())
        });
    }
}
