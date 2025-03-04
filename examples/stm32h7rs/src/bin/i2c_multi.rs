#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::join3;
use embassy_futures::select;
use embassy_stm32::i2c::{Command, CommandGuard, Error, I2c, TargetAddress};
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, i2c, peripherals, rng};
use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c::I2c as HalI2c;
use rand_core::RngCore;
use {defmt_rtt as _, panic_probe as _};

const ADDRESS1: u8 = 0x41;
const ADDRESS2: u8 = 0x42;

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    I2C2_EV => i2c::EventInterruptHandler<peripherals::I2C2>;
    I2C2_ER => i2c::ErrorInterruptHandler<peripherals::I2C2>;
    RNG => rng::InterruptHandler<peripherals::RNG>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Hello world!");
    let p = embassy_stm32::init(Default::default());

    let mut config = embassy_stm32::i2c::Config::default();
    config.sda_pullup = true;
    config.scl_pullup = true;
    config.timeout = Duration::from_millis(500);

    let i2c1 = I2c::new(
        p.I2C1,
        // SCL, Nucleo CN7 pin 2
        p.PB8,
        // SDA, Nucleo CN7 pin 4
        p.PB9,
        Irqs,
        p.GPDMA1_CH4,
        p.GPDMA1_CH5,
        Hertz(400_000),
        config,
    );

    let mut i2c2 = I2c::new(
        p.I2C2,
        // SCL, Nucleo CN9 pin 19
        p.PF1,
        // SDA, Nucleo CN9 pin 21
        p.PF0,
        Irqs,
        p.GPDMA1_CH6,
        p.GPDMA1_CH7,
        Hertz(400_000),
        config,
    );

    let mut multi1 = i2c::I2cMulti::new(i2c1);
    let (mut con1, mut tar1) = multi1.split(TargetAddress(ADDRESS1)).await.unwrap();
    let mut multi2 = i2c::I2cMulti::new(i2c2);
    let (mut con2, mut tar2) = multi2.split(TargetAddress(ADDRESS2)).await.unwrap();

    let mut rng = rng::Rng::new(p.RNG, Irqs);

    let run_tar1 = listen(&mut tar1, "tar41", rng.next_u32());
    let run_tar2 = listen(&mut tar2, "tar42", rng.next_u32());

    let run_con1 = send(&mut con1, ADDRESS2, "con42", rng.next_u32());
    let run_con2 = send(&mut con2, ADDRESS1, "con41", rng.next_u32());

    let print = async {
        loop {
            info!("tick");
            Timer::after(Duration::from_millis(2000)).await;
        }
    };

    let timeout = Timer::after(Duration::from_millis(1600));
    let run = select::select5(print, run_tar2, run_tar1, run_con1, run_con2);
    run.await;
    // select::select(run, timeout).await;
    // select::select4(print, run_tar2, run_con1, run_con2).await;
    // select::select3(print, run_tar1, run_con2).await;
    // select::select3(print, run_tar2, run_con1).await;
    // select::select3(print, run_tar1, run_con2).await;
}
async fn listen(tar: &mut i2c::I2cTarget<'_, '_>, name: &str, seed: u32) -> ! {
    let mut random = XorShift::new(seed);
    // let seq = [40, 10, 40, 40, 10].iter().cloned().chain(core::iter::repeat(10));
    // let seq = [40, 10, 40, 40];
    // let seq = seq.iter().cycle().cloned();
    let seq = random.map(|x| (x as usize) % 200 + 1 + 500);

    let mut buf = [0u8; 2000];
    for (iter, n) in seq.enumerate() {
        let buf = &mut buf[..n];
        trace!("target {} listen iter {} buf {}", name, iter, n);

        match tar.listen(buf).await {
            Ok(CommandGuard {
                command: Command::Write { len, pec_good },
                ..
            }) => {
                info!(
                    "target {} got write len={} pec_good {}: {:02x}",
                    name,
                    len,
                    pec_good,
                    buf[..len]
                );
            }
            Ok(CommandGuard {
                command: Command::Read, ..
            }) => {
                info!("target {} got read", name);
                // TODO respond
            }
            Ok(CommandGuard {
                command: Command::WriteRead(len),
                ..
            }) => {
                info!("target {} got writeread len={}", name, len);
                // TODO respond
            }
            Err(e) => {
                error!("listen {} error {}", name, e);
            }
        }
    }
    defmt::unreachable!();
}

async fn send(con: &mut impl HalI2c, dest: u8, name: &str, seed: u32) -> ! {
    let mut random = XorShift::new(seed);
    let mut buf = [0u8; 300];
    for (i, b) in buf.iter_mut().enumerate() {
        *b = (i & 0xff) as u8;
    }
    loop {
        let len = (random.get() % 300) as usize;
        let sendbuf = &buf[..len];
        info!("con {} try write len={}", name, sendbuf.len());
        match con.write(dest, sendbuf).await {
            Ok(()) => info!("con {} write len={} OK", name, sendbuf.len()),
            Err(e) => error!("con {} write len={} error", name, sendbuf.len()),
        }
        let delay = random.get() % 10;
        trace!("delay {} {}", name, delay);
        Timer::after(Duration::from_millis(delay as u64)).await;
    }
}

struct XorShift(u32);

impl XorShift {
    pub fn new(seed: u32) -> Self {
        // ensure seed isn't 0
        Self(seed | 1)
    }
    pub fn get(&mut self) -> u32 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 17;
        self.0 ^= self.0 << 5;
        self.0
    }
}

impl Iterator for XorShift {
    type Item = u32;
    fn next(&mut self) -> Option<Self::Item> {
        Some(self.get())
    }
}
