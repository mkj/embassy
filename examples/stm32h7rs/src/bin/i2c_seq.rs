#![no_std]
#![no_main]
#![allow(unused_imports)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_futures::select::{self, Either};
use embassy_stm32::i2c::{Command, CommandGuard, Error, I2c, I2cController, I2cTarget, TargetAddress};
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
        Hertz(100_000),
        config,
    );

    let i2c2 = I2c::new(
        p.I2C2,
        // SCL, Nucleo CN9 pin 19
        p.PF1,
        // SDA, Nucleo CN9 pin 21
        p.PF0,
        Irqs,
        p.GPDMA1_CH6,
        p.GPDMA1_CH7,
        Hertz(100_000),
        config,
    );

    let mut multi2 = i2c::I2cMulti::new(i2c2);
    let (mut con2, mut tar2) = multi2.split(TargetAddress(ADDRESS2)).await.unwrap();
    let mut multi1 = i2c::I2cMulti::new(i2c1);
    let (mut con1, mut tar1) = multi1.split(TargetAddress(ADDRESS1)).await.unwrap();

    // let mut rng = rng::Rng::new(p.RNG, Irqs);

    // workaround defmt slow to attach?
    for _ in 0..5 {
        Timer::after(Duration::from_millis(30)).await;
        info!("starting");
    }

    // info!("2 -> 1, len 10");
    // seq(&mut con2, &mut tar1, 10, 300).await;
    // info!("1 -> 2, len 10");
    // seq(&mut con1, &mut tar2, 10, 300).await;

    info!("1 -> 2, len 10");
    seq(&mut con1, &mut tar2, ADDRESS2, 5, 10).await;
    info!("1 -> 2, len 10");
    seq(&mut con1, &mut tar2, ADDRESS2, 400, 500).await;
    info!("1 -> 2, len 10");
    seq(&mut con1, &mut tar2, ADDRESS2, 257, 500).await;
    info!("2 -> 1, len 10");
    seq(&mut con1, &mut tar2, ADDRESS2, 257, 500).await;
    // info!("1 -> 2, len 10");
    // seq(&mut con1, &mut tar2, ADDRESS2, 10, 5).await;
    // warn!("here");
    // info!("1 -> 2, len 10");
    // seq(&mut con1, &mut tar2, ADDRESS2, 5, 10).await;
    // info!("1 -> 2, len 10");
    // seq(&mut con1, &mut tar2, ADDRESS2, 10, 20).await;
    // info!("2 -> 1, len 10");
    // seq(&mut con1, &mut tar2, ADDRESS2, 10, 300).await;
    info!("all done");
}

async fn seq(con: &mut I2cController<'_, '_>, tar: &mut I2cTarget<'_, '_>, dest: u8, sendlen: usize, recvlen: usize) {
    trace!("seq send {} recv {}", sendlen, recvlen);
    con.regtrace("con").await;
    tar.regtrace("tar").await;
    join(
        async {
            let mut buf = [0u8; 600];
            let timeout = Timer::after(Duration::from_millis(400));
            let lis = tar.listen(&mut buf[..recvlen]);
            match select::select(timeout, lis).await {
                Either::First(_) => error!("listen timeout"),
                Either::Second(r) => match r {
                    Ok(CommandGuard {
                        command: Command::Write { len, .. },
                        ..
                    }) => {
                        info!("target got write len={}: {:02x}", len, buf[..len]);
                    }
                    Err(e) => error!("listen error {}", e),
                    _ => defmt::todo!(),
                },
            }
        },
        async {
            Timer::after(Duration::from_millis(30)).await;
            let mut out = [0u8; 500];
            for (i, b) in out.iter_mut().enumerate() {
                *b = (i & 0xff) as u8;
            }
            trace!("writing {}", sendlen);
            let r = con.write(dest, &out[..sendlen]).await;
            match r {
                Ok(()) => info!("wrote {}", sendlen),
                Err(e) => error!("write error {}", e),
            }
        },
    )
    .await;
    Timer::after(Duration::from_millis(30)).await;
    trace!("seq complete");
}
