#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

// #[panic_handler]
// fn panic(_info: &core::panic::PanicInfo) -> ! {
//     loop {}
// }

use cortex_m_rt as _;
use {defmt_rtt as _, panic_probe as _}; // global logger

use embassy_executor::Spawner;
use embassy_rp::gpio::{AnyPin, Input, Level, Output, Pin, Pull};
use embassy_time::{Duration, Timer};

// Declare async tasks
#[embassy_executor::task]
async fn blink(pin: AnyPin) {
    let mut led = Output::new(pin, Level::Low);

    loop {
        // Timekeeping is globally available, no need to mess with hardware timers.
        led.set_high();
        Timer::after(Duration::from_millis(1000)).await;
        led.set_low();
        Timer::after(Duration::from_millis(1000)).await;
    }
}

// Main is itself an async task as well.
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Spawned tasks run in the background, concurrently.
    spawner.spawn(blink(p.PIN_16.degrade())).unwrap();
    // spawner.spawn(blink(p.PIN_17.degrade())).unwrap();
    // spawner.spawn(blink(p.PIN_25.degrade())).unwrap();

    let mut led = Output::new(p.PIN_25.degrade(), Level::Low);

    loop {
        // Timekeeping is globally available, no need to mess with hardware timers.
        led.set_high();
        Timer::after(Duration::from_millis(1000)).await;
        led.set_low();
        Timer::after(Duration::from_millis(1000)).await;
    }
}
