//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::fixed_point::FixedPoint;
use panic_probe as _;

use rp2040_hal::{
    gpio::FunctionPio0,
    pio::{PIOExt, PinDir, PinState},
};
// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need t&mut o change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    i2c::I2C,
    pac,
    pio::PIOBuilder,
    sio::Sio,
    watchdog::Watchdog,
};

struct NeoPixel<PIO: PIOExt> {
    pio: PIO,
}

impl<PIO: PIOExt> NeoPixel<PIO> {
    fn new(pio: PIO) -> Self {
        NeoPixel { pio }
    }
}

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let program = pio_proc::pio_asm!(
        ".wrap_target",
        "set pins, 1 [31]",
        "set pins, 0 [31]",
        ".wrap"
    )
    .program;
    let installed = pio.install(&program).unwrap();
    let (mut sm, rx, tx) = PIOBuilder::from_program(installed).build(sm0);
    sm.set_pins([(0, PinState::High)]);
    sm.set_pindirs([(0, PinDir::Output)]);
    sm.start();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let _pin0 = pins.gpio0.into_mode::<FunctionPio0>();

    // let i2c = I2C::i2c0(pac.I2C0, pins., scl_pin, freq, resets, system_clock);

    // let mut led_pin = pins.led.into_push_pull_output();
    let mut blue_pin = pins.led.into_push_pull_output();
    let mut green_pin = pins.gpio16.into_push_pull_output();
    let mut red_pin = pins.gpio17.into_push_pull_output();

    loop {
        info!("on!");
        blue_pin.set_high().unwrap();
        green_pin.set_high().unwrap();
        red_pin.set_high().unwrap();
        delay.delay_ms(500);
        info!("off!");
        blue_pin.set_low().unwrap();
        green_pin.set_low().unwrap();
        red_pin.set_low().unwrap();
        delay.delay_ms(500);
    }
}

// End of file
