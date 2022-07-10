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
    clocks::ClocksManager,
    gpio::FunctionPio0,
    pio::{self, PIOExt, Running, StateMachine, SM0},
};
// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need t&mut o change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    i2c::I2C,
    pac,
    pac::RESETS,
    pio::PIOBuilder,
    pio::PIO,
    sio::Sio,
    watchdog::Watchdog,
};

pub struct NeoPixel<const N: usize, P: PIOExt> {
    pio: PIO<P>,
    sm: StateMachine<(P, SM0), Running>,
    tx: pio::Tx<(P, SM0)>,
    pixels: [u32; N],
}

impl<const N: usize, P: PIOExt> NeoPixel<N, P> {
    pub fn new(pio: P, pin: u8, resets: &mut RESETS, clock_manager: &ClocksManager) -> Self {
        let (mut pio, sm0, _, _, _) = pio.split(resets);
        let program = pio_proc::pio_asm!(
            ".side_set 1",
            "set pindirs, 1  side 0 [0]",
            ".wrap_target",
            "out x, 1        side 0 [2]",
            "jmp !x, 3       side 1 [1]",
            "jmp 0           side 1 [4]",
            "nop             side 0 [4]",
            ".wrap"
        )
        .program;
        let installed = pio.install(&program).unwrap();
        let (sm, rx, tx) = PIOBuilder::from_program(installed)
            .side_set_pin_base(pin)
            .clock_divisor(
                (clock_manager.system_clock.freq().integer() as f32) / ((800000 * 10) as f32),
            )
            .out_shift_direction(pio::ShiftDirection::Left)
            .autopull(true)
            .buffers(pio::Buffers::OnlyRx)
            .build(sm0);
        let sm = sm.start();
        Self {
            pio,
            sm,
            tx,
            pixels: [0; N],
        }
    }
    pub fn set_pixel(&mut self, index: u16, pixel: u32) {
        let index = index as usize;
        if index < N {
            self.pixels[index] = pixel;
        }
    }
    pub fn show(&mut self) {
        for pixel in self.pixels.iter() {
            self.tx.write(pixel);
        }
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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let _pin0 = pins.gpio0.into_mode::<FunctionPio0>();

    let mut neo_power = pins.gpio11.into_push_pull_output();
    neo_power.set_high().unwrap();
    let mut neo_pixel = NeoPixel::<1, _>::new(pac.PIO0, 11, &mut pac.RESETS, &clocks);
    neo_pixel.set_pixel(0, 0x00FFFFFF);
    neo_pixel.show();

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
