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

#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

use rp2040_hal as hal;

use hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio, i2c, pac,
    pio::PIOExt,
    sio::Sio,
    watchdog::Watchdog,
};

use smart_leds::{SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812Direct;

fn h2rgb(h: u16) -> RGB8 {
    match h / 255 {
        0 => RGB8::new(255, h as u8, 0),
        1 => RGB8::new(255 - (h % 255) as u8, 255, 0),
        2 => RGB8::new(0, 255, (h % 255) as u8),
        3 => RGB8::new(0, 255 - (h % 255) as u8, 255),
        4 => RGB8::new((h % 255) as u8, 0, 255),
        5 => RGB8::new(255, 0, 255 - (h % 255) as u8),
        _ => RGB8::new(0, 0, 0),
    }
}

#[entry]
fn main() -> ! {
    info!("P&mut rogram start");
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
    let pins = gpio::bank0::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut neo_pixel_power = pins.gpio11.into_push_pull_output();
    neo_pixel_power.set_high().unwrap();
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812Direct::new(
        pins.gpio12.into_mode(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
    );

    let mut i2c_event_iterator = i2c::I2C::new_peripheral_event_iterator(
        pac.I2C1,
        pins.gpio6.into_mode(),
        pins.gpio7.into_mode(),
        &mut pac.RESETS,
        0x65,
    );

    let mut blue_pin = pins.gpio25.into_push_pull_output();
    // let mut green_pin = pins.gpio16.into_push_pull_output();
    // let mut red_pin = pins.gpio17.into_push_pull_output();

    let mut h = 0;
    const DIFFH: u16 = 50;

    enum I2CState {
        Idle,
        Start,
        Read,
        Write,
    }

    let mut i2c_state = I2CState::Idle;

    let mut regs = [0u8; 0x10];

    loop {
        loop {
            match i2c_event_iterator.next() {
                Some(i2c::peripheral::I2CEvent::Start) => {
                    i2c_state = I2CState::Start;
                }
                Some(i2c::peripheral::I2CEvent::TransferRead) => {
                    i2c_state = I2CState::Read;
                }
                Some(i2c::peripheral::I2CEvent::TransferWrite) => {
                    i2c_state = I2CState::Write;
                }
                Some(i2c::peripheral::I2CEvent::Restart) => {
                    i2c_state = I2CState::Start;
                }
                Some(i2c::peripheral::I2CEvent::Stop) => match i2c_state {
                    I2CState::Read => {}
                    I2CState::Write => {
                        let mut buffer = [0u8; 16];
                        let size = i2c_event_iterator.read(&mut buffer[..]) as u8;
                        if size > 0 {
                            let reg = buffer[0];
                            for pos in 1..size {
                                let reg = reg + pos - 1;
                                if (reg as usize) < regs.len() {
                                    regs[reg as usize] = buffer[pos as usize];
                                }
                            }
                        }
                    }
                    I2CState::Start | I2CState::Idle => {}
                },
                None => break,
            }
        }

        info!("on!");
        blue_pin.set_high().unwrap();
        ws.write([h2rgb(h)].iter().copied()).unwrap();
        h = (h + DIFFH) % (255 * 6);
        delay.delay_ms(16);
        info!("off!");
        blue_pin.set_low().unwrap();
        ws.write([h2rgb(h)].iter().copied()).unwrap();
        h = (h + DIFFH) % (255 * 6);
        delay.delay_ms(16);
    }
}

// End of file
