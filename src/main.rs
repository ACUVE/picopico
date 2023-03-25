//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use core::{borrow::BorrowMut, convert::Infallible, sync::atomic::Ordering};
use cortex_m::interrupt::InterruptNumber;
use cortex_m_rt as _;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::{OutputPin, PinState};
use embedded_time::fixed_point::FixedPoint;
use panic_probe as _;

use rp2040_hal as hal;

use hal::{
    clocks::{init_clocks_and_plls, Clock},
    entry, gpio, i2c, pac,
    pio::PIOExt,
    sio::Sio,
    watchdog::Watchdog,
};

use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

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

// use device::interrupt;
use pac::interrupt;

static LED_PIN: core::sync::atomic::AtomicPtr<
    *mut dyn OutputPin<Error = core::convert::Infallible>,
> = core::sync::atomic::AtomicPtr::new(core::ptr::null_mut());

#[interrupt]
fn USBCTRL_IRQ() {
    static FLAG: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

    if let Some(pin) = unsafe {
        LED_PIN
            .load(core::sync::atomic::Ordering::Acquire)
            .as_ref()
            .map(|p| p.as_mut())
    }
    .flatten()
    {
        pin.set_state(if FLAG.load(Ordering::Relaxed) {
            PinState::High
        } else {
            PinState::Low
        })
        .unwrap();
        FLAG.store(FLAG.load(Ordering::Acquire), Ordering::Release)
    }
}

// https://github.com/rp-rs/rp-hal/blob/12387bcf09fc0ff56f4cabaeecdb132fcbf5ba15/boards/seeeduino-xiao-rp2040/src/lib.rs#L69
const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    let pins = gpio::bank0::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // let mut neo_pixel_power = pins.gpio11.into_push_pull_output();
    // neo_pixel_power.set_high().unwrap();
    // let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    // let mut ws = Ws2812Direct::new(
    //     pins.gpio12.into_mode(),
    //     &mut pio,
    //     sm0,
    //     clocks.peripheral_clock.freq(),
    // );

    // let mut i2c_event_iterator = i2c::I2C::new_peripheral_event_iterator(
    //     pac.I2C1,
    //     pins.gpio6.into_mode(),
    //     pins.gpio7.into_mode(),
    //     &mut pac.RESETS,
    //     0x65,
    // );

    let mut blue_pin = pins.gpio25.into_push_pull_output();
    blue_pin.set_high().unwrap();
    let mut green_pin = pins.gpio16.into_push_pull_output();
    // green_pin.set_low().unwrap();
    green_pin.set_high().unwrap();
    let mut red_pin = pins.gpio17.into_push_pull_output();
    // red_pin.set_low().unwrap();
    red_pin.set_high().unwrap();
    let mut blue_pin_ptr = &mut blue_pin as *mut dyn OutputPin<Error = core::convert::Infallible>;
    LED_PIN.store(&mut blue_pin_ptr as *mut _, Ordering::Release);

    // let mut h = 0;
    // const DIFFH: u16 = 50;

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut said_hello = false;

    let mut pin_flag = true;

    let commands = [Command {
        name: b"test",
        func: &|serial| {
            // write_bytes(serial, &core.CPUID.base.read().to_ne_bytes());
            serial.write(b"\r\n");
        },
    }];
    let mut command_processor = SerialCommandProcessor::new(&commands);

    loop {
        // A welcome message at the beginning
        if !said_hello && timer.get_counter() >= 2_000_000 {
            said_hello = true;
            let _ = serial.write(b"Hello, World!\r\n");
        }

        // Check for new data
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Ok(0) | Err(_) => {
                    // Do nothing
                }
                Ok(count) => {
                    command_processor.process_buff(&buf[..count], &mut serial);
                    red_pin
                        .set_state({
                            pin_flag = !pin_flag;
                            if pin_flag {
                                PinState::High
                            } else {
                                PinState::Low
                            }
                        })
                        .unwrap();
                }
            }
        }

        // info!("on!");
        // blue_pin.set_high().unwrap();
        // ws.write([h2rgb(h)].iter().copied()).unwrap();
        // h = (h + DIFFH) % (255 * 6);
        // delay.delay_ms(16);
        // info!("off!");
        // blue_pin.set_low().unwrap();
        // ws.write([h2rgb(h)].iter().copied()).unwrap();
        // h = (h + DIFFH) % (255 * 6);
        // delay.delay_ms(16);
    }
}

trait Write {
    fn write(&mut self, data: &[u8]);
    fn flush(&mut self);
}

impl<B: UsbBus, RS: BorrowMut<[u8]>, WS: BorrowMut<[u8]>> Write for SerialPort<'_, B, RS, WS> {
    fn write(&mut self, data: &[u8]) {
        let _ = self.write(data);
    }
    fn flush(&mut self) {
        let _ = self.flush();
    }
}

struct Command<'a> {
    name: &'static [u8],
    func: &'a (dyn Fn(&mut dyn Write) + 'static),
}

struct SerialCommandProcessor<'a, 'b> {
    commands: &'a [Command<'b>],
    buffer: [u8; 32],
    end_pos: usize,
}

fn byte_to_chars(byte: u8) -> [u8; 2] {
    let u4_to_b = |u4| {
        if u4 <= 9 {
            u4 + b'0'
        } else {
            u4 - 10 + b'A'
        }
    };
    let upper = byte >> 4;
    let lower = byte & 0xF;
    [u4_to_b(upper), u4_to_b(lower)]
}

fn write_bytes(w: &mut (impl Write + ?Sized), bytes: &[u8]) {
    w.write(b"[");
    let mut first = true;
    for byte in bytes {
        if !first {
            w.write(b", ");
        }
        w.write(b"0x");
        w.write(&byte_to_chars(*byte));
        first = false;
    }
    w.write(b"]");
}

impl<'a, 'b> SerialCommandProcessor<'a, 'b> {
    fn new(commands: &'a [Command<'b>]) -> Self {
        Self {
            commands,
            buffer: Default::default(),
            end_pos: 0,
        }
    }
    fn process_buff<B: UsbBus, RS: BorrowMut<[u8]>, WS: BorrowMut<[u8]>>(
        &mut self,
        buff: &[u8],
        serial: &mut SerialPort<'_, B, RS, WS>,
    ) {
        let mut split = buff.split(|b| b == &b'\r' || b == &b'\n').peekable();
        while let Some(p) = split.next() {
            if split.peek().is_none() {
                if self.end_pos + p.len() <= self.buffer.len() {
                    self.buffer[self.end_pos..]
                        .iter_mut()
                        .take(p.len())
                        .zip(core::iter::successors(Some(0), |v| Some(v + 1)))
                        .for_each(|(v, index)| {
                            *v = p[index];
                        });
                    self.end_pos += p.len();
                    let _ = serial.write(p);
                } else {
                    self.end_pos = 0;
                }
            } else {
                let end_pos = self.end_pos;
                self.end_pos = 0;

                let cmp = |buff: &[u8]| {
                    self.buffer[..end_pos].iter().chain(p.iter()).cmp(buff)
                        == core::cmp::Ordering::Equal
                };

                if cmp(b"") {
                    // Do Nothing
                } else {
                    let _ = serial.write(b"\r\n");
                    if let Some(command) = self.commands.iter().find(|c| cmp(c.name)) {
                        (command.func)(serial);
                    } else {
                        let _ = serial.write(b"no command\r\n");
                    }
                }
            }
        }
    }
}

// End of file
