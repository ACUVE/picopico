//! This example shows powerful PIO module in the RP2040 chip.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

mod cds;
mod rbc;

use core::mem::MaybeUninit;

use embassy_executor::Spawner;
use embassy_rp::gpio::{AnyPin, Level, Output, Pin};
use embassy_rp::{bind_interrupts, config};
use embassy_time::{Duration, Timer};
use embassy_usb_driver::{Endpoint, EndpointIn, EndpointOut};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<embassy_rp::peripherals::PIO0>;
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<embassy_rp::peripherals::USB>;
});

struct State {
    control: MaybeUninit<Control>,
}

impl State {
    pub fn new() -> Self {
        Self {
            control: MaybeUninit::uninit(),
        }
    }
}

const MSD_GET_MAX_LUN: u8 = 0xfe;
const MSD_BBB_RESET: u8 = 0xff;

const MAX_PACKET_SIZE: u16 = 64;

struct Control {
    data_if: embassy_usb::types::InterfaceNumber,
}

impl embassy_usb::Handler for Control {
    fn control_in<'a>(
        &'a mut self,
        req: embassy_usb::control::Request,
        buf: &'a mut [u8],
    ) -> Option<embassy_usb::control::InResponse<'a>> {
        if (req.request_type, req.recipient, req.index)
            != (
                embassy_usb::control::RequestType::Class,
                embassy_usb::control::Recipient::Interface,
                self.data_if.0 as u16,
            )
        {
            return None;
        }
        match req.request {
            MSD_GET_MAX_LUN if req.value == 0 && req.length == 1 => {
                buf[0] = 0;
                Some(embassy_usb::control::InResponse::Accepted(&buf[..1]))
            }
            _ => Some(embassy_usb::control::InResponse::Rejected),
        }
    }
    fn control_out(
        &mut self,
        req: embassy_usb::control::Request,
        _data: &[u8],
    ) -> Option<embassy_usb::control::OutResponse> {
        if (req.request_type, req.recipient, req.index)
            != (
                embassy_usb::control::RequestType::Class,
                embassy_usb::control::Recipient::Interface,
                self.data_if.0 as u16,
            )
        {
            return None;
        }

        match req.request {
            MSD_BBB_RESET if req.value == 0 && req.length == 0 => {
                Some(embassy_usb::control::OutResponse::Accepted)
            }
            _ => Some(embassy_usb::control::OutResponse::Rejected),
        }
    }
}

struct MassStorageClassBbb<'d, D: embassy_usb_driver::Driver<'d>> {
    data_if: embassy_usb::types::InterfaceNumber,
    bulk_in_ep: D::EndpointIn,
    bulk_out_ep: D::EndpointOut,
}

impl<'d, D: embassy_usb_driver::Driver<'d>> MassStorageClassBbb<'d, D> {
    pub fn new(builder: &mut embassy_usb::Builder<'d, D>, state: &'d mut State) -> Self {
        let mut function = builder.function(0x08, 0x06, 0x50);
        let mut interface = function.interface();
        let data_if = interface.interface_number();
        let mut alt_setting = interface.alt_setting(0x08, 0x06, 0x50, None);
        let bulk_in_ep = alt_setting.endpoint_bulk_in(MAX_PACKET_SIZE);
        let bulk_out_ep = alt_setting.endpoint_bulk_out(MAX_PACKET_SIZE);

        drop(function);

        let control = state.control.write(Control { data_if });
        builder.handler(control);

        Self {
            data_if,
            bulk_in_ep,
            bulk_out_ep,
        }
    }
}

fn manage_cbw_and_data(cbw: &cds::Cbw, data: &rbc::Data) -> cds::Csw {
    let is_host_to_device = cbw.bmCBWFlags & 0x80 != 0x80;
    let host_assumed_data_length = cbw.dCBWDataTransferLength;

    let mut csw = cds::Csw {
        dCSWTag: cbw.dCBWTag,
        dCSWDataResidue: 0,
        bCSWStatus: 0,
    };

    if host_assumed_data_length == 0 {
        if let &rbc::Data::None = data {
            // do nothing
        } else {
            csw.bCSWStatus = 0x02;
        }
    } else if is_host_to_device {
        // host to device
        match data {
            rbc::Data::None => {
                csw.dCSWDataResidue = host_assumed_data_length;
                csw.bCSWStatus = 0x01;
            }
            rbc::Data::RecvDummy(len) => {
                if len.get() <= host_assumed_data_length {
                    csw.dCSWDataResidue = host_assumed_data_length - len.get();
                } else {
                    csw.bCSWStatus = 0x02;
                }
            }
            rbc::Data::Send(_) | rbc::Data::SendDummy(_) => {
                csw.bCSWStatus = 0x02;
            }
        }
    } else {
        // device to host
        match data {
            rbc::Data::None => {
                csw.dCSWDataResidue = host_assumed_data_length;
                csw.bCSWStatus = 0x01;
            }
            rbc::Data::Send(data) => {
                match data.len().cmp(&(host_assumed_data_length as _)) {
                    core::cmp::Ordering::Equal => {
                        // do nothing
                    }
                    core::cmp::Ordering::Less => {
                        csw.dCSWDataResidue = host_assumed_data_length - data.len() as u32;
                    }
                    core::cmp::Ordering::Greater => {
                        csw.bCSWStatus = 0x02;
                    }
                }
            }
            rbc::Data::SendDummy(len) => match len.get().cmp(&host_assumed_data_length) {
                core::cmp::Ordering::Equal => {
                    // do nothing
                }
                core::cmp::Ordering::Less => {
                    csw.dCSWDataResidue = host_assumed_data_length - len.get();
                }
                core::cmp::Ordering::Greater => {
                    csw.bCSWStatus = 0x02;
                }
            },
            rbc::Data::RecvDummy(_) => {
                csw.bCSWStatus = 0x02;
            }
        }
    }
    csw
}

async fn process_cbw<'d, 'a, 'b, D: embassy_usb_driver::Driver<'d>>(
    mass_storage: &mut MassStorageClassBbb<'d, D>,
    handler: &mut rbc::RbcHandler,
    cbw: &cds::Cbw<'a>,
    pin_red: &mut Output<'b, AnyPin>,
) -> Result<bool, u8> {
    let mut buffer = [0u8; 32];
    let data = handler.handle(cbw.CBWCB, &mut buffer).map_err(|_| 0x01)?;
    // 取り敢えず、Host の考える処理をする
    if cbw.dCBWDataTransferLength == 0 {
        // do nothing
    } else if cbw.bmCBWFlags & 0x80 == 0 {
        // host to device
        let mut needed = cbw.dCBWDataTransferLength;
        let mut buffer2 = [0u8; MAX_PACKET_SIZE as _];
        while needed > 0 {
            let read_size = mass_storage
                .bulk_out_ep
                .read(&mut buffer2)
                .await
                .map_err(|_| 0x04)?;
            needed = needed.saturating_sub(read_size as _);
        }
    } else {
        // device to host
        let mut needed = cbw.dCBWDataTransferLength;
        let real_send = match data {
            Some(rbc::Data::Send(data)) => data,
            _ => &[],
        };
        let mut real_send = &real_send[..core::cmp::min(needed as _, real_send.len())];
        while needed > 0 {
            let mut send_buffer = [0u8; MAX_PACKET_SIZE as _];
            let buf = &mut send_buffer[..core::cmp::min(needed, MAX_PACKET_SIZE as _) as _];
            let buf_len = buf.len();
            let read_send_len = real_send.len();
            let copy_read_send_len = core::cmp::min(buf_len, read_send_len);
            buf[..copy_read_send_len].copy_from_slice(&real_send[..copy_read_send_len]);
            mass_storage.bulk_in_ep.write(buf).await.map_err(|_| 0x08)?;
            let send_data = buf.len() as u32;
            needed = needed.saturating_sub(send_data);
            real_send = &real_send[copy_read_send_len as _..];
        }
    }
    let csw = match &data {
        Some(data) => manage_cbw_and_data(&cbw, data),
        None => cds::Csw {
            dCSWTag: cbw.dCBWTag,
            dCSWDataResidue: 0,
            bCSWStatus: 0x01,
        },
    };
    // Send Status
    mass_storage
        .bulk_in_ep
        .write(cds::byteify_csw(&mut buffer, &csw).map_err(|_| 0x04)?.0)
        .await
        .map_err(|_| 0x10)?;

    Ok(csw.bCSWStatus == 0x00)
}

async fn show_bits<'b, 'c>(
    val: u8,
    pin_red: &mut Output<'b, AnyPin>,
    pin_blue: &mut Output<'c, AnyPin>,
) {
    for i in 0..7 {
        pin_red.set_low();
        Timer::after(Duration::from_millis(500)).await;
        pin_red.set_high();
        if val & (0x1 << i) != 0 {
            pin_blue.set_low();
        } else {
            pin_blue.set_high();
        }
        Timer::after(Duration::from_millis(500)).await;
        pin_blue.set_high();
    }
}

// In は Host から見て Device が In するので、Device は Write する
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let config = config::Config::default();
    let p = embassy_rp::init(config);

    let mut pin_red = Output::new(p.PIN_17.degrade(), Level::High);
    let mut pin_blue = Output::new(p.PIN_25.degrade(), Level::High);

    // let mut _pin_green = Output::new(p.PIN_16.degrade(), Level::High);
    spawner.spawn(blink(p.PIN_16.degrade())).unwrap();

    let usb = p.USB;
    let driver = embassy_rp::usb::Driver::new(usb, Irqs);

    let mut usb_config = embassy_usb::Config::new(0xee, 0xff);
    usb_config.serial_number = Some("THISISTESTERIALNUMBER");

    let mut device_descriptor_buf = [0u8; 128];
    let mut config_descriptor_buf = [0u8; 128];
    let mut bos_descriptor_buf = [0u8; 128];
    let mut control_buf = [0u8; 128];

    let mut state = State::new();

    let mut builder = embassy_usb::Builder::new(
        driver,
        usb_config,
        &mut device_descriptor_buf,
        &mut config_descriptor_buf,
        &mut bos_descriptor_buf,
        &mut control_buf,
    );

    let mut mass_storage = MassStorageClassBbb::new(&mut builder, &mut state);

    let mut device = builder.build();

    let device_handle_future = device.run();
    let mass_storage_future = async {
        let mut first = true;
        loop {
            mass_storage.bulk_in_ep.wait_enabled().await;
            mass_storage.bulk_out_ep.wait_enabled().await;

            let mut out_packet = [0u8; MAX_PACKET_SIZE as _];

            let mut handler = rbc::RbcHandler::new();

            loop {
                match mass_storage.bulk_out_ep.read(&mut out_packet).await {
                    Ok(n) => match cds::parse_cbw(&out_packet[..n]) {
                        Ok(cbw) => {
                            match process_cbw(&mut mass_storage, &mut handler, &cbw, &mut pin_red)
                                .await
                            {
                                Ok(_success_command) => {
                                    pin_red.set_high();

                                    LED_BLINK.store(
                                        LED_BLINK.load(core::sync::atomic::Ordering::Relaxed) + 1,
                                        core::sync::atomic::Ordering::Relaxed,
                                    );
                                }
                                Err(num) => {
                                    // pin_red.set_low();
                                    show_bits(cbw.CBWCB.len() as _, &mut pin_red, &mut pin_blue)
                                        .await;
                                    pin_red.set_low();
                                    Timer::after(Duration::from_millis(1000)).await;
                                    show_bits(cbw.CBWCB[0], &mut pin_red, &mut pin_blue).await;
                                    pin_red.set_low();
                                    Timer::after(Duration::from_millis(1000)).await;
                                    show_bits(num, &mut pin_red, &mut pin_blue).await;
                                }
                            }
                        }
                        Err(_) => {
                            pin_red.set_low();
                            Timer::after(Duration::from_millis(3000)).await;
                            pin_red.set_high();
                        }
                    },
                    Err(_) => {
                        pin_red.set_low();
                        pin_red.set_low();
                        Timer::after(Duration::from_millis(3000)).await;
                        pin_red.set_high();
                        break;
                    }
                }
            }
        }
    };

    embassy_futures::join::join(device_handle_future, mass_storage_future).await;
}

static LED_BLINK: core::sync::atomic::AtomicU8 = core::sync::atomic::AtomicU8::new(0);

#[embassy_executor::task]
async fn blink(pin: AnyPin) {
    let mut led = Output::new(pin, Level::High);

    loop {
        let v1to10 = {
            let blink = LED_BLINK.load(core::sync::atomic::Ordering::Relaxed);
            if blink == 0 {
                0
            } else {
                LED_BLINK.load(core::sync::atomic::Ordering::Relaxed) % 9 + 1
            }
        };
        let v10to1 = 10 - v1to10;
        led.set_low();
        Timer::after(Duration::from_millis(100 * v1to10 as u64)).await;
        led.set_high();
        Timer::after(Duration::from_millis(100 * v10to1 as u64)).await;
    }
}
