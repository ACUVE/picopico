//! This example shows powerful PIO module in the RP2040 chip.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::mem::MaybeUninit;

use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::{bind_interrupts, config};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<embassy_rp::peripherals::PIO0>;
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<embassy_rp::peripherals::USB>;
});

mod codes {
    // Audio Interface Class Code
    pub const AUDIO: u8 = 0x01;

    // Audio Interface Subclass Codes
    pub const SUBCLASS_UNDEFINED: u8 = 0x00;
    pub const AUDIOCONTROL: u8 = 0x01;
    pub const AUDIOSTREAMING: u8 = 0x02;
    pub const MIDISTREAMING: u8 = 0x03;

    // Audio Interface Protocol Codes
    pub const PR_PROTOCOL_UNDEFINED: u8 = 0x00;

    // Audio Class-Specific Descriptor Types
    pub const CS_UNDEFINED: u8 = 0x20;
    pub const CS_DEVICE: u8 = 0x21;
    pub const CS_CONFIGURATION: u8 = 0x22;
    pub const CS_STRING: u8 = 0x23;
    pub const CS_INTERFACE: u8 = 0x24;
    pub const CS_ENDPOINT: u8 = 0x25;

    // Audio Class-Specific AC Interface Descriptor Subtypes
    pub const AC_DESCRIPTOR_UNDEFINED: u8 = 0x00;
    pub const HEADER: u8 = 0x01;
    pub const INPUT_TERMINAL: u8 = 0x02;
    pub const OUTPUT_TERMINAL: u8 = 0x03;
    pub const MIXER_UNIT: u8 = 0x04;
    pub const SELECTOR_UNIT: u8 = 0x05;
    pub const FEATURE_UNIT: u8 = 0x06;
    pub const PROCESSING_UNIT: u8 = 0x07;
    pub const EXTENSION_UNIT: u8 = 0x08;

    // Audio Class-Specific AS Interface Descriptor Subtypes
    pub const AS_DESCRIPTOR_UNDEFINED: u8 = 0x00;
    pub const AS_GENERAL: u8 = 0x01;
    pub const FORMAT_TYPE: u8 = 0x02;
    pub const FORMAT_SPECIFIC: u8 = 0x03;

    // Processing Unit Process Types
    pub const PROCESS_UNDEFINED: u8 = 0x00;
    pub const UP_DOWNMIX_PROCESS: u8 = 0x01;
    pub const DOLBY_PROLOGIC_PROCESS: u8 = 0x02;
    pub const THREED_STEREO_EXTENDER_PROCESS: u8 = 0x03;
    pub const REVERBERATION_PROCESS: u8 = 0x04;
    pub const CHORUS_PROCESS: u8 = 0x05;
    pub const DYN_RANGE_COMP_PROCESS: u8 = 0x06;

    // DESCRIPTOR_UNDEFINED
    pub const DESCRIPTOR_UNDEFINED: u8 = 0x00;
    pub const EP_GENERAL: u8 = 0x01;

    // Audio Class-Specific Request Codes
    pub const REQUEST_CODE_UNDEFINED: u8 = 0x00;
    pub const SET_CUR: u8 = 0x01;
    pub const GET_CUR: u8 = 0x81;
    pub const SET_MIN: u8 = 0x02;
    pub const GET_MIN: u8 = 0x82;
    pub const SET_MAX: u8 = 0x03;
    pub const GET_MAX: u8 = 0x83;
    pub const SET_RES: u8 = 0x04;
    pub const GET_RES: u8 = 0x84;
    pub const SET_MEM: u8 = 0x05;
    pub const GET_MEM: u8 = 0x85;
    pub const GET_STAT: u8 = 0xFF;
}

use codes::*;
use embassy_usb_driver::{Endpoint, EndpointIn};

struct Handler {}

impl embassy_usb::Handler for Handler {
    fn control_in<'a>(
        &'a mut self,
        _req: embassy_usb::control::Request,
        _buf: &'a mut [u8],
    ) -> Option<embassy_usb::control::InResponse<'a>> {
        None
    }
    fn control_out(
        &mut self,
        _req: embassy_usb::control::Request,
        _data: &[u8],
    ) -> Option<embassy_usb::control::OutResponse> {
        None
    }
}

// In は Host から見て Device が In するので、Device は Write する
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let config = config::Config::default();
    let p = embassy_rp::init(config);

    let _g = Output::new(p.PIN_16, Level::High);
    let mut r = Output::new(p.PIN_17, Level::High);
    let mut b = Output::new(p.PIN_25, Level::High);

    let usb = p.USB;
    let driver = embassy_rp::usb::Driver::new(usb, Irqs);

    let mut usb_config = embassy_usb::Config::new(0xee, 0xff);
    usb_config.serial_number = Some("THISISTESTERIALNUMBER");

    let mut device_descriptor_buf = [0u8; 128];
    let mut config_descriptor_buf = [0u8; 512];
    let mut bos_descriptor_buf = [0u8; 128];
    let mut control_buf = [0u8; 128];

    // let mut handler = Handler {};
    let mut handler = MaybeUninit::uninit();

    let mut builder = embassy_usb::Builder::new(
        driver,
        usb_config,
        &mut device_descriptor_buf,
        &mut config_descriptor_buf,
        &mut bos_descriptor_buf,
        &mut control_buf,
    );

    // device -> config(function) -> interface -> alternate setting -> endpoint
    // 通常、 1 interface が 1 driver に対応するが、IAD を使うと 1 driver が複数の interface を持てるので、統合して、 function と呼ぶ
    // そもそも configulation には class はない（IAD でないと、この function に与えた class などは無視される）
    let mut function = builder.function(0x00, 0x00, 0x00);
    let mut interface = function.interface();
    let mut alt_setting = interface.alt_setting(AUDIO, AUDIOCONTROL, PR_PROTOCOL_UNDEFINED, None);
    alt_setting.descriptor(
        CS_INTERFACE,
        &[
            HEADER, // bDescriptorSubtype
            0x00, 0x01, // bcdADC
            0x1e, 0x00, // wTotalLength
            0x01, // bInCollection
            0x01, // baInterfaceNr
        ],
    );
    alt_setting.descriptor(
        CS_INTERFACE,
        &[
            INPUT_TERMINAL, // bDescriptorSubtype
            0x01,           // bTerminalID
            0x01,
            0x02, // wTerminalType
            0x00, // bAssocTerminal
            0x01, // bNrChannels
            0x00,
            0x00, // wChannelConfig
            0x00, // iChannelNames
            0x00, // iTerminal
        ],
    );
    alt_setting.descriptor(
        CS_INTERFACE,
        &[
            OUTPUT_TERMINAL, // bDescriptorSubtype
            0x02,            // bTerminalID
            0x01,
            0x01, // wTerminalType
            0x00, // bAssocTerminal
            0x01, // bSourceID
            0x00, // iTerminal
        ],
    );
    let mut interface = function.interface();
    let mut alt_setting = interface.alt_setting(AUDIO, AUDIOSTREAMING, PR_PROTOCOL_UNDEFINED, None);
    let mut alt_setting = interface.alt_setting(AUDIO, AUDIOSTREAMING, PR_PROTOCOL_UNDEFINED, None);
    alt_setting.descriptor(
        CS_INTERFACE,
        &[
            AS_GENERAL, // bDescriptorSubtype
            0x02,       // bTerminalLink
            0x01,       // bDelay
            0x01, 0x00, // wFormatTag
        ],
    );
    alt_setting.descriptor(
        CS_INTERFACE,
        &[
            FORMAT_TYPE, // bDescriptorSubtype
            0x01,        // bFormatType
            0x01,        // bNrChannels
            0x02,        // bSubframeSize
            0x10,        // bBitResolution
            0x01,        // bSamFreqType
            0x40,
            0x1f,
            0x00, // tSamFreq
        ],
    );
    let mut endpoint = alt_setting.endpoint_isochronous_in(0x10, 0x01);
    alt_setting.descriptor(
        CS_ENDPOINT,
        &[
            EP_GENERAL, 0x00, // bmAttributes
            0x00, // bLockDelayUnits
            0x00, 0x00, // wLockDelay
        ],
    );

    let handler = handler.write(Handler {});
    drop(function);

    builder.handler(handler);

    let mut device = builder.build();

    b.set_low();

    const SQUARE: [u8; 16] = [
        0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ];

    embassy_futures::join::join(device.run(), async {
        loop {
            endpoint.wait_enabled().await;
            // write 16 bytes 0x00
            loop {
                r.set_level(r.is_set_low().into());
                match endpoint.write(&SQUARE).await {
                    Ok(()) => {}
                    Err(embassy_usb_driver::EndpointError::BufferOverflow) => {
                        continue;
                    }
                    Err(embassy_usb_driver::EndpointError::Disabled) => {
                        break;
                    }
                }
            }
        }
    })
    .await;
}
