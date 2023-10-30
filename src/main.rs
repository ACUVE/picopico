#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_rp::gpio::{AnyPin, Input, Level, Output, Pin, Pull};
use embassy_rp::uart::{self, Uart};
use embassy_rp::{bind_interrupts, config};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    UART0_IRQ => embassy_rp::uart::InterruptHandler<embassy_rp::peripherals::UART0>;
});

// ループコイルの入力を受け付ける（PullUp で High が非検知、 Low が検知）
// その入力を、精算機に UART で伝送する
// 精算機からシリアル通信を受け、サーボモータを制御する

// Signal は PubSubChannel の方が本当は良いかもしれない

async fn uart_task(
    uart: Uart<'_, impl uart::Instance, uart::Async>,
    coil: &Signal<impl embassy_sync::blocking_mutex::raw::RawMutex, bool>,
    servo: &Signal<impl embassy_sync::blocking_mutex::raw::RawMutex, bool>,
) {
    let (mut tx, mut rx) = uart.split();

    embassy_futures::join::join(
        async {
            servo.signal(false);

            let mut buff = [0; 1];
            loop {
                use embassy_rp::uart::Error::*;
                match rx.read(&mut buff).await {
                    Ok(()) => {
                        // TODO: なんか通信処理する
                        servo.signal(true);
                    }
                    Err(Overrun) => {}
                    Err(Break) => {}
                    Err(Parity) => {}
                    Err(Framing) => {}
                    Err(_) => {}
                }
            }
        },
        async {
            loop {
                // XXX: select 使えば、複数のシグナルを待てる https://docs.embassy.dev/embassy-futures/git/default/select/index.html
                let current = coil.wait().await;
                use embassy_rp::uart::Error::*;
                // TODO: なんか通信処理する
                match tx.write(&[current as u8]).await {
                    Ok(()) => {}
                    Err(Overrun) => {}
                    Err(Break) => {}
                    Err(Parity) => {}
                    Err(Framing) => {}
                    Err(_) => {}
                }
            }
        },
    )
    .await;
}

async fn coil_task(
    mut input: Input<'_, AnyPin>,
    coil: &Signal<impl embassy_sync::blocking_mutex::raw::RawMutex, bool>,
) {
    let mut current_level: bool = input.get_level().into();
    loop {
        coil.signal(!current_level);
        if current_level {
            input.wait_for_low().await;
            current_level = false;
        } else {
            input.wait_for_high().await;
            current_level = true;
        }
    }
}

async fn servo_task(
    servo: &Signal<impl embassy_sync::blocking_mutex::raw::RawMutex, bool>,
    mut output: Output<'_, AnyPin>,
) {
    loop {
        // サーボモータを制御する
        let current = servo.wait().await;

        // TODO: なんかサーボモータを制御する
        output.set_level(current.into());
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let config = config::Config::default();
    let p = embassy_rp::init(config);

    let input = Input::new(p.PIN_26.degrade(), Pull::Up);
    let output = Output::new(p.PIN_27.degrade(), Level::High);
    let uart = Uart::new(
        p.UART0,
        p.PIN_0,
        p.PIN_1,
        Irqs,
        p.DMA_CH0,
        p.DMA_CH1,
        Default::default(),
    );

    let coil_signal = Signal::<CriticalSectionRawMutex, _>::new();
    let servo_signal = Signal::<CriticalSectionRawMutex, _>::new();

    embassy_futures::join::join3(
        uart_task(uart, &coil_signal, &servo_signal),
        servo_task(&servo_signal, output),
        coil_task(input, &coil_signal),
    )
    .await;
}
