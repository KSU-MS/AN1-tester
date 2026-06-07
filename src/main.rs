#![no_std]
#![no_main]

mod messages;
mod tasks;

// use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::adc::{self, Adc, Channel};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c::{I2c};
use embassy_rp::peripherals::{USB};
use embassy_rp::spi::{Config, Spi};
use embassy_rp::usb::{self, Driver};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Delay;
use embassy_time::Timer;
use embassy_usb_logger;
use hx711_spi::Hx711;
use log::info;
use mcp2518fd::{
    id::{ExtendedId, Id},
    memory::controller::{
        configuration::OperationMode,
        fifo::{FifoNumber, PayloadSize},
        filter::FilterNumber,
    },
    message::tx::TxMessage,
    settings::{
        FifoConfiguration, FifoMode, FilterConfiguration, FilterMatchMode, RxFifoConfiguration,
        Settings,
    },
    spi::MCP2518FD,
};
use mlx9064x::Mlx90640Driver;

use crate::tasks::tire_temp::tire_temp_task;
use crate::tasks::wheel_speed::wheel_speed_task;

use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
    ADC_IRQ_FIFO => adc::InterruptHandler;
});

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(4096, log::LevelFilter::Info, driver);
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let driver = Driver::new(p.USB, Irqs);
    let _ = _spawner.spawn(logger_task(driver));
    Timer::after_secs(2).await;
    info!("Hello World!");

    let wheel_speed = Input::new(p.PIN_29, Pull::None);
    static SPEED: Signal<CriticalSectionRawMutex, u16> = Signal::new();
    let _ = _spawner.spawn(wheel_speed_task(wheel_speed, &SPEED));

    let mut adc = Adc::new(p.ADC, Irqs, adc::Config::default());
    let mut shockpot = Channel::new_pin(p.PIN_26, Pull::None);

    // let i2c0 = I2c::new_blocking(p.I2C0, p.PIN_25, p.PIN_24, embassy_rp::i2c::Config::default());
    // let cam = Mlx90640Driver::new(i2c0, 0x33).unwrap();
    // static TEMP: Signal<CriticalSectionRawMutex, f32> = Signal::new();
    // let _ = _spawner.spawn(tire_temp_task(cam, &TEMP));

    let can_miso = p.PIN_0;
    let can_mosi = p.PIN_3;
    let can_clk = p.PIN_2;

    let spi0 = Spi::new(
        p.SPI0,
        can_clk,
        can_mosi,
        can_miso,
        p.DMA_CH0,
        p.DMA_CH1,
        Config::default(),
    );
    let mut can = MCP2518FD::new(spi0, Output::new(p.PIN_1, Level::High));

    // Make sure the CAN controller gets reset (in case the Pico reboots
    // without the MCP2518FD losing power)
    can.reset().await.unwrap();

    // Configure the chip with default settings
    can.configure(Settings::default(), &mut Delay)
        .await
        .expect("Failed to configure MCP2518");

    // Configure FIFO 1 as an RX FIFO to hold up to 16 messages with a max
    // payload size of 64 bytes
    can.configure_fifo(
        FifoNumber::Fifo1,
        FifoConfiguration {
            fifo_size: 16,
            payload_size: PayloadSize::Bytes64,
            mode: FifoMode::Receive(RxFifoConfiguration::new().with_message_timestamps(true)),
        },
    )
    .await
    .expect("Failed to configure FIFO 1 as RX");

    // Configure Filter 0 to accept all frame types (Standard or Extended),
    // with any message ID (mask is all 0s)
    can.configure_filter(
        FilterNumber::Filter0,
        Some(FilterConfiguration {
            buffer_pointer: FifoNumber::Fifo1,
            mode: FilterMatchMode::Both,
            filter_bits: Id::Extended(ExtendedId::ZERO),
            mask_bits: Id::Extended(ExtendedId::ZERO),
        }),
    )
    .await
    .expect("Failed to configure Filter 0 for FIFO 1");

    // Set controller to CAN2
    can.set_op_mode(OperationMode::NormalCan2, &mut Delay)
        .await
        .expect("Failed to change chip operating mode");


    // let spi1 = Spi::new(p.SPI1, p.PIN_14, p.PIN_27, p.PIN_28, p.DMA_CH2, p.DMA_CH3, Config::default());
    // let mut hx711 = Hx711::new(spi1);

    // hx711.reset_async().await.unwrap();
    // hx711.set_mode_async(hx711_spi::Mode::ChAGain64).await.unwrap();

    // let mut counter = 0;

    loop {
        // let force = (hx711.read_async().await.unwrap() >> 9) + 40;
        // let _ = can.tx_queue_transmit_message(&TxMessage::from_frame(messages::LoadCellFrame::new(force).unwrap()).unwrap()).await;

        let shockpot_reading = adc.read(&mut shockpot).await.unwrap_or_default();
        let _ = can.tx_queue_transmit_message(&TxMessage::from_frame(messages::ShockpotFrame::new(shockpot_reading).unwrap()).unwrap()).await;

        // if temp.signaled() {
        //     let _ = can.tx_queue_transmit_message(&TxMessage::from_frame(messages::TiretempFrame::new(temp.try_take().unwrap()).unwrap()).unwrap()).await;
        // }

        if SPEED.signaled() {
            let _ = can.tx_queue_transmit_message(&TxMessage::from_frame(messages::WheelspeedFrame::new(SPEED.try_take().unwrap()).unwrap()).unwrap()).await;
        }
        // Send a message with the TXQ

        // Read the message back (we are in loopback mode)
        // match can.rx_fifo_get_next(FifoNumber::Fifo1).await {
        //     Ok(Some(frame)) => info!("Received frame {:?}", frame),
        //     Ok(None) => info!("No message to read!"),
        //     Err(e) => info!("Error reading from FIFO: {:?}", e),
        // }

        Timer::after_millis(1).await;
    }
}
