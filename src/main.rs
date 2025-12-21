#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::{
    gpio::{Level, Output},
    peripherals::USB,
    spi::{Config, Spi},
    usb::{Driver, InterruptHandler},
};
use embassy_time::Delay;
use embassy_time::Timer;
use embassy_usb_logger;
use log::info;
use mcp2518fd::{
    id::{ExtendedId, Id, StandardId},
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
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let driver = Driver::new(p.USB, Irqs);
    let _ = _spawner.spawn(logger_task(driver));
    info!("Hello World!");

    let can_miso = p.PIN_0;
    let can_mosi = p.PIN_3;
    let can_clk = p.PIN_2;

    let spi = Spi::new(
        p.SPI0,
        can_clk,
        can_mosi,
        can_miso,
        p.DMA_CH0,
        p.DMA_CH1,
        Config::default(),
    );
    let mut can = MCP2518FD::new(spi, Output::new(p.PIN_1, Level::High));

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

    // Set controller to internal loopback mode (all transmitted messages
    // will be immediately received)
    can.set_op_mode(OperationMode::InternalLoopback, &mut Delay)
        .await
        .expect("Failed to change chip operating mode");

    /* Send and receive messages forever */

    let message = TxMessage::new_2_0(
        Id::Standard(StandardId::MAX),
        &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16],
    )
    .expect("Message data is too long for frame kind (FD)")
    .with_bit_rate_switched(true);

    let mut counter = 0;

    loop {
        // Send a message with the TXQ
        can.tx_queue_transmit_message(&message)
            .await
            .expect("Failed to TX frame");

        // Read the message back (we are in loopback mode)
        match can.rx_fifo_get_next(FifoNumber::Fifo1).await {
            Ok(Some(frame)) => info!("Received frame {:?}", frame),
            Ok(None) => info!("No message to read!"),
            Err(e) => info!("Error reading from FIFO: {:?}", e),
        }

        counter += 1;
        info!("Tick {}", counter);
        Timer::after_secs(1).await;
    }
}
