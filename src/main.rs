#![no_std]
#![no_main]

// use defmt::info;
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
use embassy_usb::class::cdc_acm::CdcAcmClass;
use embassy_usb_logger;
use hx711_spi::Hx711;
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
    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB-serial logger");
    config.serial_number = None;
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut logger_state = embassy_usb::class::cdc_acm::State::new();

    let mut builder = embassy_usb::Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );
    
    // Create a class for the logger
    let logger_class = CdcAcmClass::new(&mut builder, &mut logger_state, 64);

    let log_fut = embassy_usb_logger::with_custom_style!(8192, log::LevelFilter::Info, logger_class, |record, writer| {
        use core::fmt::Write;
        // let level = record.level().as_str();
        write!(writer, "{}", record.args()).unwrap();
    });
    let mut usb = builder.build();
    let usb_fut = usb.run();

    join(usb_fut, log_fut).await; 
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let driver = Driver::new(p.USB, Irqs);
    let _ = _spawner.spawn(logger_task(driver));
    Timer::after_secs(2).await;
    info!("Hello World!");

    // let can_miso = p.PIN_0;
    // let can_mosi = p.PIN_3;
    // let can_clk = p.PIN_2;

    // let spi0 = Spi::new(
    //     p.SPI0,
    //     can_clk,
    //     can_mosi,
    //     can_miso,
    //     p.DMA_CH0,
    //     p.DMA_CH1,
    //     Config::default(),
    // );
    // let mut can = MCP2518FD::new(spi0, Output::new(p.PIN_1, Level::High));

    // // Make sure the CAN controller gets reset (in case the Pico reboots
    // // without the MCP2518FD losing power)
    // can.reset().await.unwrap();

    // // Configure the chip with default settings
    // can.configure(Settings::default(), &mut Delay)
    //     .await
    //     .expect("Failed to configure MCP2518");

    // // Configure FIFO 1 as an RX FIFO to hold up to 16 messages with a max
    // // payload size of 64 bytes
    // can.configure_fifo(
    //     FifoNumber::Fifo1,
    //     FifoConfiguration {
    //         fifo_size: 16,
    //         payload_size: PayloadSize::Bytes64,
    //         mode: FifoMode::Receive(RxFifoConfiguration::new().with_message_timestamps(true)),
    //     },
    // )
    // .await
    // .expect("Failed to configure FIFO 1 as RX");

    // // Configure Filter 0 to accept all frame types (Standard or Extended),
    // // with any message ID (mask is all 0s)
    // can.configure_filter(
    //     FilterNumber::Filter0,
    //     Some(FilterConfiguration {
    //         buffer_pointer: FifoNumber::Fifo1,
    //         mode: FilterMatchMode::Both,
    //         filter_bits: Id::Extended(ExtendedId::ZERO),
    //         mask_bits: Id::Extended(ExtendedId::ZERO),
    //     }),
    // )
    // .await
    // .expect("Failed to configure Filter 0 for FIFO 1");

    // // Set controller to internal loopback mode (all transmitted messages
    // // will be immediately received)
    // can.set_op_mode(OperationMode::InternalLoopback, &mut Delay)
    //     .await
    //     .expect("Failed to change chip operating mode");

    // /* Send and receive messages forever */

    // let message = TxMessage::new_2_0(
    //     Id::Standard(StandardId::MAX),
    //     &[1, 2, 3, 4, 5, 6, 7, 8],
    // )
    // .expect("Message data is too long for frame kind (FD)")
    // .with_bit_rate_switched(true);

    let spi1 = Spi::new(p.SPI1, p.PIN_26, p.PIN_27, p.PIN_28, p.DMA_CH2, p.DMA_CH3, Config::default());
    let mut hx711 = Hx711::new(spi1);

    hx711.reset_async().await.unwrap();
    hx711.set_mode_async(hx711_spi::Mode::ChAGain64).await.unwrap();
    let mut counter = 0;

    loop {
        // Send a message with the TXQ
        // can.tx_queue_transmit_message(&message)
        //     .await
        //     .expect("Failed to TX frame");

        // // Read the message back (we are in loopback mode)
        // match can.rx_fifo_get_next(FifoNumber::Fifo1).await {
        //     Ok(Some(frame)) => info!("Received frame {:?}", frame),
        //     Ok(None) => info!("No message to read!"),
        //     Err(e) => info!("Error reading from FIFO: {:?}", e),
        // }

        let v = (hx711.read_async().await.unwrap() >> 9) + 40;
        info!("value = {:?}", v);


        // counter += 1;
        // info!("Tick {}", counter);
        Timer::after_millis(500).await;
    }
}
