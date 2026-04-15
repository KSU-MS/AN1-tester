#![no_std]
#![no_main]

// use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::{
    gpio::{Level, Output},
    i2c::{I2c},
    peripherals::{USB, I2C0},
    spi::{Config, Spi},
    usb::{Driver, InterruptHandler},
};
use embassy_time::Delay;
use embassy_time::Timer;
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
use mlx9064x::mlx90640::Mlx90640;
use mlx9064x::{MelexisCamera, Mlx90640Driver};

use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
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

    /* Send and receive messages forever */

    // let message = TxMessage::new_2_0(
    //     Id::Standard(StandardId::MAX),
    //     &[1, 2, 3, 4, 5, 6, 7, 8],
    // )
    // .expect("Message data is too long for frame kind (FD)")
    // .with_bit_rate_switched(true);

    // let spi1 = Spi::new(p.SPI1, p.PIN_14, p.PIN_27, p.PIN_28, p.DMA_CH2, p.DMA_CH3, Config::default());
    // let mut hx711 = Hx711::new(spi1);

    // hx711.reset_async().await.unwrap();
    // hx711.set_mode_async(hx711_spi::Mode::ChAGain64).await.unwrap();
    
    // let i2c0 = I2c::new_blocking(p.I2C0, p.PIN_25, p.PIN_24, embassy_rp::i2c::Config::default());
    // let mut cam = Mlx90640Driver::new(i2c0, 0x33).unwrap();
    
    // let mut temperatures = [0f32; Mlx90640::HEIGHT * Mlx90640::WIDTH];
    // let mut counter = 0;
    
    loop {
        // let _ = cam.generate_image_if_ready(&mut temperatures);
        
        // let v = (hx711.read_async().await.unwrap() >> 9) + 40;
        // info!("value = {:?}", v);

        let message = TxMessage::from_frame(ksu_rs_dbc::messages::CornernodeFrShockpot::new(10).unwrap()).unwrap();

        // Send a message with the TXQ
        can.tx_queue_transmit_message(&message)
            .await
            .expect("Failed to TX frame");

        // Read the message back (we are in loopback mode)
        // match can.rx_fifo_get_next(FifoNumber::Fifo1).await {
        //     Ok(Some(frame)) => info!("Received frame {:?}", frame),
        //     Ok(None) => info!("No message to read!"),
        //     Err(e) => info!("Error reading from FIFO: {:?}", e),
        // }

        // counter += 1;
        // info!("Tick {}", counter);
        Timer::after_millis(10000).await;
    }
}
