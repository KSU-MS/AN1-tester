#![no_std]
#![no_main]

// use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::adc::{Adc, Channel};
use embassy_rp::gpio::{Input, Pin, Pull};
use embassy_rp::{adc, bind_interrupts};
use embassy_rp::{
    gpio::{Level, Output},
    peripherals::USB,
    spi::{Config, Spi},
    usb::{self, Driver},
};
use embassy_time::Delay;
use embassy_time::Timer;
use embassy_usb_logger;
use log::info;
use mcp2518fd::settings::{
    BitTimeConfiguration, DataBitTimeConfiguration, NominalBitTimeConfiguration,
};
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
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
});

bind_interrupts!(struct AdcIrqs {
        ADC_IRQ_FIFO => adc::InterruptHandler;
});

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(4096, log::LevelFilter::Info, driver);
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut adc = Adc::new(p.ADC, AdcIrqs, adc::Config::default());

    let driver = Driver::new(p.USB, Irqs);
    let _ = _spawner.spawn(logger_task(driver));

    let btn_5 = Input::new(p.PIN_25, Pull::Up);
    let mut steering_pot = Channel::new_pin(p.PIN_29, Pull::None);
    let mut bpr = Channel::new_pin(p.PIN_27, Pull::None);

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

    can.configure_bit_timing(BitTimeConfiguration {
        nominal: NominalBitTimeConfiguration::RATE_500_KBIT,
        data: DataBitTimeConfiguration::RATE_500_KBIT,
    })
    .await
    .expect("Failed to set CAN baudrate");

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

    loop {
        //
        //// Dash button
        let message = TxMessage::from_frame(
            ksu_rs_dbc::messages::DashButtons::new(
                false,
                btn_5.is_low(),
                false,
                false,
                false,
                false,
            )
            .unwrap(),
        )
        .unwrap();

        can.tx_queue_transmit_message(&message)
            .await
            .expect("Failed to TX frame");

        //
        //// Rear brake pressure
        let brake_pressure_uint = adc.read(&mut bpr).await.unwrap();

        let message = TxMessage::from_frame(
            ksu_rs_dbc::messages::CornernodeRearBrakepressure::new(brake_pressure_uint).unwrap(),
        )
        .unwrap();

        can.tx_queue_transmit_message(&message)
            .await
            .expect("Failed to TX frame");

        //
        //// Steering pot
        let steering_pot_uint = adc.read(&mut steering_pot).await.unwrap();

        let message = TxMessage::from_frame(
            ksu_rs_dbc::messages::CornernodeSteeringpot::new(steering_pot_uint).unwrap(),
        )
        .unwrap();

        can.tx_queue_transmit_message(&message)
            .await
            .expect("Failed to TX frame");

        Timer::after_millis(20).await; // 50hz
    }
}
