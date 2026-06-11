#![no_std]
#![no_main]

mod messages;
mod tasks;

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Delay, Timer};

use log::info;

use embassy_rp::{
    bind_interrupts, gpio,
    peripherals::{self, USB},
    spi::{self, Spi},
    uart::{self, Uart},
    usb::{self, Driver},
};

use embedded_can::{ExtendedId, Id};

use mcp2518fd::{
    self, MCP2518FD,
    memory::controller::{
        configuration::OperationMode::NormalCan2,
        fifo::{FifoNumber, PayloadSize},
        filter::FilterNumber,
    },
    message::tx::TxMessage,
    settings::{
        BitTimeConfiguration, DataBitTimeConfiguration, FifoConfiguration, FifoMode,
        FilterConfiguration, FilterMatchMode, NominalBitTimeConfiguration, RxFifoConfiguration,
        Settings, TxFifoConfiguration,
    },
};

use crate::tasks::vn_rs::{VectorNavBin20Hz, VectorNavBin400Hz, vectornav_task};

use crate::messages::{
    AccelFrame, AttitudeFrame, GyroFrame, PositionFrame, TimeFrame, VelocityFrame,
};

use {defmt_rtt as _, panic_probe as _};

static BIN_20HZ: Signal<CriticalSectionRawMutex, VectorNavBin20Hz> = Signal::new();
static BIN_400HZ: Signal<CriticalSectionRawMutex, VectorNavBin400Hz> = Signal::new();

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
});
bind_interrupts!(struct UartIrqs {
    UART1_IRQ => uart::InterruptHandler<peripherals::UART1>;
});

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(4096, log::LevelFilter::Info, driver);
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let pins = embassy_rp::init(Default::default());

    let driver = Driver::new(pins.USB, Irqs);
    let _ = spawner.spawn(logger_task(driver));

    // The VectorNav is set to a weird baudrate to cram more data
    let mut uart_cfg = uart::Config::default();
    uart_cfg.baudrate = 230_400;

    let uart = Uart::new(
        pins.UART1,
        pins.PIN_24, // As labeled on board
        pins.PIN_25, // As labeled on board
        UartIrqs,
        pins.DMA_CH0,
        pins.DMA_CH1,
        uart_cfg,
    );

    let can_miso = pins.PIN_0;
    let can_mosi = pins.PIN_3;
    let can_clk = pins.PIN_2;

    let mut spi_cfg = spi::Config::default();
    spi_cfg.frequency = 20_000_000;

    let spi0 = Spi::new(
        pins.SPI0,
        can_clk,
        can_mosi,
        can_miso,
        pins.DMA_CH2,
        pins.DMA_CH3,
        spi_cfg,
    );
    let mut can = MCP2518FD::new(spi0, gpio::Output::new(pins.PIN_1, gpio::Level::High));

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

    // Set Fifo 1 for the 20hz bin
    can.configure_fifo(
        FifoNumber::Fifo1,
        FifoConfiguration {
            fifo_size: 32,
            payload_size: PayloadSize::Bytes8,
            mode: FifoMode::Transmit(TxFifoConfiguration::new(2)),
        },
    )
    .await
    .expect("Failed to configure FIFO 1 for 20hz bin");

    // Set Fifo 2 for the 400hz bin
    can.configure_fifo(
        FifoNumber::Fifo2,
        FifoConfiguration {
            fifo_size: 32,
            payload_size: PayloadSize::Bytes8,
            mode: FifoMode::Transmit(TxFifoConfiguration::new(3)),
        },
    )
    .await
    .expect("Failed to configure FIFO 2 for 400hz bin");

    // Set controller to CAN2
    can.set_op_mode(NormalCan2, &mut Delay)
        .await
        .expect("Failed to change chip operating mode");

    let _ = spawner.spawn(vectornav_task(uart, &BIN_20HZ, &BIN_400HZ));

    loop {
        match select(BIN_20HZ.wait(), BIN_400HZ.wait()).await {
            Either::First(data) => {
                let _ = can
                    .tx_fifo_push_message(
                        FifoNumber::Fifo1,
                        &TxMessage::from_frame(TimeFrame::new(data.unix_time_ns).unwrap()).unwrap(),
                    )
                    .await;

                let _ = can
                    .tx_fifo_push_message(
                        FifoNumber::Fifo1,
                        &TxMessage::from_frame(
                            PositionFrame::new(data.position[0] as f32, data.position[1] as f32)
                                .unwrap(),
                        )
                        .unwrap(),
                    )
                    .await;

                let _ = can
                    .tx_fifo_push_message(
                        FifoNumber::Fifo1,
                        &TxMessage::from_frame(
                            ksu_rs_dbc::messages::VectornavState::new(
                                data.inertial_navigation_state,
                            )
                            .unwrap(),
                        )
                        .unwrap(),
                    )
                    .await;

                let _ = can.tx_fifo_request_transmission(FifoNumber::Fifo1).await;
            }

            Either::Second(data) => {
                let _ = can
                    .tx_fifo_push_message(
                        FifoNumber::Fifo2,
                        &TxMessage::from_frame(
                            AttitudeFrame::new(
                                data.attitude[0],
                                data.attitude[1],
                                data.attitude[2],
                            )
                            .unwrap(),
                        )
                        .unwrap(),
                    )
                    .await;

                let _ = can
                    .tx_fifo_push_message(
                        FifoNumber::Fifo2,
                        &TxMessage::from_frame(
                            GyroFrame::new(
                                data.angular_rate[0],
                                data.angular_rate[1],
                                data.angular_rate[2],
                            )
                            .unwrap(),
                        )
                        .unwrap(),
                    )
                    .await;

                let _ = can
                    .tx_fifo_push_message(
                        FifoNumber::Fifo2,
                        &TxMessage::from_frame(
                            VelocityFrame::new(
                                data.velocity[0],
                                data.velocity[1],
                                data.velocity[2],
                            )
                            .unwrap(),
                        )
                        .unwrap(),
                    )
                    .await;

                let _ = can
                    .tx_fifo_push_message(
                        FifoNumber::Fifo2,
                        &TxMessage::from_frame(
                            AccelFrame::new(data.accel[0], data.accel[1], data.accel[2]).unwrap(),
                        )
                        .unwrap(),
                    )
                    .await;

                let _ = can.tx_fifo_request_transmission(FifoNumber::Fifo2).await;
            }
        }
    }
}
