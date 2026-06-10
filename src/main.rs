#![no_std]
#![no_main]

mod vn_rs;

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::Delay;

use embassy_rp::{
    bind_interrupts, gpio,
    peripherals::{self, USB},
    spi::{self, Spi},
    uart::{self, Uart},
    usb::{self, Driver},
};

use log::{error, info};

use mcp2518fd::{
    self, MCP2518FD,
    memory::controller::configuration::OperationMode::NormalCan2,
    message::tx::TxMessage,
    settings::{
        BitTimeConfiguration, DataBitTimeConfiguration, NominalBitTimeConfiguration, Settings,
    },
};

use {defmt_rtt as _, panic_probe as _};

use crate::vn_rs::{VectorNavBin, VectorNavBin20Hz, VectorNavBin400Hz};

static BIN_20HZ: Signal<CriticalSectionRawMutex, VectorNavBin20Hz> = Signal::new();
static BIN_400HZ: Signal<CriticalSectionRawMutex, VectorNavBin400Hz> = Signal::new();

bind_interrupts!(struct UsbIrqs {
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

    let driver = Driver::new(pins.USB, UsbIrqs);
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

    let _ = spawner.spawn(uart_task(uart));

    let spi0 = Spi::new(
        pins.SPI0,
        pins.PIN_2,
        pins.PIN_3,
        pins.PIN_0,
        pins.DMA_CH2,
        pins.DMA_CH3,
        spi::Config::default(),
    );
    let mut can_controller = MCP2518FD::new(spi0, gpio::Output::new(pins.PIN_1, gpio::Level::High));

    // Make sure the can_controller controller gets reset (in case the Pico reboots
    // without the MCP2518FD losing power)
    can_controller.reset().await.unwrap();

    // Configure the chip with default settings
    can_controller
        .configure(Settings::default(), &mut Delay)
        .await
        .expect("Failed to configure MCP2518");

    can_controller
        .configure_bit_timing(BitTimeConfiguration {
            nominal: NominalBitTimeConfiguration::RATE_500_KBIT,
            data: DataBitTimeConfiguration::RATE_500_KBIT,
        })
        .await
        .expect("Failed to set can_controller baudrate");

    // Set controller to can_controller2
    can_controller
        .set_op_mode(NormalCan2, &mut Delay)
        .await
        .expect("Failed to change chip operating mode");

    loop {
        if BIN_20HZ.signaled() {
            let data = BIN_20HZ.try_take().unwrap();

            let _ = can_controller.tx_queue_transmit_message(
                &TxMessage::from_frame(
                    ksu_rs_dbc::messages::EveloggerVectornavTime::new(data.unix_time_ns).unwrap(),
                )
                .unwrap(),
            );

            let _ = can_controller.tx_queue_transmit_message(
                &TxMessage::from_frame(
                    ksu_rs_dbc::messages::EveloggerVectornavPosition::new(
                        data.position[0] as f32,
                        data.position[1] as f32,
                    )
                    .unwrap(),
                )
                .unwrap(),
            );

            BIN_20HZ.reset();

            // TODO: Finish INS state signal thing
            // let _ = can_controller.tx_queue_transmit_message(
            //     &TxMessage::from_frame(
            //         ksu_rs_dbc::messages::EveloggerVectornavTime::new(data.unix_time_ns).unwrap(),
            //     )
            //     .unwrap(),
            // );
        }

        if BIN_400HZ.signaled() {
            let data = BIN_400HZ.try_take().unwrap();

            let _ = can_controller.tx_queue_transmit_message(
                &TxMessage::from_frame(
                    ksu_rs_dbc::messages::EveloggerVectornavAttitude::new(
                        data.attitude[0],
                        data.attitude[1],
                        data.attitude[2],
                    )
                    .unwrap(),
                )
                .unwrap(),
            );

            let _ = can_controller.tx_queue_transmit_message(
                &TxMessage::from_frame(
                    ksu_rs_dbc::messages::EveloggerVectornavGyro::new(
                        data.angular_rate[0],
                        data.angular_rate[1],
                        data.angular_rate[2],
                    )
                    .unwrap(),
                )
                .unwrap(),
            );

            let _ = can_controller.tx_queue_transmit_message(
                &TxMessage::from_frame(
                    ksu_rs_dbc::messages::EveloggerVectornavVelocity::new(
                        data.velocity[0],
                        data.velocity[1],
                        data.velocity[2],
                    )
                    .unwrap(),
                )
                .unwrap(),
            );

            let _ = can_controller.tx_queue_transmit_message(
                &TxMessage::from_frame(
                    ksu_rs_dbc::messages::EveloggerVectornavAcceleration::new(
                        data.accel[0],
                        data.accel[1],
                        data.accel[2],
                    )
                    .unwrap(),
                )
                .unwrap(),
            );

            BIN_400HZ.reset();
        }
    }
}

#[embassy_executor::task]
async fn uart_task(mut uart: Uart<'static, uart::Async>) {
    // Buffers to read data into
    let mut rx_buf = [0u8; 1];
    let mut bin_buf = [0u8; 3];
    let mut bin_20hz = [0u8; 36];
    let mut bin_400hz = [0u8; 50];

    loop {
        // Check the next byte
        let read_res = uart.read(&mut rx_buf).await;

        // If its our sync byte and the result was ok, read the bin
        if rx_buf == [0xFA] && read_res.is_ok() {
            if uart.read(&mut bin_buf).await.is_err() {
                continue;
            }

            // Figure out which packet was sent, and try to parse it, if everything works, update
            // the signal for the CAN fella to yeet
            match bin_buf {
                // 20hz bin identifier
                [0x01, 0x42, 0x10] => {
                    if uart.read(&mut bin_20hz).await.is_ok() {
                        match VectorNavBin20Hz::from_bytes(&bin_buf, &bin_20hz) {
                            Ok(bin) => {
                                BIN_20HZ.signal(bin);
                            }

                            Err(_) => {
                                continue;
                            }
                        };
                    };
                }

                // 400hz bin identifier
                [0x01, 0xA8, 0x01] => {
                    if uart.read(&mut bin_400hz).await.is_ok() {
                        match VectorNavBin400Hz::from_bytes(&bin_buf, &bin_400hz) {
                            Ok(bin) => {
                                BIN_400HZ.signal(bin);
                            }

                            Err(_) => {
                                continue;
                            }
                        }
                    };
                }

                // We got some garbo, skip
                _ => {
                    continue;
                }
            }
        }
    }
}
