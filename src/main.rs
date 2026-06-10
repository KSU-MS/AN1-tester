#![no_std]
#![no_main]

mod messages;
mod tasks;

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Delay, Timer};

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

use crate::tasks::vn_rs::{VectorNavBin20Hz, VectorNavBin400Hz, vectornav_task};

use crate::messages::{
    AccelFrame, AttitudeFrame, GyroFrame, PositionFrame, TimeFrame, VelocityFrame,
};

use {defmt_rtt as _, panic_probe as _};

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

    Timer::after_secs(2).await;
    info!("fuck sd");
    Timer::after_secs(2).await;

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

    let _ = spawner.spawn(vectornav_task(uart, &BIN_20HZ, &BIN_400HZ));

    loop {
        Timer::after_millis(1).await;

        if BIN_20HZ.signaled() {
            info!("Trying to send 20hz");

            let data = BIN_20HZ.try_take();

            if data.is_some() {
                let data = data.unwrap();

                info!("what {:?}", data.unix_time_ns);

                let _ = can_controller.tx_queue_transmit_message(
                    &TxMessage::from_frame(
                        ksu_rs_dbc::messages::EveloggerVectornavTime::new(data.unix_time_ns)
                            .unwrap(),
                    )
                    .unwrap(),
                );

                // let _ = can_controller.tx_queue_transmit_message(
                //     &TxMessage::from_frame(
                //         ksu_rs_dbc::messages::EveloggerVectornavPosition::new(
                //             data.position[0] as f32,
                //             data.position[1] as f32,
                //         )
                //         .unwrap(),
                //     )
                //     .unwrap(),
                // );

                // TODO: Finish INS state signal thing
                // let _ = can_controller.tx_queue_transmit_message(
                //     &TxMessage::from_frame(
                //         ksu_rs_dbc::messages::EveloggerVectornavTime::new(data.unix_time_ns).unwrap(),
                //     )
                //     .unwrap(),
                // );

                BIN_20HZ.reset();
            }
        }

        // if BIN_400HZ.signaled() {
        //     let data = BIN_400HZ.try_take();
        //
        //     if data.is_some() {
        //         let data = data.unwrap();
        //
        //         // let _ = can_controller.tx_queue_transmit_message(
        //         //     &TxMessage::from_frame(
        //         //         AttitudeFrame::new(data.attitude[0], data.attitude[1], data.attitude[2])
        //         //             .unwrap(),
        //         //     )
        //         //     .unwrap(),
        //         // );
        //         //
        //         // let _ = can_controller.tx_queue_transmit_message(
        //         //     &TxMessage::from_frame(
        //         //         GyroFrame::new(
        //         //             data.angular_rate[0],
        //         //             data.angular_rate[1],
        //         //             data.angular_rate[2],
        //         //         )
        //         //         .unwrap(),
        //         //     )
        //         //     .unwrap(),
        //         // );
        //         //
        //         // let _ = can_controller.tx_queue_transmit_message(
        //         //     &TxMessage::from_frame(
        //         //         VelocityFrame::new(data.velocity[0], data.velocity[1], data.velocity[2])
        //         //             .unwrap(),
        //         //     )
        //         //     .unwrap(),
        //         // );
        //         //
        //         // let _ = can_controller.tx_queue_transmit_message(
        //         //     &TxMessage::from_frame(
        //         //         AccelFrame::new(data.accel[0], data.accel[1], data.accel[2]).unwrap(),
        //         //     )
        //         //     .unwrap(),
        //         // );
        //     }
        //
        //     BIN_400HZ.reset();
        // }
    }
}
