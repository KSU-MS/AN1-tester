#![no_std]
#![no_main]

mod messages;
mod tasks;

// use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    gpio::{Level, Output},
    peripherals::{self, USB},
    spi::{self, Spi},
    uart::{self, Async, Uart},
    usb::{self, Driver},
};
use embassy_time::{Delay, Timer};
use embassy_usb_logger;
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
        BitTimeConfiguration, DataBitTimeConfiguration, FifoConfiguration, FifoMode,
        FilterConfiguration, FilterMatchMode, NominalBitTimeConfiguration, RxFifoConfiguration,
        Settings,
    },
    spi::MCP2518FD,
};

use crate::vn_rs::VectorNav;
// use crate::{can_helpers::set_joe_can, vn_rs::VectorNav};

use {defmt_rtt as _, panic_probe as _};

mod can_helpers;
mod vn_rs;

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
async fn main(_spawner: Spawner) {
    // Get our peripherals ready
    let pins = embassy_rp::init(Default::default());

    // Start serial print type shi
    let driver = Driver::new(pins.USB, UsbIrqs);
    let _ = _spawner.spawn(logger_task(driver)).unwrap();

    // Setup a SPI for the CAN Controller
    // let spi0 = Spi::new(
    //     pins.SPI0,
    //     pins.PIN_2,
    //     pins.PIN_3,
    //     pins.PIN_0,
    //     pins.DMA_CH0,
    //     pins.DMA_CH1,
    //     spi::Config::default(),
    // );
    // let mut can = MCP2518FD::new(spi0, Output::new(pins.PIN_1, Level::High));
    //
    // set_joe_can(can);

    // Setup a UART for the VN
    let mut uart_config_tech = uart::Config::default();
    uart_config_tech.baudrate = 230_400;

    let mut uart_controller = Uart::new(
        pins.UART1,
        pins.PIN_24,  // Board TX, VN RX
        pins.PIN_25,  // Board RX, VN TX
        UartIrqs,     // Given by the bind_interrupts! macro above
        pins.DMA_CH0, // Unused DMA channels?
        pins.DMA_CH1,
        uart_config_tech, // Adjusted baudrate
    );

    // info!("uart_controller return: {:?}", uart_controller.);

    info!(
        "Spawner return: {:?}",
        _spawner.spawn(uart_task(uart_controller)).unwrap()
    );

    loop {
        //
        //// VN Stage
        info!("morw");

        //
        //// CAN Stage
        // let message = TxMessage::from_frame(
        //     ksu_rs_dbc::messages::DashButtons::new(
        //         false,
        //         btn_5.is_low(),
        //         false,
        //         false,
        //         false,
        //         false,
        //     )
        //     .unwrap(),
        // )
        // .unwrap();
        //
        // // Send a message with the TXQ
        // can.tx_queue_transmit_message(&message)
        //     .await
        //     .expect("Failed to TX frame");

        Timer::after_millis(1000).await;
    }
}

#[embassy_executor::task]
async fn uart_task(mut uart_controller: Uart<'static, Async>) {
    let mut vn = VectorNav::new();

    let mut input_buf = [0u8; 1];
    let mut bin_buffer = [0u8; 2];

    loop {
        if uart_controller.read(&mut input_buf).await.is_err() {
            continue;
        };

        if !vn.check_sync_byte(input_buf) {
            continue;
        }

        if uart_controller.read(&mut bin_buffer).await.is_err() {
            continue;
        };

        match vn.check_bin(bin_buffer) {
            Ok(vn_rs::VectorNavBin::Bin20hz) => {
                unsafe {
                    if uart_controller
                        .read(&mut vn.bin1_data.buffer)
                        .await
                        .is_err()
                    {
                        continue;
                    };
                }

                let crc = vn.calc_crc(vn_rs::VectorNavBin::Bin20hz);

                if vn.load_values(crc).is_err() {
                    continue;
                };
            }
            Ok(vn_rs::VectorNavBin::Bin400hz) => {
                unsafe {
                    if uart_controller
                        .read(&mut vn.bin2_data.buffer)
                        .await
                        .is_err()
                    {
                        continue;
                    };
                }

                let crc = vn.calc_crc(vn_rs::VectorNavBin::Bin400hz);

                if vn.load_values(crc).is_err() {
                    continue;
                };
            }
            Err(_) => {
                continue;
            }
        };
    }
}
