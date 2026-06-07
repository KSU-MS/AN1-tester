#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    peripherals::{self, USB},
    uart::{self, Async, Uart},
    usb::{self, Driver},
};
use embassy_time::Timer;
use embassy_usb_logger;
use log::{error, info};

use {defmt_rtt as _, panic_probe as _};

mod vn_rs;

use vn_rs::VectorNavData;

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

#[embassy_executor::task]
async fn uart_task(mut uart: Uart<'static, Async>) {
    let mut vn = VectorNavData::new();

    let mut rx_buf = [0u8; 64];

    loop {
        let read_res = uart.read(&mut rx_buf).await;
        if read_res.is_err() {
            info!("{:?}", read_res);
            continue;
        }

        vn.update(rx_buf);
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let driver = Driver::new(p.USB, UsbIrqs);
    spawner.spawn(logger_task(driver)).unwrap();

    let mut uart_cfg = uart::Config::default();
    uart_cfg.baudrate = 230_400;

    let uart = Uart::new(
        p.UART1, p.PIN_24, p.PIN_25, UartIrqs, p.DMA_CH2, p.DMA_CH3, uart_cfg,
    );

    spawner.spawn(uart_task(uart)).unwrap();

    loop {}
}
