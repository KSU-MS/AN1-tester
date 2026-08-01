#![no_std]
#![no_main]

mod messages;
mod tasks;

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Delay, Timer};

use embassy_rp::{
    adc::{self, Adc, Channel},
    bind_interrupts,
    gpio::{self, Input, Pull},
    peripherals::USB,
    spi::{self, Spi},
    usb::{self, Driver},
};

use mcp2518fd::{
    self, MCP2518FD,
    memory::controller::{
        configuration::OperationMode::NormalCan2,
        fifo::{FifoNumber, PayloadSize},
    },
    message::tx::TxMessage,
    settings::{
        BitTimeConfiguration, DataBitTimeConfiguration, FifoConfiguration, FifoMode,
        NominalBitTimeConfiguration, Settings, TxFifoConfiguration,
    },
};

use crate::{
    messages::{ButtonFrame, RearBPFrame, SteeringFrame},
    tasks::generic_button::generic_button_task,
};

use crate::tasks::generic_adc::generic_adc_task;
use crate::tasks::wheel_speed::wheel_speed_task;

use {defmt_rtt as _, panic_probe as _};

static REARBRAKE_RAW: Signal<CriticalSectionRawMutex, (u16, f32)> = Signal::new();
static STEERING_RAW: Signal<CriticalSectionRawMutex, (u16, f32)> = Signal::new();
static RTD_BUTTON: Signal<CriticalSectionRawMutex, bool> = Signal::new();

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
async fn main(spawner: Spawner) {
    // Get the GPIOs setup and get little fellas to fuck with them
    let pins = embassy_rp::init(Default::default());
    let adc = Adc::new(pins.ADC, AdcIrqs, adc::Config::default());

    // This gives us the ability to use the info! macro
    let driver = Driver::new(pins.USB, Irqs);
    let _ = spawner.spawn(logger_task(driver));

    // Set the spi config to 20mHz, its the max the chip can do
    let mut spi_cfg = spi::Config::default();
    spi_cfg.frequency = 20_000_000;

    let spi0 = Spi::new(
        pins.SPI0,
        pins.PIN_2,
        pins.PIN_3,
        pins.PIN_0,
        pins.DMA_CH2,
        pins.DMA_CH3,
        spi_cfg,
    );

    // Start with CS HIGH bc SPI, and pass the refrence of the pin in
    let mut can = MCP2518FD::new(spi0, gpio::Output::new(pins.PIN_1, gpio::Level::High));

    // Make sure the CAN controller gets reset (in case the Pico reboots
    // without the MCP2518FD losing power)
    can.reset().await.expect("Failed to reset MCP2518");

    // Configure the chip with 500K baud settings
    can.configure(Settings::default(), &mut Delay)
        .await
        .expect("Failed to configure MCP2518");

    can.configure_bit_timing(BitTimeConfiguration {
        nominal: NominalBitTimeConfiguration::RATE_500_KBIT,
        data: DataBitTimeConfiguration::RATE_500_KBIT,
    })
    .await
    .expect("Failed to set CAN baudrate");

    // Set Fifo 1 for the 50hz bin
    can.configure_fifo(
        FifoNumber::Fifo1,
        FifoConfiguration {
            fifo_size: 32,                                         // The max
            payload_size: PayloadSize::Bytes8, // None of our frames are more than 8 bytes rn
            mode: FifoMode::Transmit(TxFifoConfiguration::new(2)), // Not 0 for headroom
        },
    )
    .await
    .expect("Failed to configure FIFO 1 for 50hz bin");

    // Set controller to CAN2
    can.set_op_mode(NormalCan2, &mut Delay)
        .await
        .expect("Failed to change chip operating mode");

    // Set up the brake pressure reading task
    let rear_bp_adc_channel = Channel::new_pin(pins.PIN_27, Pull::None);
    let _ = spawner.spawn(generic_adc_task(adc, rear_bp_adc_channel, &REARBRAKE_RAW));

    // TODO: Make it so the generic_adc_task doesn't consume the ADC peripheral
    // let steering_adc_channel = Channel::new_pin(pins.PIN_29, Pull::None);
    // let _ = spawner.spawn(generic_adc_task(adc, steering_adc_channel, &STEERING_RAW));

    let rtd_button_pin = Input::new(pins.PIN_25, Pull::Up);
    let _ = spawner.spawn(generic_button_task(
        rtd_button_pin,
        &RTD_BUTTON,
        tasks::generic_button::Config::default(),
    ));

    loop {
        //
        //// The brake pressure bit
        if let Some(data) = REARBRAKE_RAW.try_take() {
            let k_pa: f32;

            // Vin = data * (3.3v/2^12 bit)
            // Vreal = Vin * Vdiv^-1
            // Vdiv = (1.5035 kOhm) / (1.0008 kOhm + 1.5035 kOhm)
            //
            // The sensor outputs 0-1500psi/0-10342kPa from 0.5v to 4.5v, assume anything past range that is clipping
            // out the sensor and is not good data
            //
            // Bf = (10342 - 0) / (4.5 - 0.5)
            //
            // This can be baked into the following check
            // 372 is 0.5 / (3.3/2^12 * Vdiv^-1)
            if data.0 < 372 {
                k_pa = 0_f32;
            }
            // 3354 is 4.5 / (3.3/2^12 * Vdiv^-1)
            else if data.0 > 3354 {
                k_pa = 10342_f32;
            } else {
                //kPa = (V - 0.5) * (10342 / (4.5 - 0.5))
                //kPa = (V - 372) * (10342 / 2980)
                k_pa = (data.0 - 372) as f32 * 3.47046979866_f32;
            }

            let an1_uint12 = data.0;

            let _ = can
                .tx_fifo_push_message(
                    FifoNumber::Fifo1,
                    &TxMessage::from_frame(
                        RearBPFrame::new(0_f32, k_pa as u16, an1_uint12).unwrap(),
                    )
                    .unwrap(),
                )
                .await;
        }

        //
        //// The steering pot bit
        // if let Some(data) = STEERING_RAW.try_take() {
        //     // TODO: Later /-\
        // }

        if let Some(data) = RTD_BUTTON.try_take() {
            let _ = can
                .tx_fifo_push_message(
                    FifoNumber::Fifo1,
                    &TxMessage::from_frame(
                        ButtonFrame::new(false, data, false, false, false, false).unwrap(),
                    )
                    .unwrap(),
                )
                .await;
        }

        // Throw the messages out onto the bus
        let _ = can.tx_fifo_request_transmission(FifoNumber::Fifo1).await;

        // 50 hz lol
        Timer::after_millis(20).await;
    }
}
