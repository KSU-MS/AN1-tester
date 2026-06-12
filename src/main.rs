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

use crate::messages::{ShockFrame, SpeedFrame};

use crate::tasks::generic_adc::generic_adc_task;
use crate::tasks::wheel_speed::wheel_speed_task;

use {defmt_rtt as _, panic_probe as _};

static SHOCKPOT_RAW: Signal<CriticalSectionRawMutex, (u16, f32)> = Signal::new();
static WHEEL_SPEED: Signal<CriticalSectionRawMutex, (u16, f32)> = Signal::new();

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
    let mut adc = Adc::new(pins.ADC, AdcIrqs, adc::Config::default());

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

    // Configure the chip with 1000K baud settings
    can.configure(Settings::default(), &mut Delay)
        .await
        .expect("Failed to configure MCP2518");

    can.configure_bit_timing(BitTimeConfiguration {
        nominal: NominalBitTimeConfiguration::RATE_1_MBIT,
        data: DataBitTimeConfiguration::RATE_1_MBIT,
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

    // Set up the shockpot reading task
    let shock_pot_adc_channel = Channel::new_pin(pins.PIN_26, Pull::None);
    let _ = spawner.spawn(generic_adc_task(adc, shock_pot_adc_channel, &SHOCKPOT_RAW));

    // Set up the wheelspeed reading task
    let wheel_speed_pin = Input::new(pins.PIN_29, Pull::Down);
    let _ = spawner.spawn(wheel_speed_task(wheel_speed_pin, &WHEEL_SPEED));

    loop {
        //
        //// The shockpot bit
        if let Some(data) = SHOCKPOT_RAW.try_take() {
            // V = x bit * (3.3v/2^12 bit)

            // From this point on, we assume we have the blue shockpot with 75mm stroke, the max
            // voltage should be 99.5% of vin for 0mm extended, and 0.05% of vin for 75mm extended

            // length = (((3.3v * .995) - V) / ((3.3v * .995) - (3.3v * 0.05))) * 75mm

            // This can be baked into the following
            // (4076 - x) * 0.018495501894
            let length_mm = (4076 - data.0) as f32 * 0.018495501894_f32;

            let length_delta = (4076_f32 - data.1) * 0.018495501894_f32;

            let an1_uint12 = data.0;

            let _ = can
                .tx_fifo_push_message(
                    FifoNumber::Fifo1,
                    &TxMessage::from_frame(
                        ShockFrame::new(length_delta, length_mm, an1_uint12).unwrap(),
                    )
                    .unwrap(),
                )
                .await;
        }

        //
        //// The wheel speed bit
        if let Some(data) = WHEEL_SPEED.try_take() {
            let rpm_delta = data.1;
            let rpm = data.0;

            let _ = can
                .tx_fifo_push_message(
                    FifoNumber::Fifo1,
                    &TxMessage::from_frame(SpeedFrame::new(rpm_delta, rpm).unwrap()).unwrap(),
                )
                .await;
        }

        //
        //// The loadcell bit
        // NOTE: We don't have the pushrods yet lol

        // Throw the messages out
        let _ = can.tx_fifo_request_transmission(FifoNumber::Fifo1).await;

        // 50 hz lol
        Timer::after_millis(20).await;
    }
}
