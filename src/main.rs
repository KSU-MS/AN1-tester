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

use crate::messages::*;

use crate::tasks::generic_adc::generic_adc_task;
use crate::tasks::wheel_speed::wheel_speed_task;

use {defmt_rtt as _, panic_reset as _};

cfg_select! {
    not(feature = "rear_unified") => {
        static SHOCKPOT_RAW: Signal<CriticalSectionRawMutex, (u16, f32)> = Signal::new();
        static WHEEL_SPEED: Signal<CriticalSectionRawMutex, (u16, f32)> = Signal::new();
    }
    feature = "rear_unified" => {
        static SHOCKPOT_R_RAW: Signal<CriticalSectionRawMutex, (u16, f32)> = Signal::new();
        static SHOCKPOT_L_RAW: Signal<CriticalSectionRawMutex, (u16, f32)> = Signal::new();
        static WHEEL_R_SPEED: Signal<CriticalSectionRawMutex, (u16, f32)> = Signal::new();
        static WHEEL_L_SPEED: Signal<CriticalSectionRawMutex, (u16, f32)> = Signal::new();
    }
}

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
    let p = embassy_rp::init(Default::default());
    let adc = Adc::new(p.ADC, AdcIrqs, adc::Config::default());

    // This gives us the ability to use the info! macro
    // let driver = Driver::new(p.USB, Irqs);
    // let _ = spawner.spawn(logger_task(driver));

    // Set the spi config to 20mHz, its the max the chip can do
    let mut spi_cfg = spi::Config::default();
    spi_cfg.frequency = 20_000_000;

    let spi0 = Spi::new(
        p.SPI0, p.PIN_2, p.PIN_3, p.PIN_0, p.DMA_CH2, p.DMA_CH3, spi_cfg,
    );

    // Start with CS HIGH bc SPI, and pass the reference of the pin in
    let mut can = MCP2518FD::new(spi0, gpio::Output::new(p.PIN_1, gpio::Level::High));

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
    #[cfg(feature = "rear_unified")]
    let _ = spawner.spawn(generic_adc_task(
        adc,
        [
            Channel::new_pin(p.PIN_29, Pull::None),
            Channel::new_pin(p.PIN_28, Pull::None),
        ],
        [&SHOCKPOT_L_RAW, &SHOCKPOT_R_RAW],
    ));
    #[cfg(not(feature = "rear_unified"))]
    let _ = spawner.spawn(generic_adc_task(
        adc,
        [Channel::new_pin(p.PIN_29, Pull::None)],
        [&SHOCKPOT_RAW],
    ));

    // Set up the wheelspeed reading task
    #[cfg(feature = "rear_unified")]
    {
        let _ = spawner.spawn(wheel_speed_task(
            Input::new(p.PIN_25, Pull::None),
            &WHEEL_L_SPEED,
        ));
        let _ = spawner.spawn(wheel_speed_task(
            Input::new(p.PIN_24, Pull::None),
            &WHEEL_R_SPEED,
        ));
    }
    #[cfg(not(feature = "rear_unified"))]
    let _ = spawner.spawn(wheel_speed_task(
        Input::new(p.PIN_25, Pull::None),
        &WHEEL_SPEED,
    ));

    loop {
        // * The shockpot bit
        #[cfg(not(feature = "rear_unified"))]
        if let Some(msg) = shock(&SHOCKPOT_RAW, |d: f32, l: f32, r: u16| {
            TxMessage::from_frame(ShockFrame::new(d, l, r).ok()?)
        }) {
            let _ = can.tx_fifo_transmit_message(FifoNumber::Fifo1, &msg).await;
        }

        #[cfg(feature = "rear_unified")]
        {
            if let Some(msg) = shock(&SHOCKPOT_L_RAW, |d: f32, l: f32, r: u16| {
                TxMessage::from_frame(ShockFrameL::new(d, l, r).ok()?)
            }) {
                let _ = can.tx_fifo_transmit_message(FifoNumber::Fifo1, &msg).await;
            }

            if let Some(msg) = shock(&SHOCKPOT_R_RAW, |d: f32, l: f32, r: u16| {
                TxMessage::from_frame(ShockFrameR::new(d, l, r).ok()?)
            }) {
                let _ = can.tx_fifo_transmit_message(FifoNumber::Fifo1, &msg).await;
            }
        }

        //// * The wheel speed bit
        #[cfg(not(feature = "rear_unified"))]
        if let Some(msg) = speed(&WHEEL_SPEED, |d: f32, r: u16| {
            TxMessage::from_frame(SpeedFrame::new(d, r).ok()?)
        }) {
            let _ = can.tx_fifo_transmit_message(FifoNumber::Fifo1, &msg).await;
        }

        // Left Wheel Speed
        #[cfg(feature = "rear_unified")]
        if let Some(msg) = speed(&WHEEL_L_SPEED, |d: f32, r: u16| {
            TxMessage::from_frame(SpeedFrameL::new(d, r).ok()?)
        }) {
            let _ = can.tx_fifo_transmit_message(FifoNumber::Fifo1, &msg).await;
        }

        // Right Wheel Speed
        #[cfg(feature = "rear_unified")]
        if let Some(msg) = speed(&WHEEL_R_SPEED, |d: f32, r: u16| {
            TxMessage::from_frame(SpeedFrameR::new(d, r).ok()?)
        }) {
            let _ = can.tx_fifo_transmit_message(FifoNumber::Fifo1, &msg).await;
        }

        //// * The loadcell bit
        // NOTE: We don't have the pushrods yet lol

        // Throw the messages out
        let _ = can.tx_fifo_request_transmission(FifoNumber::Fifo1).await;

        // 50 hz lol
        Timer::after_millis(20).await;
    }
}

fn speed(
    sig: &Signal<CriticalSectionRawMutex, (u16, f32)>,
    new: fn(f32, u16) -> Option<TxMessage>,
) -> Option<TxMessage> {
    let data = sig.try_take()?;

    let rpm_delta = data.1;
    let rpm = data.0;

    return new(
        rpm_delta.clamp(-3276.8_f32, 3276.7_f32),
        rpm.clamp(0, 65535_u16),
    );
}

fn shock(
    sig: &Signal<CriticalSectionRawMutex, (u16, f32)>,
    new: fn(f32, f32, u16) -> Option<TxMessage>,
) -> Option<TxMessage> {
    let data = sig.try_take()?;
    // V = x bit * (3.3v/2^12 bit)

    // From this point on, we assume we have the blue shockpot with 75mm stroke, the max
    // voltage should be 99.5% of vin for 0mm extended, and 0.05% of vin for 75mm extended

    // length = (((3.3v * .995) - V) / ((3.3v * .995) - (3.3v * 0.05))) * 75mm

    // This can be baked into the following
    // (4076 - x) * 0.018495501894
    let length_mm = (4096 - data.0) as f32 * 0.018495501894_f32;
    let length_delta = data.1 * 0.018495501894_f32;

    let an1_uint12 = data.0;

    return new(
        length_delta.clamp(-3276.8_f32, 3276_f32),
        length_mm.clamp(0_f32, 6553.5_f32),
        an1_uint12.clamp(0, 65535_u16),
    );
}
