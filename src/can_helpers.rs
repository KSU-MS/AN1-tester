use embassy_time::Delay;
use mcp2518fd::{
    id::{ExtendedId, Id, StandardId},
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

// pub async fn set_joe_can<SPI, CS>(can: MCP2518FD<SPI, CS>) {
//     // Make sure the CAN controller gets reset (in case the Pico reboots
//     // without the MCP2518FD losing power)
//     can.reset().await.unwrap();
//
//     // Configure the chip with default settings
//     can.configure(Settings::default(), &mut Delay)
//         .await
//         .expect("Failed to configure MCP2518");
//
//     can.configure_bit_timing(BitTimeConfiguration {
//         nominal: NominalBitTimeConfiguration::RATE_500_KBIT,
//         data: DataBitTimeConfiguration::RATE_500_KBIT,
//     })
//     .await
//     .expect("Failed to set CAN baudrate");
//
//     // Configure FIFO 1 as an RX FIFO to hold up to 16 messages with a max
//     // payload size of 64 bytes
//     can.configure_fifo(
//         FifoNumber::Fifo1,
//         FifoConfiguration {
//             fifo_size: 16,
//             payload_size: PayloadSize::Bytes64,
//             mode: FifoMode::Receive(RxFifoConfiguration::new().with_message_timestamps(true)),
//         },
//     )
//     .await
//     .expect("Failed to configure FIFO 1 as RX");
//
//     // Configure Filter 0 to accept all frame types (Standard or Extended),
//     // with any message ID (mask is all 0s)
//     can.configure_filter(
//         FilterNumber::Filter0,
//         Some(FilterConfiguration {
//             buffer_pointer: FifoNumber::Fifo1,
//             mode: FilterMatchMode::Both,
//             filter_bits: Id::Extended(ExtendedId::ZERO),
//             mask_bits: Id::Extended(ExtendedId::ZERO),
//         }),
//     )
//     .await
//     .expect("Failed to configure Filter 0 for FIFO 1");
//
//     // Set controller to CAN2
//     can.set_op_mode(OperationMode::NormalCan2, &mut Delay)
//         .await
//         .expect("Failed to change chip operating mode");
// }
