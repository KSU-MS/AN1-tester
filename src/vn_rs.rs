use embassy_rp::{
    interrupt::UART1_IRQ,
    peripherals::UART1,
    uart::{self, Async, Uart, UartRx},
};
use log::info;
use mcp2518fd::MCP2518FD;

pub struct VectorNav {
    // uart_controller: Uart<'d, Async>,
    // can_controller: MCP2518FD<SPI, CS>,
    // buffer: [u8; 90],
    time: u64,

    ins: u16,

    yaw: f32,
    pitch: f32,
    roll: f32,

    w_x: f32,
    w_y: f32,
    w_z: f32,

    velocity_n: f32,
    velocity_e: f32,
    velocity_d: f32,

    accel_x: f32,
    accel_y: f32,
    accel_z: f32,

    latitude: f64,
    longitude: f64,
    altitude: f64,

    checksum: u16,
}

// impl<SPI, CS> VectorNav {
impl VectorNav {
    pub fn new(// uart_controller: Uart<'d, Async>,
        // can_controller: MCP2518FD<SPI, CS>,
    ) -> VectorNav {
        info!("Initializing VectorNav...");

        // TODO: Setup VN message bins and other params every boot to prevent fuckery from people

        return VectorNav {
            // uart_controller,
            // can_controller,
            // buffer: [0; 90],
            time: 0,
            ins: 0,
            yaw: 0_f32,
            pitch: 0_f32,
            roll: 0_f32,
            w_x: 0_f32,
            w_y: 0_f32,
            w_z: 0_f32,
            velocity_n: 0_f32,
            velocity_e: 0_f32,
            velocity_d: 0_f32,
            accel_x: 0_f32,
            accel_y: 0_f32,
            accel_z: 0_f32,
            latitude: 0_f64,
            longitude: 0_f64,
            altitude: 0_f64,
            checksum: 0,
        };
    }

    pub fn update(&self, input: [u8; 90]) -> bool {
        return false;
    }

    pub fn check_sync_byte(&self, input: [u8; 1]) -> bool {
        if input[0] == 0xFA {
            return true;
        } else {
            return false;
        }
    }

    fn calc_crc(self, input: [u8; 90]) -> u16 {
        let mut crc = 0_u16;

        for &b in input.iter() {
            crc = (crc >> 8) | (crc << 8); // Rotate crc left 8 bits
            crc ^= b as u16; // XOR crc with data[i]
            crc ^= (crc & 0x00FF) >> 4; // XOR crc with lower 4 bits of crc
            crc ^= crc << 12; // Rotate crc left 12 bits
            crc ^= (crc & 0x00FF) << 5; // XOR crc w lower 8 bits & shift left 5 bits
        }

        crc
    }
}
