use log::info;

pub enum VectorNavError {
    BadCRC,
    NoBinFound,
}
pub enum VectorNavBin {
    Bin20hz,
    Bin400hz,
}

#[repr(C)]
#[derive(Debug, Copy, Clone, Default)]
pub struct Bin20hzType {
    pub time: u64, // 8

    pub latitude: f64,  // 16
    pub longitude: f64, // 24
    pub altitude: f64,  // 32

    pub ins: u16, // 34

    pub checksum: u16, // 36
}

#[repr(C)]
#[derive(Debug, Copy, Clone, Default)]
pub struct Bin400hzType {
    pub yaw: f32,   // 4
    pub pitch: f32, //8
    pub roll: f32,  // 12

    pub w_x: f32, // 16
    pub w_y: f32, // 20
    pub w_z: f32, // 24

    pub velocity_n: f32, // 28
    pub velocity_e: f32, // 32
    pub velocity_d: f32, // 36

    pub accel_x: f32, // 40
    pub accel_y: f32, // 44
    pub accel_z: f32, // 48

    pub checksum: u16, // 50
}

#[repr(C)]
pub union Bin20hzUnion {
    pub buffer: [u8; 36],
    pub data: Bin20hzType,
}

#[repr(C)]
pub union Bin400hzUnion {
    pub buffer: [u8; 50],
    pub data: Bin400hzType,
}

pub struct VectorNav {
    // uart_controller: Uart<'d, Async>,
    // can_controller: MCP2518FD<SPI, CS>,
    pub bin1_data: Bin20hzUnion,
    pub bin2_data: Bin400hzUnion,
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
            bin1_data: Bin20hzUnion {
                data: Bin20hzType::default(),
            },
            bin2_data: Bin400hzUnion {
                data: Bin400hzType::default(),
            },
        };
    }

    pub fn check_sync_byte(&self, buffer: [u8; 1]) -> bool {
        if buffer[0] == 0xFA {
            return true;
        } else {
            return false;
        }
    }

    pub fn check_bin(&self, buffer: [u8; 2]) -> Result<VectorNavBin, VectorNavError> {
        if buffer[0] == 0x42 && buffer[1] == 0x10 {
            info!("Got 20hz bin");
            Ok(VectorNavBin::Bin20hz)
        } else if buffer[0] == 0xA8 && buffer[1] == 0x01 {
            Ok(VectorNavBin::Bin400hz)
        } else {
            Err(VectorNavError::NoBinFound)
        }
    }

    pub fn load_values(&self, crc: u16) -> Result<VectorNavBin, VectorNavError> {
        return Err(VectorNavError::BadCRC);
    }

    pub fn calc_crc(&self, bin: VectorNavBin) -> u16 {
        let mut crc = 0_u16;

        match bin {
            VectorNavBin::Bin20hz => {
                unsafe {
                    for &b in self.bin1_data.buffer.iter() {
                        crc = (crc >> 8) | (crc << 8); // Rotate crc left 8 bits
                        crc ^= b as u16; // XOR crc with data[i]
                        crc ^= (crc & 0x00FF) >> 4; // XOR crc with lower 4 bits of crc
                        crc ^= crc << 12; // Rotate crc left 12 bits
                        crc ^= (crc & 0x00FF) << 5; // XOR crc w lower 8 bits & shift left 5 bits
                    }
                }
            }
            VectorNavBin::Bin400hz => {
                unsafe {
                    for &b in self.bin2_data.buffer.iter() {
                        crc = (crc >> 8) | (crc << 8); // Rotate crc left 8 bits
                        crc ^= b as u16; // XOR crc with data[i]
                        crc ^= (crc & 0x00FF) >> 4; // XOR crc with lower 4 bits of crc
                        crc ^= crc << 12; // Rotate crc left 12 bits
                        crc ^= (crc & 0x00FF) << 5; // XOR crc w lower 8 bits & shift left 5 bits
                    }
                }
            }
        }

        crc
    }
}
