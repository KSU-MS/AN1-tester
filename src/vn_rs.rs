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

const MAX_PACKET_SIZE: usize = size_of::<Bin400hzUnion>();
enum ParseState {
    WaitingForSync,
    ReadingBin {
        buffer: [u8; 2],
        received: usize,
    },
    ReadingPacket {
        bin: VectorNavBin,
        len: usize,
        received: usize,
        buffer: [u8; MAX_PACKET_SIZE],
    },
}

pub struct VectorNav {
    // uart_controller: Uart<'d, Async>,
    // can_controller: MCP2518FD<SPI, CS>,
    pub bin1_data: Bin20hzUnion,
    pub bin2_data: Bin400hzUnion,
    state: ParseState,
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
            state: ParseState::WaitingForSync,
        };
    }

    pub fn update(&mut self, buffer: &[u8]) -> Result<VectorNavBin, VectorNavError> {
        for &byte in buffer.iter() {
            match &mut self.state {
                // Check if we have a sync byte
                ParseState::WaitingForSync => {
                    if byte == 0xFA {
                        self.state = ParseState::ReadingBin {
                            buffer: [0; 2],
                            received: 0,
                        }
                    }
                }

                // Figure out which bin it is
                ParseState::ReadingBin { buffer, received } => {
                    buffer[*received] = byte;
                    *received += 1;

                    if *received == 2 {
                        match *buffer {
                            [0x42, 0x10] => {
                                self.state = ParseState::ReadingPacket {
                                    bin: VectorNavBin::Bin20hz,
                                    len: size_of::<Bin20hzUnion>(),
                                    received: 0,
                                    buffer: [0; MAX_PACKET_SIZE],
                                };
                            }
                            [0xA8, 0x01] => {
                                self.state = ParseState::ReadingPacket {
                                    bin: VectorNavBin::Bin20hz,
                                    len: size_of::<Bin400hzUnion>(),
                                    received: 0,
                                    buffer: [0; MAX_PACKET_SIZE],
                                };
                            }
                            _ => {
                                self.state = ParseState::WaitingForSync;
                                return Err(VectorNavError::NoBinFound);
                            }
                        };
                    }
                }

                // Read the packet out
                ParseState::ReadingPacket {
                    buffer,
                    received,
                    len,
                    bin,
                } => {
                    buffer[*received] = byte;
                    *received += 1;

                    if *received == *len {
                        self.check_values(bin);

                        self.state = ParseState::WaitingForSync;
                    }
                }
            }
        }

        Err(VectorNavError::NoBinFound)
    }

    pub fn check_bin(&self, buffer: &[u8]) -> Result<VectorNavBin, VectorNavError> {
        if buffer[0] == 0x42 && buffer[1] == 0x10 {
            Ok(VectorNavBin::Bin20hz)
        } else if buffer[0] == 0xA8 && buffer[1] == 0x01 {
            Ok(VectorNavBin::Bin400hz)
        } else {
            Err(VectorNavError::NoBinFound)
        }
    }

    pub fn check_values(&self, bin: &VectorNavBin) -> Result<VectorNavBin, VectorNavError> {
        let crc = self.calc_crc(&bin);

        match bin {
            VectorNavBin::Bin20hz => unsafe {
                if self.bin1_data.data.checksum == crc {
                    info!("Got good 20hz bin!");
                    return Ok(VectorNavBin::Bin20hz);
                } else {
                    return Err(VectorNavError::BadCRC);
                }
            },

            VectorNavBin::Bin400hz => unsafe {
                if self.bin2_data.data.checksum == crc {
                    info!("Got good 400hz bin!");
                    return Ok(VectorNavBin::Bin400hz);
                } else {
                    return Err(VectorNavError::BadCRC);
                }
            },
        }
    }

    fn calc_crc(&self, bin: &VectorNavBin) -> u16 {
        let mut crc = 0_u16;

        match bin {
            VectorNavBin::Bin20hz => {
                unsafe {
                    // We need all the bytes except the last 2 which is the VNs CRC
                    for b in 0..(self.bin1_data.buffer.len() - 2) {
                        crc = (crc >> 8) | (crc << 8); // Rotate crc left 8 bits
                        crc ^= self.bin1_data.buffer[b] as u16; // XOR crc with data[i]
                        crc ^= (crc & 0x00FF) >> 4; // XOR crc with lower 4 bits of crc
                        crc ^= crc << 12; // Rotate crc left 12 bits
                        crc ^= (crc & 0x00FF) << 5; // XOR crc w lower 8 bits & shift left 5 bits
                    }
                }
            }
            VectorNavBin::Bin400hz => {
                unsafe {
                    // We need all the bytes except the last 2 which is the VNs CRC
                    for b in 0..(self.bin2_data.buffer.len() - 2) {
                        crc = (crc >> 8) | (crc << 8); // Rotate crc left 8 bits
                        crc ^= self.bin2_data.buffer[b] as u16; // XOR crc with data[i]
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
