use log::info;

#[derive(Clone, Copy, Debug)]
pub enum VectorNavError {
    BadCRC,
    NoBinFound,
}

#[derive(Clone, Copy, Debug)]
enum VectorNavBin {
    Bin20Hz,
    Bin400Hz,
}

impl VectorNavBin {
    const fn id(&self) -> [u8; 2] {
        match self {
            VectorNavBin::Bin20Hz => [0x42, 0x10],
            VectorNavBin::Bin400Hz => [0xA8, 0x01],
        }
    }

    const fn size(&self) -> usize {
        match self {
            VectorNavBin::Bin20Hz => 36,
            VectorNavBin::Bin400Hz => 50,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct Bin20Hz {
    pub time: u64, // 8

    pub latitude: f64,  // 16
    pub longitude: f64, // 24
    pub altitude: f64,  // 32

    pub ins: u16, // 34

    pub checksum: u16, // 36
}

#[derive(Clone, Copy, Debug, Default)]
struct Bin400hz {
    pub yaw: f32,   // 4
    pub pitch: f32, // 8
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

enum ParseState {
    WaitingForSync {
        buffer: [u8; 2],
        received: usize,
    },
    ReadingBin {
        buffer: [u8; 2],
        received: usize,
    },
    ReadingPacket {
        bin: VectorNavBin,
        len: usize,
        received: usize,
        buffer: [u8; 50],
    },
}

pub struct VectorNavData {
    bin_20hz: Bin20Hz,
    bin_400hz: Bin400hz,
    state: ParseState,
}

impl VectorNavData {
    pub fn new() -> VectorNavData {
        VectorNavData {
            bin_20hz: Bin20Hz::default(),
            bin_400hz: Bin400hz::default(),
            state: ParseState::WaitingForSync {
                buffer: [0_u8; 2],
                received: 0,
            },
        }
    }

    pub fn update(&mut self, input: [u8; 1]) {
        for byte in input.iter() {
            match &mut self.state {
                ParseState::WaitingForSync { buffer, received } => {
                    buffer[*received] = *byte;
                    *received += 1;

                    if *received == 2 {
                        match *buffer {
                            [0xFA, 0x01] => {
                                info!("Found sync");

                                self.state = ParseState::ReadingBin {
                                    buffer: [0_u8; 2],
                                    received: 0,
                                }
                            }

                            _ => {
                                self.state = ParseState::WaitingForSync {
                                    buffer: [0_u8; 2],
                                    received: 0,
                                };
                            }
                        };
                    }
                }

                ParseState::ReadingBin { buffer, received } => {
                    buffer[*received] = *byte;
                    *received += 1;

                    if *received == 2 {
                        match *buffer {
                            [0x42, 0x10] => {
                                info!("Got bin: 20hz");

                                self.state = ParseState::ReadingPacket {
                                    bin: VectorNavBin::Bin20Hz,
                                    len: VectorNavBin::Bin20Hz.size(),
                                    received: 0,
                                    buffer: [0_u8; 50],
                                }
                            }
                            [0xA8, 0x01] => {
                                info!("Got bin: 400hz");

                                self.state = ParseState::ReadingPacket {
                                    bin: VectorNavBin::Bin400Hz,
                                    len: VectorNavBin::Bin400Hz.size(),
                                    received: 0,
                                    buffer: [0_u8; 50],
                                }
                            }
                            _ => {
                                info!("Got garbo: {:?} {:?}", buffer[0], buffer[1]);
                                self.state = ParseState::WaitingForSync {
                                    buffer: [0_u8; 2],
                                    received: 0,
                                };
                            }
                        };
                    }
                }

                ParseState::ReadingPacket {
                    bin,
                    len,
                    received,
                    buffer,
                } => {
                    buffer[*received] = *byte;
                    *received += 1;

                    if *received == *len {
                        info!("Got full bin: {:?}", bin);

                        self.state = ParseState::WaitingForSync {
                            buffer: [0_u8; 2],
                            received: 0,
                        };
                    }
                }
            }
        }
    }

    fn get_crc(buffer: &[u8]) -> u16 {
        let mut crc = 0_u16;

        // We need all the bytes except the last 2 which is the VNs CRC
        for b in 0..(buffer.len() - 2) {
            crc = (crc >> 8) | (crc << 8); // Rotate crc left 8 bits
            crc ^= buffer[b] as u16; // XOR crc with data[i]
            crc ^= (crc & 0x00FF) >> 4; // XOR crc with lower 4 bits of crc
            crc ^= crc << 12; // Rotate crc left 12 bits
            crc ^= (crc & 0x00FF) << 5; // XOR crc w lower 8 bits & shift left 5 bits
        }

        crc
    }

    // fn f32_le(bytes: &[u8]) -> f32 {
    //     f32::from_le_bytes(bytes.try_into().unwrap())
    // }
    //
    // fn f64_le(bytes: &[u8]) -> f64 {
    //     f64::from_le_bytes(bytes.try_into().unwrap())
    // }
    //
    // fn u64_le(bytes: &[u8]) -> u64 {
    //     u64::from_le_bytes(bytes.try_into().unwrap())
    // }
    //
    // fn u16_le(bytes: &[u8]) -> u16 {
    //     u16::from_le_bytes(bytes.try_into().unwrap())
    // }
}
