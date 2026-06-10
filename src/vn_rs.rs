pub enum VectorNavError {
    BadCRC,
}

pub trait VectorNavBin {
    fn from_bytes(bin: &[u8], bytes: &[u8]) -> Result<Self, VectorNavError>
    where
        Self: Sized;
}

pub struct VectorNavBin20Hz {
    pub unix_time_ns: u64, // Time since 1980 in nanoseconds 8

    pub position: [f64; 3], // Latitude, Longitude, Altitude 32

    pub inertial_navigation_state: u16, // VN tracking state 34
}

impl VectorNavBin for VectorNavBin20Hz {
    fn from_bytes(bin: &[u8], bytes: &[u8]) -> Result<Self, VectorNavError> {
        // NOTE: To whatever dev decided that the checksum should be the only thing in big endian
        // from this fucking board, I hope that you suffer a thousand byte alignment errors in hell
        let checksum = u16::from_be_bytes(bytes[34..36].try_into().unwrap());

        let crc = get_crc(bin.iter().chain(bytes));

        if checksum == crc {
            return Ok(VectorNavBin20Hz {
                unix_time_ns: u64::from_le_bytes(bytes[0..8].try_into().unwrap()),

                position: [
                    f64::from_le_bytes(bytes[8..16].try_into().unwrap()),
                    f64::from_le_bytes(bytes[16..24].try_into().unwrap()),
                    f64::from_le_bytes(bytes[24..32].try_into().unwrap()),
                ],

                inertial_navigation_state: u16::from_le_bytes(bytes[32..34].try_into().unwrap()),
            });
        } else {
            return Err(VectorNavError::BadCRC);
        }
    }
}

pub struct VectorNavBin400Hz {
    pub attitude: [f32; 3],     // Yaw, Pitch, Roll     12
    pub angular_rate: [f32; 3], // W_x, W_y, W_z        24
    pub velocity: [f32; 3],     // North, East, Down    36
    pub accel: [f32; 3],        // North, East, Down    48
}

impl VectorNavBin for VectorNavBin400Hz {
    fn from_bytes(bin: &[u8], bytes: &[u8]) -> Result<Self, VectorNavError> {
        let checksum = u16::from_be_bytes(bytes[48..50].try_into().unwrap());

        let crc = get_crc(bin.iter().chain(bytes));
        if checksum == crc {
            return Ok(VectorNavBin400Hz {
                attitude: [
                    f32::from_le_bytes(bytes[0..4].try_into().unwrap()),
                    f32::from_le_bytes(bytes[4..8].try_into().unwrap()),
                    f32::from_le_bytes(bytes[8..12].try_into().unwrap()),
                ],
                angular_rate: [
                    f32::from_le_bytes(bytes[12..16].try_into().unwrap()),
                    f32::from_le_bytes(bytes[16..20].try_into().unwrap()),
                    f32::from_le_bytes(bytes[20..24].try_into().unwrap()),
                ],
                velocity: [
                    f32::from_le_bytes(bytes[24..28].try_into().unwrap()),
                    f32::from_le_bytes(bytes[28..32].try_into().unwrap()),
                    f32::from_le_bytes(bytes[32..36].try_into().unwrap()),
                ],
                accel: [
                    f32::from_le_bytes(bytes[36..40].try_into().unwrap()),
                    f32::from_le_bytes(bytes[40..44].try_into().unwrap()),
                    f32::from_le_bytes(bytes[44..48].try_into().unwrap()),
                ],
            });
        } else {
            return Err(VectorNavError::BadCRC);
        }
    }
}

fn get_crc<'a, I>(bytes: I) -> u16
where
    I: IntoIterator<Item = &'a u8>,
{
    let mut crc = 0_u16;

    for &byte in bytes {
        crc = crc.swap_bytes(); // Flip the low and high bytes
        crc ^= byte as u16; // XOR crc with data[i]
        crc ^= (crc & 0x00FF) >> 4; // XOR crc with lower 4 bits of crc
        crc ^= crc.wrapping_shl(12); // Rotate crc left 12 bits
        crc ^= (crc & 0x00FF) << 5; // XOR crc w lower 8 bits & shift left 5 bits
    }

    crc
}
