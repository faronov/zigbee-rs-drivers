//! CRC routines used by several sensor families.

/// CRC-8 with polynomial 0x31, init 0xFF, no reflection, no final XOR.
///
/// Used by Sensirion (SHT3x/SHT4x/SCD4x/SGP4x/SPS30...), Aosong AHT2x,
/// ScioSense ENS210 and others.
pub fn crc8_sensirion(bytes: &[u8]) -> u8 {
    crc8_msb(bytes, 0x31, 0xFF)
}

/// Dallas/Maxim CRC-8 (1-Wire ROM and scratchpad): reflected polynomial
/// 0x8C, init 0x00.
pub fn crc8_maxim(bytes: &[u8]) -> u8 {
    let mut crc = 0u8;
    for &byte in bytes {
        let mut b = byte;
        for _ in 0..8 {
            let mix = (crc ^ b) & 1;
            crc >>= 1;
            if mix != 0 {
                crc ^= 0x8C;
            }
            b >>= 1;
        }
    }
    crc
}

/// Generic MSB-first CRC-8.
pub fn crc8_msb(bytes: &[u8], polynomial: u8, init: u8) -> u8 {
    let mut crc = init;
    for &byte in bytes {
        crc ^= byte;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ polynomial
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// Modbus RTU CRC-16 (reflected 0xA001, init 0xFFFF). The result is sent
/// low byte first.
pub fn crc16_modbus(bytes: &[u8]) -> u16 {
    let mut crc = 0xFFFFu16;
    for &byte in bytes {
        crc ^= u16::from(byte);
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xA001
            } else {
                crc >> 1
            };
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sensirion_datasheet_vector() {
        assert_eq!(crc8_sensirion(&[0xBE, 0xEF]), 0x92);
    }

    #[test]
    fn maxim_rom_vector() {
        // DS18B20 ROM 28-xx: family + serial, CRC byte last.
        let rom = [0x28, 0xFF, 0x4C, 0x3C, 0x64, 0x16, 0x03];
        let full = [0x28, 0xFF, 0x4C, 0x3C, 0x64, 0x16, 0x03, crc8_maxim(&rom)];
        assert_eq!(crc8_maxim(&full), 0);
        assert_eq!(crc8_maxim(b"123456789"), 0xA1);
    }

    #[test]
    fn modbus_vector() {
        assert_eq!(crc16_modbus(b"123456789"), 0x4B37);
    }
}
