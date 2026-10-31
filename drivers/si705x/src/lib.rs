//! Silicon Labs Si7050 / Si7051 / Si7053 / Si7054 / Si7055 digital
//! temperature sensors (I2C).
//!
//! The variants differ only in accuracy (±4 °C .. ±0.1 °C); the command set
//! is the temperature subset of the Si7021: no-hold-master measurement
//! (0xF3), user register (0xE6 / 0xE7), reset (0xFE), electronic ID and
//! firmware revision. Measurement words carry a CRC-8 (polynomial 0x31,
//! init 0x00). The part enters standby (60 nA) after every conversion.
//!
//! Reference: Silicon Labs "Si7050/1/3/4/5-A20" datasheet, Rev. 1.2 (revision
//! not re-verified: the datasheet could not be downloaded while writing this driver; values were
//! cross-checked against the Si7021-A20 Rev. 1.2 temperature path, which
//! uses the identical commands, formula and timing).
//!
//! ```ignore
//! let mut si = Si705x::new(i2c, Variant::Si7051);
//! let wait_us = si.start_measurement()?;
//! // ... sleep wait_us ...
//! let m = si.read_measurement()?;
//! ```

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs, crc::crc8_msb};

/// Fixed I2C address.
pub const ADDRESS: u8 = 0x40;

const CMD_MEASURE_T: u8 = 0xF3;
const CMD_RESET: u8 = 0xFE;
const CMD_WRITE_USER: u8 = 0xE6;
const CMD_READ_USER: u8 = 0xE7;
const CMD_EID_FIRST: [u8; 2] = [0xFA, 0x0F];
const CMD_EID_SECOND: [u8; 2] = [0xFC, 0xC9];
const CMD_FIRMWARE: [u8; 2] = [0x84, 0xB8];

/// Soft reset time (t_RST max 15 ms).
pub const RESET_WAIT_US: WaitUs = 15_000;
/// Power-up time to first command (80 ms max over temperature).
pub const POWER_UP_WAIT_US: WaitUs = 80_000;

/// Accuracy grade.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// ±4 °C.
    Si7050,
    /// ±0.1 °C at 25 °C (±0.13 °C 20..70 °C).
    Si7051,
    /// ±0.3 °C.
    Si7053,
    /// ±0.4 °C.
    Si7054,
    /// ±0.5 °C.
    Si7055,
}

impl Variant {
    /// SNB_3 device code of the electronic ID.
    pub const fn device_code(self) -> u8 {
        match self {
            Self::Si7050 => 0x32,
            Self::Si7051 => 0x33,
            Self::Si7053 => 0x35,
            Self::Si7054 => 0x36,
            Self::Si7055 => 0x37,
        }
    }
}

/// Temperature resolution (user register bits D7 and D0).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Resolution {
    /// 14 bit, 10.8 ms max (power-on default).
    #[default]
    Bits14 = 0,
    /// 12 bit, 3.8 ms max.
    Bits12 = 1,
    /// 13 bit, 6.2 ms max.
    Bits13 = 2,
    /// 11 bit, 2.4 ms max.
    Bits11 = 3,
}

impl Resolution {
    /// Maximum conversion time.
    pub const fn wait_us(self) -> WaitUs {
        [10_800, 3_800, 6_200, 2_400][self as usize]
    }

    const fn from_register(reg: u8) -> Self {
        match ((reg >> 6) & 0b10) | (reg & 0b01) {
            0 => Self::Bits14,
            1 => Self::Bits12,
            2 => Self::Bits13,
            _ => Self::Bits11,
        }
    }

    /// User register with this resolution; other bits preserved.
    pub const fn apply(self, reg: u8) -> u8 {
        let v = self as u8;
        (reg & !0x81) | ((v & 0b10) << 6) | (v & 0b01)
    }
}

/// One temperature sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Temperature in 0.01 °C.
    pub temperature: i32,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Temperature(self.temperature));
    }
}

/// CRC-8 on measurement words (x^8 + x^5 + x^4 + 1, init 0x00).
pub fn crc8(bytes: &[u8]) -> u8 {
    crc8_msb(bytes, 0x31, 0x00)
}

/// Temperature code to 0.01 °C: T = 175.72 × code / 65536 − 46.85.
pub fn convert_temperature(code: u16) -> i32 {
    ((17_572 * i64::from(code & !0x3) + 32_768) >> 16) as i32 - 4_685
}

/// Decode a 3-byte measurement frame (MSB, LSB, CRC).
pub fn decode_measurement<E>(frame: &[u8; 3]) -> Result<Measurement, Error<E>> {
    if crc8(&frame[..2]) != frame[2] {
        return Err(Error::Crc);
    }
    Ok(Measurement {
        temperature: convert_temperature(u16::from_be_bytes([frame[0], frame[1]])),
    })
}

/// 64-bit electronic ID from the two raw reads (CRC bytes skipped).
pub const fn decode_electronic_id(first: &[u8; 8], second: &[u8; 6]) -> u64 {
    u64::from_be_bytes([
        first[0], first[2], first[4], first[6], second[0], second[1], second[3], second[4],
    ])
}

macro_rules! constructors {
    () => {
        /// Create a driver; no bus traffic.
        pub const fn new(i2c: I2C, variant: Variant) -> Self {
            Self {
                i2c,
                variant,
                resolution: Resolution::Bits14,
            }
        }

        /// Tell the driver which resolution the part is configured for
        /// (wait time only; `set_resolution` writes it).
        pub const fn with_resolution(mut self, resolution: Resolution) -> Self {
            self.resolution = resolution;
            self
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }

        fn check_id<E>(&self, id: u64) -> Result<u64, Error<E>> {
            let code = (id >> 24) as u8;
            // Engineering samples report 0x00 or 0xFF.
            if code == self.variant.device_code() || code == 0x00 || code == 0xFF {
                Ok(id)
            } else {
                Err(Error::UnexpectedId(u32::from(code)))
            }
        }
    };
}

/// Blocking driver.
pub struct Si705x<I2C> {
    i2c: I2C,
    variant: Variant,
    resolution: Resolution,
}

impl<I2C> Si705x<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Si705x<I2C> {
    fn write_read(&mut self, cmd: &[u8], buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c.write_read(ADDRESS, cmd, buf).map_err(Error::bus)
    }

    /// Soft reset; restores the default resolution. Wait [`RESET_WAIT_US`].
    pub fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.i2c.write(ADDRESS, &[CMD_RESET]).map_err(Error::bus)?;
        self.resolution = Resolution::Bits14;
        Ok(RESET_WAIT_US)
    }

    /// Set the resolution (read-modify-write of the user register).
    pub fn set_resolution(&mut self, resolution: Resolution) -> Result<(), Error<I2C::Error>> {
        let mut reg = [0u8; 1];
        self.write_read(&[CMD_READ_USER], &mut reg)?;
        self.i2c
            .write(ADDRESS, &[CMD_WRITE_USER, resolution.apply(reg[0])])
            .map_err(Error::bus)?;
        self.resolution = resolution;
        Ok(())
    }

    /// Read the resolution from the user register.
    pub fn read_resolution(&mut self) -> Result<Resolution, Error<I2C::Error>> {
        let mut reg = [0u8; 1];
        self.write_read(&[CMD_READ_USER], &mut reg)?;
        Ok(Resolution::from_register(reg[0]))
    }

    /// Read the 64-bit electronic ID and check the SNB_3 device code
    /// against the variant ([`Error::UnexpectedId`] on mismatch).
    pub fn read_electronic_id(&mut self) -> Result<u64, Error<I2C::Error>> {
        let mut first = [0u8; 8];
        let mut second = [0u8; 6];
        self.write_read(&CMD_EID_FIRST, &mut first)?;
        self.write_read(&CMD_EID_SECOND, &mut second)?;
        self.check_id(decode_electronic_id(&first, &second))
    }

    /// Firmware revision byte (0xFF = 1.0, 0x20 = 2.0).
    pub fn firmware_revision(&mut self) -> Result<u8, Error<I2C::Error>> {
        let mut buf = [0u8; 1];
        self.write_read(&CMD_FIRMWARE, &mut buf)?;
        Ok(buf[0])
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Si705x<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.i2c
            .write(ADDRESS, &[CMD_MEASURE_T])
            .map_err(Error::bus)?;
        Ok(self.resolution.wait_us())
    }

    /// Returns [`Error::Bus`] (NACK) while the conversion is running.
    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        let mut frame = [0u8; 3];
        self.i2c.read(ADDRESS, &mut frame).map_err(Error::bus)?;
        decode_measurement(&frame)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        ADDRESS, CMD_EID_FIRST, CMD_EID_SECOND, CMD_FIRMWARE, CMD_MEASURE_T, CMD_READ_USER,
        CMD_RESET, CMD_WRITE_USER, Measurement, RESET_WAIT_US, Resolution, Variant,
        decode_electronic_id, decode_measurement,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Si705x`].
    pub struct Si705x<I2C> {
        i2c: I2C,
        variant: Variant,
        resolution: Resolution,
    }

    impl<I2C> Si705x<I2C> {
        constructors!();
    }

    impl<I2C: I2c> Si705x<I2C> {
        async fn write_read(
            &mut self,
            cmd: &[u8],
            buf: &mut [u8],
        ) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write_read(ADDRESS, cmd, buf)
                .await
                .map_err(Error::bus)
        }

        /// See [`super::Si705x::soft_reset`].
        pub async fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.i2c
                .write(ADDRESS, &[CMD_RESET])
                .await
                .map_err(Error::bus)?;
            self.resolution = Resolution::Bits14;
            Ok(RESET_WAIT_US)
        }

        /// See [`super::Si705x::set_resolution`].
        pub async fn set_resolution(
            &mut self,
            resolution: Resolution,
        ) -> Result<(), Error<I2C::Error>> {
            let mut reg = [0u8; 1];
            self.write_read(&[CMD_READ_USER], &mut reg).await?;
            self.i2c
                .write(ADDRESS, &[CMD_WRITE_USER, resolution.apply(reg[0])])
                .await
                .map_err(Error::bus)?;
            self.resolution = resolution;
            Ok(())
        }

        /// See [`super::Si705x::read_resolution`].
        pub async fn read_resolution(&mut self) -> Result<Resolution, Error<I2C::Error>> {
            let mut reg = [0u8; 1];
            self.write_read(&[CMD_READ_USER], &mut reg).await?;
            Ok(Resolution::from_register(reg[0]))
        }

        /// See [`super::Si705x::read_electronic_id`].
        pub async fn read_electronic_id(&mut self) -> Result<u64, Error<I2C::Error>> {
            let mut first = [0u8; 8];
            let mut second = [0u8; 6];
            self.write_read(&CMD_EID_FIRST, &mut first).await?;
            self.write_read(&CMD_EID_SECOND, &mut second).await?;
            self.check_id(decode_electronic_id(&first, &second))
        }

        /// See [`super::Si705x::firmware_revision`].
        pub async fn firmware_revision(&mut self) -> Result<u8, Error<I2C::Error>> {
            let mut buf = [0u8; 1];
            self.write_read(&CMD_FIRMWARE, &mut buf).await?;
            Ok(buf[0])
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Si705x<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.i2c
                .write(ADDRESS, &[CMD_MEASURE_T])
                .await
                .map_err(Error::bus)?;
            Ok(self.resolution.wait_us())
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let mut frame = [0u8; 3];
            self.i2c
                .read(ADDRESS, &mut frame)
                .await
                .map_err(Error::bus)?;
            decode_measurement(&frame)
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use std::vec;
    use zigbee_sensor::blocking::Sensor;

    fn frame(word: u16) -> std::vec::Vec<u8> {
        let [a, b] = word.to_be_bytes();
        vec![a, b, crc8(&[a, b])]
    }

    #[test]
    fn conversion_matches_formula() {
        // 0x683A (Si7021 / HTU21D example) -> 24.69 °C.
        assert_eq!(convert_temperature(0x683A), 2_469);
        assert_eq!(convert_temperature(0), -4_685);
        // 25.00 °C: code = (25 + 46.85) × 65536 / 175.72 = 26797.
        assert_eq!(convert_temperature(26_797), 2_500);
        assert_eq!(crc8(&[0x68, 0x3A]), 0x7C);
    }

    #[test]
    fn measure_with_resolution() {
        let expectations = [
            Transaction::write_read(ADDRESS, vec![0xE7], vec![0x3A]),
            Transaction::write(ADDRESS, vec![0xE6, 0xBB]),
            Transaction::write(ADDRESS, vec![0xF3]),
            Transaction::read(ADDRESS, frame(0x683A)),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut si = Si705x::new(&mut i2c, Variant::Si7051);
        si.set_resolution(Resolution::Bits11).unwrap();
        assert_eq!(si.start_measurement().unwrap(), 2_400);
        assert_eq!(si.read_measurement().unwrap().temperature, 2_469);
        i2c.done();
    }

    #[test]
    fn crc_error() {
        let mut bad = frame(0x683A);
        bad[2] ^= 0x80;
        let mut i2c = Mock::new(&[Transaction::read(ADDRESS, bad)]);
        let mut si = Si705x::new(&mut i2c, Variant::Si7050);
        assert_eq!(si.read_measurement(), Err(Error::Crc));
        i2c.done();
    }

    #[test]
    fn electronic_id_checks_device_code() {
        let first = vec![1, 0, 2, 0, 3, 0, 4, 0];
        let expectations = [
            Transaction::write_read(ADDRESS, vec![0xFA, 0x0F], first.clone()),
            Transaction::write_read(ADDRESS, vec![0xFC, 0xC9], vec![0x37, 0xFF, 0, 5, 6, 0]),
            Transaction::write_read(ADDRESS, vec![0xFA, 0x0F], first),
            Transaction::write_read(ADDRESS, vec![0xFC, 0xC9], vec![0x15, 0xFF, 0, 5, 6, 0]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut si = Si705x::new(&mut i2c, Variant::Si7055);
        assert_eq!(si.read_electronic_id().unwrap(), 0x0102_0304_37FF_0506);
        assert_eq!(si.read_electronic_id(), Err(Error::UnexpectedId(0x15)));
        i2c.done();
    }

    #[test]
    fn async_measure() {
        use zigbee_sensor::asynch::Sensor as _;

        struct RecordingDelay(u32);
        impl embedded_hal_async::delay::DelayNs for RecordingDelay {
            async fn delay_ns(&mut self, ns: u32) {
                self.0 += ns / 1_000;
            }
        }

        let expectations = [
            Transaction::write(ADDRESS, vec![0xF3]),
            Transaction::read(ADDRESS, frame(26_797)),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut si = asynch::Si705x::new(&mut i2c, Variant::Si7053);
        let mut delay = RecordingDelay(0);
        let m = block_on(si.measure(&mut delay)).unwrap();
        assert_eq!(m.temperature, 2_500);
        assert_eq!(delay.0, 10_800);
        i2c.done();
    }

    fn block_on<F: core::future::Future>(fut: F) -> F::Output {
        let mut cx = core::task::Context::from_waker(core::task::Waker::noop());
        let mut fut = core::pin::pin!(fut);
        loop {
            if let core::task::Poll::Ready(v) = fut.as_mut().poll(&mut cx) {
                return v;
            }
        }
    }
}
