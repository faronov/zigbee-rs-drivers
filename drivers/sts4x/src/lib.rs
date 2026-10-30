//! Sensirion STS40 / STS45 digital temperature sensor (I2C).
//!
//! Temperature-only sibling of the SHT4x: same 8-bit command set without
//! humidity and heater. Variants differ in accuracy (STS40 ±0.2 °C,
//! STS45 ±0.1 °C) and in the factory-set address (0x44 / 0x45 / 0x46). The
//! sensor idles at 80 nA between measurements and has no sleep command.
//!
//! Reference: Sensirion "Datasheet STS4x", version 3.1 (February 2025).
//!
//! ```ignore
//! let mut sts = Sts4x::new(i2c, ADDRESS_A);
//! let wait_us = sts.start_measurement()?;
//! // ... let the Zigbee stack run / sleep for wait_us ...
//! let t = sts.read_measurement()?;
//! ```

#![no_std]

use zigbee_sensirion_common::{decode_word, temperature_from_ticks};
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address of the -A variants.
pub const ADDRESS_A: u8 = 0x44;
/// I2C address of the -B variants.
pub const ADDRESS_B: u8 = 0x45;
/// I2C address of the -C variants.
pub const ADDRESS_C: u8 = 0x46;

const CMD_SERIAL: u8 = 0x89;
const CMD_SOFT_RESET: u8 = 0x94;

/// Time between the soft-reset ACK and the idle state (t_SR max, table 3).
pub const SOFT_RESET_WAIT_US: WaitUs = 1_000;
/// Wait before reading the serial number. The datasheet gives no execution
/// time beyond the 1 ms minimum gap t_W between commands; 10 ms is a safe
/// margin.
pub const SERIAL_WAIT_US: u32 = 10_000;

/// Measurement precision (repeatability), datasheet table 3.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Precision {
    /// 8.3 ms max.
    #[default]
    High,
    /// 4.5 ms max.
    Medium,
    /// 1.6 ms max.
    Low,
}

impl Precision {
    const fn command(self) -> u8 {
        match self {
            Self::High => 0xFD,
            Self::Medium => 0xF6,
            Self::Low => 0xE0,
        }
    }

    /// Maximum measurement duration, in microseconds.
    pub const fn wait_us(self) -> WaitUs {
        match self {
            Self::High => 8_300,
            Self::Medium => 4_500,
            Self::Low => 1_600,
        }
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

/// Convert raw ticks: `T = -45 + 175 × S / 65535` (section 4.5, eq. 1).
pub fn convert(raw_temperature: u16) -> Measurement {
    Measurement {
        temperature: temperature_from_ticks(raw_temperature),
    }
}

/// Decode the 3-byte frame (T msb, lsb, crc).
pub fn decode_measurement<E>(frame: &[u8; 3]) -> Result<Measurement, Error<E>> {
    Ok(convert(decode_word(frame)?))
}

/// Decode the 6-byte serial-number frame (two CRC-protected words).
pub fn decode_serial<E>(frame: &[u8; 6]) -> Result<u32, Error<E>> {
    let hi = decode_word(&frame[..3])?;
    let lo = decode_word(&frame[3..])?;
    Ok((u32::from(hi) << 16) | u32::from(lo))
}

macro_rules! constructors {
    () => {
        /// Create a driver; no bus traffic.
        pub const fn new(i2c: I2C, address: u8) -> Self {
            Self {
                i2c,
                address,
                precision: Precision::High,
            }
        }

        /// Select the precision used by `start_measurement`.
        pub const fn with_precision(mut self, precision: Precision) -> Self {
            self.precision = precision;
            self
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }
    };
}

/// Blocking driver.
pub struct Sts4x<I2C> {
    i2c: I2C,
    address: u8,
    precision: Precision,
}

impl<I2C> Sts4x<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Sts4x<I2C> {
    fn command(&mut self, cmd: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(self.address, &[cmd]).map_err(Error::bus)
    }

    /// Soft reset. Wait [`SOFT_RESET_WAIT_US`] before the next command.
    pub fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.command(CMD_SOFT_RESET)?;
        Ok(SOFT_RESET_WAIT_US)
    }

    /// Read the unique 32-bit serial number; also serves as a presence
    /// probe. Waits [`SERIAL_WAIT_US`] on `delay` between command and read.
    pub fn serial_number<D: embedded_hal::delay::DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<u32, Error<I2C::Error>> {
        self.command(CMD_SERIAL)?;
        delay.delay_us(SERIAL_WAIT_US);
        let mut frame = [0u8; 6];
        self.i2c
            .read(self.address, &mut frame)
            .map_err(Error::bus)?;
        decode_serial(&frame)
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Sts4x<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.command(self.precision.command())?;
        Ok(self.precision.wait_us())
    }

    /// Returns [`Error::Bus`] (NACK) if the conversion is still running.
    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        let mut frame = [0u8; 3];
        self.i2c
            .read(self.address, &mut frame)
            .map_err(Error::bus)?;
        decode_measurement(&frame)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        CMD_SERIAL, CMD_SOFT_RESET, Measurement, Precision, SERIAL_WAIT_US, SOFT_RESET_WAIT_US,
        decode_measurement, decode_serial,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Sts4x`].
    pub struct Sts4x<I2C> {
        i2c: I2C,
        address: u8,
        precision: Precision,
    }

    impl<I2C> Sts4x<I2C> {
        constructors!();
    }

    impl<I2C: I2c> Sts4x<I2C> {
        async fn command(&mut self, cmd: u8) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write(self.address, &[cmd])
                .await
                .map_err(Error::bus)
        }

        /// See [`super::Sts4x::soft_reset`].
        pub async fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.command(CMD_SOFT_RESET).await?;
            Ok(SOFT_RESET_WAIT_US)
        }

        /// See [`super::Sts4x::serial_number`].
        pub async fn serial_number<D: embedded_hal_async::delay::DelayNs>(
            &mut self,
            delay: &mut D,
        ) -> Result<u32, Error<I2C::Error>> {
            self.command(CMD_SERIAL).await?;
            delay.delay_us(SERIAL_WAIT_US).await;
            let mut frame = [0u8; 6];
            self.i2c
                .read(self.address, &mut frame)
                .await
                .map_err(Error::bus)?;
            decode_serial(&frame)
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Sts4x<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.command(self.precision.command()).await?;
            Ok(self.precision.wait_us())
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let mut frame = [0u8; 3];
            self.i2c
                .read(self.address, &mut frame)
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
    use embedded_hal_mock::eh1::delay::{CheckedDelay, Transaction as Delay};
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use std::vec;
    use zigbee_sensirion_common::encode_word;
    use zigbee_sensor::blocking::Sensor;

    #[test]
    fn conversion_matches_datasheet_equation() {
        assert_eq!(convert(0).temperature, -4_500);
        assert_eq!(convert(0x6666).temperature, 2_500);
        assert_eq!(convert(u16::MAX).temperature, 13_000);
    }

    #[test]
    fn start_then_read() {
        let expectations = [
            Transaction::write(ADDRESS_B, vec![0xE0]),
            Transaction::read(ADDRESS_B, encode_word(0x6666).to_vec()),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut sts = Sts4x::new(&mut i2c, ADDRESS_B).with_precision(Precision::Low);
        assert_eq!(sts.start_measurement().unwrap(), 1_600);
        assert_eq!(sts.read_measurement().unwrap().temperature, 2_500);
        i2c.done();
    }

    #[test]
    fn serial_reset_and_crc() {
        let mut serial = encode_word(0xCAFE).to_vec();
        serial.extend_from_slice(&encode_word(0xF00D));
        let expectations = [
            Transaction::write(ADDRESS_A, vec![0x89]),
            Transaction::read(ADDRESS_A, serial),
            Transaction::write(ADDRESS_A, vec![0x94]),
            Transaction::read(ADDRESS_A, vec![0x66, 0x66, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut delay = CheckedDelay::new(&[Delay::blocking_delay_us(SERIAL_WAIT_US)]);
        let mut sts = Sts4x::new(&mut i2c, ADDRESS_A);
        assert_eq!(sts.serial_number(&mut delay).unwrap(), 0xCAFE_F00D);
        assert_eq!(sts.soft_reset().unwrap(), SOFT_RESET_WAIT_US);
        assert_eq!(sts.read_measurement(), Err(Error::Crc));
        i2c.done();
        delay.done();
    }

    #[test]
    fn async_measure() {
        use zigbee_sensor::asynch::Sensor as _;

        let expectations = [
            Transaction::write(ADDRESS_C, vec![0xFD]),
            Transaction::read(ADDRESS_C, encode_word(0x8000).to_vec()),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut delay = CheckedDelay::new(&[Delay::async_delay_us(8_300)]);
        let mut sts = asynch::Sts4x::new(&mut i2c, ADDRESS_C);
        assert_eq!(
            block_on(sts.measure(&mut delay)).unwrap().temperature,
            4_250
        );
        i2c.done();
        delay.done();
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
