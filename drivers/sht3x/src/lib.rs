//! Sensirion SHT30 / SHT31 / SHT35 temperature and humidity sensor (I2C).
//!
//! Single-shot mode without clock stretching: the sensor idles at < 2 µA
//! between conversions, so no explicit sleep command is needed.
//!
//! Reference: Sensirion "Datasheet SHT3x-DIS", v7 (2023).
//!
//! ```ignore
//! let mut sht = Sht3x::new(i2c, PRIMARY_ADDRESS);
//! let wait_us = sht.start_measurement()?;
//! // ... let the Zigbee stack run / sleep for wait_us ...
//! let m = sht.read_measurement()?;
//! ```

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs, crc::crc8_sensirion};

/// I2C address with ADDR pin low.
pub const PRIMARY_ADDRESS: u8 = 0x44;
/// I2C address with ADDR pin high.
pub const SECONDARY_ADDRESS: u8 = 0x45;

const CMD_SOFT_RESET: [u8; 2] = [0x30, 0xA2];
const CMD_READ_STATUS: [u8; 2] = [0xF3, 0x2D];
const CMD_CLEAR_STATUS: [u8; 2] = [0x30, 0x41];
const CMD_HEATER_ON: [u8; 2] = [0x30, 0x6D];
const CMD_HEATER_OFF: [u8; 2] = [0x30, 0x66];

/// Time the sensor needs after a soft reset (datasheet t_SR max 1.5 ms).
pub const SOFT_RESET_WAIT_US: WaitUs = 1_500;

/// Measurement repeatability: trades noise for conversion time and energy.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Repeatability {
    /// 0.04 °C / 0.08 %RH noise, 15.5 ms max.
    #[default]
    High,
    /// 0.08 °C / 0.15 %RH noise, 6.5 ms max.
    Medium,
    /// 0.15 °C / 0.21 %RH noise, 4.5 ms max.
    Low,
}

impl Repeatability {
    const fn command(self) -> [u8; 2] {
        match self {
            Self::High => [0x24, 0x00],
            Self::Medium => [0x24, 0x0B],
            Self::Low => [0x24, 0x16],
        }
    }

    /// Maximum conversion time from the datasheet, in microseconds.
    pub const fn wait_us(self) -> WaitUs {
        match self {
            Self::High => 15_500,
            Self::Medium => 6_500,
            Self::Low => 4_500,
        }
    }
}

/// One temperature + humidity sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Temperature in 0.01 °C.
    pub temperature: i32,
    /// Relative humidity in 0.01 %RH.
    pub humidity: u16,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Temperature(self.temperature));
        f(Quantity::Humidity(self.humidity));
    }
}

/// Status register.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Status(pub u16);

impl Status {
    /// Heater is on.
    pub const fn heater_on(self) -> bool {
        self.0 & (1 << 13) != 0
    }

    /// A reset (power-on, soft or brown-out) happened since the last clear.
    pub const fn reset_detected(self) -> bool {
        self.0 & (1 << 4) != 0
    }
}

/// Convert raw sensor words to engineering units (datasheet section 4.13).
pub fn convert(raw_temperature: u16, raw_humidity: u16) -> Measurement {
    // T = -45 + 175 × raw / 65535, RH = 100 × raw / 65535, rounded.
    let t = (17_500 * i64::from(raw_temperature) + 32_767) / 65_535 - 4_500;
    let rh = (10_000 * u32::from(raw_humidity) + 32_767) / 65_535;
    Measurement {
        temperature: t as i32,
        humidity: rh as u16,
    }
}

fn check_word<E>(bytes: &[u8]) -> Result<u16, Error<E>> {
    if crc8_sensirion(&bytes[..2]) != bytes[2] {
        return Err(Error::Crc);
    }
    Ok(u16::from_be_bytes([bytes[0], bytes[1]]))
}

/// Decode the 6-byte measurement frame (T msb, lsb, crc, RH msb, lsb, crc).
pub fn decode_measurement<E>(frame: &[u8; 6]) -> Result<Measurement, Error<E>> {
    Ok(convert(check_word(&frame[..3])?, check_word(&frame[3..])?))
}

macro_rules! constructors {
    () => {
        /// Create a driver; no bus traffic.
        pub const fn new(i2c: I2C, address: u8) -> Self {
            Self {
                i2c,
                address,
                repeatability: Repeatability::High,
            }
        }

        /// Select the repeatability used by `start_measurement`.
        pub const fn with_repeatability(mut self, repeatability: Repeatability) -> Self {
            self.repeatability = repeatability;
            self
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }
    };
}

/// Blocking driver.
pub struct Sht3x<I2C> {
    i2c: I2C,
    address: u8,
    repeatability: Repeatability,
}

impl<I2C> Sht3x<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Sht3x<I2C> {
    fn command(&mut self, cmd: [u8; 2]) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(self.address, &cmd).map_err(Error::bus)
    }

    /// Soft reset. Wait [`SOFT_RESET_WAIT_US`] before the next command.
    pub fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.command(CMD_SOFT_RESET)?;
        Ok(SOFT_RESET_WAIT_US)
    }

    /// Read the status register.
    pub fn read_status(&mut self) -> Result<Status, Error<I2C::Error>> {
        let mut buf = [0u8; 3];
        self.i2c
            .write_read(self.address, &CMD_READ_STATUS, &mut buf)
            .map_err(Error::bus)?;
        Ok(Status(check_word(&buf)?))
    }

    /// Clear the status register flags.
    pub fn clear_status(&mut self) -> Result<(), Error<I2C::Error>> {
        self.command(CMD_CLEAR_STATUS)
    }

    /// Switch the internal heater (condensation removal / plausibility test).
    pub fn set_heater(&mut self, on: bool) -> Result<(), Error<I2C::Error>> {
        self.command(if on { CMD_HEATER_ON } else { CMD_HEATER_OFF })
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Sht3x<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.command(self.repeatability.command())?;
        Ok(self.repeatability.wait_us())
    }

    /// Returns [`Error::Bus`] (NACK) if the conversion is still running.
    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        let mut frame = [0u8; 6];
        self.i2c
            .read(self.address, &mut frame)
            .map_err(Error::bus)?;
        decode_measurement(&frame)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        CMD_CLEAR_STATUS, CMD_HEATER_OFF, CMD_HEATER_ON, CMD_READ_STATUS, CMD_SOFT_RESET,
        Measurement, Repeatability, SOFT_RESET_WAIT_US, Status, check_word, decode_measurement,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Sht3x`].
    pub struct Sht3x<I2C> {
        i2c: I2C,
        address: u8,
        repeatability: Repeatability,
    }

    impl<I2C> Sht3x<I2C> {
        constructors!();
    }

    impl<I2C: I2c> Sht3x<I2C> {
        async fn command(&mut self, cmd: [u8; 2]) -> Result<(), Error<I2C::Error>> {
            self.i2c.write(self.address, &cmd).await.map_err(Error::bus)
        }

        /// See [`super::Sht3x::soft_reset`].
        pub async fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.command(CMD_SOFT_RESET).await?;
            Ok(SOFT_RESET_WAIT_US)
        }

        /// See [`super::Sht3x::read_status`].
        pub async fn read_status(&mut self) -> Result<Status, Error<I2C::Error>> {
            let mut buf = [0u8; 3];
            self.i2c
                .write_read(self.address, &CMD_READ_STATUS, &mut buf)
                .await
                .map_err(Error::bus)?;
            Ok(Status(check_word(&buf)?))
        }

        /// See [`super::Sht3x::clear_status`].
        pub async fn clear_status(&mut self) -> Result<(), Error<I2C::Error>> {
            self.command(CMD_CLEAR_STATUS).await
        }

        /// See [`super::Sht3x::set_heater`].
        pub async fn set_heater(&mut self, on: bool) -> Result<(), Error<I2C::Error>> {
            self.command(if on { CMD_HEATER_ON } else { CMD_HEATER_OFF })
                .await
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Sht3x<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.command(self.repeatability.command()).await?;
            Ok(self.repeatability.wait_us())
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let mut frame = [0u8; 6];
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
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use std::vec;
    use zigbee_sensor::blocking::Sensor;

    fn frame(raw_t: u16, raw_rh: u16) -> [u8; 6] {
        let [t0, t1] = raw_t.to_be_bytes();
        let [h0, h1] = raw_rh.to_be_bytes();
        [
            t0,
            t1,
            crc8_sensirion(&[t0, t1]),
            h0,
            h1,
            crc8_sensirion(&[h0, h1]),
        ]
    }

    #[test]
    fn conversion_matches_datasheet_formula() {
        assert_eq!(
            convert(0, 0),
            Measurement {
                temperature: -4_500,
                humidity: 0
            }
        );
        assert_eq!(
            convert(u16::MAX, u16::MAX),
            Measurement {
                temperature: 13_000,
                humidity: 10_000
            }
        );
        // 0x6666: T = -45 + 175 × 0.4 = 25.00 °C, RH = 40.00 %.
        assert_eq!(
            convert(0x6666, 0x6666),
            Measurement {
                temperature: 2_500,
                humidity: 4_000
            }
        );
    }

    #[test]
    fn start_then_read_without_internal_delay() {
        let expectations = [
            Transaction::write(PRIMARY_ADDRESS, vec![0x24, 0x0B]),
            Transaction::read(PRIMARY_ADDRESS, frame(0x6666, 0x6666).to_vec()),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut sht =
            Sht3x::new(&mut i2c, PRIMARY_ADDRESS).with_repeatability(Repeatability::Medium);

        assert_eq!(sht.start_measurement().unwrap(), 6_500);
        assert_eq!(
            sht.read_measurement().unwrap(),
            Measurement {
                temperature: 2_500,
                humidity: 4_000
            }
        );
        i2c.done();
    }

    #[test]
    fn corrupt_crc_is_reported() {
        let mut bad = frame(0x6666, 0x6666);
        bad[5] ^= 1;
        let expectations = [Transaction::read(PRIMARY_ADDRESS, bad.to_vec())];
        let mut i2c = Mock::new(&expectations);
        let mut sht = Sht3x::new(&mut i2c, PRIMARY_ADDRESS);
        assert_eq!(sht.read_measurement(), Err(Error::Crc));
        i2c.done();
    }

    #[test]
    fn status_flags() {
        let expectations = [Transaction::write_read(
            SECONDARY_ADDRESS,
            CMD_READ_STATUS.to_vec(),
            vec![0x20, 0x10, crc8_sensirion(&[0x20, 0x10])],
        )];
        let mut i2c = Mock::new(&expectations);
        let mut sht = Sht3x::new(&mut i2c, SECONDARY_ADDRESS);
        let status = sht.read_status().unwrap();
        assert!(status.heater_on());
        assert!(status.reset_detected());
        i2c.done();
    }

    #[test]
    fn async_measure_awaits_conversion_time() {
        use zigbee_sensor::asynch::Sensor as _;

        struct RecordingDelay(u32);
        impl embedded_hal_async::delay::DelayNs for RecordingDelay {
            async fn delay_ns(&mut self, ns: u32) {
                self.0 += ns / 1_000;
            }
        }

        let expectations = [
            Transaction::write(PRIMARY_ADDRESS, vec![0x24, 0x00]),
            Transaction::read(PRIMARY_ADDRESS, frame(0x6666, 0x6666).to_vec()),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut sht = asynch::Sht3x::new(&mut i2c, PRIMARY_ADDRESS);
        let mut delay = RecordingDelay(0);
        let m = block_on(sht.measure(&mut delay)).unwrap();
        assert_eq!(m.temperature, 2_500);
        assert_eq!(delay.0, 15_500);
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
