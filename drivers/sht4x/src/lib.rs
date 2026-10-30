//! Sensirion SHT40 / SHT41 / SHT43 / SHT45 temperature and humidity sensor
//! (I2C).
//!
//! All variants share one command set; they differ in accuracy and in the
//! factory-set I2C address (0x44, 0x45 or 0x46, see the order code). The
//! sensor idles at 80 nA between measurements and has no sleep command.
//!
//! Reference: Sensirion "Datasheet SHT4x", version 7.3 (June 2026).
//!
//! ```ignore
//! let mut sht = Sht4x::new(i2c, ADDRESS_A).with_precision(Precision::Medium);
//! let wait_us = sht.start_measurement()?;
//! // ... let the Zigbee stack run / sleep for wait_us ...
//! let m = sht.read_measurement()?;
//! ```

#![no_std]

use zigbee_sensirion_common::{decode_word, temperature_from_ticks};
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address of the -A (and -AD1x) variants.
pub const ADDRESS_A: u8 = 0x44;
/// I2C address of the -B variants.
pub const ADDRESS_B: u8 = 0x45;
/// I2C address of the -C variants.
pub const ADDRESS_C: u8 = 0x46;

const CMD_SERIAL: u8 = 0x89;
const CMD_SOFT_RESET: u8 = 0x94;

/// Time between the soft-reset ACK and the idle state (t_SR max, table 5).
pub const SOFT_RESET_WAIT_US: WaitUs = 1_000;
/// Wait before reading the serial number. The datasheet gives no execution
/// time (the sensor NACKs while busy); 10 ms is a safe margin.
pub const SERIAL_WAIT_US: u32 = 10_000;

/// Measurement precision (repeatability), datasheet table 5.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Precision {
    /// 0.04 %RH / 0.04 °C (SHT40) noise, 8.3 ms max.
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

/// Heater power (typical at VDD = 3.3 V), datasheet table 9.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HeaterPower {
    /// 200 mW.
    High,
    /// 110 mW.
    Medium,
    /// 20 mW.
    Low,
}

/// Heater-on duration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HeaterDuration {
    /// 1 s (1.1 s max).
    Long,
    /// 0.1 s (0.11 s max).
    Short,
}

/// Command byte of a heater pulse (datasheet table 8).
pub const fn heater_command(power: HeaterPower, duration: HeaterDuration) -> u8 {
    match (power, duration) {
        (HeaterPower::High, HeaterDuration::Long) => 0x39,
        (HeaterPower::High, HeaterDuration::Short) => 0x32,
        (HeaterPower::Medium, HeaterDuration::Long) => 0x2F,
        (HeaterPower::Medium, HeaterDuration::Short) => 0x24,
        (HeaterPower::Low, HeaterDuration::Long) => 0x1E,
        (HeaterPower::Low, HeaterDuration::Short) => 0x15,
    }
}

/// Maximum time of a heater pulse plus the high-precision measurement
/// taken at its end, in microseconds.
pub const fn heater_wait_us(duration: HeaterDuration) -> WaitUs {
    let pulse = match duration {
        HeaterDuration::Long => 1_100_000,
        HeaterDuration::Short => 110_000,
    };
    pulse + Precision::High.wait_us()
}

/// One temperature + humidity sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Temperature in 0.01 °C.
    pub temperature: i32,
    /// Relative humidity in 0.01 %RH, cropped to 0..=100 %.
    pub humidity: u16,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Temperature(self.temperature));
        f(Quantity::Humidity(self.humidity));
    }
}

/// Convert raw ticks (datasheet section 4.6, equations 1 and 2).
///
/// `RH = -6 + 125 × S / 65535` is cropped to 0..=100 %RH as the datasheet
/// advises.
pub fn convert(raw_temperature: u16, raw_humidity: u16) -> Measurement {
    let rh = (12_500 * u32::from(raw_humidity) + 32_767) / 65_535;
    Measurement {
        temperature: temperature_from_ticks(raw_temperature),
        humidity: (rh as i32 - 600).clamp(0, 10_000) as u16,
    }
}

/// Decode the 6-byte frame (T msb, lsb, crc, RH msb, lsb, crc).
pub fn decode_measurement<E>(frame: &[u8; 6]) -> Result<Measurement, Error<E>> {
    Ok(convert(
        decode_word(&frame[..3])?,
        decode_word(&frame[3..])?,
    ))
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
pub struct Sht4x<I2C> {
    i2c: I2C,
    address: u8,
    precision: Precision,
}

impl<I2C> Sht4x<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Sht4x<I2C> {
    fn command(&mut self, cmd: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(self.address, &[cmd]).map_err(Error::bus)
    }

    fn read_frame(&mut self) -> Result<[u8; 6], Error<I2C::Error>> {
        let mut frame = [0u8; 6];
        self.i2c
            .read(self.address, &mut frame)
            .map_err(Error::bus)?;
        Ok(frame)
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
        decode_serial(&self.read_frame()?)
    }

    /// Start a heater pulse. The sensor measures with high precision just
    /// before switching the heater off; fetch that sample with
    /// `read_measurement` after the returned wait. Keep the heater duty
    /// cycle below 10 % and use it only below 65 °C ambient (section 4.9).
    pub fn start_heater(
        &mut self,
        power: HeaterPower,
        duration: HeaterDuration,
    ) -> Result<WaitUs, Error<I2C::Error>> {
        self.command(heater_command(power, duration))?;
        Ok(heater_wait_us(duration))
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Sht4x<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.command(self.precision.command())?;
        Ok(self.precision.wait_us())
    }

    /// Returns [`Error::Bus`] (NACK) if the conversion is still running.
    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        decode_measurement(&self.read_frame()?)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        CMD_SERIAL, CMD_SOFT_RESET, HeaterDuration, HeaterPower, Measurement, Precision,
        SERIAL_WAIT_US, SOFT_RESET_WAIT_US, decode_measurement, decode_serial, heater_command,
        heater_wait_us,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Sht4x`].
    pub struct Sht4x<I2C> {
        i2c: I2C,
        address: u8,
        precision: Precision,
    }

    impl<I2C> Sht4x<I2C> {
        constructors!();
    }

    impl<I2C: I2c> Sht4x<I2C> {
        async fn command(&mut self, cmd: u8) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write(self.address, &[cmd])
                .await
                .map_err(Error::bus)
        }

        async fn read_frame(&mut self) -> Result<[u8; 6], Error<I2C::Error>> {
            let mut frame = [0u8; 6];
            self.i2c
                .read(self.address, &mut frame)
                .await
                .map_err(Error::bus)?;
            Ok(frame)
        }

        /// See [`super::Sht4x::soft_reset`].
        pub async fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.command(CMD_SOFT_RESET).await?;
            Ok(SOFT_RESET_WAIT_US)
        }

        /// See [`super::Sht4x::serial_number`].
        pub async fn serial_number<D: embedded_hal_async::delay::DelayNs>(
            &mut self,
            delay: &mut D,
        ) -> Result<u32, Error<I2C::Error>> {
            self.command(CMD_SERIAL).await?;
            delay.delay_us(SERIAL_WAIT_US).await;
            decode_serial(&self.read_frame().await?)
        }

        /// See [`super::Sht4x::start_heater`].
        pub async fn start_heater(
            &mut self,
            power: HeaterPower,
            duration: HeaterDuration,
        ) -> Result<WaitUs, Error<I2C::Error>> {
            self.command(heater_command(power, duration)).await?;
            Ok(heater_wait_us(duration))
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Sht4x<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.command(self.precision.command()).await?;
            Ok(self.precision.wait_us())
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            decode_measurement(&self.read_frame().await?)
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

    fn frame(a: u16, b: u16) -> std::vec::Vec<u8> {
        let mut v = encode_word(a).to_vec();
        v.extend_from_slice(&encode_word(b));
        v
    }

    #[test]
    fn conversion_matches_datasheet_equations() {
        // Section 4.6: T = -45 + 175 S/65535, RH = -6 + 125 S/65535.
        assert_eq!(
            convert(0x6666, 0x8000),
            Measurement {
                temperature: 2_500,
                humidity: 5_650
            }
        );
        // Below 0 %RH and above 100 %RH are cropped.
        assert_eq!(convert(0, 0).humidity, 0);
        assert_eq!(convert(u16::MAX, u16::MAX).humidity, 10_000);
        assert_eq!(convert(u16::MAX, 0).temperature, 13_000);
    }

    #[test]
    fn precision_commands_and_waits() {
        for (p, cmd, wait) in [
            (Precision::High, 0xFD, 8_300),
            (Precision::Medium, 0xF6, 4_500),
            (Precision::Low, 0xE0, 1_600),
        ] {
            let expectations = [
                Transaction::write(ADDRESS_A, vec![cmd]),
                Transaction::read(ADDRESS_A, frame(0x6666, 0x8000)),
            ];
            let mut i2c = Mock::new(&expectations);
            let mut sht = Sht4x::new(&mut i2c, ADDRESS_A).with_precision(p);
            assert_eq!(sht.start_measurement().unwrap(), wait);
            assert_eq!(sht.read_measurement().unwrap().temperature, 2_500);
            i2c.done();
        }
    }

    #[test]
    fn heater_pulse_then_read() {
        let expectations = [
            Transaction::write(ADDRESS_B, vec![0x24]),
            Transaction::read(ADDRESS_B, frame(0x8000, 0x4000)),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut sht = Sht4x::new(&mut i2c, ADDRESS_B);
        let wait = sht
            .start_heater(HeaterPower::Medium, HeaterDuration::Short)
            .unwrap();
        assert_eq!(wait, 118_300);
        assert_eq!(sht.read_measurement().unwrap().temperature, 4_250);
        i2c.done();
        assert_eq!(
            heater_command(HeaterPower::High, HeaterDuration::Long),
            0x39
        );
        assert_eq!(
            heater_command(HeaterPower::Low, HeaterDuration::Short),
            0x15
        );
    }

    #[test]
    fn serial_and_reset() {
        let expectations = [
            Transaction::write(ADDRESS_C, vec![0x89]),
            Transaction::read(ADDRESS_C, frame(0x1234, 0x5678)),
            Transaction::write(ADDRESS_C, vec![0x94]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut delay = CheckedDelay::new(&[Delay::blocking_delay_us(SERIAL_WAIT_US)]);
        let mut sht = Sht4x::new(&mut i2c, ADDRESS_C);
        assert_eq!(sht.serial_number(&mut delay).unwrap(), 0x1234_5678);
        assert_eq!(sht.soft_reset().unwrap(), 1_000);
        i2c.done();
        delay.done();
    }

    #[test]
    fn corrupt_crc_is_reported() {
        let mut bad = frame(0x6666, 0x8000);
        bad[2] ^= 1;
        let expectations = [Transaction::read(ADDRESS_A, bad)];
        let mut i2c = Mock::new(&expectations);
        let mut sht = Sht4x::new(&mut i2c, ADDRESS_A);
        assert_eq!(sht.read_measurement(), Err(Error::Crc));
        i2c.done();
    }

    #[test]
    fn async_measure_awaits_conversion_time() {
        use zigbee_sensor::asynch::Sensor as _;

        let expectations = [
            Transaction::write(ADDRESS_A, vec![0xFD]),
            Transaction::read(ADDRESS_A, frame(0x6666, 0x8000)),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut delay = CheckedDelay::new(&[Delay::async_delay_us(8_300)]);
        let mut sht = asynch::Sht4x::new(&mut i2c, ADDRESS_A);
        let m = block_on(sht.measure(&mut delay)).unwrap();
        assert_eq!(m.humidity, 5_650);
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
