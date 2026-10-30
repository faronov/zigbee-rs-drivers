//! TE Connectivity HTU31D temperature and humidity sensor (I2C).
//!
//! One Conversion command (with the oversampling ratio of each channel
//! encoded in the command byte) measures T and RH; after the conversion
//! time, Read T&RH returns `T msb, lsb, crc, RH msb, lsb, crc`. The
//! sensor does not signal completion and keeps the previous result until
//! the conversion finishes, so the driver waits the typical conversion
//! time plus 50 %. CRC-8 is polynomial 0x31 with init 0x00.
//!
//! Reference: TE Connectivity "HTU31D RH/T SENSOR IC" datasheet (05/2020).
//!
//! ```ignore
//! let mut htu = Htu31d::new(i2c);
//! let serial = htu.serial_number()?;
//! let wait_us = htu.start_measurement()?;
//! // ... sleep wait_us ...
//! let m = htu.read_measurement()?;
//! ```

#![no_std]

use zigbee_sensor::crc::crc8_msb;
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address with IC_ADD tied to GND.
pub const PRIMARY_ADDRESS: u8 = 0x40;
/// I2C address with IC_ADD tied to VDD.
pub const SECONDARY_ADDRESS: u8 = 0x41;

const CMD_READ_T_RH: u8 = 0x00;
const CMD_HEATER_OFF: u8 = 0x02;
const CMD_HEATER_ON: u8 = 0x04;
const CMD_READ_DIAGNOSTIC: u8 = 0x08;
const CMD_READ_SERIAL: u8 = 0x0A;
const CMD_RESET: u8 = 0x1E;
const CMD_CONVERSION: u8 = 0x40;

/// Power-up time.
pub const POWER_UP_US: WaitUs = 10_000;
/// Soft reset time (timing table; the command description says < 5 ms).
pub const RESET_WAIT_US: WaitUs = 15_000;

/// Diagnostic register: NVM CRC error.
pub const DIAG_NVM_ERROR: u8 = 1 << 7;
/// Diagnostic register: humidity truncated to 0 or 2^16 − 1.
pub const DIAG_HUMIDITY_OVERRUN: u8 = 1 << 6;
/// Diagnostic register: humidity above 120 %RH.
pub const DIAG_HUMIDITY_HIGH: u8 = 1 << 5;
/// Diagnostic register: humidity below −10 %RH.
pub const DIAG_HUMIDITY_LOW: u8 = 1 << 4;
/// Diagnostic register: temperature truncated to 0 or 2^16 − 1.
pub const DIAG_TEMPERATURE_OVERRUN: u8 = 1 << 3;
/// Diagnostic register: temperature above 150 °C.
pub const DIAG_TEMPERATURE_HIGH: u8 = 1 << 2;
/// Diagnostic register: temperature below −50 °C.
pub const DIAG_TEMPERATURE_LOW: u8 = 1 << 1;
/// Diagnostic register: heater on.
pub const DIAG_HEATER_ON: u8 = 1 << 0;

/// Oversampling ratio of one channel.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Osr {
    /// OSR 0 (0.020 %RH / 0.040 °C).
    Osr0 = 0,
    /// OSR 1 (0.014 %RH / 0.025 °C).
    Osr1 = 1,
    /// OSR 2 (0.010 %RH / 0.016 °C).
    #[default]
    Osr2 = 2,
    /// OSR 3 (0.007 %RH / 0.012 °C).
    Osr3 = 3,
}

impl Osr {
    /// Typical humidity conversion time.
    pub const fn humidity_us(self) -> WaitUs {
        [1_110, 2_140, 4_210, 8_340][self as usize]
    }

    /// Typical temperature conversion time.
    pub const fn temperature_us(self) -> WaitUs {
        [1_570, 3_060, 6_030, 11_980][self as usize]
    }
}

/// Conversion settings.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// Humidity oversampling.
    pub humidity: Osr,
    /// Temperature oversampling.
    pub temperature: Osr,
}

impl Config {
    /// Conversion command byte: `0x40 | osr_rh << 3 | osr_t << 1`.
    pub const fn command(self) -> u8 {
        CMD_CONVERSION | ((self.humidity as u8) << 3) | ((self.temperature as u8) << 1)
    }

    /// Wait for one conversion: both typical times + 50 %.
    pub const fn wait_us(self) -> WaitUs {
        (self.humidity.humidity_us() + self.temperature.temperature_us()) * 3 / 2
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

/// HTU31D CRC-8 (polynomial 0x31, init 0x00).
pub fn crc8(bytes: &[u8]) -> u8 {
    crc8_msb(bytes, 0x31, 0x00)
}

/// Raw signals to engineering units:
/// T = −40 + 165 × S_T / (2^16 − 1), RH = 100 × S_RH / (2^16 − 1).
pub fn convert(raw_temperature: u16, raw_humidity: u16) -> Measurement {
    let t = (16_500 * u32::from(raw_temperature) + 32_767) / 65_535;
    let rh = (10_000 * u32::from(raw_humidity) + 32_767) / 65_535;
    Measurement {
        temperature: t as i32 - 4_000,
        humidity: rh as u16,
    }
}

fn checked_word<E>(chunk: &[u8]) -> Result<u16, Error<E>> {
    if crc8(&chunk[..2]) == chunk[2] {
        Ok(u16::from_be_bytes([chunk[0], chunk[1]]))
    } else {
        Err(Error::Crc)
    }
}

/// Decode a Read T&RH answer (T word + CRC, RH word + CRC).
pub fn decode_frame<E>(frame: &[u8; 6]) -> Result<Measurement, Error<E>> {
    let t = checked_word(&frame[..3])?;
    let rh = checked_word(&frame[3..])?;
    Ok(convert(t, rh))
}

fn decode_serial<E>(answer: &[u8; 4]) -> Result<u32, Error<E>> {
    if crc8(&answer[..3]) != answer[3] {
        return Err(Error::Crc);
    }
    Ok(u32::from_be_bytes([0, answer[0], answer[1], answer[2]]))
}

fn decode_diagnostic<E>(answer: &[u8; 2]) -> Result<u8, Error<E>> {
    if crc8(&answer[..1]) != answer[1] {
        return Err(Error::Crc);
    }
    Ok(answer[0])
}

macro_rules! constructors {
    () => {
        /// Create a driver at [`PRIMARY_ADDRESS`](crate::PRIMARY_ADDRESS)
        /// with OSR 2 for both channels; no bus traffic.
        pub const fn new(i2c: I2C) -> Self {
            Self {
                i2c,
                address: PRIMARY_ADDRESS,
                config: Config {
                    humidity: Osr::Osr2,
                    temperature: Osr::Osr2,
                },
            }
        }

        /// Use another I2C address.
        pub const fn with_address(mut self, address: u8) -> Self {
            self.address = address;
            self
        }

        /// Oversampling used by `start_measurement`.
        pub const fn with_config(mut self, config: Config) -> Self {
            self.config = config;
            self
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }
    };
}

/// Blocking driver.
pub struct Htu31d<I2C> {
    i2c: I2C,
    address: u8,
    config: Config,
}

impl<I2C> Htu31d<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Htu31d<I2C> {
    fn command(&mut self, cmd: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(self.address, &[cmd]).map_err(Error::bus)
    }

    fn command_read(&mut self, cmd: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write_read(self.address, &[cmd], buf)
            .map_err(Error::bus)
    }

    /// Read the 24-bit serial number (CRC-checked).
    pub fn serial_number(&mut self) -> Result<u32, Error<I2C::Error>> {
        let mut answer = [0u8; 4];
        self.command_read(CMD_READ_SERIAL, &mut answer)?;
        decode_serial(&answer)
    }

    /// Read the diagnostic register (see the `DIAG_*` bits).
    pub fn diagnostic(&mut self) -> Result<u8, Error<I2C::Error>> {
        let mut answer = [0u8; 2];
        self.command_read(CMD_READ_DIAGNOSTIC, &mut answer)?;
        decode_diagnostic(&answer)
    }

    /// Switch the heater; a reset also turns it off.
    pub fn set_heater(&mut self, on: bool) -> Result<(), Error<I2C::Error>> {
        self.command(if on { CMD_HEATER_ON } else { CMD_HEATER_OFF })
    }

    /// Soft reset. Wait [`RESET_WAIT_US`].
    pub fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.command(CMD_RESET)?;
        Ok(RESET_WAIT_US)
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Htu31d<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.command(self.config.command())?;
        Ok(self.config.wait_us())
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        let mut frame = [0u8; 6];
        self.command_read(CMD_READ_T_RH, &mut frame)?;
        decode_frame(&frame)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        CMD_HEATER_OFF, CMD_HEATER_ON, CMD_READ_DIAGNOSTIC, CMD_READ_SERIAL, CMD_READ_T_RH,
        CMD_RESET, Config, Measurement, Osr, PRIMARY_ADDRESS, RESET_WAIT_US, decode_diagnostic,
        decode_frame, decode_serial,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Htu31d`].
    pub struct Htu31d<I2C> {
        i2c: I2C,
        address: u8,
        config: Config,
    }

    impl<I2C> Htu31d<I2C> {
        constructors!();
    }

    impl<I2C: I2c> Htu31d<I2C> {
        async fn command(&mut self, cmd: u8) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write(self.address, &[cmd])
                .await
                .map_err(Error::bus)
        }

        async fn command_read(&mut self, cmd: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write_read(self.address, &[cmd], buf)
                .await
                .map_err(Error::bus)
        }

        /// See [`super::Htu31d::serial_number`].
        pub async fn serial_number(&mut self) -> Result<u32, Error<I2C::Error>> {
            let mut answer = [0u8; 4];
            self.command_read(CMD_READ_SERIAL, &mut answer).await?;
            decode_serial(&answer)
        }

        /// See [`super::Htu31d::diagnostic`].
        pub async fn diagnostic(&mut self) -> Result<u8, Error<I2C::Error>> {
            let mut answer = [0u8; 2];
            self.command_read(CMD_READ_DIAGNOSTIC, &mut answer).await?;
            decode_diagnostic(&answer)
        }

        /// See [`super::Htu31d::set_heater`].
        pub async fn set_heater(&mut self, on: bool) -> Result<(), Error<I2C::Error>> {
            self.command(if on { CMD_HEATER_ON } else { CMD_HEATER_OFF })
                .await
        }

        /// See [`super::Htu31d::soft_reset`].
        pub async fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.command(CMD_RESET).await?;
            Ok(RESET_WAIT_US)
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Htu31d<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.command(self.config.command()).await?;
            Ok(self.config.wait_us())
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let mut frame = [0u8; 6];
            self.command_read(CMD_READ_T_RH, &mut frame).await?;
            decode_frame(&frame)
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

    // Words from the datasheet CRC examples: 0x683A → 0x7C, 0x4E85 → 0x6B.
    const FRAME: [u8; 6] = [0x68, 0x3A, 0x7C, 0x4E, 0x85, 0x6B];
    const EXPECTED: Measurement = Measurement {
        temperature: 2_718,
        humidity: 3_067,
    };

    #[test]
    fn datasheet_crc_and_formulas() {
        assert_eq!(crc8(&[0xDC]), 0x79);
        assert_eq!(crc8(&[0x68, 0x3A]), 0x7C);
        assert_eq!(crc8(&[0x4E, 0x85]), 0x6B);
        // −40 + 165 × 26682 / 65535 = 27.18 °C; 100 × 20101 / 65535 = 30.67 %RH.
        assert_eq!(decode_frame::<()>(&FRAME), Ok(EXPECTED));
        assert_eq!(
            convert(0xFFFF, 0xFFFF),
            Measurement {
                temperature: 12_500,
                humidity: 10_000
            }
        );
        assert_eq!(convert(0, 0).temperature, -4_000);
        let mut bad = FRAME;
        bad[5] ^= 1;
        assert_eq!(decode_frame::<()>(&bad), Err(Error::Crc));
        // Command bits: 0 1 0 RH1 RH0 T1 T0 X.
        let max = Config {
            humidity: Osr::Osr3,
            temperature: Osr::Osr3,
        };
        assert_eq!(max.command(), 0x5E);
        assert_eq!(max.wait_us(), (8_340 + 11_980) * 3 / 2);
        assert_eq!(Config::default().command(), 0x54);
    }

    #[test]
    fn conversion_and_read() {
        let config = Config {
            humidity: Osr::Osr1,
            temperature: Osr::Osr0,
        };
        let expectations = [
            Transaction::write(SECONDARY_ADDRESS, vec![0x48]),
            Transaction::write_read(SECONDARY_ADDRESS, vec![0x00], FRAME.to_vec()),
            Transaction::write_read(SECONDARY_ADDRESS, vec![0x00], FRAME.to_vec())
                .with_error(embedded_hal::i2c::ErrorKind::Other),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut htu = Htu31d::new(&mut i2c)
            .with_address(SECONDARY_ADDRESS)
            .with_config(config);
        assert_eq!(htu.start_measurement().unwrap(), (2_140 + 1_570) * 3 / 2);
        assert_eq!(htu.read_measurement(), Ok(EXPECTED));
        assert!(matches!(htu.read_measurement(), Err(Error::Bus(_))));
        i2c.done();
    }

    #[test]
    fn serial_diagnostic_heater_reset() {
        let expectations = [
            Transaction::write_read(PRIMARY_ADDRESS, vec![0x0A], vec![0x12, 0x34, 0x56, 0xC1]),
            Transaction::write_read(PRIMARY_ADDRESS, vec![0x0A], vec![0x12, 0x34, 0x56, 0xC0]),
            Transaction::write_read(PRIMARY_ADDRESS, vec![0x08], vec![0x01, 0x31]),
            Transaction::write(PRIMARY_ADDRESS, vec![0x04]),
            Transaction::write(PRIMARY_ADDRESS, vec![0x02]),
            Transaction::write(PRIMARY_ADDRESS, vec![0x1E]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut htu = Htu31d::new(&mut i2c);
        assert_eq!(htu.serial_number(), Ok(0x12_3456));
        assert_eq!(htu.serial_number(), Err(Error::Crc));
        assert_eq!(htu.diagnostic(), Ok(DIAG_HEATER_ON));
        htu.set_heater(true).unwrap();
        htu.set_heater(false).unwrap();
        assert_eq!(htu.soft_reset(), Ok(RESET_WAIT_US));
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
            Transaction::write(PRIMARY_ADDRESS, vec![0x54]),
            Transaction::write_read(PRIMARY_ADDRESS, vec![0x00], FRAME.to_vec()),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut htu = asynch::Htu31d::new(&mut i2c);
        let mut delay = RecordingDelay(0);
        assert_eq!(block_on(htu.measure(&mut delay)), Ok(EXPECTED));
        assert_eq!(delay.0, (4_210 + 6_030) * 3 / 2);
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
