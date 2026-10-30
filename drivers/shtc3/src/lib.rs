//! Sensirion SHTC1 / SHTC3 temperature and humidity sensor (I2C, 0x70).
//!
//! * SHTC3: normal (12.1 ms) and low-power (0.8 ms) measurement modes,
//!   sleep (0.3 µA typ.) / wake-up commands. Idle current without sleep is
//!   45 µA, so a battery device should wake, measure, read and sleep every
//!   cycle (datasheet section 5.4) using [`zigbee_sensor::blocking::LowPower`].
//! * SHTC1: same measurement commands, idles at 0.7 µA automatically; no
//!   sleep command ([`Error::Unsupported`]).
//!
//! Measurements use the "clock stretching disabled, temperature first"
//! commands; the sensor NACKs its read header until the conversion ends.
//!
//! References: Sensirion "Datasheet SHTC3", version 4 (December 2022);
//! "Datasheet SHTC1", version 5 (December 2022); application note "Low
//! Power Measurement Mode for SHTC1, SHTW2 and STSC1", version 4 (May 2016).
//!
//! ```ignore
//! let mut shtc = Shtc3::new(i2c);
//! shtc.probe()?;                     // detects SHTC1 / SHTC3
//! let wait = shtc.wake()?;           // LowPower::wake, 240 µs
//! // ... wait ...
//! let wait = shtc.start_measurement()?;
//! // ... wait ...
//! let m = shtc.read_measurement()?;
//! shtc.sleep()?;
//! ```

#![no_std]

use zigbee_sensirion_common::blocking::Interface;
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Fixed I2C address of SHTC1 and SHTC3.
pub const ADDRESS: u8 = 0x70;

const CMD_SLEEP: u16 = 0xB098;
const CMD_WAKEUP: u16 = 0x3517;
const CMD_SOFT_RESET: u16 = 0x805D;
const CMD_READ_ID: u16 = 0xEFC8;

/// Wake-up / soft-reset / power-up time (SHTC3 t_PU and t_SR max 240 µs,
/// SHTC1 239 µs / 230 µs).
pub const WAKE_WAIT_US: WaitUs = 240;
/// Soft-reset time, see [`WAKE_WAIT_US`].
pub const SOFT_RESET_WAIT_US: WaitUs = 240;

/// Part detected from the ID register.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Variant {
    /// SHTC1 (no sleep command).
    Shtc1,
    /// SHTC3.
    #[default]
    Shtc3,
}

/// Classify the 16-bit ID register (SHTC3 table 15, SHTC1 table 12).
///
/// SHTC3: bits 11 and 5:0 are `1` and `00_0111`. SHTC1: bits 5:0 are
/// `00_0111`, the rest unspecified.
pub const fn variant_from_id(id: u16) -> Option<Variant> {
    if id & 0x083F == 0x0807 {
        Some(Variant::Shtc3)
    } else if id & 0x003F == 0x0007 {
        Some(Variant::Shtc1)
    } else {
        None
    }
}

/// Measurement mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Mode {
    /// Full repeatability.
    #[default]
    Normal,
    /// Shorter conversion and less energy, higher noise (0.2 %RH / 0.3 °C
    /// repeatability on SHTC1).
    LowPower,
}

impl Mode {
    /// Measurement command, clock stretching disabled, T first (SHTC3
    /// table 11, SHTC1 table 9 and low-power application note table 1).
    pub const fn command(self) -> u16 {
        match self {
            Self::Normal => 0x7866,
            Self::LowPower => 0x609C,
        }
    }

    /// Maximum measurement duration for `variant`, in microseconds.
    pub const fn wait_us(self, variant: Variant) -> WaitUs {
        match (self, variant) {
            (Self::Normal, Variant::Shtc3) => 12_100,
            (Self::Normal, Variant::Shtc1) => 14_400,
            (Self::LowPower, Variant::Shtc3) => 800,
            (Self::LowPower, Variant::Shtc1) => 940,
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

/// Convert raw ticks (SHTC3 section 5.11): `T = -45 + 175 × S / 2^16`,
/// `RH = 100 × S / 2^16`.
pub fn convert(raw_temperature: u16, raw_humidity: u16) -> Measurement {
    let t = (17_500 * u32::from(raw_temperature) + 32_768) >> 16;
    let rh = (10_000 * u32::from(raw_humidity) + 32_768) >> 16;
    Measurement {
        temperature: t as i32 - 4_500,
        humidity: rh as u16,
    }
}

macro_rules! constructors {
    () => {
        /// Create a driver for an SHTC3; no bus traffic. Call `probe` or
        /// [`Self::with_variant`] for an SHTC1.
        pub const fn new(i2c: I2C) -> Self {
            Self {
                bus: Interface::new(i2c, ADDRESS),
                variant: Variant::Shtc3,
                mode: Mode::Normal,
            }
        }

        /// Set the part without probing.
        pub const fn with_variant(mut self, variant: Variant) -> Self {
            self.variant = variant;
            self
        }

        /// Select the measurement mode.
        pub const fn with_mode(mut self, mode: Mode) -> Self {
            self.mode = mode;
            self
        }

        /// Part in use.
        pub const fn variant(&self) -> Variant {
            self.variant
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.bus.release()
        }
    };
}

/// Blocking driver.
pub struct Shtc3<I2C> {
    bus: Interface<I2C>,
    variant: Variant,
    mode: Mode,
}

impl<I2C> Shtc3<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Shtc3<I2C> {
    /// Read the ID register, detect SHTC1 / SHTC3 and remember it. The
    /// SHTC3 must be awake.
    pub fn probe(&mut self) -> Result<Variant, Error<I2C::Error>> {
        self.bus.write(CMD_READ_ID, &[])?;
        let mut id = [0u16];
        self.bus.read(&mut id)?;
        self.variant = variant_from_id(id[0]).ok_or(Error::UnexpectedId(u32::from(id[0])))?;
        Ok(self.variant)
    }

    /// Soft reset (only while idle). Wait [`SOFT_RESET_WAIT_US`].
    pub fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.bus.write(CMD_SOFT_RESET, &[])?;
        Ok(SOFT_RESET_WAIT_US)
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Shtc3<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.bus.write(self.mode.command(), &[])?;
        Ok(self.mode.wait_us(self.variant))
    }

    /// Returns [`Error::Bus`] (NACK) if the conversion is still running.
    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        let mut raw = [0u16; 2];
        self.bus.read(&mut raw)?;
        Ok(convert(raw[0], raw[1]))
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::LowPower for Shtc3<I2C> {
    type BusError = I2C::Error;

    /// SHTC3 sleep (0.3 µA). [`Error::Unsupported`] on SHTC1, which idles
    /// at 0.7 µA by itself.
    fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
        if self.variant == Variant::Shtc1 {
            return Err(Error::Unsupported);
        }
        self.bus.write(CMD_SLEEP, &[])
    }

    /// Wake-up command (SHTC3); returns 0 on SHTC1 without bus traffic.
    fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        if self.variant == Variant::Shtc1 {
            return Ok(0);
        }
        self.bus.write(CMD_WAKEUP, &[])?;
        Ok(WAKE_WAIT_US)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        ADDRESS, CMD_READ_ID, CMD_SLEEP, CMD_SOFT_RESET, CMD_WAKEUP, Measurement, Mode,
        SOFT_RESET_WAIT_US, Variant, WAKE_WAIT_US, convert, variant_from_id,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensirion_common::asynch::Interface;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Shtc3`].
    pub struct Shtc3<I2C> {
        bus: Interface<I2C>,
        variant: Variant,
        mode: Mode,
    }

    impl<I2C> Shtc3<I2C> {
        constructors!();
    }

    impl<I2C: I2c> Shtc3<I2C> {
        /// See [`super::Shtc3::probe`].
        pub async fn probe(&mut self) -> Result<Variant, Error<I2C::Error>> {
            self.bus.write(CMD_READ_ID, &[]).await?;
            let mut id = [0u16];
            self.bus.read(&mut id).await?;
            self.variant = variant_from_id(id[0]).ok_or(Error::UnexpectedId(u32::from(id[0])))?;
            Ok(self.variant)
        }

        /// See [`super::Shtc3::soft_reset`].
        pub async fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.bus.write(CMD_SOFT_RESET, &[]).await?;
            Ok(SOFT_RESET_WAIT_US)
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Shtc3<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.bus.write(self.mode.command(), &[]).await?;
            Ok(self.mode.wait_us(self.variant))
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let mut raw = [0u16; 2];
            self.bus.read(&mut raw).await?;
            Ok(convert(raw[0], raw[1]))
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::LowPower for Shtc3<I2C> {
        type BusError = I2C::Error;

        async fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
            if self.variant == Variant::Shtc1 {
                return Err(Error::Unsupported);
            }
            self.bus.write(CMD_SLEEP, &[]).await
        }

        async fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            if self.variant == Variant::Shtc1 {
                return Ok(0);
            }
            self.bus.write(CMD_WAKEUP, &[]).await?;
            Ok(WAKE_WAIT_US)
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use std::vec;
    use std::vec::Vec;
    use zigbee_sensirion_common::encode_word;
    use zigbee_sensor::blocking::{LowPower, Sensor};

    fn words(w: &[u16]) -> Vec<u8> {
        w.iter().flat_map(|&x| encode_word(x)).collect()
    }

    #[test]
    fn conversion_matches_datasheet_formula() {
        // Section 5.11 with 2^16 divisor: 0x8000 → 42.5 °C, 50 %RH.
        assert_eq!(
            convert(0x8000, 0x8000),
            Measurement {
                temperature: 4_250,
                humidity: 5_000
            }
        );
        assert_eq!(convert(0, 0).temperature, -4_500);
    }

    #[test]
    fn id_classification() {
        assert_eq!(variant_from_id(0x0807), Some(Variant::Shtc3));
        assert_eq!(variant_from_id(0xF8C7), Some(Variant::Shtc3));
        assert_eq!(variant_from_id(0x0007), Some(Variant::Shtc1));
        assert_eq!(variant_from_id(0x1234), None);
    }

    #[test]
    fn full_low_power_cycle() {
        let expectations = [
            Transaction::write(ADDRESS, vec![0x35, 0x17]),
            Transaction::write(ADDRESS, vec![0xEF, 0xC8]),
            Transaction::read(ADDRESS, words(&[0x0887])),
            Transaction::write(ADDRESS, vec![0x60, 0x9C]),
            Transaction::read(ADDRESS, words(&[0x6666, 0x8000])),
            Transaction::write(ADDRESS, vec![0xB0, 0x98]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut shtc = Shtc3::new(&mut i2c).with_mode(Mode::LowPower);
        assert_eq!(shtc.wake().unwrap(), 240);
        assert_eq!(shtc.probe().unwrap(), Variant::Shtc3);
        assert_eq!(shtc.start_measurement().unwrap(), 800);
        let m = shtc.read_measurement().unwrap();
        assert_eq!(m.humidity, 5_000);
        shtc.sleep().unwrap();
        i2c.done();
    }

    #[test]
    fn shtc1_has_no_sleep_and_wrong_id_is_reported() {
        let expectations = [
            Transaction::write(ADDRESS, vec![0xEF, 0xC8]),
            Transaction::read(ADDRESS, words(&[0x0047])),
            Transaction::write(ADDRESS, vec![0x78, 0x66]),
            Transaction::write(ADDRESS, vec![0xEF, 0xC8]),
            Transaction::read(ADDRESS, words(&[0x1234])),
            Transaction::write(ADDRESS, vec![0x80, 0x5D]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut shtc = Shtc3::new(&mut i2c);
        assert_eq!(shtc.probe().unwrap(), Variant::Shtc1);
        assert_eq!(shtc.sleep(), Err(Error::Unsupported));
        assert_eq!(shtc.wake().unwrap(), 0);
        assert_eq!(shtc.start_measurement().unwrap(), 14_400);
        assert_eq!(shtc.probe(), Err(Error::UnexpectedId(0x1234)));
        assert_eq!(shtc.soft_reset().unwrap(), 240);
        i2c.done();
    }

    #[test]
    fn crc_error() {
        let mut bad = words(&[0x6666, 0x8000]);
        bad[5] ^= 0x10;
        let expectations = [Transaction::read(ADDRESS, bad)];
        let mut i2c = Mock::new(&expectations);
        let mut shtc = Shtc3::new(&mut i2c);
        assert_eq!(shtc.read_measurement(), Err(Error::Crc));
        i2c.done();
    }

    #[test]
    fn async_measure() {
        use zigbee_sensor::asynch::{LowPower as _, Sensor as _};

        struct RecordingDelay(u32);
        impl embedded_hal_async::delay::DelayNs for RecordingDelay {
            async fn delay_ns(&mut self, ns: u32) {
                self.0 += ns / 1_000;
            }
        }

        let expectations = [
            Transaction::write(ADDRESS, vec![0x35, 0x17]),
            Transaction::write(ADDRESS, vec![0x78, 0x66]),
            Transaction::read(ADDRESS, words(&[0x6666, 0x8000])),
            Transaction::write(ADDRESS, vec![0xB0, 0x98]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut shtc = asynch::Shtc3::new(&mut i2c);
        let mut delay = RecordingDelay(0);
        block_on(async {
            assert_eq!(shtc.wake().await.unwrap(), 240);
            let m = shtc.measure(&mut delay).await.unwrap();
            assert_eq!(m.temperature, 2_500);
            shtc.sleep().await.unwrap();
        });
        assert_eq!(delay.0, 12_100);
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
