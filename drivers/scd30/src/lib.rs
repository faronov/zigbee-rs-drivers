//! Sensirion SCD30 NDIR CO2 sensor module with temperature and humidity
//! (I2C, 0x61).
//!
//! Commands: continuous measurement with optional ambient pressure
//! compensation, stop, measurement interval, data-ready status, read
//! measurement, automatic self-calibration, forced recalibration,
//! temperature offset, altitude, firmware version and soft reset.
//!
//! The SCD30 needs an I2C master with clock stretching (up to 30 ms, once a
//! day up to 150 ms), at most 100 kHz (50 kHz recommended), and no repeated
//! start. The read header must follow the command by more than 3 ms.
//!
//! Measured values are IEEE 754 single-precision floats on the wire. They
//! are converted to integer units with integer-only bit manipulation
//! ([`float_bits_to_scaled`]); no floating point is used.
//!
//! # Measurement through the [`Sensor`](zigbee_sensor::blocking::Sensor) trait
//!
//! The SCD30 measures continuously (default every 2 s, 19 mA average); the
//! measuring state and interval survive power cycles. The first
//! `start_measurement` (re)starts continuous measurement with the configured
//! ambient pressure and returns the interval; the following
//! `read_measurement` returns [`Error::NotReady`]. Every later
//! `start_measurement` sends `read_measurement` and returns 3 ms; the next
//! `read_measurement` returns the latest sample. Call
//! `start_continuous_measurement` at initialisation to skip the empty first
//! cycle, and poll no faster than the measurement interval (or check
//! `get_data_ready_status` first).
//!
//! There is no sleep command; switch the supply off for low power.
//!
//! Reference: Sensirion "Interface Description Sensirion SCD30 Sensor
//! Module", version 1.0 (May 2020); "Datasheet SCD30", version 1.0 (May
//! 2020).

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Fixed I2C address.
pub const ADDRESS: u8 = 0x61;

mod cmd {
    pub const TRIGGER_CONTINUOUS: u16 = 0x0010;
    pub const STOP_CONTINUOUS: u16 = 0x0104;
    pub const MEASUREMENT_INTERVAL: u16 = 0x4600;
    pub const GET_DATA_READY: u16 = 0x0202;
    pub const READ_MEASUREMENT: u16 = 0x0300;
    pub const ASC: u16 = 0x5306;
    pub const FRC: u16 = 0x5204;
    pub const TEMPERATURE_OFFSET: u16 = 0x5403;
    pub const ALTITUDE: u16 = 0x5102;
    pub const FIRMWARE_VERSION: u16 = 0xD100;
    pub const SOFT_RESET: u16 = 0xD304;
}

/// Minimum gap between a command and its read header (> 3 ms); also
/// returned by the configuration commands as the time before the next
/// command.
pub const COMMAND_WAIT_US: WaitUs = 3_000;
/// Boot time after power-up or soft reset (< 2 s).
pub const BOOT_WAIT_US: WaitUs = 2_000_000;
/// Default measurement interval in seconds.
pub const DEFAULT_INTERVAL_S: u16 = 2;

/// One sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// CO2 in ppm (0–10 000; negative readings are clamped to 0).
    pub co2: u32,
    /// Temperature in 0.01 °C.
    pub temperature: i32,
    /// Relative humidity in 0.01 %RH, clamped to 0–100 %.
    pub humidity: u16,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Co2(self.co2));
        f(Quantity::Temperature(self.temperature));
        f(Quantity::Humidity(self.humidity));
    }
}

/// Convert the bit pattern of an IEEE 754 single-precision float to
/// `round(value × scale)` using integer arithmetic only.
///
/// Returns `None` for NaN, infinity or results outside `i32`. Subnormals
/// round to 0.
pub fn float_bits_to_scaled(bits: u32, scale: u32) -> Option<i32> {
    let negative = bits >> 31 != 0;
    let exponent = ((bits >> 23) & 0xFF) as i32;
    if exponent == 0xFF {
        return None;
    }
    if exponent == 0 {
        return Some(0);
    }
    // value = mantissa × 2^(exponent - 150), mantissa with implicit 1.
    let product = u64::from((bits & 0x7F_FFFF) | 0x80_0000) * u64::from(scale);
    let shift = exponent - 150;
    let magnitude = if shift >= 0 {
        if shift > 31 {
            return None;
        }
        product.checked_shl(shift as u32)?
    } else {
        let s = (-shift) as u32;
        if s >= 63 {
            0
        } else {
            (product + (1 << (s - 1))) >> s
        }
    };
    let magnitude = i64::try_from(magnitude).ok()?;
    i32::try_from(if negative { -magnitude } else { magnitude }).ok()
}

/// Decode the six `read_measurement` words (CO2, T, RH as big-endian
/// floats, interface description section 1.5).
pub fn convert(words: &[u16; 6]) -> Option<Measurement> {
    let float = |i: usize| (u32::from(words[i]) << 16) | u32::from(words[i + 1]);
    let co2 = float_bits_to_scaled(float(0), 1)?;
    let temperature = float_bits_to_scaled(float(2), 100)?;
    let humidity = float_bits_to_scaled(float(4), 100)?;
    Some(Measurement {
        co2: co2.max(0) as u32,
        temperature,
        humidity: humidity.clamp(0, 10_000) as u16,
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Pending {
    Nothing,
    Read,
}

macro_rules! constructors {
    () => {
        /// Create the driver; no bus traffic. Ambient pressure compensation
        /// off, interval assumed 2 s until [`Self::with_interval`] or
        /// `set_measurement_interval`.
        pub const fn new(i2c: I2C) -> Self {
            Self {
                bus: Interface::new(i2c, ADDRESS),
                pressure_mbar: 0,
                interval_s: DEFAULT_INTERVAL_S,
                running: false,
                pending: Pending::Nothing,
            }
        }

        /// Ambient pressure (mbar, 700–1400, 0 = off) sent with the start
        /// command by the `Sensor` trait. Not range-checked here.
        pub const fn with_ambient_pressure(mut self, mbar: u16) -> Self {
            self.pressure_mbar = mbar;
            self
        }

        /// Interval (s) the sensor is known to be configured to; returned
        /// by the first `start_measurement`. The setting is stored in the
        /// sensor's non-volatile memory.
        pub const fn with_interval(mut self, seconds: u16) -> Self {
            self.interval_s = seconds;
            self
        }

        /// `true` once the driver has started continuous measurement.
        pub const fn is_running(&self) -> bool {
            self.running
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.bus.release()
        }
    };
}

// Bus methods shared by the blocking and async drivers: `$async` is empty or
// `async`, `$aw` is empty or `.await`.
macro_rules! methods {
    ([$($async:tt)*] [$($aw:tt)*] $Delay:path) => {
        $($async)* fn send(&mut self, cmd: u16, args: &[u16]) -> Result<WaitUs, Error<I2C::Error>> {
            self.bus.write(cmd, args) $($aw)* ?;
            Ok(COMMAND_WAIT_US)
        }

        $($async)* fn get<D: $Delay>(&mut self, cmd: u16, delay: &mut D) -> Result<u16, Error<I2C::Error>> {
            let mut word = [0u16];
            self.bus.fetch(cmd, &[], COMMAND_WAIT_US, delay, &mut word) $($aw)* ?;
            Ok(word[0])
        }

        /// Trigger continuous measurement with ambient pressure
        /// compensation (`mbar` 700–1400, 0 = off; overrides the altitude).
        /// Also used to update the pressure while running. Returns the
        /// configured interval; invalid pressures give
        /// [`Error::InvalidArgument`] without bus traffic.
        pub $($async)* fn start_continuous_measurement(
            &mut self,
            mbar: u16,
        ) -> Result<WaitUs, Error<I2C::Error>> {
            if mbar != 0 && !(700..=1400).contains(&mbar) {
                return Err(Error::InvalidArgument);
            }
            self.send(cmd::TRIGGER_CONTINUOUS, &[mbar]) $($aw)* ?;
            self.pressure_mbar = mbar;
            self.running = true;
            self.pending = Pending::Nothing;
            Ok(u32::from(self.interval_s) * 1_000_000)
        }

        /// Stop continuous measurement.
        pub $($async)* fn stop_continuous_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::STOP_CONTINUOUS, &[]) $($aw)* ?;
            self.running = false;
            self.pending = Pending::Nothing;
            Ok(COMMAND_WAIT_US)
        }

        /// Set the measurement interval, 2–1800 s (non-volatile). Other
        /// values give [`Error::InvalidArgument`].
        pub $($async)* fn set_measurement_interval(
            &mut self,
            seconds: u16,
        ) -> Result<WaitUs, Error<I2C::Error>> {
            if !(2..=1800).contains(&seconds) {
                return Err(Error::InvalidArgument);
            }
            self.send(cmd::MEASUREMENT_INTERVAL, &[seconds]) $($aw)* ?;
            self.interval_s = seconds;
            Ok(COMMAND_WAIT_US)
        }

        /// Measurement interval in seconds (also remembered by the driver).
        pub $($async)* fn get_measurement_interval<D: $Delay>(
            &mut self,
            delay: &mut D,
        ) -> Result<u16, Error<I2C::Error>> {
            let seconds = self.get(cmd::MEASUREMENT_INTERVAL, delay) $($aw)* ?;
            self.interval_s = seconds;
            Ok(seconds)
        }

        /// `true` if a sample is waiting in the buffer.
        pub $($async)* fn get_data_ready_status<D: $Delay>(
            &mut self,
            delay: &mut D,
        ) -> Result<bool, Error<I2C::Error>> {
            Ok(self.get(cmd::GET_DATA_READY, delay) $($aw)* ? == 1)
        }

        /// Enable or disable continuous automatic self-calibration
        /// (non-volatile, default off; needs 7 days and 1 h of fresh air
        /// per day).
        pub $($async)* fn set_automatic_self_calibration(
            &mut self,
            enabled: bool,
        ) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::ASC, &[u16::from(enabled)]) $($aw)*
        }

        /// `true` if automatic self-calibration is enabled.
        pub $($async)* fn get_automatic_self_calibration<D: $Delay>(
            &mut self,
            delay: &mut D,
        ) -> Result<bool, Error<I2C::Error>> {
            Ok(self.get(cmd::ASC, delay) $($aw)* ? == 1)
        }

        /// Forced recalibration to a reference of 400–2000 ppm (run the
        /// sensor ≥ 2 min at a 2 s interval first). Other values give
        /// [`Error::InvalidArgument`].
        pub $($async)* fn set_forced_recalibration(
            &mut self,
            ppm: u16,
        ) -> Result<WaitUs, Error<I2C::Error>> {
            if !(400..=2000).contains(&ppm) {
                return Err(Error::InvalidArgument);
            }
            self.send(cmd::FRC, &[ppm]) $($aw)*
        }

        /// Last FRC reference in ppm (400 after power-up).
        pub $($async)* fn get_forced_recalibration<D: $Delay>(
            &mut self,
            delay: &mut D,
        ) -> Result<u16, Error<I2C::Error>> {
            self.get(cmd::FRC, delay) $($aw)*
        }

        /// Temperature offset in 0.01 °C (non-volatile).
        pub $($async)* fn set_temperature_offset(
            &mut self,
            centi_celsius: u16,
        ) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::TEMPERATURE_OFFSET, &[centi_celsius]) $($aw)*
        }

        /// Temperature offset in 0.01 °C.
        pub $($async)* fn get_temperature_offset<D: $Delay>(
            &mut self,
            delay: &mut D,
        ) -> Result<u16, Error<I2C::Error>> {
            self.get(cmd::TEMPERATURE_OFFSET, delay) $($aw)*
        }

        /// Altitude in metres above sea level (non-volatile; ignored while
        /// an ambient pressure is set).
        pub $($async)* fn set_altitude(&mut self, metres: u16) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::ALTITUDE, &[metres]) $($aw)*
        }

        /// Altitude in metres.
        pub $($async)* fn get_altitude<D: $Delay>(
            &mut self,
            delay: &mut D,
        ) -> Result<u16, Error<I2C::Error>> {
            self.get(cmd::ALTITUDE, delay) $($aw)*
        }

        /// Firmware version `(major, minor)`.
        pub $($async)* fn firmware_version<D: $Delay>(
            &mut self,
            delay: &mut D,
        ) -> Result<(u8, u8), Error<I2C::Error>> {
            let [major, minor] = self.get(cmd::FIRMWARE_VERSION, delay) $($aw)* ?.to_be_bytes();
            Ok((major, minor))
        }

        /// Restart the sensor (state as after power-up; < 2 s boot).
        pub $($async)* fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::SOFT_RESET, &[]) $($aw)* ?;
            self.running = false;
            self.pending = Pending::Nothing;
            Ok(BOOT_WAIT_US)
        }

        $($async)* fn start(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            if !self.running {
                return self.start_continuous_measurement(self.pressure_mbar) $($aw)*;
            }
            self.send(cmd::READ_MEASUREMENT, &[]) $($aw)* ?;
            self.pending = Pending::Read;
            Ok(COMMAND_WAIT_US)
        }

        $($async)* fn read(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            if self.pending != Pending::Read {
                return Err(Error::NotReady);
            }
            self.pending = Pending::Nothing;
            let mut w = [0u16; 6];
            self.bus.read(&mut w) $($aw)* .map_err(nack_as_not_ready)?;
            convert(&w).ok_or(Error::InvalidData)
        }
    };
}

macro_rules! trait_impls {
    ($Sensor:path, $I2c:path, [$($async:tt)*] [$($aw:tt)*]) => {
        impl<I2C: $I2c> $Sensor for Scd30<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// Starts continuous measurement on the first call, then
            /// requests the latest sample (3 ms).
            $($async)* fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.start() $($aw)*
            }

            /// [`Error::NotReady`] if no sample was requested;
            /// [`Error::InvalidData`] for NaN / infinite values.
            $($async)* fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                self.read() $($aw)*
            }
        }
    };
}

use zigbee_sensirion_common::blocking::Interface;
use zigbee_sensirion_common::nack_as_not_ready;

/// Blocking driver.
#[derive(Debug)]
pub struct Scd30<I2C> {
    bus: Interface<I2C>,
    pressure_mbar: u16,
    interval_s: u16,
    running: bool,
    pending: Pending,
}

impl<I2C> Scd30<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Scd30<I2C> {
    methods!([] [] embedded_hal::delay::DelayNs);
}

trait_impls!(zigbee_sensor::blocking::Sensor, embedded_hal::i2c::I2c, [] []);

/// Async driver.
pub mod asynch {
    use super::{
        ADDRESS, BOOT_WAIT_US, COMMAND_WAIT_US, DEFAULT_INTERVAL_S, Measurement, Pending, cmd,
        convert,
    };
    use zigbee_sensirion_common::asynch::Interface;
    use zigbee_sensirion_common::nack_as_not_ready;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Scd30`].
    #[derive(Debug)]
    pub struct Scd30<I2C> {
        bus: Interface<I2C>,
        pressure_mbar: u16,
        interval_s: u16,
        running: bool,
        pending: Pending,
    }

    impl<I2C> Scd30<I2C> {
        constructors!();
    }

    impl<I2C: embedded_hal_async::i2c::I2c> Scd30<I2C> {
        methods!([async] [.await] embedded_hal_async::delay::DelayNs);
    }

    trait_impls!(
        zigbee_sensor::asynch::Sensor,
        embedded_hal_async::i2c::I2c,
        [async] [.await]
    );
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::delay::{CheckedDelay, Transaction as D};
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use std::vec;
    use zigbee_sensor::blocking::Sensor;

    // Interface description section 1.1.2: 439 ppm, 27.2 °C, 48.8 %RH.
    const FRAME: [u8; 18] = [
        0x43, 0xDB, 0xCB, 0x8C, 0x2E, 0x8F, 0x41, 0xD9, 0x70, 0xE7, 0xFF, 0xF5, 0x42, 0x43, 0xBF,
        0x3A, 0x1B, 0x74,
    ];

    #[test]
    fn float_decoding_without_floats() {
        assert_eq!(float_bits_to_scaled(0x43C8_0000, 1), Some(400));
        assert_eq!(float_bits_to_scaled(0x43DB_8C2E, 1), Some(439));
        assert_eq!(float_bits_to_scaled(0x41D9_70E7, 100), Some(2_718));
        assert_eq!(float_bits_to_scaled(0x4243_3A1B, 100), Some(4_881));
        assert_eq!(float_bits_to_scaled(0xC120_0000, 100), Some(-1_000)); // -10.0
        assert_eq!(float_bits_to_scaled(0x3F00_0000, 1), Some(1)); // 0.5 rounds up
        assert_eq!(float_bits_to_scaled(0x0000_0000, 100), Some(0));
        assert_eq!(float_bits_to_scaled(0x8000_0001, 100), Some(0));
        assert_eq!(float_bits_to_scaled(0x7FC0_0000, 1), None); // NaN
        assert_eq!(float_bits_to_scaled(0x7F80_0000, 1), None); // +inf
        assert_eq!(float_bits_to_scaled(0x4F00_0000, 1), None); // 2^31
    }

    #[test]
    fn continuous_cycle_with_datasheet_frame() {
        let expectations = [
            Transaction::write(ADDRESS, vec![0x00, 0x10, 0x00, 0x00, 0x81]),
            Transaction::write(ADDRESS, vec![0x03, 0x00]),
            Transaction::read(ADDRESS, FRAME.to_vec()),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut scd = Scd30::new(&mut i2c);
        assert_eq!(scd.start_measurement().unwrap(), 2_000_000);
        assert_eq!(scd.read_measurement(), Err(Error::NotReady));
        assert_eq!(scd.start_measurement().unwrap(), 3_000);
        assert_eq!(
            scd.read_measurement().unwrap(),
            Measurement {
                co2: 439,
                temperature: 2_724,
                humidity: 4_881
            }
        );
        i2c.done();
    }

    #[test]
    fn configuration_matches_datasheet_frames() {
        let expectations = [
            Transaction::write(ADDRESS, vec![0x46, 0x00, 0x00, 0x02, 0xE3]),
            Transaction::write(ADDRESS, vec![0x46, 0x00]),
            Transaction::read(ADDRESS, vec![0x00, 0x02, 0xE3]),
            Transaction::write(ADDRESS, vec![0x02, 0x02]),
            Transaction::read(ADDRESS, vec![0x00, 0x01, 0xB0]),
            Transaction::write(ADDRESS, vec![0x53, 0x06, 0x00, 0x00, 0x81]),
            Transaction::write(ADDRESS, vec![0x52, 0x04, 0x01, 0xC2, 0x50]),
            Transaction::write(ADDRESS, vec![0x54, 0x03, 0x01, 0xF4, 0x33]),
            Transaction::write(ADDRESS, vec![0x51, 0x02, 0x03, 0xE8, 0xD4]),
            Transaction::write(ADDRESS, vec![0xD1, 0x00]),
            Transaction::read(ADDRESS, vec![0x03, 0x42, 0xF3]),
            Transaction::write(ADDRESS, vec![0x01, 0x04]),
            Transaction::write(ADDRESS, vec![0xD3, 0x04]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut delay = CheckedDelay::new(&[
            D::blocking_delay_us(3_000),
            D::blocking_delay_us(3_000),
            D::blocking_delay_us(3_000),
        ]);
        let mut scd = Scd30::new(&mut i2c);
        assert_eq!(scd.set_measurement_interval(2).unwrap(), 3_000);
        assert_eq!(scd.get_measurement_interval(&mut delay).unwrap(), 2);
        assert!(scd.get_data_ready_status(&mut delay).unwrap());
        scd.set_automatic_self_calibration(false).unwrap();
        scd.set_forced_recalibration(450).unwrap();
        scd.set_temperature_offset(500).unwrap();
        scd.set_altitude(1_000).unwrap();
        assert_eq!(scd.firmware_version(&mut delay).unwrap(), (3, 0x42));
        scd.stop_continuous_measurement().unwrap();
        assert_eq!(scd.soft_reset().unwrap(), 2_000_000);
        assert_eq!(scd.set_measurement_interval(1), Err(Error::InvalidArgument));
        assert_eq!(
            scd.set_forced_recalibration(300),
            Err(Error::InvalidArgument)
        );
        assert_eq!(
            scd.start_continuous_measurement(500),
            Err(Error::InvalidArgument)
        );
        delay.done();
        i2c.done();
    }

    #[test]
    fn crc_and_nan() {
        let mut bad = FRAME;
        bad[2] ^= 1;
        let mut nan = FRAME;
        nan[0..3].copy_from_slice(&[0x7F, 0xC0, zigbee_sensirion_common::crc8(&[0x7F, 0xC0])]);
        let expectations = [
            Transaction::write(ADDRESS, vec![0x00, 0x10, 0x03, 0xF5, 0xDB]),
            Transaction::write(ADDRESS, vec![0x03, 0x00]),
            Transaction::read(ADDRESS, bad.to_vec()),
            Transaction::write(ADDRESS, vec![0x03, 0x00]),
            Transaction::read(ADDRESS, nan.to_vec()),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut scd = Scd30::new(&mut i2c)
            .with_ambient_pressure(1013)
            .with_interval(5);
        assert_eq!(scd.start_measurement().unwrap(), 5_000_000);
        scd.start_measurement().unwrap();
        assert_eq!(scd.read_measurement(), Err(Error::Crc));
        scd.start_measurement().unwrap();
        assert_eq!(scd.read_measurement(), Err(Error::InvalidData));
        i2c.done();
    }

    #[test]
    fn async_measure() {
        use zigbee_sensor::asynch::Sensor as _;

        let expectations = [
            Transaction::write(ADDRESS, vec![0x00, 0x10, 0x00, 0x00, 0x81]),
            Transaction::write(ADDRESS, vec![0x03, 0x00]),
            Transaction::read(ADDRESS, FRAME.to_vec()),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut delay = CheckedDelay::new(&[D::async_delay_us(3_000)]);
        let mut scd = asynch::Scd30::new(&mut i2c);
        block_on(async {
            assert_eq!(
                scd.start_continuous_measurement(0).await.unwrap(),
                2_000_000
            );
            let m = scd.measure(&mut delay).await.unwrap();
            assert_eq!(m.co2, 439);
        });
        delay.done();
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
