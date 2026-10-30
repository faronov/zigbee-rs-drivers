//! Sensirion SCD40 / SCD41 / SCD43 photoacoustic CO2 sensor with on-board
//! temperature and humidity (I2C, 0x62).
//!
//! The driver covers the full command set of the datasheet: periodic,
//! low-power periodic and single-shot measurement (SCD41/SCD43 only),
//! data-ready status, temperature offset, altitude and ambient pressure
//! compensation, automatic self-calibration (enable, target, initial and
//! standard period), forced recalibration, persist settings, serial number,
//! self test, factory reset, re-init, sensor variant and power down /
//! wake up ([`zigbee_sensor::blocking::LowPower`]).
//!
//! # Measurement through the [`Sensor`](zigbee_sensor::blocking::Sensor) trait
//!
//! The mode is chosen with [`Scd4x::with_mode`], or automatically: after
//! [`Scd4x::with_variant`] or a successful `probe`, SCD41/SCD43 use
//! single-shot mode (0.45 mA average at one sample per 5 min instead of
//! 15 mA in periodic mode) unless a mode was set explicitly.
//!
//! * **Single shot** (SCD41/SCD43): `start_measurement` sends
//!   `measure_single_shot` and returns 5 s (50 ms for
//!   [`Mode::SingleShotRhtOnly`]). `read_measurement` sends
//!   `read_measurement` and fetches the result by I2C acknowledge polling
//!   (the 1 ms execution time is bridged by the sensor NACKing its read
//!   header, bounded by [`POLL_ATTEMPTS`]); no delay is used.
//! * **Periodic / low-power periodic**: the sensor measures every 5 s / 30 s
//!   by itself. The first `start_measurement` starts periodic measurement
//!   and returns the interval; the following `read_measurement` returns
//!   [`Error::NotReady`] (nothing was requested yet). Every later
//!   `start_measurement` sends `read_measurement` (1 ms) and the next
//!   `read_measurement` returns the sample, or [`Error::NotReady`] if the
//!   sensor NACKs because no new sample is buffered. Calling
//!   `start_periodic_measurement` during initialisation avoids the empty
//!   first cycle.
//!
//! While periodic measurement runs, only `read_measurement`,
//! `get_data_ready_status`, `stop_periodic_measurement`,
//! `set_ambient_pressure` and `get_ambient_pressure` are accepted by the
//! sensor (datasheet table 9); stop it (500 ms) to change settings.
//!
//! Getters that return data take a `delay` for the command execution time
//! (1 ms); they are meant for initialisation and maintenance. The long
//! commands (forced recalibration 400 ms, self test 10 s) are split into
//! `start_*` returning the wait and `read_*` fetching the result.
//!
//! Reference: Sensirion "Datasheet SCD4x", version 1.7 (April 2025).
//!
//! ```ignore
//! let mut scd = Scd4x::new(i2c);
//! let variant = scd.probe(&mut delay)?;     // SCD41 → single shot
//! let wait = scd.start_measurement()?;      // 5 s
//! // ... sleep ...
//! let m = scd.read_measurement()?;
//! ```

#![no_std]

use zigbee_sensirion_common::{humidity_from_ticks, is_nack, temperature_from_ticks};
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Fixed I2C address.
pub const ADDRESS: u8 = 0x62;

mod cmd {
    pub const START_PERIODIC: u16 = 0x21B1;
    pub const READ_MEASUREMENT: u16 = 0xEC05;
    pub const STOP_PERIODIC: u16 = 0x3F86;
    pub const SET_TEMPERATURE_OFFSET: u16 = 0x241D;
    pub const GET_TEMPERATURE_OFFSET: u16 = 0x2318;
    pub const SET_SENSOR_ALTITUDE: u16 = 0x2427;
    pub const GET_SENSOR_ALTITUDE: u16 = 0x2322;
    pub const AMBIENT_PRESSURE: u16 = 0xE000;
    pub const PERFORM_FORCED_RECALIBRATION: u16 = 0x362F;
    pub const SET_ASC_ENABLED: u16 = 0x2416;
    pub const GET_ASC_ENABLED: u16 = 0x2313;
    pub const SET_ASC_TARGET: u16 = 0x243A;
    pub const GET_ASC_TARGET: u16 = 0x233F;
    pub const START_LOW_POWER_PERIODIC: u16 = 0x21AC;
    pub const GET_DATA_READY_STATUS: u16 = 0xE4B8;
    pub const PERSIST_SETTINGS: u16 = 0x3615;
    pub const GET_SERIAL_NUMBER: u16 = 0x3682;
    pub const PERFORM_SELF_TEST: u16 = 0x3639;
    pub const PERFORM_FACTORY_RESET: u16 = 0x3632;
    pub const REINIT: u16 = 0x3646;
    pub const GET_SENSOR_VARIANT: u16 = 0x202F;
    pub const MEASURE_SINGLE_SHOT: u16 = 0x219D;
    pub const MEASURE_SINGLE_SHOT_RHT_ONLY: u16 = 0x2196;
    pub const POWER_DOWN: u16 = 0x36E0;
    pub const WAKE_UP: u16 = 0x36F6;
    pub const SET_ASC_INITIAL_PERIOD: u16 = 0x2445;
    pub const GET_ASC_INITIAL_PERIOD: u16 = 0x2340;
    pub const SET_ASC_STANDARD_PERIOD: u16 = 0x244E;
    pub const GET_ASC_STANDARD_PERIOD: u16 = 0x234B;
}

/// Execution time of most commands (1 ms).
pub const COMMAND_WAIT_US: WaitUs = 1_000;
/// Power-up, `wake_up` and `reinit` time (30 ms).
pub const WAKE_WAIT_US: WaitUs = 30_000;
/// Measurement interval in periodic mode.
pub const PERIODIC_INTERVAL_US: WaitUs = 5_000_000;
/// Measurement interval in low-power periodic mode.
pub const LOW_POWER_INTERVAL_US: WaitUs = 30_000_000;
/// `measure_single_shot` duration.
pub const SINGLE_SHOT_WAIT_US: WaitUs = 5_000_000;
/// `measure_single_shot_rht_only` duration.
pub const SINGLE_SHOT_RHT_WAIT_US: WaitUs = 50_000;
/// `stop_periodic_measurement` duration.
pub const STOP_WAIT_US: WaitUs = 500_000;
/// `perform_forced_recalibration` duration.
pub const FRC_WAIT_US: WaitUs = 400_000;
/// `persist_settings` duration.
pub const PERSIST_WAIT_US: WaitUs = 800_000;
/// `perform_self_test` duration.
pub const SELF_TEST_WAIT_US: WaitUs = 10_000_000;
/// `perform_factory_reset` duration.
pub const FACTORY_RESET_WAIT_US: WaitUs = 1_200_000;
/// Read attempts while acknowledge-polling the 1 ms `read_measurement`
/// execution in single-shot mode (one NACKed address byte is ~90 µs at
/// 100 kHz, so 100 attempts span ≥ 9 ms at up to 400 kHz ≥ 2.5 ms).
pub const POLL_ATTEMPTS: u16 = 100;

/// SCD4x part.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Variant {
    /// SCD40 (periodic modes only).
    #[default]
    Scd40,
    /// SCD41.
    Scd41,
    /// SCD43.
    Scd43,
}

impl Variant {
    /// Decode `get_sensor_variant` (bits 15..12, datasheet section 3.10.6).
    pub const fn from_word(word: u16) -> Option<Self> {
        match word >> 12 {
            0b0000 => Some(Self::Scd40),
            0b0001 => Some(Self::Scd41),
            0b0101 => Some(Self::Scd43),
            _ => None,
        }
    }

    /// `true` for the parts with single-shot commands (SCD41, SCD43).
    pub const fn has_single_shot(self) -> bool {
        !matches!(self, Self::Scd40)
    }
}

/// How the [`Sensor`](zigbee_sensor::blocking::Sensor) trait measures.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Mode {
    /// Periodic measurement, one sample every 5 s (15 mA average at 3.3 V).
    #[default]
    Periodic,
    /// Low-power periodic measurement, one sample every 30 s (3.2 mA).
    LowPowerPeriodic,
    /// On-demand CO2 + RH/T, 5 s (SCD41/SCD43).
    SingleShot,
    /// On-demand RH/T only, 50 ms; CO2 is not reported (SCD41/SCD43).
    SingleShotRhtOnly,
}

/// One sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// CO2 in ppm; `None` after a RH/T-only single shot.
    pub co2: Option<u16>,
    /// Temperature in 0.01 °C.
    pub temperature: i32,
    /// Relative humidity in 0.01 %RH.
    pub humidity: u16,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        if let Some(co2) = self.co2 {
            f(Quantity::Co2(u32::from(co2)));
        }
        f(Quantity::Temperature(self.temperature));
        f(Quantity::Humidity(self.humidity));
    }
}

/// Convert the three `read_measurement` words (section 3.6.2): CO2 in ppm,
/// `T = -45 + 175 × S / (2^16 - 1)`, `RH = 100 × S / (2^16 - 1)`.
pub fn convert(words: [u16; 3], with_co2: bool) -> Measurement {
    Measurement {
        co2: with_co2.then_some(words[0]),
        temperature: temperature_from_ticks(words[1]),
        humidity: humidity_from_ticks(words[2]),
    }
}

/// Temperature offset in 0.01 °C to the `set_temperature_offset` word
/// (`T × (2^16 - 1) / 175`), clamped to 175 °C.
pub fn offset_to_word(centi_celsius: u16) -> u16 {
    let t = u32::from(centi_celsius.min(17_500));
    ((t * 65_535 + 8_750) / 17_500) as u16
}

/// `get_temperature_offset` word to 0.01 °C.
pub fn offset_from_word(word: u16) -> u16 {
    ((u32::from(word) * 17_500 + 32_767) / 65_535) as u16
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Pending {
    Nothing,
    Read,
    SingleShot { co2: bool },
}

macro_rules! constructors {
    () => {
        /// Create an SCD40 driver in periodic mode; no bus traffic.
        pub const fn new(i2c: I2C) -> Self {
            Self {
                bus: Interface::new(i2c, ADDRESS),
                variant: Variant::Scd40,
                mode: Mode::Periodic,
                mode_set: false,
                running: false,
                pending: Pending::Nothing,
            }
        }

        /// Set the part without probing; selects single-shot mode on
        /// SCD41/SCD43 unless [`Self::with_mode`] was used.
        pub const fn with_variant(mut self, variant: Variant) -> Self {
            self.set_variant(variant);
            self
        }

        /// Choose the measurement mode used by the `Sensor` trait.
        pub const fn with_mode(mut self, mode: Mode) -> Self {
            self.mode = mode;
            self.mode_set = true;
            self
        }

        /// Part in use.
        pub const fn variant(&self) -> Variant {
            self.variant
        }

        /// Measurement mode in use.
        pub const fn mode(&self) -> Mode {
            self.mode
        }

        /// `true` while the driver has (low-power) periodic measurement
        /// running.
        pub const fn is_running(&self) -> bool {
            self.running
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.bus.release()
        }

        const fn set_variant(&mut self, variant: Variant) {
            self.variant = variant;
            if !self.mode_set {
                self.mode = if variant.has_single_shot() {
                    Mode::SingleShot
                } else {
                    Mode::Periodic
                };
            }
        }
    };
}

// Bus methods shared by the blocking and async drivers: `$async` is empty or
// `async`, `$aw` is empty or `.await`.
macro_rules! methods {
    ([$($async:tt)*] [$($aw:tt)*] $Delay:path) => {
        $($async)* fn send(
            &mut self,
            cmd: u16,
            args: &[u16],
            wait: WaitUs,
        ) -> Result<WaitUs, Error<I2C::Error>> {
            self.bus.write(cmd, args) $($aw)* ?;
            Ok(wait)
        }

        $($async)* fn get<D: $Delay>(
            &mut self,
            cmd: u16,
            delay: &mut D,
        ) -> Result<u16, Error<I2C::Error>> {
            let mut word = [0u16];
            self.bus.fetch(cmd, &[], COMMAND_WAIT_US, delay, &mut word) $($aw)* ?;
            Ok(word[0])
        }

        /// Read `get_sensor_variant` and remember the part (see
        /// [`Self::with_variant`]). Sensor must be idle.
        pub $($async)* fn probe<D: $Delay>(
            &mut self,
            delay: &mut D,
        ) -> Result<Variant, Error<I2C::Error>> {
            let word = self.get(cmd::GET_SENSOR_VARIANT, delay) $($aw)* ?;
            let variant = Variant::from_word(word).ok_or(Error::UnexpectedId(u32::from(word)))?;
            self.set_variant(variant);
            Ok(variant)
        }

        /// Start periodic measurement; first sample after the returned 5 s.
        pub $($async)* fn start_periodic_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::START_PERIODIC, &[], PERIODIC_INTERVAL_US) $($aw)* ?;
            self.running = true;
            self.pending = Pending::Nothing;
            Ok(PERIODIC_INTERVAL_US)
        }

        /// Start low-power periodic measurement (one sample every 30 s).
        pub $($async)* fn start_low_power_periodic_measurement(
            &mut self,
        ) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::START_LOW_POWER_PERIODIC, &[], LOW_POWER_INTERVAL_US) $($aw)* ?;
            self.running = true;
            self.pending = Pending::Nothing;
            Ok(LOW_POWER_INTERVAL_US)
        }

        /// Stop (low-power) periodic measurement; wait 500 ms before the next
        /// command.
        pub $($async)* fn stop_periodic_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::STOP_PERIODIC, &[], STOP_WAIT_US) $($aw)* ?;
            self.running = false;
            self.pending = Pending::Nothing;
            Ok(STOP_WAIT_US)
        }

        /// Start a CO2 + RH/T single shot (SCD41/SCD43); fetch it with
        /// `read_measurement` after the returned 5 s.
        pub $($async)* fn measure_single_shot(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.single_shot(true) $($aw)*
        }

        /// Start a RH/T-only single shot (SCD41/SCD43); 50 ms.
        pub $($async)* fn measure_single_shot_rht_only(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.single_shot(false) $($aw)*
        }

        $($async)* fn single_shot(&mut self, co2: bool) -> Result<WaitUs, Error<I2C::Error>> {
            if !self.variant.has_single_shot() {
                return Err(Error::Unsupported);
            }
            let (command, wait) = if co2 {
                (cmd::MEASURE_SINGLE_SHOT, SINGLE_SHOT_WAIT_US)
            } else {
                (cmd::MEASURE_SINGLE_SHOT_RHT_ONLY, SINGLE_SHOT_RHT_WAIT_US)
            };
            self.send(command, &[], wait) $($aw)* ?;
            self.pending = Pending::SingleShot { co2 };
            Ok(wait)
        }

        /// `get_data_ready_status`: `true` if a sample is buffered (lower
        /// 11 bits non-zero).
        pub $($async)* fn get_data_ready_status<D: $Delay>(
            &mut self,
            delay: &mut D,
        ) -> Result<bool, Error<I2C::Error>> {
            let word = self.get(cmd::GET_DATA_READY_STATUS, delay) $($aw)* ?;
            Ok(word & 0x07FF != 0)
        }

        /// Set the temperature offset in 0.01 °C (default 4 °C, recommended
        /// 0–20 °C). Volatile until `persist_settings`.
        pub $($async)* fn set_temperature_offset(
            &mut self,
            centi_celsius: u16,
        ) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::SET_TEMPERATURE_OFFSET, &[offset_to_word(centi_celsius)], COMMAND_WAIT_US) $($aw)*
        }

        /// Temperature offset in 0.01 °C.
        pub $($async)* fn get_temperature_offset<D: $Delay>(
            &mut self,
            delay: &mut D,
        ) -> Result<u16, Error<I2C::Error>> {
            Ok(offset_from_word(self.get(cmd::GET_TEMPERATURE_OFFSET, delay) $($aw)* ?))
        }

        /// Set the altitude in metres above sea level (0–3000, default 0).
        pub $($async)* fn set_sensor_altitude(&mut self, metres: u16) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::SET_SENSOR_ALTITUDE, &[metres], COMMAND_WAIT_US) $($aw)*
        }

        /// Altitude in metres.
        pub $($async)* fn get_sensor_altitude<D: $Delay>(
            &mut self,
            delay: &mut D,
        ) -> Result<u16, Error<I2C::Error>> {
            self.get(cmd::GET_SENSOR_ALTITUDE, delay) $($aw)*
        }

        /// Set the ambient pressure in Pa (70 000–120 000, default 101 300;
        /// overrides the altitude). Allowed during periodic measurement.
        /// Out-of-range values give [`Error::InvalidData`] without bus
        /// traffic.
        pub $($async)* fn set_ambient_pressure(&mut self, pascal: u32) -> Result<WaitUs, Error<I2C::Error>> {
            if !(70_000..=120_000).contains(&pascal) {
                return Err(Error::InvalidData);
            }
            let word = ((pascal + 50) / 100) as u16;
            self.send(cmd::AMBIENT_PRESSURE, &[word], COMMAND_WAIT_US) $($aw)*
        }

        /// Ambient pressure in Pa.
        pub $($async)* fn get_ambient_pressure<D: $Delay>(
            &mut self,
            delay: &mut D,
        ) -> Result<u32, Error<I2C::Error>> {
            Ok(u32::from(self.get(cmd::AMBIENT_PRESSURE, delay) $($aw)* ?) * 100)
        }

        /// Send `perform_forced_recalibration` with the reference CO2 in ppm
        /// (sensor idle, operated ≥ 3 min before, stopped ≥ 500 ms). Fetch
        /// the result with `read_forced_recalibration` after the returned
        /// 400 ms.
        pub $($async)* fn start_forced_recalibration(
            &mut self,
            target_ppm: u16,
        ) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::PERFORM_FORCED_RECALIBRATION, &[target_ppm], FRC_WAIT_US) $($aw)*
        }

        /// Applied FRC correction in ppm; [`Error::InvalidData`] if the
        /// sensor reports a failed FRC (0xFFFF).
        pub $($async)* fn read_forced_recalibration(&mut self) -> Result<i16, Error<I2C::Error>> {
            let mut word = [0u16];
            self.bus.read(&mut word) $($aw)* ?;
            if word[0] == 0xFFFF {
                return Err(Error::InvalidData);
            }
            Ok((i32::from(word[0]) - 0x8000) as i16)
        }

        /// Enable or disable automatic self-calibration (default enabled).
        pub $($async)* fn set_automatic_self_calibration_enabled(
            &mut self,
            enabled: bool,
        ) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::SET_ASC_ENABLED, &[u16::from(enabled)], COMMAND_WAIT_US) $($aw)*
        }

        /// `true` if automatic self-calibration is enabled.
        pub $($async)* fn get_automatic_self_calibration_enabled<D: $Delay>(
            &mut self,
            delay: &mut D,
        ) -> Result<bool, Error<I2C::Error>> {
            Ok(self.get(cmd::GET_ASC_ENABLED, delay) $($aw)* ? != 0)
        }

        /// Set the ASC baseline target in ppm (default 400).
        pub $($async)* fn set_automatic_self_calibration_target(
            &mut self,
            ppm: u16,
        ) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::SET_ASC_TARGET, &[ppm], COMMAND_WAIT_US) $($aw)*
        }

        /// ASC baseline target in ppm.
        pub $($async)* fn get_automatic_self_calibration_target<D: $Delay>(
            &mut self,
            delay: &mut D,
        ) -> Result<u16, Error<I2C::Error>> {
            self.get(cmd::GET_ASC_TARGET, delay) $($aw)*
        }

        /// Set the ASC initial period in hours (multiple of 4, default 44;
        /// single shot counts one sample as 5 min). Other values give
        /// [`Error::InvalidData`].
        pub $($async)* fn set_automatic_self_calibration_initial_period(
            &mut self,
            hours: u16,
        ) -> Result<WaitUs, Error<I2C::Error>> {
            if hours % 4 != 0 {
                return Err(Error::InvalidData);
            }
            self.send(cmd::SET_ASC_INITIAL_PERIOD, &[hours], COMMAND_WAIT_US) $($aw)*
        }

        /// ASC initial period in hours.
        pub $($async)* fn get_automatic_self_calibration_initial_period<D: $Delay>(
            &mut self,
            delay: &mut D,
        ) -> Result<u16, Error<I2C::Error>> {
            self.get(cmd::GET_ASC_INITIAL_PERIOD, delay) $($aw)*
        }

        /// Set the ASC standard period in hours (multiple of 4, default
        /// 156). Other values give [`Error::InvalidData`].
        pub $($async)* fn set_automatic_self_calibration_standard_period(
            &mut self,
            hours: u16,
        ) -> Result<WaitUs, Error<I2C::Error>> {
            if hours % 4 != 0 {
                return Err(Error::InvalidData);
            }
            self.send(cmd::SET_ASC_STANDARD_PERIOD, &[hours], COMMAND_WAIT_US) $($aw)*
        }

        /// ASC standard period in hours.
        pub $($async)* fn get_automatic_self_calibration_standard_period<D: $Delay>(
            &mut self,
            delay: &mut D,
        ) -> Result<u16, Error<I2C::Error>> {
            self.get(cmd::GET_ASC_STANDARD_PERIOD, delay) $($aw)*
        }

        /// Store offset, altitude, ASC and FRC settings in EEPROM (800 ms,
        /// limited write endurance: only call when settings changed).
        pub $($async)* fn persist_settings(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::PERSIST_SETTINGS, &[], PERSIST_WAIT_US) $($aw)*
        }

        /// 48-bit serial number. Also verifies the sensor is idle after
        /// `wake`.
        pub $($async)* fn serial_number<D: $Delay>(
            &mut self,
            delay: &mut D,
        ) -> Result<u64, Error<I2C::Error>> {
            let mut w = [0u16; 3];
            self.bus.fetch(cmd::GET_SERIAL_NUMBER, &[], COMMAND_WAIT_US, delay, &mut w) $($aw)* ?;
            Ok(u64::from(w[0]) << 32 | u64::from(w[1]) << 16 | u64::from(w[2]))
        }

        /// Start the end-of-line self test; fetch the result with
        /// `read_self_test` after the returned 10 s.
        pub $($async)* fn start_self_test(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::PERFORM_SELF_TEST, &[], SELF_TEST_WAIT_US) $($aw)*
        }

        /// `true` if the self test found no malfunction.
        pub $($async)* fn read_self_test(&mut self) -> Result<bool, Error<I2C::Error>> {
            let mut word = [0u16];
            self.bus.read(&mut word) $($aw)* ?;
            Ok(word[0] == 0)
        }

        /// Reset all settings in EEPROM and the FRC/ASC history (1.2 s).
        pub $($async)* fn perform_factory_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::PERFORM_FACTORY_RESET, &[], FACTORY_RESET_WAIT_US) $($aw)*
        }

        /// Reload user settings from EEPROM (sensor idle; 30 ms).
        pub $($async)* fn reinit(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::REINIT, &[], WAKE_WAIT_US) $($aw)*
        }

        $($async)* fn start(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            match self.mode {
                Mode::SingleShot => self.single_shot(true) $($aw)*,
                Mode::SingleShotRhtOnly => self.single_shot(false) $($aw)*,
                Mode::Periodic | Mode::LowPowerPeriodic if !self.running => {
                    if self.mode == Mode::Periodic {
                        self.start_periodic_measurement() $($aw)*
                    } else {
                        self.start_low_power_periodic_measurement() $($aw)*
                    }
                }
                Mode::Periodic | Mode::LowPowerPeriodic => {
                    self.send(cmd::READ_MEASUREMENT, &[], COMMAND_WAIT_US) $($aw)* ?;
                    self.pending = Pending::Read;
                    Ok(COMMAND_WAIT_US)
                }
            }
        }

        $($async)* fn read(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let mut w = [0u16; 3];
            match self.pending {
                Pending::Nothing => return Err(Error::NotReady),
                Pending::Read => {
                    self.pending = Pending::Nothing;
                    self.bus.read(&mut w) $($aw)* .map_err(nack_as_not_ready)?;
                    Ok(convert(w, true))
                }
                Pending::SingleShot { co2 } => {
                    // A NACK here means the single shot is still running.
                    self.bus.write(cmd::READ_MEASUREMENT, &[]) $($aw)* .map_err(nack_as_not_ready)?;
                    self.pending = Pending::Nothing;
                    match self.bus.read_polling(&mut w, POLL_ATTEMPTS) $($aw)* {
                        Err(Error::Timeout) => return Err(Error::NotReady),
                        other => other?,
                    }
                    Ok(convert(w, co2))
                }
            }
        }

        $($async)* fn power_down(&mut self) -> Result<(), Error<I2C::Error>> {
            self.bus.write(cmd::POWER_DOWN, &[]) $($aw)* ?;
            self.running = false;
            self.pending = Pending::Nothing;
            Ok(())
        }

        $($async)* fn wake_up(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            // The SCD4x does not acknowledge wake_up (section 3.11.4).
            match self.bus.write(cmd::WAKE_UP, &[]) $($aw)* {
                Err(Error::Bus(e)) if !is_nack(&e) => Err(Error::Bus(e)),
                _ => Ok(WAKE_WAIT_US),
            }
        }
    };
}

// Trait impls shared by the blocking and async drivers.
macro_rules! trait_impls {
    ($Sensor:path, $LowPower:path, $I2c:path, [$($async:tt)*] [$($aw:tt)*]) => {
        impl<I2C: $I2c> $Sensor for Scd4x<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// See the crate documentation for the per-mode behaviour.
            $($async)* fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.start() $($aw)*
            }

            /// [`Error::NotReady`] if no sample was requested or none is
            /// available yet.
            $($async)* fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                self.read() $($aw)*
            }
        }

        impl<I2C: $I2c> $LowPower for Scd4x<I2C> {
            type BusError = I2C::Error;

            /// `power_down` (sensor must be idle: stop periodic measurement
            /// and wait 500 ms first). Settings not persisted are lost.
            $($async)* fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.power_down() $($aw)*
            }

            /// `wake_up` (not acknowledged by the sensor); 30 ms.
            $($async)* fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.wake_up() $($aw)*
            }
        }
    };
}

use zigbee_sensirion_common::blocking::Interface;
use zigbee_sensirion_common::nack_as_not_ready;

/// Blocking driver.
#[derive(Debug)]
pub struct Scd4x<I2C> {
    bus: Interface<I2C>,
    variant: Variant,
    mode: Mode,
    mode_set: bool,
    running: bool,
    pending: Pending,
}

impl<I2C> Scd4x<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Scd4x<I2C> {
    methods!([] [] embedded_hal::delay::DelayNs);
}

trait_impls!(
    zigbee_sensor::blocking::Sensor,
    zigbee_sensor::blocking::LowPower,
    embedded_hal::i2c::I2c,
    [] []
);

/// Async driver.
pub mod asynch {
    use super::{
        ADDRESS, COMMAND_WAIT_US, FACTORY_RESET_WAIT_US, FRC_WAIT_US, LOW_POWER_INTERVAL_US,
        Measurement, Mode, PERIODIC_INTERVAL_US, PERSIST_WAIT_US, POLL_ATTEMPTS, Pending,
        SELF_TEST_WAIT_US, SINGLE_SHOT_RHT_WAIT_US, SINGLE_SHOT_WAIT_US, STOP_WAIT_US, Variant,
        WAKE_WAIT_US, cmd, convert, offset_from_word, offset_to_word,
    };
    use zigbee_sensirion_common::asynch::Interface;
    use zigbee_sensirion_common::{is_nack, nack_as_not_ready};
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Scd4x`].
    #[derive(Debug)]
    pub struct Scd4x<I2C> {
        bus: Interface<I2C>,
        variant: Variant,
        mode: Mode,
        mode_set: bool,
        running: bool,
        pending: Pending,
    }

    impl<I2C> Scd4x<I2C> {
        constructors!();
    }

    impl<I2C: embedded_hal_async::i2c::I2c> Scd4x<I2C> {
        methods!([async] [.await] embedded_hal_async::delay::DelayNs);
    }

    trait_impls!(
        zigbee_sensor::asynch::Sensor,
        zigbee_sensor::asynch::LowPower,
        embedded_hal_async::i2c::I2c,
        [async] [.await]
    );
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};
    use embedded_hal_mock::eh1::delay::{CheckedDelay, Transaction as D};
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use std::vec;
    use std::vec::Vec;
    use zigbee_sensirion_common::encode_word;
    use zigbee_sensor::blocking::{LowPower, Sensor};

    fn words(w: &[u16]) -> Vec<u8> {
        w.iter().flat_map(|&x| encode_word(x)).collect()
    }

    fn cmd_bytes(cmd: u16, args: &[u16]) -> Vec<u8> {
        let mut v = cmd.to_be_bytes().to_vec();
        v.extend(words(args));
        v
    }

    fn nack(t: Transaction) -> Transaction {
        t.with_error(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address))
    }

    // Datasheet section 3.6.2 example: 500 ppm, 25 °C, 37 %RH.
    const SAMPLE: [u16; 3] = [0x01F4, 0x6667, 0x5EB9];

    #[test]
    fn conversion_matches_datasheet_example() {
        let m = convert(SAMPLE, true);
        assert_eq!(
            m,
            Measurement {
                co2: Some(500),
                temperature: 2_500,
                humidity: 3_700
            }
        );
        assert_eq!(offset_to_word(540), 0x07E6); // 5.4 °C, section 3.7.1
        assert_eq!(offset_from_word(0x07E6), 540);
        assert_eq!(Variant::from_word(0x1440), Some(Variant::Scd41));
        assert_eq!(Variant::from_word(0x0440), Some(Variant::Scd40));
        assert_eq!(Variant::from_word(0x5441), Some(Variant::Scd43));
        assert_eq!(Variant::from_word(0x3000), None);
    }

    #[test]
    fn periodic_cycle() {
        let expectations = [
            Transaction::write(ADDRESS, vec![0x21, 0xB1]),
            Transaction::write(ADDRESS, vec![0xEC, 0x05]),
            Transaction::read(ADDRESS, words(&SAMPLE)),
            Transaction::write(ADDRESS, vec![0xEC, 0x05]),
            nack(Transaction::read(ADDRESS, vec![0; 9])),
            Transaction::write(ADDRESS, vec![0x3F, 0x86]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut scd = Scd4x::new(&mut i2c);
        assert_eq!(scd.start_measurement().unwrap(), 5_000_000);
        assert_eq!(scd.read_measurement(), Err(Error::NotReady));
        assert_eq!(scd.start_measurement().unwrap(), 1_000);
        assert_eq!(scd.read_measurement().unwrap().co2, Some(500));
        assert_eq!(scd.start_measurement().unwrap(), 1_000);
        assert_eq!(scd.read_measurement(), Err(Error::NotReady));
        assert_eq!(scd.stop_periodic_measurement().unwrap(), 500_000);
        assert!(!scd.is_running());
        i2c.done();
    }

    #[test]
    fn probe_selects_single_shot_and_polls_result() {
        let expectations = [
            Transaction::write(ADDRESS, vec![0x20, 0x2F]),
            Transaction::read(ADDRESS, vec![0x14, 0x40, 0x51]),
            Transaction::write(ADDRESS, vec![0x21, 0x9D]),
            nack(Transaction::write(ADDRESS, vec![0xEC, 0x05])),
            Transaction::write(ADDRESS, vec![0xEC, 0x05]),
            nack(Transaction::read(ADDRESS, vec![0; 9])),
            Transaction::read(ADDRESS, words(&SAMPLE)),
            Transaction::write(ADDRESS, vec![0x20, 0x2F]),
            Transaction::read(ADDRESS, words(&[0x3000])),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut delay =
            CheckedDelay::new(&[D::blocking_delay_us(1_000), D::blocking_delay_us(1_000)]);
        let mut scd = Scd4x::new(&mut i2c);
        assert_eq!(scd.probe(&mut delay).unwrap(), Variant::Scd41);
        assert_eq!(scd.mode(), Mode::SingleShot);
        assert_eq!(scd.start_measurement().unwrap(), 5_000_000);
        assert_eq!(scd.read_measurement(), Err(Error::NotReady)); // still measuring
        let m = scd.read_measurement().unwrap();
        assert_eq!(m.temperature, 2_500);
        assert_eq!(scd.probe(&mut delay), Err(Error::UnexpectedId(0x3000)));
        delay.done();
        i2c.done();
    }

    #[test]
    fn rht_only_and_scd40_restrictions() {
        let expectations = [
            Transaction::write(ADDRESS, vec![0x21, 0x96]),
            Transaction::write(ADDRESS, vec![0xEC, 0x05]),
            Transaction::read(ADDRESS, words(&[0, 0x6667, 0x5EB9])),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut scd = Scd4x::new(&mut i2c)
            .with_mode(Mode::SingleShotRhtOnly)
            .with_variant(Variant::Scd43);
        assert_eq!(scd.mode(), Mode::SingleShotRhtOnly);
        assert_eq!(scd.start_measurement().unwrap(), 50_000);
        let m = scd.read_measurement().unwrap();
        assert_eq!(m.co2, None);
        let mut n = 0;
        m.for_each(&mut |_| n += 1);
        assert_eq!(n, 2);
        i2c.done();

        let mut i2c = Mock::new(&[]);
        let mut scd = Scd4x::new(&mut i2c).with_mode(Mode::SingleShot);
        assert_eq!(scd.start_measurement(), Err(Error::Unsupported));
        assert_eq!(scd.set_ambient_pressure(60_000), Err(Error::InvalidData));
        assert_eq!(
            scd.set_automatic_self_calibration_initial_period(45),
            Err(Error::InvalidData)
        );
        i2c.done();
    }

    #[test]
    fn configuration_commands() {
        let expectations = [
            Transaction::write(ADDRESS, cmd_bytes(0x241D, &[0x07E6])),
            Transaction::write(ADDRESS, cmd_bytes(0x2318, &[])),
            Transaction::read(ADDRESS, words(&[0x07E6])),
            Transaction::write(ADDRESS, cmd_bytes(0x2427, &[300])),
            Transaction::write(ADDRESS, vec![0xE0, 0x00, 0x03, 0xDB, 0x42]),
            Transaction::write(ADDRESS, cmd_bytes(0xE000, &[])),
            Transaction::read(ADDRESS, words(&[987])),
            Transaction::write(ADDRESS, cmd_bytes(0x2416, &[0])),
            Transaction::write(ADDRESS, cmd_bytes(0x243A, &[420])),
            Transaction::write(ADDRESS, cmd_bytes(0x2445, &[48])),
            Transaction::write(ADDRESS, cmd_bytes(0x244E, &[160])),
            Transaction::write(ADDRESS, vec![0x36, 0x15]),
            Transaction::write(ADDRESS, vec![0x36, 0x2F, 0x01, 0xE0, 0xB4]),
            Transaction::read(ADDRESS, vec![0x7F, 0xCE, 0x7B]),
            Transaction::write(ADDRESS, cmd_bytes(0x362F, &[400])),
            Transaction::read(ADDRESS, words(&[0xFFFF])),
            Transaction::write(ADDRESS, vec![0x36, 0x82]),
            Transaction::read(
                ADDRESS,
                vec![0xF8, 0x96, 0x31, 0x9F, 0x07, 0xC2, 0x3B, 0xBE, 0x89],
            ),
            Transaction::write(ADDRESS, vec![0x36, 0x39]),
            Transaction::read(ADDRESS, words(&[0])),
            Transaction::write(ADDRESS, vec![0x36, 0x32]),
            Transaction::write(ADDRESS, vec![0x36, 0x46]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut delay = CheckedDelay::new(&[
            D::blocking_delay_us(1_000),
            D::blocking_delay_us(1_000),
            D::blocking_delay_us(1_000),
        ]);
        let mut scd = Scd4x::new(&mut i2c);
        assert_eq!(scd.set_temperature_offset(540).unwrap(), 1_000);
        assert_eq!(scd.get_temperature_offset(&mut delay).unwrap(), 540);
        scd.set_sensor_altitude(300).unwrap();
        scd.set_ambient_pressure(98_700).unwrap();
        assert_eq!(scd.get_ambient_pressure(&mut delay).unwrap(), 98_700);
        scd.set_automatic_self_calibration_enabled(false).unwrap();
        scd.set_automatic_self_calibration_target(420).unwrap();
        scd.set_automatic_self_calibration_initial_period(48)
            .unwrap();
        scd.set_automatic_self_calibration_standard_period(160)
            .unwrap();
        assert_eq!(scd.persist_settings().unwrap(), 800_000);
        assert_eq!(scd.start_forced_recalibration(480).unwrap(), 400_000);
        assert_eq!(scd.read_forced_recalibration().unwrap(), -50);
        scd.start_forced_recalibration(400).unwrap();
        assert_eq!(scd.read_forced_recalibration(), Err(Error::InvalidData));
        assert_eq!(scd.serial_number(&mut delay).unwrap(), 273_325_796_834_238);
        assert_eq!(scd.start_self_test().unwrap(), 10_000_000);
        assert!(scd.read_self_test().unwrap());
        assert_eq!(scd.perform_factory_reset().unwrap(), 1_200_000);
        assert_eq!(scd.reinit().unwrap(), 30_000);
        delay.done();
        i2c.done();
    }

    #[test]
    fn power_down_wake_up_ignores_nack_and_crc_error() {
        let mut bad = words(&SAMPLE);
        bad[8] ^= 1;
        let expectations = [
            Transaction::write(ADDRESS, vec![0x36, 0xE0]),
            nack(Transaction::write(ADDRESS, vec![0x36, 0xF6])),
            Transaction::write(ADDRESS, vec![0x21, 0xAC]),
            Transaction::write(ADDRESS, vec![0xEC, 0x05]),
            Transaction::read(ADDRESS, bad),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut scd = Scd4x::new(&mut i2c).with_mode(Mode::LowPowerPeriodic);
        scd.sleep().unwrap();
        assert_eq!(scd.wake().unwrap(), 30_000);
        assert_eq!(scd.start_measurement().unwrap(), 30_000_000);
        scd.start_measurement().unwrap();
        assert_eq!(scd.read_measurement(), Err(Error::Crc));
        i2c.done();
    }

    #[test]
    fn async_single_shot_and_data_ready() {
        use zigbee_sensor::asynch::Sensor as _;

        let expectations = [
            Transaction::write(ADDRESS, vec![0xE4, 0xB8]),
            Transaction::read(ADDRESS, words(&[0x8006])),
            Transaction::write(ADDRESS, vec![0x21, 0x9D]),
            Transaction::write(ADDRESS, vec![0xEC, 0x05]),
            Transaction::read(ADDRESS, words(&SAMPLE)),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut delay =
            CheckedDelay::new(&[D::async_delay_us(1_000), D::async_delay_us(5_000_000)]);
        let mut scd = asynch::Scd4x::new(&mut i2c).with_variant(Variant::Scd41);
        block_on(async {
            assert!(scd.get_data_ready_status(&mut delay).await.unwrap());
            let m = scd.measure(&mut delay).await.unwrap();
            assert_eq!(m.humidity, 3_700);
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
