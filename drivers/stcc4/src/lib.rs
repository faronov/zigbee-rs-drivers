//! Sensirion STCC4 thermal-conductivity CO2 sensor (I2C, 0x64 / 0x65).
//!
//! The STCC4 needs temperature and humidity compensation: either an SHT4x
//! wired to its dedicated controller pins (SDA_C/SCL_C, recommended; the
//! STCC4 then reports that sensor's T/RH), or values written by the host
//! with `set_rht_compensation` (configure the driver with
//! [`Stcc4::with_sht4x`]`(false)`; T/RH are then not reported).
//!
//! Commands: continuous (1 s) and single-shot (500 ms) measurement, RH/T
//! and pressure compensation input, sleep / exit sleep
//! ([`zigbee_sensor::blocking::LowPower`]), conditioning, soft reset
//! (general call), factory reset, self test, testing mode, forced
//! recalibration and product ID / serial number (`probe`).
//!
//! # Measurement through the [`Sensor`](zigbee_sensor::blocking::Sensor) trait
//!
//! * [`Mode::SingleShot`] (default; 100 µA average at one sample per 10 s
//!   with sleep): `start_measurement` sends `measure_single_shot` and
//!   returns 500 ms; `read_measurement` sends `read_measurement` and fetches
//!   the result by I2C acknowledge polling over its 1 ms execution time
//!   (bounded by [`POLL_ATTEMPTS`]). A recommended cycle is wake → start →
//!   read → sleep with a 5–600 s interval.
//! * [`Mode::Continuous`] (950 µA): the first `start_measurement` starts
//!   continuous measurement and returns 1 s, the next read is
//!   [`Error::NotReady`]; afterwards `start_measurement` sends
//!   `read_measurement` (1 ms) and `read_measurement` returns the sample or
//!   [`Error::NotReady`] when the sensor NACKs (no new data).
//!
//! Commands with a result and a long execution time (factory reset 90 ms,
//! self test 360 ms, FRC 90 ms) are split into `start_*` / `read_*`; the
//! sensor does not stretch the clock.
//!
//! Reference: Sensirion "Datasheet STCC4", version 1 (June 2025).

#![no_std]

use zigbee_sensirion_common::{ignore_nack, temperature_from_ticks, ticks_from_temperature};
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address with ADDR = GND.
pub const ADDRESS: u8 = 0x64;
/// I2C address with ADDR = VDD.
pub const ADDRESS_ALT: u8 = 0x65;
/// Product ID returned by `get_product_id`.
pub const PRODUCT_ID: u32 = 0x0901_018A;

mod cmd {
    pub const START_CONTINUOUS: u16 = 0x218B;
    pub const STOP_CONTINUOUS: u16 = 0x3F86;
    pub const READ_MEASUREMENT: u16 = 0xEC05;
    pub const SET_RHT_COMPENSATION: u16 = 0xE000;
    pub const SET_PRESSURE_COMPENSATION: u16 = 0xE016;
    pub const MEASURE_SINGLE_SHOT: u16 = 0x219D;
    pub const ENTER_SLEEP: u16 = 0x3650;
    pub const PERFORM_CONDITIONING: u16 = 0x29BC;
    pub const PERFORM_FACTORY_RESET: u16 = 0x3632;
    pub const PERFORM_SELF_TEST: u16 = 0x278C;
    pub const ENABLE_TESTING_MODE: u16 = 0x3FBC;
    pub const DISABLE_TESTING_MODE: u16 = 0x3F3D;
    pub const PERFORM_FORCED_RECALIBRATION: u16 = 0x362F;
    pub const GET_PRODUCT_ID: u16 = 0x365B;
}

/// Execution time of `read_measurement`, the compensation inputs,
/// `enter_sleep_mode` and `get_product_id`.
pub const COMMAND_WAIT_US: WaitUs = 1_000;
/// Sampling interval in continuous mode.
pub const CONTINUOUS_INTERVAL_US: WaitUs = 1_000_000;
/// `stop_continuous_measurement` duration.
pub const STOP_WAIT_US: WaitUs = 1_200_000;
/// `measure_single_shot` duration.
pub const SINGLE_SHOT_WAIT_US: WaitUs = 500_000;
/// `exit_sleep_mode` duration.
pub const WAKE_WAIT_US: WaitUs = 5_000;
/// `perform_conditioning` duration.
pub const CONDITIONING_WAIT_US: WaitUs = 22_000_000;
/// Soft reset (and power-up) time.
pub const RESET_WAIT_US: WaitUs = 10_000;
/// `perform_factory_reset` duration.
pub const FACTORY_RESET_WAIT_US: WaitUs = 90_000;
/// `perform_forced_recalibration` duration.
pub const FRC_WAIT_US: WaitUs = 90_000;
/// `perform_self_test` duration.
pub const SELF_TEST_WAIT_US: WaitUs = 360_000;
/// Read attempts while acknowledge-polling the 1 ms `read_measurement`
/// execution in single-shot mode (≥ 1 ms even at 1 MHz).
pub const POLL_ATTEMPTS: u16 = 100;

/// Status bit set in the measurement while testing mode is active
/// (datasheet section 3.4.13).
const STATUS_TESTING_MODE: u16 = 0x0040;

/// How the `Sensor` trait measures.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Mode {
    /// On-demand measurement, 500 ms.
    #[default]
    SingleShot,
    /// Continuous measurement every 1 s.
    Continuous,
}

/// One sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// CO2 in ppm (380–32 000; negative outputs clamp to 0).
    pub co2: u16,
    /// Temperature from the attached SHT4x in 0.01 °C (`None` when the
    /// host provides compensation values).
    pub temperature: Option<i32>,
    /// Relative humidity from the attached SHT4x in 0.01 %RH.
    pub humidity: Option<u16>,
    /// Testing mode active (ASC paused).
    pub testing_mode: bool,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Co2(u32::from(self.co2)));
        if let Some(t) = self.temperature {
            f(Quantity::Temperature(t));
        }
        if let Some(rh) = self.humidity {
            f(Quantity::Humidity(rh));
        }
    }
}

/// STCC4 humidity output `RH = 125 × S / (2^16 - 1) - 6` in 0.01 %RH,
/// clamped to 0–100 % (datasheet table 11).
pub fn humidity_from_ticks(ticks: u16) -> u16 {
    let rh = ((12_500 * u32::from(ticks) + 32_767) / 65_535) as i32 - 600;
    rh.clamp(0, 10_000) as u16
}

/// STCC4 humidity input `(RH + 6) × (2^16 - 1) / 125` from 0.01 %RH
/// (0–100 %).
pub fn ticks_from_humidity(centi_percent: u16) -> u16 {
    let rh = u32::from(centi_percent.min(10_000)) + 600;
    ((rh * 65_535 + 6_250) / 12_500) as u16
}

/// Decode the four `read_measurement` words (CO2 as i16, T, RH, status).
pub fn convert(words: [u16; 4], with_rht: bool) -> Measurement {
    Measurement {
        co2: (words[0] as i16).max(0) as u16,
        temperature: with_rht.then(|| temperature_from_ticks(words[1])),
        humidity: with_rht.then(|| humidity_from_ticks(words[2])),
        testing_mode: words[3] & STATUS_TESTING_MODE != 0,
    }
}

/// `true` if a `perform_self_test` result means "pass": 0x0000, or 0x0010
/// (no SHT4x on the controller interface, expected with host
/// compensation).
pub const fn self_test_passed(result: u16) -> bool {
    result & !0x0010 == 0
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Pending {
    Nothing,
    Read,
    SingleShot,
}

macro_rules! constructors {
    () => {
        /// Create a driver at [`ADDRESS`] in single-shot mode with an
        /// SHT4x on the controller interface; no bus traffic.
        pub const fn new(i2c: I2C) -> Self {
            Self::new_with_address(i2c, ADDRESS)
        }

        /// Create a driver at `address` ([`ADDRESS`](crate::ADDRESS) or [`ADDRESS_ALT`](crate::ADDRESS_ALT)).
        pub const fn new_with_address(i2c: I2C, address: u8) -> Self {
            Self {
                bus: Interface::new(i2c, address),
                mode: Mode::SingleShot,
                sht4x: true,
                running: false,
                pending: Pending::Nothing,
            }
        }

        /// Choose the measurement mode used by the `Sensor` trait.
        pub const fn with_mode(mut self, mode: Mode) -> Self {
            self.mode = mode;
            self
        }

        /// `false` if no SHT4x is wired to SDA_C/SCL_C (the host supplies
        /// compensation values; T/RH are not reported).
        pub const fn with_sht4x(mut self, present: bool) -> Self {
            self.sht4x = present;
            self
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
        $($async)* fn send(&mut self, cmd: u16, args: &[u16], wait: WaitUs) -> Result<WaitUs, Error<I2C::Error>> {
            self.bus.write(cmd, args) $($aw)* ?;
            Ok(wait)
        }

        $($async)* fn read_word(&mut self) -> Result<u16, Error<I2C::Error>> {
            let mut word = [0u16];
            self.bus.read(&mut word) $($aw)* ?;
            Ok(word[0])
        }

        /// Read product ID and serial number; returns the 64-bit serial,
        /// [`Error::UnexpectedId`] if the product ID is not [`PRODUCT_ID`].
        /// Sensor must be idle (awake, not measuring).
        pub $($async)* fn probe<D: $Delay>(&mut self, delay: &mut D) -> Result<u64, Error<I2C::Error>> {
            let mut w = [0u16; 6];
            self.bus.fetch(cmd::GET_PRODUCT_ID, &[], COMMAND_WAIT_US, delay, &mut w) $($aw)* ?;
            let id = u32::from(w[0]) << 16 | u32::from(w[1]);
            if id != PRODUCT_ID {
                return Err(Error::UnexpectedId(id));
            }
            Ok(w[2..].iter().fold(0u64, |acc, &x| acc << 16 | u64::from(x)))
        }

        /// Start continuous measurement (1 s interval).
        pub $($async)* fn start_continuous_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::START_CONTINUOUS, &[], CONTINUOUS_INTERVAL_US) $($aw)* ?;
            self.running = true;
            self.pending = Pending::Nothing;
            Ok(CONTINUOUS_INTERVAL_US)
        }

        /// Stop continuous measurement (1.2 s, sensor unresponsive meanwhile).
        pub $($async)* fn stop_continuous_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::STOP_CONTINUOUS, &[], STOP_WAIT_US) $($aw)* ?;
            self.running = false;
            self.pending = Pending::Nothing;
            Ok(STOP_WAIT_US)
        }

        /// Start a single shot (sensor idle); read it after 500 ms.
        pub $($async)* fn measure_single_shot(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::MEASURE_SINGLE_SHOT, &[], SINGLE_SHOT_WAIT_US) $($aw)* ?;
            self.pending = Pending::SingleShot;
            Ok(SINGLE_SHOT_WAIT_US)
        }

        /// Write host T (0.01 °C) and RH (0.01 %RH) for compensation; used
        /// from the next interval, reset to 25 °C / 50 %RH on power-up. Do
        /// not use while an SHT4x is attached to the controller interface.
        pub $($async)* fn set_rht_compensation(
            &mut self,
            centi_celsius: i32,
            centi_percent: u16,
        ) -> Result<WaitUs, Error<I2C::Error>> {
            let args = [ticks_from_temperature(centi_celsius), ticks_from_humidity(centi_percent)];
            self.send(cmd::SET_RHT_COMPENSATION, &args, COMMAND_WAIT_US) $($aw)*
        }

        /// Write the ambient pressure in Pa (clipped to 40 000–110 000;
        /// default 101 300).
        pub $($async)* fn set_pressure_compensation(&mut self, pascal: u32) -> Result<WaitUs, Error<I2C::Error>> {
            let word = (pascal.clamp(40_000, 110_000) / 2) as u16;
            self.send(cmd::SET_PRESSURE_COMPENSATION, &[word], COMMAND_WAIT_US) $($aw)*
        }

        /// Condition the sensor (22 s) after > 3 h without measurements;
        /// sensor idle.
        pub $($async)* fn perform_conditioning(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::PERFORM_CONDITIONING, &[], CONDITIONING_WAIT_US) $($aw)*
        }

        /// Soft reset through the I2C general call (also resets other
        /// devices on the bus that implement it); 10 ms.
        pub $($async)* fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.bus.general_call_reset() $($aw)* ?;
            self.running = false;
            self.pending = Pending::Nothing;
            Ok(RESET_WAIT_US)
        }

        /// Start a factory reset (FRC and ASC history, bypass phase);
        /// check it with `read_factory_reset` after 90 ms.
        pub $($async)* fn start_factory_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::PERFORM_FACTORY_RESET, &[], FACTORY_RESET_WAIT_US) $($aw)*
        }

        /// [`Error::InvalidData`] if the factory reset failed (0xFFFF).
        pub $($async)* fn read_factory_reset(&mut self) -> Result<(), Error<I2C::Error>> {
            match self.read_word() $($aw)* ? {
                0xFFFF => Err(Error::InvalidData),
                _ => Ok(()),
            }
        }

        /// Start the self test; fetch the result after 360 ms.
        pub $($async)* fn start_self_test(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::PERFORM_SELF_TEST, &[], SELF_TEST_WAIT_US) $($aw)*
        }

        /// Raw self-test word (bit 0 supply, bit 4 no SHT4x, bits 6:5
        /// memory); see [`self_test_passed`](crate::self_test_passed).
        pub $($async)* fn read_self_test(&mut self) -> Result<u16, Error<I2C::Error>> {
            self.read_word() $($aw)*
        }

        /// Pause ASC updates (testing mode).
        pub $($async)* fn enable_testing_mode(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::ENABLE_TESTING_MODE, &[], 0) $($aw)*
        }

        /// Leave testing mode.
        pub $($async)* fn disable_testing_mode(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::DISABLE_TESTING_MODE, &[], 0) $($aw)*
        }

        /// Start a forced recalibration to `target_ppm` (0–32 000; sensor
        /// idle, operated ≥ 30 s before). Fetch the correction after 90 ms.
        pub $($async)* fn start_forced_recalibration(
            &mut self,
            target_ppm: u16,
        ) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::PERFORM_FORCED_RECALIBRATION, &[target_ppm], FRC_WAIT_US) $($aw)*
        }

        /// Applied FRC correction in ppm; [`Error::InvalidData`] on failure
        /// (0xFFFF).
        pub $($async)* fn read_forced_recalibration(&mut self) -> Result<i16, Error<I2C::Error>> {
            match self.read_word() $($aw)* ? {
                0xFFFF => Err(Error::InvalidData),
                word => Ok((i32::from(word) - 32_768) as i16),
            }
        }

        $($async)* fn start(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            match self.mode {
                Mode::SingleShot => self.measure_single_shot() $($aw)*,
                Mode::Continuous if !self.running => self.start_continuous_measurement() $($aw)*,
                Mode::Continuous => {
                    self.send(cmd::READ_MEASUREMENT, &[], COMMAND_WAIT_US) $($aw)* ?;
                    self.pending = Pending::Read;
                    Ok(COMMAND_WAIT_US)
                }
            }
        }

        $($async)* fn read(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let mut w = [0u16; 4];
            match self.pending {
                Pending::Nothing => return Err(Error::NotReady),
                Pending::Read => {
                    self.pending = Pending::Nothing;
                    self.bus.read(&mut w) $($aw)* .map_err(nack_as_not_ready)?;
                }
                Pending::SingleShot => {
                    // NACK: the single shot is still running.
                    self.bus.write(cmd::READ_MEASUREMENT, &[]) $($aw)* .map_err(nack_as_not_ready)?;
                    self.pending = Pending::Nothing;
                    match self.bus.read_polling(&mut w, POLL_ATTEMPTS) $($aw)* {
                        Err(Error::Timeout) => return Err(Error::NotReady),
                        other => other?,
                    }
                }
            }
            Ok(convert(w, self.sht4x))
        }

        $($async)* fn enter_sleep(&mut self) -> Result<(), Error<I2C::Error>> {
            self.bus.write(cmd::ENTER_SLEEP, &[]) $($aw)* ?;
            self.pending = Pending::Nothing;
            Ok(())
        }

        $($async)* fn exit_sleep(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            // Address + payload byte 0x00, not acknowledged (section 3.4.8).
            ignore_nack(self.bus.write_bytes(&[0x00]) $($aw)*)?;
            Ok(WAKE_WAIT_US)
        }
    };
}

macro_rules! trait_impls {
    ($Sensor:path, $LowPower:path, $I2c:path, [$($async:tt)*] [$($aw:tt)*]) => {
        impl<I2C: $I2c> $Sensor for Stcc4<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// See the crate documentation for the per-mode behaviour.
            $($async)* fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.start() $($aw)*
            }

            /// [`Error::NotReady`] if nothing was requested or no data is
            /// available yet.
            $($async)* fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                self.read() $($aw)*
            }
        }

        impl<I2C: $I2c> $LowPower for Stcc4<I2C> {
            type BusError = I2C::Error;

            /// `enter_sleep_mode` (1 µA; sensor must be idle — stop
            /// continuous measurement first). Compensation values and ASC
            /// state are retained.
            $($async)* fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.enter_sleep() $($aw)*
            }

            /// `exit_sleep_mode`; 5 ms.
            $($async)* fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.exit_sleep() $($aw)*
            }
        }
    };
}

use zigbee_sensirion_common::blocking::Interface;
use zigbee_sensirion_common::nack_as_not_ready;

/// Blocking driver.
#[derive(Debug)]
pub struct Stcc4<I2C> {
    bus: Interface<I2C>,
    mode: Mode,
    sht4x: bool,
    running: bool,
    pending: Pending,
}

impl<I2C> Stcc4<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Stcc4<I2C> {
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
        ADDRESS, COMMAND_WAIT_US, CONDITIONING_WAIT_US, CONTINUOUS_INTERVAL_US,
        FACTORY_RESET_WAIT_US, FRC_WAIT_US, Measurement, Mode, POLL_ATTEMPTS, PRODUCT_ID, Pending,
        RESET_WAIT_US, SELF_TEST_WAIT_US, SINGLE_SHOT_WAIT_US, STOP_WAIT_US, WAKE_WAIT_US, cmd,
        convert, ticks_from_humidity,
    };
    use zigbee_sensirion_common::asynch::Interface;
    use zigbee_sensirion_common::{ignore_nack, nack_as_not_ready, ticks_from_temperature};
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Stcc4`].
    #[derive(Debug)]
    pub struct Stcc4<I2C> {
        bus: Interface<I2C>,
        mode: Mode,
        sht4x: bool,
        running: bool,
        pending: Pending,
    }

    impl<I2C> Stcc4<I2C> {
        constructors!();
    }

    impl<I2C: embedded_hal_async::i2c::I2c> Stcc4<I2C> {
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

    fn nack(t: Transaction) -> Transaction {
        t.with_error(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address))
    }

    // 500 ppm, 25 °C (26214), 50 %RH (29360), testing mode off.
    const SAMPLE: [u16; 4] = [500, 26_214, 29_360, 0];

    #[test]
    fn conversions_match_table_11() {
        assert_eq!(ticks_from_humidity(5_000), 29_360);
        assert_eq!(humidity_from_ticks(29_360), 5_000);
        assert_eq!(humidity_from_ticks(0), 0); // -6 % clamps
        assert_eq!(
            convert(SAMPLE, true),
            Measurement {
                co2: 500,
                temperature: Some(2_500),
                humidity: Some(5_000),
                testing_mode: false
            }
        );
        let m = convert([0xFFF0, 0, 0, 0x0040], false);
        assert_eq!((m.co2, m.temperature, m.testing_mode), (0, None, true));
        assert!(self_test_passed(0x0010));
        assert!(!self_test_passed(0x0001));
    }

    #[test]
    fn single_shot_cycle_with_sleep() {
        let expectations = [
            nack(Transaction::write(ADDRESS, vec![0x00])),
            Transaction::write(ADDRESS, vec![0x36, 0x5B]),
            Transaction::read(
                ADDRESS,
                [
                    vec![0x09, 0x01, 0x73, 0x01, 0x8A, 0xD4],
                    words(&[0, 0, 0x1234, 0x5678]),
                ]
                .concat(),
            ),
            Transaction::write(ADDRESS, vec![0x21, 0x9D]),
            Transaction::write(ADDRESS, vec![0xEC, 0x05]),
            nack(Transaction::read(ADDRESS, vec![0; 12])),
            Transaction::read(ADDRESS, words(&SAMPLE)),
            Transaction::write(ADDRESS, vec![0x36, 0x50]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut delay = CheckedDelay::new(&[D::blocking_delay_us(1_000)]);
        let mut stcc = Stcc4::new(&mut i2c);
        assert_eq!(stcc.wake().unwrap(), 5_000);
        assert_eq!(stcc.probe(&mut delay).unwrap(), 0x1234_5678);
        assert_eq!(stcc.start_measurement().unwrap(), 500_000);
        assert_eq!(stcc.read_measurement().unwrap().humidity, Some(5_000));
        stcc.sleep().unwrap();
        delay.done();
        i2c.done();
    }

    #[test]
    fn continuous_mode_and_compensation_inputs() {
        let expectations = [
            // Datasheet 3.4.4 / 3.4.5 examples.
            Transaction::write(
                ADDRESS_ALT,
                vec![0xE0, 0x00, 0x66, 0x66, 0x93, 0x72, 0xB0, 0xDC],
            ),
            Transaction::write(ADDRESS_ALT, vec![0xE0, 0x16, 0xC5, 0xDA, 0x83]),
            Transaction::write(ADDRESS_ALT, vec![0x21, 0x8B]),
            Transaction::write(ADDRESS_ALT, vec![0xEC, 0x05]),
            Transaction::read(ADDRESS_ALT, words(&SAMPLE)),
            Transaction::write(ADDRESS_ALT, vec![0xEC, 0x05]),
            nack(Transaction::read(ADDRESS_ALT, vec![0; 12])),
            Transaction::write(ADDRESS_ALT, vec![0x3F, 0x86]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut stcc = Stcc4::new_with_address(&mut i2c, ADDRESS_ALT)
            .with_mode(Mode::Continuous)
            .with_sht4x(false);
        stcc.set_rht_compensation(2_500, 5_000).unwrap();
        stcc.set_pressure_compensation(101_300).unwrap();
        assert_eq!(stcc.start_measurement().unwrap(), 1_000_000);
        assert_eq!(stcc.read_measurement(), Err(Error::NotReady));
        assert_eq!(stcc.start_measurement().unwrap(), 1_000);
        let m = stcc.read_measurement().unwrap();
        assert_eq!((m.co2, m.temperature), (500, None));
        stcc.start_measurement().unwrap();
        assert_eq!(stcc.read_measurement(), Err(Error::NotReady));
        assert_eq!(stcc.stop_continuous_measurement().unwrap(), 1_200_000);
        i2c.done();
    }

    #[test]
    fn maintenance_commands() {
        let expectations = [
            Transaction::write(ADDRESS, vec![0x36, 0x2F, 0x01, 0xF4, 0x33]),
            Transaction::read(ADDRESS, words(&[32_668])),
            Transaction::write(ADDRESS, vec![0x36, 0x32]),
            Transaction::read(ADDRESS, words(&[0xFFFF])),
            Transaction::write(ADDRESS, vec![0x27, 0x8C]),
            Transaction::read(ADDRESS, words(&[0x0010])),
            Transaction::write(ADDRESS, vec![0x29, 0xBC]),
            Transaction::write(ADDRESS, vec![0x3F, 0xBC]),
            Transaction::write(ADDRESS, vec![0x3F, 0x3D]),
            nack(Transaction::write(0x00, vec![0x06])),
            Transaction::write(ADDRESS, vec![0x36, 0x5B]),
            Transaction::read(ADDRESS, words(&[0x0901, 0x018B, 0, 0, 0, 0])),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut delay = CheckedDelay::new(&[D::blocking_delay_us(1_000)]);
        let mut stcc = Stcc4::new(&mut i2c);
        assert_eq!(stcc.start_forced_recalibration(500).unwrap(), 90_000);
        assert_eq!(stcc.read_forced_recalibration().unwrap(), -100);
        stcc.start_factory_reset().unwrap();
        assert_eq!(stcc.read_factory_reset(), Err(Error::InvalidData));
        assert_eq!(stcc.start_self_test().unwrap(), 360_000);
        assert!(self_test_passed(stcc.read_self_test().unwrap()));
        assert_eq!(stcc.perform_conditioning().unwrap(), 22_000_000);
        stcc.enable_testing_mode().unwrap();
        stcc.disable_testing_mode().unwrap();
        assert_eq!(stcc.soft_reset().unwrap(), 10_000);
        assert_eq!(
            stcc.probe(&mut delay),
            Err(Error::UnexpectedId(0x0901_018B))
        );
        delay.done();
        i2c.done();
    }

    #[test]
    fn async_single_shot() {
        use zigbee_sensor::asynch::{LowPower as _, Sensor as _};

        let mut bad = words(&SAMPLE);
        bad[11] ^= 1;
        let expectations = [
            Transaction::write(ADDRESS, vec![0x21, 0x9D]),
            Transaction::write(ADDRESS, vec![0xEC, 0x05]),
            Transaction::read(ADDRESS, words(&SAMPLE)),
            Transaction::write(ADDRESS, vec![0x21, 0x9D]),
            Transaction::write(ADDRESS, vec![0xEC, 0x05]),
            Transaction::read(ADDRESS, bad),
            Transaction::write(ADDRESS, vec![0x36, 0x50]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut delay = CheckedDelay::new(&[D::async_delay_us(500_000)]);
        let mut stcc = asynch::Stcc4::new(&mut i2c);
        block_on(async {
            let m = stcc.measure(&mut delay).await.unwrap();
            assert_eq!(m.temperature, Some(2_500));
            stcc.start_measurement().await.unwrap();
            assert_eq!(stcc.read_measurement().await, Err(Error::Crc));
            stcc.sleep().await.unwrap();
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
