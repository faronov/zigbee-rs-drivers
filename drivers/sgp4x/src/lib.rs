//! Sensirion SGP40 (VOC) and SGP41 (VOC + NOx) metal-oxide gas sensors
//! (I2C, 0x59), with Sensirion's gas index algorithm ported to fixed point
//! in [`gas_index`].
//!
//! Commands: raw signal measurement with optional RH/T compensation,
//! SGP41 conditioning, heater off ([`zigbee_sensor::blocking::LowPower`]),
//! built-in self test, serial number (`probe`) and soft reset (general
//! call).
//!
//! # Measurement through the [`Sensor`](zigbee_sensor::blocking::Sensor) trait
//!
//! `start_measurement` sends `measure_raw_signal(s)` (SGP40 30 ms, SGP41
//! 50 ms) with the configured compensation values; on the SGP41 the first
//! [`CONDITIONING_SAMPLES`] starts after power-up or heater off send
//! `execute_conditioning` instead (NOx reported as `None`, indices not
//! updated). `read_measurement` reads the raw signals ([`Error::NotReady`]
//! while the sensor NACKs) and feeds them to the VOC / NOx gas index
//! algorithms. **The algorithms and the conditioning count assume one
//! measurement per second with the heater kept on** — the cadence is the
//! caller's responsibility. The indices are 0 (not reported) during the
//! first 45 s.
//!
//! References: Sensirion "Datasheet SGP40" version 1.2 (February 2022),
//! "Datasheet SGP41" version 1.0 (December 2021).

#![no_std]

pub mod gas_index;

use gas_index::{Algorithm, GasIndexAlgorithm};
use zigbee_sensirion_common::{ticks_from_humidity, ticks_from_temperature};
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address of all SGP4x parts.
pub const ADDRESS: u8 = 0x59;

mod cmd {
    pub const SGP40_MEASURE_RAW: u16 = 0x260F;
    pub const SGP41_CONDITIONING: u16 = 0x2612;
    pub const SGP41_MEASURE_RAW: u16 = 0x2619;
    pub const SELF_TEST: u16 = 0x280E;
    pub const HEATER_OFF: u16 = 0x3615;
    pub const GET_SERIAL: u16 = 0x3682;
}

/// SGP40 `measure_raw_signal` maximum duration.
pub const SGP40_MEASURE_WAIT_US: WaitUs = 30_000;
/// SGP41 `measure_raw_signals` / `execute_conditioning` maximum duration.
pub const SGP41_MEASURE_WAIT_US: WaitUs = 50_000;
/// `execute_self_test` maximum duration.
pub const SELF_TEST_WAIT_US: WaitUs = 320_000;
/// `turn_heater_off` and `get_serial_number` maximum duration.
pub const COMMAND_WAIT_US: WaitUs = 1_000;
/// Soft reset / power-up time.
pub const RESET_WAIT_US: WaitUs = 600;
/// Default number of 1 s conditioning cycles of the SGP41 (10 s; never
/// exceed 10 s).
pub const CONDITIONING_SAMPLES: u8 = 10;

/// Compensation words for 50 %RH / 25 °C, which disable the on-chip
/// humidity compensation.
const DEFAULT_COMPENSATION: [u16; 2] = [0x8000, 0x6666];

/// Sensor variant.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// SGP40: VOC pixel only.
    Sgp40,
    /// SGP41: VOC and NOx pixels, needs conditioning.
    Sgp41,
}

impl Variant {
    const fn measure_wait(self) -> WaitUs {
        match self {
            Variant::Sgp40 => SGP40_MEASURE_WAIT_US,
            Variant::Sgp41 => SGP41_MEASURE_WAIT_US,
        }
    }

    /// Interpret an `execute_self_test` result word: SGP40 passes with
    /// MSB 0xD4; SGP41 passes with bits 0 (VOC) and 1 (NOx) clear.
    pub const fn self_test_passed(self, result: u16) -> bool {
        match self {
            Variant::Sgp40 => result >> 8 == 0xD4,
            Variant::Sgp41 => result & 0x0003 == 0,
        }
    }
}

/// Raw signals in ticks (proportional to the log of the sensing-layer
/// resistance).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RawSignals {
    /// SRAW_VOC.
    pub voc: u16,
    /// SRAW_NOX (SGP41 measurement mode only).
    pub nox: Option<u16>,
}

/// One sample: raw signals plus gas indices.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Raw signals.
    pub raw: RawSignals,
    /// VOC index 1–500 (100 = average); 0 while the algorithm is warming
    /// up or the SGP41 is conditioning.
    pub voc_index: u16,
    /// NOx index 1–500 (1 = average); `None` on the SGP40 and during
    /// conditioning, 0 while warming up.
    pub nox_index: Option<u16>,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        if self.voc_index != 0 {
            f(Quantity::VocIndex(self.voc_index));
        }
        if let Some(nox) = self.nox_index.filter(|&n| n != 0) {
            f(Quantity::NoxIndex(nox));
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Pending {
    Nothing,
    Conditioning,
    Measurement,
}

macro_rules! constructors {
    () => {
        /// Create a driver; no bus traffic. Compensation is disabled until
        /// [`set_compensation`](Self::set_compensation).
        pub fn new(i2c: I2C, variant: Variant) -> Self {
            Self {
                bus: Interface::new(i2c, ADDRESS),
                variant,
                compensation: DEFAULT_COMPENSATION,
                conditioning_samples: CONDITIONING_SAMPLES,
                conditioning_left: CONDITIONING_SAMPLES,
                pending: Pending::Nothing,
                voc: GasIndexAlgorithm::new(Algorithm::Voc),
                nox: GasIndexAlgorithm::new(Algorithm::Nox),
            }
        }

        /// Number of 1 s SGP41 conditioning cycles after power-up or heater
        /// off (default [`CONDITIONING_SAMPLES`](crate::CONDITIONING_SAMPLES); at most 10).
        pub fn with_conditioning_samples(mut self, samples: u8) -> Self {
            self.conditioning_samples = samples.min(CONDITIONING_SAMPLES);
            self.conditioning_left = self.conditioning_samples;
            self
        }

        /// Sensor variant.
        pub const fn variant(&self) -> Variant {
            self.variant
        }

        /// Ambient temperature (0.01 °C) and humidity (0.01 %RH) sent with
        /// the next measurements to enable on-chip humidity compensation.
        pub fn set_compensation(&mut self, centi_celsius: i32, centi_percent: u16) {
            self.compensation = [
                ticks_from_humidity(centi_percent),
                ticks_from_temperature(centi_celsius),
            ];
        }

        /// Send the default values again (compensation off).
        pub fn clear_compensation(&mut self) {
            self.compensation = DEFAULT_COMPENSATION;
        }

        /// VOC gas index algorithm (states, tuning).
        pub fn voc_algorithm(&mut self) -> &mut GasIndexAlgorithm {
            &mut self.voc
        }

        /// NOx gas index algorithm (SGP41).
        pub fn nox_algorithm(&mut self) -> &mut GasIndexAlgorithm {
            &mut self.nox
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
        /// Read the 48-bit serial number; serves as presence check (the
        /// SGP4x has no product ID). Sensor must be idle.
        pub $($async)* fn probe<D: $Delay>(&mut self, delay: &mut D) -> Result<u64, Error<I2C::Error>> {
            let mut w = [0u16; 3];
            self.bus.fetch(cmd::GET_SERIAL, &[], COMMAND_WAIT_US, delay, &mut w) $($aw)* ?;
            Ok(w.iter().fold(0u64, |acc, &x| acc << 16 | u64::from(x)))
        }

        /// Start one raw measurement with the configured compensation
        /// (starts / continues measurement mode); read with
        /// [`read_raw`](Self::read_raw).
        pub $($async)* fn start_raw_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            let command = match self.variant {
                Variant::Sgp40 => cmd::SGP40_MEASURE_RAW,
                Variant::Sgp41 => cmd::SGP41_MEASURE_RAW,
            };
            self.bus.write(command, &self.compensation) $($aw)* ?;
            self.pending = Pending::Measurement;
            Ok(self.variant.measure_wait())
        }

        /// SGP41: start one conditioning cycle (call every second for up to
        /// 10 s after power-up / heater off); [`Error::Unsupported`] on the
        /// SGP40. [`read_raw`](Self::read_raw) then returns SRAW_VOC only.
        pub $($async)* fn start_conditioning(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            if self.variant != Variant::Sgp41 {
                return Err(Error::Unsupported);
            }
            self.bus.write(cmd::SGP41_CONDITIONING, &DEFAULT_COMPENSATION) $($aw)* ?;
            self.pending = Pending::Conditioning;
            Ok(SGP41_MEASURE_WAIT_US)
        }

        /// Read the started measurement or conditioning result;
        /// [`Error::NotReady`] if nothing was started or the sensor is still
        /// busy (NACK; retry later).
        pub $($async)* fn read_raw(&mut self) -> Result<RawSignals, Error<I2C::Error>> {
            let mut w = [0u16; 2];
            let n = match (self.pending, self.variant) {
                (Pending::Nothing, _) => return Err(Error::NotReady),
                (Pending::Measurement, Variant::Sgp41) => 2,
                _ => 1,
            };
            self.bus.read(&mut w[..n]) $($aw)* .map_err(nack_as_not_ready)?;
            self.pending = Pending::Nothing;
            Ok(RawSignals { voc: w[0], nox: (n == 2).then_some(w[1]) })
        }

        /// Turn the hotplate off and return to idle (34 µA); the SGP41
        /// conditions again afterwards.
        pub $($async)* fn turn_heater_off(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.bus.write(cmd::HEATER_OFF, &[]) $($aw)* ?;
            self.idle();
            Ok(COMMAND_WAIT_US)
        }

        /// Start the built-in self test; read it after 320 ms.
        pub $($async)* fn start_self_test(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.bus.write(cmd::SELF_TEST, &[]) $($aw)* ?;
            Ok(SELF_TEST_WAIT_US)
        }

        /// Raw self-test word; see [`Variant::self_test_passed`].
        pub $($async)* fn read_self_test(&mut self) -> Result<u16, Error<I2C::Error>> {
            let mut w = [0u16];
            self.bus.read(&mut w) $($aw)* .map_err(nack_as_not_ready)?;
            Ok(w[0])
        }

        /// Soft reset through the I2C general call (also resets other
        /// devices on the bus that implement it); 0.6 ms.
        pub $($async)* fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.bus.general_call_reset() $($aw)* ?;
            self.idle();
            Ok(RESET_WAIT_US)
        }

        $($async)* fn start(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            if self.variant == Variant::Sgp41 && self.conditioning_left > 0 {
                let wait = self.start_conditioning() $($aw)* ?;
                self.conditioning_left -= 1;
                Ok(wait)
            } else {
                self.start_raw_measurement() $($aw)*
            }
        }

        $($async)* fn read(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let conditioning = self.pending == Pending::Conditioning;
            let raw = self.read_raw() $($aw)* ?;
            Ok(self.process(raw, conditioning))
        }
    };
}

macro_rules! helpers {
    () => {
        fn idle(&mut self) {
            self.pending = Pending::Nothing;
            self.conditioning_left = self.conditioning_samples;
        }

        fn process(&mut self, raw: RawSignals, conditioning: bool) -> Measurement {
            let voc_index = if conditioning {
                0
            } else {
                self.voc.process(raw.voc)
            };
            Measurement {
                raw,
                voc_index,
                nox_index: raw.nox.map(|nox| self.nox.process(nox)),
            }
        }
    };
}

macro_rules! trait_impls {
    ($Sensor:path, $LowPower:path, $I2c:path, [$($async:tt)*] [$($aw:tt)*]) => {
        impl<I2C: $I2c> $Sensor for Sgp4x<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// Conditioning (SGP41, first cycles) or raw measurement; call
            /// once per second.
            $($async)* fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.start() $($aw)*
            }

            /// Raw signals and gas indices; [`Error::NotReady`] if nothing
            /// was started or the sensor is still busy.
            $($async)* fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                self.read() $($aw)*
            }
        }

        impl<I2C: $I2c> $LowPower for Sgp4x<I2C> {
            type BusError = I2C::Error;

            /// `turn_heater_off` (idle, 34 µA). The gas index algorithms keep
            /// their state; call `reset()` on them after long pauses.
            $($async)* fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.turn_heater_off() $($aw)* ?;
                Ok(())
            }

            /// No command needed: the next measurement turns the heater on.
            $($async)* fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                Ok(0)
            }
        }
    };
}

use zigbee_sensirion_common::blocking::Interface;
use zigbee_sensirion_common::nack_as_not_ready;

/// Blocking driver.
#[derive(Debug)]
pub struct Sgp4x<I2C> {
    bus: Interface<I2C>,
    variant: Variant,
    compensation: [u16; 2],
    conditioning_samples: u8,
    conditioning_left: u8,
    pending: Pending,
    voc: GasIndexAlgorithm,
    nox: GasIndexAlgorithm,
}

impl<I2C> Sgp4x<I2C> {
    constructors!();
    helpers!();
}

impl<I2C: embedded_hal::i2c::I2c> Sgp4x<I2C> {
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
    use super::gas_index::{Algorithm, GasIndexAlgorithm};
    use super::{
        ADDRESS, COMMAND_WAIT_US, CONDITIONING_SAMPLES, DEFAULT_COMPENSATION, Measurement, Pending,
        RESET_WAIT_US, RawSignals, SELF_TEST_WAIT_US, SGP41_MEASURE_WAIT_US, Variant, cmd,
    };
    use zigbee_sensirion_common::asynch::Interface;
    use zigbee_sensirion_common::{nack_as_not_ready, ticks_from_humidity, ticks_from_temperature};
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Sgp4x`].
    #[derive(Debug)]
    pub struct Sgp4x<I2C> {
        bus: Interface<I2C>,
        variant: Variant,
        compensation: [u16; 2],
        conditioning_samples: u8,
        conditioning_left: u8,
        pending: Pending,
        voc: GasIndexAlgorithm,
        nox: GasIndexAlgorithm,
    }

    impl<I2C> Sgp4x<I2C> {
        constructors!();
        helpers!();
    }

    impl<I2C: embedded_hal_async::i2c::I2c> Sgp4x<I2C> {
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

    #[test]
    fn sgp40_commands_match_datasheet() {
        let expectations = [
            // Table 9: measurement without compensation.
            Transaction::write(
                ADDRESS,
                vec![0x26, 0x0F, 0x80, 0x00, 0xA2, 0x66, 0x66, 0x93],
            ),
            nack(Transaction::read(ADDRESS, vec![0; 3])),
            Transaction::read(ADDRESS, words(&[30_000])),
            // Table 10 minimum: 0 %RH, -45 °C.
            Transaction::write(
                ADDRESS,
                vec![0x26, 0x0F, 0x00, 0x00, 0x81, 0x00, 0x00, 0x81],
            ),
            Transaction::read(ADDRESS, words(&[30_000])),
            Transaction::write(ADDRESS, vec![0x28, 0x0E]),
            Transaction::read(ADDRESS, words(&[0xD400])),
            Transaction::write(ADDRESS, vec![0x36, 0x82]),
            Transaction::read(ADDRESS, words(&[0x0000, 0x0123, 0x4567])),
            Transaction::write(ADDRESS, vec![0x36, 0x15]),
            nack(Transaction::write(0x00, vec![0x06])),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut delay = CheckedDelay::new(&[D::blocking_delay_us(1_000)]);
        let mut sgp = Sgp4x::new(&mut i2c, Variant::Sgp40);
        assert_eq!(sgp.read_measurement(), Err(Error::NotReady));
        assert_eq!(sgp.start_measurement().unwrap(), 30_000);
        assert_eq!(sgp.read_measurement(), Err(Error::NotReady));
        let m = sgp.read_measurement().unwrap();
        assert_eq!(
            m.raw,
            RawSignals {
                voc: 30_000,
                nox: None
            }
        );
        assert_eq!((m.voc_index, m.nox_index), (0, None));
        sgp.set_compensation(-4_500, 0);
        sgp.start_measurement().unwrap();
        sgp.read_measurement().unwrap();
        assert_eq!(sgp.start_self_test().unwrap(), 320_000);
        assert!(Variant::Sgp40.self_test_passed(sgp.read_self_test().unwrap()));
        assert_eq!(sgp.probe(&mut delay).unwrap(), 0x0123_4567);
        sgp.sleep().unwrap();
        assert_eq!(sgp.soft_reset().unwrap(), 600);
        delay.done();
        i2c.done();
    }

    #[test]
    fn sgp41_conditioning_then_measurement() {
        let expectations = [
            // Table 9: conditioning with default parameters.
            Transaction::write(
                ADDRESS,
                vec![0x26, 0x12, 0x80, 0x00, 0xA2, 0x66, 0x66, 0x93],
            ),
            Transaction::read(ADDRESS, words(&[29_000])),
            Transaction::write(
                ADDRESS,
                vec![0x26, 0x19, 0x80, 0x00, 0xA2, 0x66, 0x66, 0x93],
            ),
            Transaction::read(ADDRESS, words(&[30_000, 15_000])),
            Transaction::write(ADDRESS, vec![0x36, 0x15]),
            Transaction::write(
                ADDRESS,
                vec![0x26, 0x12, 0x80, 0x00, 0xA2, 0x66, 0x66, 0x93],
            ),
            Transaction::read(ADDRESS, words(&[29_000])),
            Transaction::write(ADDRESS, vec![0x28, 0x0E]),
            Transaction::read(ADDRESS, words(&[0x0002])),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut sgp = Sgp4x::new(&mut i2c, Variant::Sgp41).with_conditioning_samples(1);
        assert_eq!(sgp.start_measurement().unwrap(), 50_000);
        let m = sgp.read_measurement().unwrap();
        assert_eq!((m.raw.voc, m.raw.nox, m.nox_index), (29_000, None, None));
        sgp.start_measurement().unwrap();
        let m = sgp.read_measurement().unwrap();
        assert_eq!(m.raw.nox, Some(15_000));
        assert_eq!(m.nox_index, Some(0));
        let mut seen = 0;
        m.for_each(&mut |_| seen += 1);
        assert_eq!(seen, 0);
        sgp.sleep().unwrap();
        sgp.start_measurement().unwrap();
        assert_eq!(sgp.read_measurement().unwrap().raw.nox, None);
        sgp.start_self_test().unwrap();
        assert!(!Variant::Sgp41.self_test_passed(sgp.read_self_test().unwrap()));
        i2c.done();
        let mut idle = Mock::new(&[]);
        assert_eq!(
            Sgp4x::new(&mut idle, Variant::Sgp40).start_conditioning(),
            Err(Error::Unsupported)
        );
        idle.done();
    }

    #[test]
    fn indices_after_warm_up() {
        let mut expectations = Vec::new();
        for _ in 0..47 {
            expectations.push(Transaction::write(
                ADDRESS,
                vec![0x26, 0x19, 0x80, 0x00, 0xA2, 0x66, 0x66, 0x93],
            ));
            expectations.push(Transaction::read(ADDRESS, words(&[30_000, 15_000])));
        }
        let mut i2c = Mock::new(&expectations);
        let mut sgp = Sgp4x::new(&mut i2c, Variant::Sgp41).with_conditioning_samples(0);
        let mut last = None;
        for _ in 0..47 {
            sgp.start_measurement().unwrap();
            last = Some(sgp.read_measurement().unwrap());
        }
        let m = last.unwrap();
        assert_eq!(m.nox_index, Some(1));
        let mut seen = Vec::new();
        m.for_each(&mut |q| seen.push(q));
        assert_eq!(
            seen,
            [Quantity::VocIndex(m.voc_index), Quantity::NoxIndex(1)]
        );
        assert_ne!(m.voc_index, 0);
        i2c.done();
    }

    #[test]
    fn async_measurement_and_crc() {
        use zigbee_sensor::asynch::{LowPower as _, Sensor as _};

        let mut bad = words(&[30_000, 15_000]);
        bad[5] ^= 1;
        let expectations = [
            Transaction::write(
                ADDRESS,
                vec![0x26, 0x19, 0x80, 0x00, 0xA2, 0x66, 0x66, 0x93],
            ),
            Transaction::read(ADDRESS, words(&[30_000, 15_000])),
            Transaction::write(
                ADDRESS,
                vec![0x26, 0x19, 0x80, 0x00, 0xA2, 0x66, 0x66, 0x93],
            ),
            Transaction::read(ADDRESS, bad),
            Transaction::write(ADDRESS, vec![0x36, 0x15]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut delay = CheckedDelay::new(&[D::async_delay_us(50_000)]);
        let mut sgp = asynch::Sgp4x::new(&mut i2c, Variant::Sgp41).with_conditioning_samples(0);
        block_on(async {
            let m = sgp.measure(&mut delay).await.unwrap();
            assert_eq!(m.raw.nox, Some(15_000));
            sgp.start_measurement().await.unwrap();
            assert_eq!(sgp.read_measurement().await, Err(Error::Crc));
            sgp.sleep().await.unwrap();
            assert_eq!(sgp.wake().await.unwrap(), 0);
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
