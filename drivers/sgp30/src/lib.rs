//! Sensirion SGP30 multi-pixel gas sensor (I2C, 0x58): TVOC (ppb) and CO2eq
//! (ppm) from the on-chip baseline compensation algorithm, raw H2 / ethanol
//! signals, baseline get/set, absolute-humidity compensation, feature set
//! (`probe`), measure test and soft reset (general call).
//!
//! # Measurement through the [`Sensor`](zigbee_sensor::blocking::Sensor) trait
//!
//! The first `start_measurement` sends `sgp30_iaq_init` (10 ms) and the next
//! read is [`Error::NotReady`]; later starts send `sgp30_measure_iaq`
//! (12 ms). **`sgp30_measure_iaq` must be sent once per second** for the
//! dynamic baseline compensation to work — the caller owns that cadence.
//! During the first 15 s after `iaq_init` the sensor returns the fixed
//! values 400 ppm / 0 ppb.
//!
//! The SGP30 has no sleep command: it sleeps (2 µA) only after power-up or
//! a soft reset, and `iaq_init` keeps it measuring (48 mA). There is
//! therefore no `LowPower` implementation; use [`Sgp30::soft_reset`] (an
//! I2C general call that resets every device on the bus supporting it) or
//! switch the supply.
//!
//! Reference: Sensirion "Datasheet SGP30", version 1.0 (May 2020, D1).

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address.
pub const ADDRESS: u8 = 0x58;

mod cmd {
    pub const IAQ_INIT: u16 = 0x2003;
    pub const MEASURE_IAQ: u16 = 0x2008;
    pub const GET_IAQ_BASELINE: u16 = 0x2015;
    pub const SET_IAQ_BASELINE: u16 = 0x201E;
    pub const SET_ABSOLUTE_HUMIDITY: u16 = 0x2061;
    pub const MEASURE_TEST: u16 = 0x2032;
    pub const GET_FEATURE_SET: u16 = 0x202F;
    pub const MEASURE_RAW: u16 = 0x2050;
    pub const GET_TVOC_INCEPTIVE_BASELINE: u16 = 0x20B3;
    pub const SET_TVOC_BASELINE: u16 = 0x2077;
}

/// `sgp30_measure_iaq` maximum duration.
pub const MEASURE_IAQ_WAIT_US: WaitUs = 12_000;
/// `sgp30_measure_raw` maximum duration.
pub const MEASURE_RAW_WAIT_US: WaitUs = 25_000;
/// `sgp30_measure_test` maximum duration.
pub const MEASURE_TEST_WAIT_US: WaitUs = 220_000;
/// Maximum duration of `iaq_init`, the baseline, humidity and feature-set
/// commands.
pub const COMMAND_WAIT_US: WaitUs = 10_000;
/// Soft reset / power-up time.
pub const RESET_WAIT_US: WaitUs = 600;
/// `sgp30_measure_test` result when the self test passed.
pub const TEST_PASSED: u16 = 0xD400;

/// One `sgp30_measure_iaq` result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// CO2 equivalent in ppm (400–60 000).
    pub co2eq: u16,
    /// Total VOC in ppb (0–60 000).
    pub tvoc: u16,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Eco2(u32::from(self.co2eq)));
        f(Quantity::Tvoc(u32::from(self.tvoc)));
    }
}

/// Baseline of the compensation algorithm (store regularly, restore after
/// `iaq_init`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Baseline {
    /// CO2eq baseline.
    pub co2eq: u16,
    /// TVOC baseline.
    pub tvoc: u16,
}

/// Raw signals of `sgp30_measure_raw`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RawSignals {
    /// H2 signal (sout_H2).
    pub h2: u16,
    /// Ethanol signal (sout_EtOH).
    pub ethanol: u16,
}

/// Saturation water vapour density in mg/m³ from -20 °C to 70 °C in 5 °C
/// steps (Magnus formula of datasheet section 6.3).
const SATURATION_MG_M3: [u32; 19] = [
    1_078, 1_611, 2_364, 3_412, 4_849, 6_792, 9_383, 12_797, 17_243, 22_968, 30_264, 39_471,
    50_983, 65_250, 82_785, 104_168, 130_048, 161_150, 198_277,
];

/// Absolute humidity in the 8.8 fixed-point g/m³ format of
/// `set_absolute_humidity`, from temperature (0.01 °C, clamped to
/// -20–70 °C) and relative humidity (0.01 %RH); linear interpolation of a
/// saturation table (≤ 1.5 % high between table points). Returns at least
/// 1 (0 disables the compensation).
pub fn absolute_humidity(centi_celsius: i32, centi_percent: u16) -> u16 {
    let t = centi_celsius.clamp(-2_000, 7_000) + 2_000;
    let i = ((t / 500) as usize).min(SATURATION_MG_M3.len() - 2);
    let frac = (t - i as i32 * 500) as u32;
    let (lo, hi) = (SATURATION_MG_M3[i], SATURATION_MG_M3[i + 1]);
    let saturation = lo + ((hi - lo) * frac + 250) / 500;
    let mg = saturation * u32::from(centi_percent.min(10_000)) / 10_000;
    ((mg * 256 + 500) / 1_000).clamp(1, 0xFFFF) as u16
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Pending {
    Nothing,
    Iaq,
    Raw,
    Test,
}

macro_rules! constructors {
    () => {
        /// Create a driver; no bus traffic.
        pub const fn new(i2c: I2C) -> Self {
            Self {
                bus: Interface::new(i2c, ADDRESS),
                initialized: false,
                pending: Pending::Nothing,
            }
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
        $($async)* fn send(&mut self, cmd: u16, args: &[u16], wait: WaitUs, pending: Pending) -> Result<WaitUs, Error<I2C::Error>> {
            self.bus.write(cmd, args) $($aw)* ?;
            self.pending = pending;
            Ok(wait)
        }

        $($async)* fn read_pending<const N: usize>(&mut self, expected: Pending) -> Result<[u16; N], Error<I2C::Error>> {
            if self.pending != expected {
                return Err(Error::NotReady);
            }
            let mut w = [0u16; N];
            self.bus.read(&mut w) $($aw)* .map_err(nack_as_not_ready)?;
            self.pending = Pending::Nothing;
            Ok(w)
        }

        /// Read the feature set (product type in bits 15:12 must be 0 for
        /// the SGP30, else [`Error::UnexpectedId`]); returns it.
        pub $($async)* fn probe<D: $Delay>(&mut self, delay: &mut D) -> Result<u16, Error<I2C::Error>> {
            let mut w = [0u16];
            self.bus.fetch(cmd::GET_FEATURE_SET, &[], COMMAND_WAIT_US, delay, &mut w) $($aw)* ?;
            if w[0] & 0xF000 != 0 {
                return Err(Error::UnexpectedId(u32::from(w[0])));
            }
            Ok(w[0])
        }

        /// `sgp30_iaq_init`: start the air-quality measurement (after every
        /// power-up / soft reset); restore a baseline afterwards if stored.
        pub $($async)* fn init_air_quality(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::IAQ_INIT, &[], COMMAND_WAIT_US, Pending::Nothing) $($aw)* ?;
            self.initialized = true;
            Ok(COMMAND_WAIT_US)
        }

        /// `sgp30_measure_iaq`; send once per second, read after 12 ms with
        /// [`read_air_quality`](Self::read_air_quality).
        pub $($async)* fn start_air_quality(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::MEASURE_IAQ, &[], MEASURE_IAQ_WAIT_US, Pending::Iaq) $($aw)*
        }

        /// CO2eq / TVOC; [`Error::NotReady`] if not started or still busy.
        pub $($async)* fn read_air_quality(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let [co2eq, tvoc] = self.read_pending(Pending::Iaq) $($aw)* ?;
            Ok(Measurement { co2eq, tvoc })
        }

        /// Read the current baseline.
        pub $($async)* fn get_baseline<D: $Delay>(&mut self, delay: &mut D) -> Result<Baseline, Error<I2C::Error>> {
            let mut w = [0u16; 2];
            self.bus.fetch(cmd::GET_IAQ_BASELINE, &[], COMMAND_WAIT_US, delay, &mut w) $($aw)* ?;
            Ok(Baseline { co2eq: w[0], tvoc: w[1] })
        }

        /// Restore a baseline (after `init_air_quality`); sent in the
        /// datasheet order TVOC, CO2eq.
        pub $($async)* fn set_baseline(&mut self, baseline: Baseline) -> Result<WaitUs, Error<I2C::Error>> {
            let args = [baseline.tvoc, baseline.co2eq];
            self.send(cmd::SET_IAQ_BASELINE, &args, COMMAND_WAIT_US, Pending::Nothing) $($aw)*
        }

        /// Absolute humidity for on-chip compensation, 8.8 fixed-point g/m³
        /// (see [`absolute_humidity`](crate::absolute_humidity)); 0 disables
        /// the compensation.
        pub $($async)* fn set_absolute_humidity(&mut self, g_m3_8_8: u16) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::SET_ABSOLUTE_HUMIDITY, &[g_m3_8_8], COMMAND_WAIT_US, Pending::Nothing) $($aw)*
        }

        /// Start `sgp30_measure_raw` (25 ms).
        pub $($async)* fn start_raw(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::MEASURE_RAW, &[], MEASURE_RAW_WAIT_US, Pending::Raw) $($aw)*
        }

        /// H2 and ethanol raw signals.
        pub $($async)* fn read_raw(&mut self) -> Result<RawSignals, Error<I2C::Error>> {
            let [h2, ethanol] = self.read_pending(Pending::Raw) $($aw)* ?;
            Ok(RawSignals { h2, ethanol })
        }

        /// Start `sgp30_measure_test` (220 ms; production test only, not
        /// after `iaq_init`).
        pub $($async)* fn start_measure_test(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::MEASURE_TEST, &[], MEASURE_TEST_WAIT_US, Pending::Test) $($aw)*
        }

        /// `true` if the self test returned [`TEST_PASSED`](crate::TEST_PASSED).
        pub $($async)* fn read_measure_test(&mut self) -> Result<bool, Error<I2C::Error>> {
            let [result] = self.read_pending(Pending::Test) $($aw)* ?;
            Ok(result == TEST_PASSED)
        }

        /// Read the pre-calibrated inceptive TVOC baseline (feature set ≥ 34).
        pub $($async)* fn get_tvoc_inceptive_baseline<D: $Delay>(&mut self, delay: &mut D) -> Result<u16, Error<I2C::Error>> {
            let mut w = [0u16];
            self.bus.fetch(cmd::GET_TVOC_INCEPTIVE_BASELINE, &[], COMMAND_WAIT_US, delay, &mut w) $($aw)* ?;
            Ok(w[0])
        }

        /// Activate the inceptive TVOC baseline (first start-up only).
        pub $($async)* fn set_tvoc_baseline(&mut self, baseline: u16) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::SET_TVOC_BASELINE, &[baseline], COMMAND_WAIT_US, Pending::Nothing) $($aw)*
        }

        /// Soft reset through the I2C general call (also resets other
        /// devices on the bus that implement it); the sensor sleeps
        /// afterwards (2 µA) until the next `iaq_init`.
        pub $($async)* fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.bus.general_call_reset() $($aw)* ?;
            self.initialized = false;
            self.pending = Pending::Nothing;
            Ok(RESET_WAIT_US)
        }

        $($async)* fn start(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            if self.initialized {
                self.start_air_quality() $($aw)*
            } else {
                self.init_air_quality() $($aw)*
            }
        }
    };
}

macro_rules! trait_impls {
    ($Sensor:path, $I2c:path, [$($async:tt)*] [$($aw:tt)*]) => {
        impl<I2C: $I2c> $Sensor for Sgp30<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// `iaq_init` the first time, then `measure_iaq`; call at 1 Hz.
            $($async)* fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.start() $($aw)*
            }

            /// [`Error::NotReady`] after `iaq_init` or while busy.
            $($async)* fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                self.read_air_quality() $($aw)*
            }
        }
    };
}

use zigbee_sensirion_common::blocking::Interface;
use zigbee_sensirion_common::nack_as_not_ready;

/// Blocking driver.
#[derive(Debug)]
pub struct Sgp30<I2C> {
    bus: Interface<I2C>,
    initialized: bool,
    pending: Pending,
}

impl<I2C> Sgp30<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Sgp30<I2C> {
    methods!([] [] embedded_hal::delay::DelayNs);
}

trait_impls!(
    zigbee_sensor::blocking::Sensor,
    embedded_hal::i2c::I2c,
    [] []
);

/// Async driver.
pub mod asynch {
    use super::{
        ADDRESS, Baseline, COMMAND_WAIT_US, MEASURE_IAQ_WAIT_US, MEASURE_RAW_WAIT_US,
        MEASURE_TEST_WAIT_US, Measurement, Pending, RESET_WAIT_US, RawSignals, TEST_PASSED, cmd,
    };
    use zigbee_sensirion_common::asynch::Interface;
    use zigbee_sensirion_common::nack_as_not_ready;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Sgp30`].
    #[derive(Debug)]
    pub struct Sgp30<I2C> {
        bus: Interface<I2C>,
        initialized: bool,
        pending: Pending,
    }

    impl<I2C> Sgp30<I2C> {
        constructors!();
    }

    impl<I2C: embedded_hal_async::i2c::I2c> Sgp30<I2C> {
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
    use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};
    use embedded_hal_mock::eh1::delay::{CheckedDelay, Transaction as D};
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use std::vec;
    use std::vec::Vec;
    use zigbee_sensirion_common::encode_word;
    use zigbee_sensor::blocking::Sensor;

    fn words(w: &[u16]) -> Vec<u8> {
        w.iter().flat_map(|&x| encode_word(x)).collect()
    }

    fn nack(t: Transaction) -> Transaction {
        t.with_error(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address))
    }

    #[test]
    fn absolute_humidity_conversion() {
        // 25 °C / 50 %RH: 11.48 g/m³ by the section 6.3 formula.
        assert_eq!(absolute_humidity(2_500, 5_000), 2_940);
        // 27 °C / 60 %RH: exact 15.41 g/m³ = 3944; interpolated 0.8 % high.
        assert_eq!(absolute_humidity(2_700, 6_000), 3_976);
        assert_eq!(absolute_humidity(-4_000, 0), 1);
        assert_eq!(absolute_humidity(9_000, 10_000), 50_759);
    }

    #[test]
    fn iaq_cycle_and_baseline() {
        let expectations = [
            Transaction::write(ADDRESS, vec![0x20, 0x2F]),
            Transaction::read(ADDRESS, words(&[0x0022])),
            Transaction::write(ADDRESS, vec![0x20, 0x03]),
            // TVOC first, then CO2eq.
            Transaction::write(
                ADDRESS,
                vec![0x20, 0x1E, 0x8A, 0x9C, 0x08, 0x8E, 0x5E, 0xAE],
            ),
            // Section 6.3 example: 0x0F80 = 15.5 g/m³.
            Transaction::write(ADDRESS, vec![0x20, 0x61, 0x0F, 0x80, 0x62]),
            Transaction::write(ADDRESS, vec![0x20, 0x08]),
            nack(Transaction::read(ADDRESS, vec![0; 6])),
            Transaction::read(ADDRESS, words(&[400, 0])),
            Transaction::write(ADDRESS, vec![0x20, 0x15]),
            Transaction::read(ADDRESS, words(&[0x8E5E, 0x8A9C])),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut delay =
            CheckedDelay::new(&[D::blocking_delay_us(10_000), D::blocking_delay_us(10_000)]);
        let mut sgp = Sgp30::new(&mut i2c);
        assert_eq!(sgp.probe(&mut delay).unwrap(), 0x0022);
        assert_eq!(sgp.start_measurement().unwrap(), 10_000);
        assert_eq!(sgp.read_measurement(), Err(Error::NotReady));
        let baseline = Baseline {
            co2eq: 0x8E5E,
            tvoc: 0x8A9C,
        };
        sgp.set_baseline(baseline).unwrap();
        sgp.set_absolute_humidity(0x0F80).unwrap();
        assert_eq!(sgp.start_measurement().unwrap(), 12_000);
        assert_eq!(sgp.read_measurement(), Err(Error::NotReady));
        let m = sgp.read_measurement().unwrap();
        assert_eq!(
            m,
            Measurement {
                co2eq: 400,
                tvoc: 0
            }
        );
        let mut seen = Vec::new();
        m.for_each(&mut |q| seen.push(q));
        assert_eq!(seen, [Quantity::Eco2(400), Quantity::Tvoc(0)]);
        assert_eq!(sgp.get_baseline(&mut delay).unwrap(), baseline);
        delay.done();
        i2c.done();
    }

    #[test]
    fn raw_test_and_reset() {
        let expectations = [
            Transaction::write(ADDRESS, vec![0x20, 0x2F]),
            Transaction::read(ADDRESS, words(&[0x1022])),
            Transaction::write(ADDRESS, vec![0x20, 0x32]),
            Transaction::read(ADDRESS, words(&[TEST_PASSED])),
            Transaction::write(ADDRESS, vec![0x20, 0x50]),
            Transaction::read(ADDRESS, words(&[13_000, 18_000])),
            Transaction::write(ADDRESS, vec![0x20, 0xB3]),
            Transaction::read(ADDRESS, words(&[0x1234])),
            Transaction::write(ADDRESS, vec![0x20, 0x77, 0x12, 0x34, 0x37]),
            nack(Transaction::write(0x00, vec![0x06])),
            Transaction::write(ADDRESS, vec![0x20, 0x03]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut delay =
            CheckedDelay::new(&[D::blocking_delay_us(10_000), D::blocking_delay_us(10_000)]);
        let mut sgp = Sgp30::new(&mut i2c);
        assert_eq!(sgp.probe(&mut delay), Err(Error::UnexpectedId(0x1022)));
        assert_eq!(sgp.start_measure_test().unwrap(), 220_000);
        assert_eq!(sgp.read_raw(), Err(Error::NotReady));
        assert!(sgp.read_measure_test().unwrap());
        assert_eq!(sgp.start_raw().unwrap(), 25_000);
        assert_eq!(
            sgp.read_raw().unwrap(),
            RawSignals {
                h2: 13_000,
                ethanol: 18_000
            }
        );
        let inceptive = sgp.get_tvoc_inceptive_baseline(&mut delay).unwrap();
        sgp.set_tvoc_baseline(inceptive).unwrap();
        assert_eq!(sgp.soft_reset().unwrap(), 600);
        // After reset the next start initialises again.
        assert_eq!(sgp.start_measurement().unwrap(), 10_000);
        delay.done();
        i2c.done();
    }

    #[test]
    fn async_measurement_and_crc() {
        use zigbee_sensor::asynch::Sensor as _;

        let mut bad = words(&[450, 12]);
        bad[2] ^= 1;
        let expectations = [
            Transaction::write(ADDRESS, vec![0x20, 0x03]),
            Transaction::write(ADDRESS, vec![0x20, 0x08]),
            Transaction::read(ADDRESS, words(&[450, 12])),
            Transaction::write(ADDRESS, vec![0x20, 0x08]),
            Transaction::read(ADDRESS, bad),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut delay = CheckedDelay::new(&[D::async_delay_us(12_000)]);
        let mut sgp = asynch::Sgp30::new(&mut i2c);
        block_on(async {
            sgp.start_measurement().await.unwrap();
            let m = sgp.measure(&mut delay).await.unwrap();
            assert_eq!(
                m,
                Measurement {
                    co2eq: 450,
                    tvoc: 12
                }
            );
            sgp.start_measurement().await.unwrap();
            assert_eq!(sgp.read_measurement().await, Err(Error::Crc));
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
