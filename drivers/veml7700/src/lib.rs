//! Vishay VEML7700 and VEML6030 ambient light sensors (I2C).
//!
//! Both parts share the register map: 16-bit little-endian registers,
//! gain ×1/8…×2, integration time 25…800 ms, a power-saving mode for
//! continuous operation and a 0.5 µA shutdown.
//!
//! By default the driver works *one-shot*: `start_measurement` powers the
//! ALS on, `read_measurement` reads ALS + WHITE and shuts it down again. For
//! continuous operation with the power-saving mode (PSM) call
//! [`Veml7700::start_continuous`]; reads then leave the part running.
//!
//! Above ~1000 lx the response at gain ×1/4 and ×1/8 is non-linear; the
//! application note's 4th-order correction polynomial is applied in fixed
//! point (see [`correct_millilux`]).
//!
//! References: Vishay VEML7700 datasheet rev. 1.8 (doc. 84286), VEML6030
//! datasheet (doc. 84366), application notes "Designing the VEML7700 /
//! VEML6030 Into an Application" (doc. 84323 / 84367).

#![no_std]

use embedded_hal::i2c::I2c;
use zigbee_sensor::blocking::{LowPower, Sensor};
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// VEML7700 address, VEML6030 with ADDR low.
pub const ADDRESS: u8 = 0x10;
/// VEML6030 with ADDR high.
pub const ADDRESS_VEML6030_HIGH: u8 = 0x48;

const REG_CONF: u8 = 0x00;
const REG_WH: u8 = 0x01;
const REG_WL: u8 = 0x02;
const REG_PSM: u8 = 0x03;
const REG_ALS: u8 = 0x04;
const REG_WHITE: u8 = 0x05;
const REG_INT: u8 = 0x06;
const REG_ID: u8 = 0x07;

/// Low byte of the ID register.
pub const DEVICE_ID: u8 = 0x81;

const CONF_SD: u16 = 1;
const CONF_INT_EN: u16 = 1 << 1;
const INT_LOW: u16 = 1 << 15;
const INT_HIGH: u16 = 1 << 14;

/// Start-up time after clearing ALS_SD (2.5 ms VEML7700, 4 ms VEML6030).
const STARTUP_US: u32 = 4_000;

/// ALS gain (`ALS_GAIN`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Gain {
    /// ×1.
    X1 = 0,
    /// ×2 (most sensitive).
    X2 = 1,
    /// ×1/8 (default; the application note starts here).
    #[default]
    X1_8 = 2,
    /// ×1/4.
    X1_4 = 3,
}

impl Gain {
    /// Resolution multiplier relative to gain ×2.
    const fn factor(self) -> u32 {
        match self {
            Self::X2 => 1,
            Self::X1 => 2,
            Self::X1_4 => 8,
            Self::X1_8 => 16,
        }
    }
}

/// Integration time (`ALS_IT`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum IntegrationTime {
    Ms25 = 0b1100,
    Ms50 = 0b1000,
    #[default]
    Ms100 = 0b0000,
    Ms200 = 0b0001,
    Ms400 = 0b0010,
    Ms800 = 0b0011,
}

impl IntegrationTime {
    /// Integration time in microseconds.
    pub const fn us(self) -> u32 {
        match self {
            Self::Ms25 => 25_000,
            Self::Ms50 => 50_000,
            Self::Ms100 => 100_000,
            Self::Ms200 => 200_000,
            Self::Ms400 => 400_000,
            Self::Ms800 => 800_000,
        }
    }

    const fn factor(self) -> u32 {
        800_000 / self.us()
    }
}

/// Interrupt persistence (`ALS_PERS`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum Persistence {
    #[default]
    One = 0,
    Two = 1,
    Four = 2,
    Eight = 3,
}

/// Power-saving mode for continuous operation (`PSM`): adds 0.5/1/2/4 s
/// of standby between conversions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(missing_docs)]
pub enum PowerSaving {
    Mode1 = 0,
    Mode2 = 1,
    Mode3 = 2,
    Mode4 = 3,
}

impl PowerSaving {
    /// Refresh time in microseconds ("Refresh Time" table): IT + 0.5 s × 2^mode.
    pub const fn refresh_us(self, it: IntegrationTime) -> u32 {
        it.us() + (500_000 << self as u32)
    }
}

/// Resolution in 1/10 mlx per count (4.2 mlx at ×2 / 800 ms).
pub const fn resolution_decimillilux(gain: Gain, it: IntegrationTime) -> u32 {
    42 * gain.factor() * it.factor()
}

/// Uncorrected illuminance in milli-lux.
pub const fn counts_to_millilux(counts: u16, gain: Gain, it: IntegrationTime) -> u32 {
    counts as u32 * resolution_decimillilux(gain, it) / 10
}

/// Application-note non-linearity correction
/// `y = 6.0135e-13 x⁴ − 9.3924e-9 x³ + 8.1488e-5 x² + 1.0023 x` (x, y in
/// lux), evaluated in integer Horner form with 1e-17-scaled coefficients.
/// Saturates at `u32::MAX`.
pub fn correct_millilux(millilux: u32) -> u32 {
    const A: i128 = 60_135;
    const B: i128 = -939_240_000;
    const C: i128 = 8_148_800_000_000;
    const D: i128 = 100_230_000_000_000_000;
    let m = i128::from(millilux);
    let p = A * m / 1000 + B;
    let p = p * m / 1000 + C;
    let p = p * m / 1000 + D;
    let y = p * m / 100_000_000_000_000_000;
    y.clamp(0, u32::MAX as i128) as u32
}

/// Setting for [`auto_range`].
pub type Range = (Gain, IntegrationTime);

const LADDER: [Range; 9] = [
    (Gain::X1_8, IntegrationTime::Ms25),
    (Gain::X1_8, IntegrationTime::Ms50),
    (Gain::X1_8, IntegrationTime::Ms100),
    (Gain::X1_4, IntegrationTime::Ms100),
    (Gain::X1, IntegrationTime::Ms100),
    (Gain::X2, IntegrationTime::Ms100),
    (Gain::X2, IntegrationTime::Ms200),
    (Gain::X2, IntegrationTime::Ms400),
    (Gain::X2, IntegrationTime::Ms800),
];

/// Application-note auto-range step: raise gain, then integration time,
/// while a sample is ≤ 100 counts; step back when the ADC nears saturation
/// (≥ 60 000 counts) or gain ×1/×2 sees more than 100 lx (non-linear there).
/// Returns `None` when the current range is fine.
pub fn auto_range(current: Range, counts: u16) -> Option<Range> {
    let pos = LADDER.iter().position(|r| *r == current).unwrap_or(2);
    let (gain, it) = LADDER[pos];
    let high_gain = matches!(gain, Gain::X1 | Gain::X2);
    if counts >= 60_000 || (high_gain && counts_to_millilux(counts, gain, it) > 100_000) {
        pos.checked_sub(1).map(|p| LADDER[p])
    } else if counts <= 100 {
        LADDER.get(pos + 1).copied()
    } else if LADDER[pos] != current {
        Some(LADDER[pos])
    } else {
        None
    }
}

/// Measurement configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// ALS gain.
    pub gain: Gain,
    /// Integration time.
    pub integration_time: IntegrationTime,
    /// Interrupt persistence.
    pub persistence: Persistence,
    /// Enable the threshold interrupt (VEML6030 INT pin; flags on both).
    pub interrupt: bool,
    /// Skip the non-linearity correction even at gain ×1/4 and ×1/8.
    pub no_correction: bool,
}

impl Config {
    /// `ALS_CONF_0` value.
    pub const fn word(&self, shutdown: bool) -> u16 {
        (self.gain as u16) << 11
            | (self.integration_time as u16) << 6
            | (self.persistence as u16) << 4
            | (if self.interrupt { CONF_INT_EN } else { 0 })
            | (if shutdown { CONF_SD } else { 0 })
    }

    /// Wait for one conversion after power-on: start-up + IT + 10 %.
    pub const fn wait_us(&self) -> WaitUs {
        STARTUP_US + self.integration_time.us() * 11 / 10
    }

    /// Counts → milli-lux, corrected if the gain is ×1/4 or ×1/8.
    pub fn millilux(&self, counts: u16) -> u32 {
        let raw = counts_to_millilux(counts, self.gain, self.integration_time);
        if !self.no_correction && matches!(self.gain, Gain::X1_4 | Gain::X1_8) {
            correct_millilux(raw)
        } else {
            raw
        }
    }

    /// Milli-lux → threshold counts (linear, saturating).
    pub const fn millilux_to_counts(&self, millilux: u32) -> u16 {
        let res = resolution_decimillilux(self.gain, self.integration_time) as u64;
        let counts = millilux as u64 * 10 / res;
        if counts > u16::MAX as u64 {
            u16::MAX
        } else {
            counts as u16
        }
    }
}

/// One sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Illuminance in milli-lux.
    pub illuminance: u32,
    /// Raw ALS counts.
    pub als: u16,
    /// Raw WHITE channel counts.
    pub white: u16,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Illuminance(self.illuminance));
    }
}

/// Threshold interrupt flags (register 0x06, cleared on read).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InterruptStatus {
    /// Crossed the low threshold.
    pub low: bool,
    /// Crossed the high threshold.
    pub high: bool,
}

macro_rules! driver {
    ($($async_:ident)?, $($await_:tt)*) => {
        /// VEML7700 / VEML6030 driver.
        pub struct Veml7700<I2C> {
            i2c: I2C,
            address: u8,
            config: Config,
            power_saving: Option<PowerSaving>,
            continuous: bool,
        }

        impl<I2C> Veml7700<I2C> {
            /// Create a driver; no bus traffic.
            pub const fn new(i2c: I2C, address: u8) -> Self {
                Self {
                    i2c,
                    address,
                    config: Config {
                        gain: Gain::X1_8,
                        integration_time: IntegrationTime::Ms100,
                        persistence: Persistence::One,
                        interrupt: false,
                        no_correction: false,
                    },
                    power_saving: None,
                    continuous: false,
                }
            }

            /// Set the configuration (written on the next start).
            pub const fn with_config(mut self, config: Config) -> Self {
                self.config = config;
                self
            }

            /// Current configuration.
            pub const fn config(&self) -> Config {
                self.config
            }

            /// Change gain and integration time, e.g. from [`auto_range`].
            pub fn set_range(&mut self, range: Range) {
                self.config.gain = range.0;
                self.config.integration_time = range.1;
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.i2c
            }
        }

        impl<I2C: I2c> Veml7700<I2C> {
            $($async_)? fn read_reg(&mut self, reg: u8) -> Result<u16, Error<I2C::Error>> {
                let mut buf = [0u8; 2];
                self.i2c
                    .write_read(self.address, &[reg], &mut buf)
                    $($await_)*
                    .map_err(Error::bus)?;
                Ok(u16::from_le_bytes(buf))
            }

            $($async_)? fn write_reg(&mut self, reg: u8, value: u16) -> Result<(), Error<I2C::Error>> {
                let [lo, hi] = value.to_le_bytes();
                self.i2c
                    .write(self.address, &[reg, lo, hi])
                    $($await_)*
                    .map_err(Error::bus)
            }

            /// Check the device ID (low byte 0x81).
            pub $($async_)? fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
                let id = self.read_reg(REG_ID) $($await_)* ?;
                if id & 0xFF != u16::from(DEVICE_ID) {
                    return Err(Error::UnexpectedId(u32::from(id)));
                }
                Ok(())
            }

            /// Program the interrupt window in milli-lux (linear counts at the
            /// current gain / integration time).
            pub $($async_)? fn set_thresholds(&mut self, low: u32, high: u32) -> Result<(), Error<I2C::Error>> {
                let low = self.config.millilux_to_counts(low);
                let high = self.config.millilux_to_counts(high);
                self.write_reg(REG_WL, low) $($await_)* ?;
                self.write_reg(REG_WH, high) $($await_)*
            }

            /// Read and clear the threshold flags.
            pub $($async_)? fn interrupt_status(&mut self) -> Result<InterruptStatus, Error<I2C::Error>> {
                let v = self.read_reg(REG_INT) $($await_)* ?;
                Ok(InterruptStatus {
                    low: v & INT_LOW != 0,
                    high: v & INT_HIGH != 0,
                })
            }

            /// Run continuously, optionally with a power-saving mode.
            /// Returns the time until the first result.
            pub $($async_)? fn start_continuous(&mut self, psm: Option<PowerSaving>) -> Result<WaitUs, Error<I2C::Error>> {
                let psm_word = match psm {
                    Some(m) => (m as u16) << 1 | 1,
                    None => 0,
                };
                self.write_reg(REG_PSM, psm_word) $($await_)* ?;
                self.write_reg(REG_CONF, self.config.word(false)) $($await_)* ?;
                self.power_saving = psm;
                self.continuous = true;
                Ok(self.config.wait_us())
            }

            /// Back to one-shot operation (shuts the ALS down).
            pub $($async_)? fn stop_continuous(&mut self) -> Result<(), Error<I2C::Error>> {
                self.continuous = false;
                self.write_reg(REG_CONF, self.config.word(true)) $($await_)*
            }

            /// Refresh period in continuous mode.
            pub const fn refresh_us(&self) -> u32 {
                match self.power_saving {
                    Some(m) => m.refresh_us(self.config.integration_time),
                    None => self.config.integration_time.us(),
                }
            }
        }

        impl<I2C: I2c> Sensor for Veml7700<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// One-shot: power on and integrate. Continuous: nothing to do,
            /// returns 0.
            $($async_)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                if self.continuous {
                    return Ok(0);
                }
                self.write_reg(REG_CONF, self.config.word(false)) $($await_)* ?;
                Ok(self.config.wait_us())
            }

            /// Read ALS + WHITE; in one-shot mode shut the ALS down after.
            $($async_)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let als = self.read_reg(REG_ALS) $($await_)* ?;
                let white = self.read_reg(REG_WHITE) $($await_)* ?;
                if !self.continuous {
                    self.write_reg(REG_CONF, self.config.word(true)) $($await_)* ?;
                }
                Ok(Measurement {
                    illuminance: self.config.millilux(als),
                    als,
                    white,
                })
            }
        }

        impl<I2C: I2c> LowPower for Veml7700<I2C> {
            type BusError = I2C::Error;

            /// Set ALS_SD (0.5 µA).
            $($async_)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write_reg(REG_CONF, self.config.word(true)) $($await_)*
            }

            /// Continuous mode: power back on. One-shot: nothing to do.
            $($async_)? fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                if !self.continuous {
                    return Ok(0);
                }
                self.write_reg(REG_CONF, self.config.word(false)) $($await_)* ?;
                Ok(self.config.wait_us())
            }
        }
    };
}

driver!(,);

/// Async driver.
pub mod asynch {
    #[allow(unused_imports)]
    use super::auto_range;
    use super::{
        Config, DEVICE_ID, Gain, INT_HIGH, INT_LOW, IntegrationTime, InterruptStatus, Measurement,
        Persistence, PowerSaving, REG_ALS, REG_CONF, REG_ID, REG_INT, REG_PSM, REG_WH, REG_WHITE,
        REG_WL, Range,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::asynch::{LowPower, Sensor};
    use zigbee_sensor::{Error, WaitUs};

    driver!(async, .await);
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use std::vec;

    #[test]
    fn resolution_table() {
        // Application note "Resolution and maximum detection range".
        assert_eq!(
            resolution_decimillilux(Gain::X2, IntegrationTime::Ms800),
            42
        );
        assert_eq!(
            resolution_decimillilux(Gain::X1_4, IntegrationTime::Ms100),
            2_688
        );
        assert_eq!(
            resolution_decimillilux(Gain::X1_8, IntegrationTime::Ms25),
            21_504
        );
        // Max 65535 × 2.1504 lx = 140 926 lx.
        assert_eq!(
            counts_to_millilux(0xFFFF, Gain::X1_8, IntegrationTime::Ms25),
            140_926_464
        );
    }

    #[test]
    fn correction_matches_app_note() {
        // AN example: 5581 counts at ×1/4, 100 ms = 1500 lx → 1658 lx.
        let raw = counts_to_millilux(5581, Gain::X1_4, IntegrationTime::Ms100);
        assert_eq!(raw, 1_500_172);
        assert_eq!(correct_millilux(1_500_000), 1_658_142);
        assert_eq!(correct_millilux(0), 0);
        assert_eq!(correct_millilux(1_000), 1_002);
        assert_eq!(correct_millilux(140_926_464), u32::MAX);
        let config = Config {
            gain: Gain::X1_4,
            ..Config::default()
        };
        assert_eq!(config.millilux(5581), correct_millilux(raw));
        let config = Config {
            gain: Gain::X1,
            ..Config::default()
        };
        assert_eq!(config.millilux(100), 6_720);
    }

    #[test]
    fn auto_range_ladder() {
        let start = (Gain::X1_8, IntegrationTime::Ms100);
        assert_eq!(auto_range(start, 1_000), None);
        assert_eq!(
            auto_range(start, 50),
            Some((Gain::X1_4, IntegrationTime::Ms100))
        );
        assert_eq!(
            auto_range(start, 65_000),
            Some((Gain::X1_8, IntegrationTime::Ms50))
        );
        // ×2 / 100 ms: 3000 counts ≈ 101 lx → leave the non-linear gain.
        let g2 = (Gain::X2, IntegrationTime::Ms100);
        assert_eq!(
            auto_range(g2, 3_000),
            Some((Gain::X1, IntegrationTime::Ms100))
        );
        let top = (Gain::X2, IntegrationTime::Ms800);
        assert_eq!(auto_range(top, 10), None);
    }

    #[test]
    fn one_shot_cycle() {
        let expectations = [
            Transaction::write_read(ADDRESS, vec![0x07], vec![0x81, 0xC4]),
            Transaction::write(ADDRESS, vec![0x00, 0x00, 0x18]),
            Transaction::write_read(ADDRESS, vec![0x04], vec![0xCD, 0x15]),
            Transaction::write_read(ADDRESS, vec![0x05], vec![0x34, 0x12]),
            Transaction::write(ADDRESS, vec![0x00, 0x01, 0x18]),
            Transaction::write(ADDRESS, vec![0x02, 0x64, 0x00]),
            Transaction::write(ADDRESS, vec![0x01, 0xFF, 0xFF]),
            Transaction::write_read(ADDRESS, vec![0x06], vec![0x00, 0x80]),
        ];
        let mut i2c = Mock::new(&expectations);
        let config = Config {
            gain: Gain::X1_4,
            ..Config::default()
        };
        let mut veml = Veml7700::new(&mut i2c, ADDRESS).with_config(config);
        veml.probe().unwrap();
        assert_eq!(veml.start_measurement().unwrap(), 114_000);
        let m = veml.read_measurement().unwrap();
        assert_eq!(m.als, 5581);
        assert_eq!(m.white, 0x1234);
        assert_eq!(m.illuminance, correct_millilux(1_500_172));
        // 100 counts × 268.8 mlx.
        veml.set_thresholds(26_880, u32::MAX).unwrap();
        let st = veml.interrupt_status().unwrap();
        assert!(st.low && !st.high);
        i2c.done();
    }

    #[test]
    fn continuous_with_psm() {
        let expectations = [
            Transaction::write(ADDRESS_VEML6030_HIGH, vec![0x03, 0x07, 0x00]),
            Transaction::write(ADDRESS_VEML6030_HIGH, vec![0x00, 0xC0, 0x18]),
            Transaction::write_read(ADDRESS_VEML6030_HIGH, vec![0x04], vec![0x0A, 0x00]),
            Transaction::write_read(ADDRESS_VEML6030_HIGH, vec![0x05], vec![0x0B, 0x00]),
            Transaction::write(ADDRESS_VEML6030_HIGH, vec![0x00, 0xC1, 0x18]),
            Transaction::write(ADDRESS_VEML6030_HIGH, vec![0x00, 0xC0, 0x18]),
            Transaction::write_read(ADDRESS_VEML6030_HIGH, vec![0x07], vec![0x80, 0xD4]),
        ];
        let mut i2c = Mock::new(&expectations);
        let config = Config {
            gain: Gain::X1_4,
            integration_time: IntegrationTime::Ms800,
            no_correction: true,
            ..Config::default()
        };
        let mut veml = Veml7700::new(&mut i2c, ADDRESS_VEML6030_HIGH).with_config(config);
        assert_eq!(
            veml.start_continuous(Some(PowerSaving::Mode4)).unwrap(),
            884_000
        );
        assert_eq!(veml.refresh_us(), 4_800_000);
        assert_eq!(veml.start_measurement().unwrap(), 0);
        let m = veml.read_measurement().unwrap();
        assert_eq!(m.illuminance, 336);
        veml.sleep().unwrap();
        assert_eq!(veml.wake().unwrap(), 884_000);
        assert_eq!(veml.probe(), Err(Error::UnexpectedId(0xD480)));
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
            Transaction::write(ADDRESS, vec![0x00, 0x40, 0x08]),
            Transaction::write_read(ADDRESS, vec![0x04], vec![0x64, 0x00]),
            Transaction::write_read(ADDRESS, vec![0x05], vec![0x00, 0x00]),
            Transaction::write(ADDRESS, vec![0x00, 0x41, 0x08]),
        ];
        let mut i2c = Mock::new(&expectations);
        let config = Config {
            gain: Gain::X2,
            integration_time: IntegrationTime::Ms200,
            ..Config::default()
        };
        let mut veml = asynch::Veml7700::new(&mut i2c, ADDRESS).with_config(config);
        let mut delay = RecordingDelay(0);
        let m = block_on(veml.measure(&mut delay)).unwrap();
        // 100 × 16.8 mlx, gain ×2 is not corrected.
        assert_eq!(m.illuminance, 1_680);
        assert_eq!(delay.0, 224_000);
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
