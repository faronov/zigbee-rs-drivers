//! Vishay VEML6031X00 and VEML6035 ambient light sensors (I2C, 0x29).
//!
//! * **VEML6031X00** — ALS + IR channels, 8-bit register map, *active
//!   force* (single-shot) mode with a data-ready flag, gain ×0.5…×2,
//!   ¼ or full photodiode, IT 3.125–400 ms, 0.0026 lx/count to 172 klx.
//! * **VEML6035** — VEML7700-style 16-bit register map (ALS + WHITE),
//!   GAIN/DG/SENS sensitivity bits, IT 25–800 ms, 0.0004 lx/count,
//!   power-saving mode.
//!
//! Both drivers power the part on for one conversion and shut it down
//! again in `read_measurement` (0.5 µA shutdown).
//!
//! References: Vishay VEML6031X00 datasheet rev. 1.4 (doc. 80007,
//! Dec 2025); VEML6035 datasheet rev. 1.3 (doc. 84889, Nov 2024).

#![no_std]

use embedded_hal::i2c::I2c;
use zigbee_sensor::blocking::{LowPower, Sensor};
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// VEML6035 and VEML6031X00 standard address option.
pub const ADDRESS: u8 = 0x29;
/// VEML6031X00 alternative address option.
pub const ADDRESS_ALT: u8 = 0x10;

/// Start-up allowance after leaving shutdown.
const STARTUP_US: u32 = 4_000;

/// Interrupt persistence.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum Persistence {
    #[default]
    One = 0,
    Two = 1,
    Four = 2,
    Eight = 3,
}

/// Threshold interrupt flags (cleared on read).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InterruptStatus {
    /// Crossed the low threshold.
    pub low: bool,
    /// Crossed the high threshold.
    pub high: bool,
}

/// One sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Illuminance in milli-lux.
    pub illuminance: u32,
    /// Raw ALS counts.
    pub als: u16,
    /// Raw second channel: IR (VEML6031X00) or WHITE (VEML6035).
    pub aux: u16,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Illuminance(self.illuminance));
    }
}

const fn saturate_u16(v: u64) -> u16 {
    if v > u16::MAX as u64 {
        u16::MAX
    } else {
        v as u16
    }
}

// ---------------------------------------------------------------- VEML6031

const R31_CONF0: u8 = 0x00;
const R31_THDH: u8 = 0x04;
const R31_THDL: u8 = 0x06;
const R31_DATA: u8 = 0x10;
const R31_ID: u8 = 0x14;
const R31_INT: u8 = 0x17;

/// `VEML6031X00_ID_L` value.
pub const VEML6031_ID: u8 = 0x01;

const C0_AF: u8 = 1 << 3;
const C0_TRIG: u8 = 1 << 2;
const C0_INT: u8 = 1 << 1;
const C0_SD: u8 = 1;
const C1_SD: u8 = 1 << 7;
const C1_CAL: u8 = 1;
const F31_READY: u8 = 1 << 3;
const F31_LOW: u8 = 1 << 2;
const F31_HIGH: u8 = 1 << 1;

/// VEML6031X00 gain (`ALS_GAIN`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum Gain31 {
    #[default]
    X1 = 0,
    X2 = 1,
    X0_66 = 2,
    X0_5 = 3,
}

/// VEML6031X00 integration time (`ALS_IT`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum IntegrationTime31 {
    Us3125 = 0,
    Us6250 = 1,
    Ms12_5 = 2,
    Ms25 = 3,
    Ms50 = 4,
    #[default]
    Ms100 = 5,
    Ms200 = 6,
    Ms400 = 7,
}

impl IntegrationTime31 {
    /// Integration time in microseconds.
    pub const fn us(self) -> u32 {
        3_125 << self as u32
    }
}

/// VEML6031X00 configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config31 {
    /// Gain.
    pub gain: Gain31,
    /// Integration time.
    pub integration_time: IntegrationTime31,
    /// Use ¼ of the photodiode (`ALS_PDDIV`): 4× range, ¼ resolution.
    pub quarter_pd: bool,
    /// Interrupt persistence.
    pub persistence: Persistence,
    /// Enable the threshold interrupt.
    pub interrupt: bool,
}

impl Config31 {
    /// `[ALS_CONF_0, ALS_CONF_1]`; `on` triggers an active-force conversion.
    pub const fn bytes(&self, on: bool) -> [u8; 2] {
        let int = if self.interrupt { C0_INT } else { 0 };
        let it = (self.integration_time as u8) << 4;
        let c1 = (if self.quarter_pd { 1 << 6 } else { 0 })
            | (self.gain as u8) << 3
            | (self.persistence as u8) << 1;
        if on {
            [it | C0_AF | C0_TRIG | int, c1 | C1_CAL]
        } else {
            [it | C0_AF | int | C0_SD, c1 | C1_SD]
        }
    }

    /// Wait after a trigger: start-up + IT + 10 %.
    pub const fn wait_us(&self) -> WaitUs {
        STARTUP_US + self.integration_time.us() * 11 / 10
    }

    /// Counts → milli-lux (datasheet rev. 1.4 Tables 11/12: 2.568 mlx/count
    /// at ×2, 400 ms, full photodiode).
    pub const fn millilux(&self, counts: u16) -> u32 {
        let (num, den): (u64, u64) = match self.gain {
            Gain31::X2 => (1, 1),
            Gain31::X1 => (2, 1),
            Gain31::X0_66 => (100, 33),
            Gain31::X0_5 => (4, 1),
        };
        let it = 128 >> self.integration_time as u64;
        let pd = if self.quarter_pd { 4 } else { 1 };
        (counts as u64 * 2_568 * num * it * pd / (den * 1_000)) as u32
    }

    /// Milli-lux → counts (saturating).
    pub const fn counts(&self, millilux: u32) -> u16 {
        let per_count_x1000 = self.millilux(1_000) as u64;
        saturate_u16(millilux as u64 * 1_000 / per_count_x1000)
    }
}

// ---------------------------------------------------------------- VEML6035

const R35_CONF: u8 = 0x00;
const R35_WH: u8 = 0x01;
const R35_WL: u8 = 0x02;
const R35_PSM: u8 = 0x03;
const R35_ALS: u8 = 0x04;
const R35_WHITE: u8 = 0x05;
const R35_INT: u8 = 0x06;

const F35_LOW: u16 = 1 << 15;
const F35_HIGH: u16 = 1 << 14;

/// VEML6035 integration time (`ALS_IT`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum IntegrationTime35 {
    Ms25 = 0b1100,
    Ms50 = 0b1000,
    #[default]
    Ms100 = 0b0000,
    Ms200 = 0b0001,
    Ms400 = 0b0010,
    Ms800 = 0b0011,
}

impl IntegrationTime35 {
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
}

/// VEML6035 power-saving wait between conversions (`PSM_WAIT`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(missing_docs)]
pub enum PsmWait {
    Ms400 = 0,
    Ms800 = 1,
    Ms1600 = 2,
    Ms3200 = 3,
}

/// VEML6035 configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config35 {
    /// `GAIN` = 1: double sensitivity.
    pub gain: bool,
    /// `DG` = 1: double gain.
    pub double_gain: bool,
    /// `SENS` = 1: 1/8 sensitivity (up to ~6.7 klx).
    pub low_sensitivity: bool,
    /// Integration time.
    pub integration_time: IntegrationTime35,
    /// Interrupt persistence.
    pub persistence: Persistence,
    /// Enable the threshold interrupt (on the ALS channel).
    pub interrupt: bool,
}

impl Default for Config35 {
    fn default() -> Self {
        Self {
            gain: false,
            double_gain: false,
            low_sensitivity: true,
            integration_time: IntegrationTime35::Ms100,
            persistence: Persistence::One,
            interrupt: false,
        }
    }
}

impl Config35 {
    /// `ALS_CONF` value (WHITE channel enabled).
    pub const fn word(&self, shutdown: bool) -> u16 {
        (if self.low_sensitivity { 1 << 12 } else { 0 })
            | (if self.double_gain { 1 << 11 } else { 0 })
            | (if self.gain { 1 << 10 } else { 0 })
            | (self.integration_time as u16) << 6
            | (self.persistence as u16) << 4
            | 1 << 2
            | (if self.interrupt { 1 << 1 } else { 0 })
            | (if shutdown { 1 } else { 0 })
    }

    /// Wait after power-on: start-up + IT + 10 %.
    pub const fn wait_us(&self) -> WaitUs {
        STARTUP_US + self.integration_time.us() * 11 / 10
    }

    /// Resolution in 1/10 mlx per count (0.4 mlx at GAIN=DG=1, SENS=0,
    /// 800 ms).
    pub const fn resolution_decimillilux(&self) -> u32 {
        4 * (if self.gain { 1 } else { 2 })
            * (if self.double_gain { 1 } else { 2 })
            * (if self.low_sensitivity { 8 } else { 1 })
            * (800_000 / self.integration_time.us())
    }

    /// Counts → milli-lux.
    pub const fn millilux(&self, counts: u16) -> u32 {
        counts as u32 * self.resolution_decimillilux() / 10
    }

    /// Milli-lux → counts (saturating).
    pub const fn counts(&self, millilux: u32) -> u16 {
        saturate_u16(millilux as u64 * 10 / self.resolution_decimillilux() as u64)
    }
}

macro_rules! driver {
    ($($async_:ident)?, $($await_:tt)*) => {
        /// VEML6031X00 driver (active force mode).
        pub struct Veml6031<I2C> {
            i2c: I2C,
            address: u8,
            config: Config31,
        }

        impl<I2C> Veml6031<I2C> {
            /// Create a driver; no bus traffic.
            pub const fn new(i2c: I2C, address: u8) -> Self {
                Self {
                    i2c,
                    address,
                    config: Config31 {
                        gain: Gain31::X1,
                        integration_time: IntegrationTime31::Ms100,
                        quarter_pd: false,
                        persistence: Persistence::One,
                        interrupt: false,
                    },
                }
            }

            /// Set the configuration (used from the next trigger).
            pub const fn with_config(mut self, config: Config31) -> Self {
                self.config = config;
                self
            }

            /// Change the configuration.
            pub fn set_config(&mut self, config: Config31) {
                self.config = config;
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.i2c
            }
        }

        impl<I2C: I2c> Veml6031<I2C> {
            $($async_)? fn write3(&mut self, reg: u8, data: [u8; 2]) -> Result<(), Error<I2C::Error>> {
                self.i2c
                    .write(self.address, &[reg, data[0], data[1]])
                    $($await_)*
                    .map_err(Error::bus)
            }

            $($async_)? fn read_byte(&mut self, reg: u8) -> Result<u8, Error<I2C::Error>> {
                let mut buf = [0u8; 1];
                self.i2c
                    .write_read(self.address, &[reg], &mut buf)
                    $($await_)*
                    .map_err(Error::bus)?;
                Ok(buf[0])
            }

            /// Check `ID_L` = 0x01.
            pub $($async_)? fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
                let mut id = [0u8; 2];
                self.i2c
                    .write_read(self.address, &[R31_ID], &mut id)
                    $($await_)*
                    .map_err(Error::bus)?;
                if id[0] != VEML6031_ID {
                    return Err(Error::UnexpectedId(u32::from(u16::from_le_bytes(id))));
                }
                Ok(())
            }

            /// Program the interrupt window in milli-lux.
            pub $($async_)? fn set_thresholds(&mut self, low: u32, high: u32) -> Result<(), Error<I2C::Error>> {
                let low = self.config.counts(low).to_le_bytes();
                let high = self.config.counts(high).to_le_bytes();
                self.write3(R31_THDL, low) $($await_)* ?;
                self.write3(R31_THDH, high) $($await_)*
            }

            /// Read and clear the threshold flags.
            pub $($async_)? fn interrupt_status(&mut self) -> Result<InterruptStatus, Error<I2C::Error>> {
                let f = self.read_byte(R31_INT) $($await_)* ?;
                Ok(InterruptStatus {
                    low: f & F31_LOW != 0,
                    high: f & F31_HIGH != 0,
                })
            }
        }

        impl<I2C: I2c> Sensor for Veml6031<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// Power on and trigger one active-force conversion.
            $($async_)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                // ALS_ON_0 and ALS_ON_1 must change in the same transaction.
                self.write3(R31_CONF0, self.config.bytes(true)) $($await_)* ?;
                Ok(self.config.wait_us())
            }

            /// [`Error::NotReady`] until `AF_DATA_READY`; then reads ALS + IR
            /// and shuts down.
            $($async_)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let flags = self.read_byte(R31_INT) $($await_)* ?;
                if flags & F31_READY == 0 {
                    return Err(Error::NotReady);
                }
                let mut buf = [0u8; 4];
                self.i2c
                    .write_read(self.address, &[R31_DATA], &mut buf)
                    $($await_)*
                    .map_err(Error::bus)?;
                self.write3(R31_CONF0, self.config.bytes(false)) $($await_)* ?;
                let als = u16::from_le_bytes([buf[0], buf[1]]);
                Ok(Measurement {
                    illuminance: self.config.millilux(als),
                    als,
                    aux: u16::from_le_bytes([buf[2], buf[3]]),
                })
            }
        }

        impl<I2C: I2c> LowPower for Veml6031<I2C> {
            type BusError = I2C::Error;

            /// Shut down ALS and IR (`ALS_ON_0` = `ALS_ON_1` = 1).
            $($async_)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write3(R31_CONF0, self.config.bytes(false)) $($await_)*
            }

            /// Nothing to do: each trigger powers the part on.
            $($async_)? fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                Ok(0)
            }
        }

        /// VEML6035 driver.
        pub struct Veml6035<I2C> {
            i2c: I2C,
            config: Config35,
            psm: Option<PsmWait>,
            continuous: bool,
        }

        impl<I2C> Veml6035<I2C> {
            /// Create a driver (fixed address 0x29); no bus traffic.
            pub fn new(i2c: I2C) -> Self {
                Self {
                    i2c,
                    config: Config35::default(),
                    psm: None,
                    continuous: false,
                }
            }

            /// Set the configuration (used from the next start).
            pub fn with_config(mut self, config: Config35) -> Self {
                self.config = config;
                self
            }

            /// Change the configuration.
            pub fn set_config(&mut self, config: Config35) {
                self.config = config;
            }

            /// Refresh period in continuous mode.
            pub const fn refresh_us(&self) -> u32 {
                let it = self.config.integration_time.us();
                match self.psm {
                    Some(w) => it + (400_000 << w as u32),
                    None => it,
                }
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.i2c
            }
        }

        impl<I2C: I2c> Veml6035<I2C> {
            $($async_)? fn read_reg(&mut self, reg: u8) -> Result<u16, Error<I2C::Error>> {
                let mut buf = [0u8; 2];
                self.i2c
                    .write_read(ADDRESS, &[reg], &mut buf)
                    $($await_)*
                    .map_err(Error::bus)?;
                Ok(u16::from_le_bytes(buf))
            }

            $($async_)? fn write_reg(&mut self, reg: u8, value: u16) -> Result<(), Error<I2C::Error>> {
                let [lo, hi] = value.to_le_bytes();
                self.i2c
                    .write(ADDRESS, &[reg, lo, hi])
                    $($await_)*
                    .map_err(Error::bus)
            }

            /// The VEML6035 has no ID register: check that the reserved
            /// configuration bits 15:13 read back as zero.
            pub $($async_)? fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
                let conf = self.read_reg(R35_CONF) $($await_)* ?;
                if conf & 0xE000 != 0 {
                    return Err(Error::UnexpectedId(u32::from(conf)));
                }
                Ok(())
            }

            /// Program the interrupt window in milli-lux.
            pub $($async_)? fn set_thresholds(&mut self, low: u32, high: u32) -> Result<(), Error<I2C::Error>> {
                let low = self.config.counts(low);
                let high = self.config.counts(high);
                self.write_reg(R35_WL, low) $($await_)* ?;
                self.write_reg(R35_WH, high) $($await_)*
            }

            /// Read and clear the threshold flags.
            pub $($async_)? fn interrupt_status(&mut self) -> Result<InterruptStatus, Error<I2C::Error>> {
                let v = self.read_reg(R35_INT) $($await_)* ?;
                Ok(InterruptStatus {
                    low: v & F35_LOW != 0,
                    high: v & F35_HIGH != 0,
                })
            }

            /// Run continuously, optionally with a power-saving wait.
            pub $($async_)? fn start_continuous(&mut self, psm: Option<PsmWait>) -> Result<WaitUs, Error<I2C::Error>> {
                let word = match psm {
                    Some(w) => (w as u16) << 1 | 1,
                    None => 0,
                };
                self.write_reg(R35_PSM, word) $($await_)* ?;
                self.write_reg(R35_CONF, self.config.word(false)) $($await_)* ?;
                self.psm = psm;
                self.continuous = true;
                Ok(self.config.wait_us())
            }

            /// Back to one-shot operation (shuts down).
            pub $($async_)? fn stop_continuous(&mut self) -> Result<(), Error<I2C::Error>> {
                self.continuous = false;
                self.write_reg(R35_CONF, self.config.word(true)) $($await_)*
            }
        }

        impl<I2C: I2c> Sensor for Veml6035<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// One-shot: power on and integrate. Continuous: returns 0.
            $($async_)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                if self.continuous {
                    return Ok(0);
                }
                self.write_reg(R35_CONF, self.config.word(false)) $($await_)* ?;
                Ok(self.config.wait_us())
            }

            /// Read ALS + WHITE; in one-shot mode shut down after.
            $($async_)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let als = self.read_reg(R35_ALS) $($await_)* ?;
                let white = self.read_reg(R35_WHITE) $($await_)* ?;
                if !self.continuous {
                    self.write_reg(R35_CONF, self.config.word(true)) $($await_)* ?;
                }
                Ok(Measurement {
                    illuminance: self.config.millilux(als),
                    als,
                    aux: white,
                })
            }
        }

        impl<I2C: I2c> LowPower for Veml6035<I2C> {
            type BusError = I2C::Error;

            /// Set SD.
            $($async_)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write_reg(R35_CONF, self.config.word(true)) $($await_)*
            }

            /// Continuous mode: power back on. One-shot: nothing to do.
            $($async_)? fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                if !self.continuous {
                    return Ok(0);
                }
                self.write_reg(R35_CONF, self.config.word(false)) $($await_)* ?;
                Ok(self.config.wait_us())
            }
        }
    };
}

driver!(,);

/// Async drivers.
pub mod asynch {
    use super::{
        ADDRESS, Config31, Config35, F31_HIGH, F31_LOW, F31_READY, F35_HIGH, F35_LOW, Gain31,
        IntegrationTime31, InterruptStatus, Measurement, Persistence, PsmWait, R31_CONF0, R31_DATA,
        R31_ID, R31_INT, R31_THDH, R31_THDL, R35_ALS, R35_CONF, R35_INT, R35_PSM, R35_WH,
        R35_WHITE, R35_WL, VEML6031_ID,
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
    fn veml6031_resolution_tables() {
        let c = |gain, integration_time, quarter_pd| Config31 {
            gain,
            integration_time,
            quarter_pd,
            ..Config31::default()
        };
        // Table 11: ×2 / 400 ms → 0.0026 lx; max 168 lx.
        let best = c(Gain31::X2, IntegrationTime31::Ms400, false);
        assert_eq!(best.millilux(0xFFFF), 168_293);
        // ×0.66 / 400 ms → 0.0078 lx.
        assert_eq!(
            c(Gain31::X0_66, IntegrationTime31::Ms400, false).millilux(1_000),
            7_781
        );
        // Table 12: ×0.5 / 6.25 ms / ¼ PD → 2.6296 lx; max 172 333 lx.
        let wide = c(Gain31::X0_5, IntegrationTime31::Us6250, true);
        assert_eq!(wide.millilux(1), 2_629);
        assert_eq!(wide.millilux(0xFFFF), 172_332_933);
        assert_eq!(wide.counts(2_630), 1);
    }

    #[test]
    fn veml6031_config_bytes() {
        let c = Config31 {
            gain: Gain31::X0_5,
            integration_time: IntegrationTime31::Ms400,
            quarter_pd: true,
            persistence: Persistence::Four,
            interrupt: true,
        };
        assert_eq!(c.bytes(true), [0x7E, 0x5D]);
        assert_eq!(c.bytes(false), [0x7B, 0xDC]);
        assert_eq!(IntegrationTime31::Us3125.us(), 3_125);
        assert_eq!(IntegrationTime31::Ms400.us(), 400_000);
    }

    #[test]
    fn veml6031_active_force() {
        let a = ADDRESS;
        let expectations = [
            Transaction::write_read(a, vec![0x14], vec![0x01, 0x00]),
            Transaction::write(a, vec![0x00, 0x5C, 0x01]),
            Transaction::write_read(a, vec![0x17], vec![0x00]),
            Transaction::write_read(a, vec![0x17], vec![0x08]),
            Transaction::write_read(a, vec![0x10], vec![0xE8, 0x03, 0x10, 0x00]),
            Transaction::write(a, vec![0x00, 0x59, 0x80]),
            Transaction::write_read(a, vec![0x14], vec![0x81, 0xC4]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut veml = Veml6031::new(&mut i2c, a);
        veml.probe().unwrap();
        assert_eq!(veml.start_measurement().unwrap(), 114_000);
        assert_eq!(veml.read_measurement(), Err(Error::NotReady));
        let m = veml.read_measurement().unwrap();
        // ×1, 100 ms: 2.568 × 2 × 4 = 20.544 mlx/count.
        assert_eq!(m.illuminance, 20_544);
        assert_eq!((m.als, m.aux), (1_000, 16));
        assert_eq!(veml.probe(), Err(Error::UnexpectedId(0xC481)));
        i2c.done();
    }

    #[test]
    fn veml6035_resolution_and_cycle() {
        // Datasheet example: 1480 counts, GAIN=1, DG=1, SENS=1, 100 ms → 37.888 lx.
        let c = Config35 {
            gain: true,
            double_gain: true,
            low_sensitivity: true,
            integration_time: IntegrationTime35::Ms100,
            ..Config35::default()
        };
        assert_eq!(c.millilux(1480), 37_888);
        let best = Config35 {
            low_sensitivity: false,
            integration_time: IntegrationTime35::Ms800,
            ..c
        };
        assert_eq!(best.resolution_decimillilux(), 4);
        // Default (GAIN=DG=0, SENS=1, 100 ms): 0.1024 lx/count.
        assert_eq!(Config35::default().resolution_decimillilux(), 1_024);

        let expectations = [
            Transaction::write_read(ADDRESS, vec![0x00], vec![0x01, 0x00]),
            Transaction::write(ADDRESS, vec![0x00, 0x04, 0x1C]),
            Transaction::write_read(ADDRESS, vec![0x04], vec![0xC8, 0x05]),
            Transaction::write_read(ADDRESS, vec![0x05], vec![0x00, 0x01]),
            Transaction::write(ADDRESS, vec![0x00, 0x05, 0x1C]),
            Transaction::write(ADDRESS, vec![0x02, 0x0A, 0x00]),
            Transaction::write(ADDRESS, vec![0x01, 0xFF, 0xFF]),
            Transaction::write_read(ADDRESS, vec![0x06], vec![0x00, 0x40]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut veml = Veml6035::new(&mut i2c).with_config(c);
        veml.probe().unwrap();
        assert_eq!(veml.start_measurement().unwrap(), 114_000);
        let m = veml.read_measurement().unwrap();
        assert_eq!((m.illuminance, m.als, m.aux), (37_888, 1480, 256));
        veml.set_thresholds(256, u32::MAX).unwrap();
        let st = veml.interrupt_status().unwrap();
        assert!(!st.low && st.high);
        i2c.done();
    }

    #[test]
    fn async_veml6035_continuous() {
        use zigbee_sensor::asynch::{LowPower as _, Sensor as _};

        struct RecordingDelay(u32);
        impl embedded_hal_async::delay::DelayNs for RecordingDelay {
            async fn delay_ns(&mut self, ns: u32) {
                self.0 += ns / 1_000;
            }
        }

        let expectations = [
            Transaction::write(ADDRESS, vec![0x03, 0x07, 0x00]),
            Transaction::write(ADDRESS, vec![0x00, 0x04, 0x10]),
            Transaction::write_read(ADDRESS, vec![0x04], vec![0x0A, 0x00]),
            Transaction::write_read(ADDRESS, vec![0x05], vec![0x00, 0x00]),
            Transaction::write(ADDRESS, vec![0x00, 0x05, 0x10]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut veml = asynch::Veml6035::new(&mut i2c);
        assert_eq!(
            block_on(veml.start_continuous(Some(PsmWait::Ms3200))).unwrap(),
            114_000
        );
        assert_eq!(veml.refresh_us(), 3_300_000);
        let mut delay = RecordingDelay(0);
        let m = block_on(veml.measure(&mut delay)).unwrap();
        assert_eq!(m.illuminance, 1_024);
        assert_eq!(delay.0, 0);
        block_on(veml.sleep()).unwrap();
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
