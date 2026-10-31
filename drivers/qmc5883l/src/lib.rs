//! QST QMC5883L and QMC5883P 3-axis magnetometers over I2C.
//!
//! The two parts share a name but not a register map, address or ID;
//! pick the [`Variant`] when creating the driver:
//!
//! | | QMC5883L | QMC5883P |
//! |---|---|---|
//! | Address | 0x0D | 0x2C |
//! | Chip ID | reg 0x0D = 0xFF | reg 0x00 = 0x80 |
//! | Ranges | ±2 / ±8 gauss | ±2 / ±8 / ±12 / ±30 gauss |
//! | Single measurement | no (emulated) | yes |
//! | DRDY pin | yes | no |
//!
//! By default every
//! [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//! takes one measurement and the part returns to standby/suspend. The
//! QMC5883L has no single mode, so the driver switches it to continuous
//! mode at 200 Hz and back to standby in `read_measurement()`. With
//! [`Config::continuous`] the part measures periodically at
//! [`Config::data_rate`]. Neither part has a threshold interrupt.
//!
//! The field is reported as [`Quantity::MagneticField`] in nT.
//!
//! References: QST "QMC5883L Datasheet" Doc 13-52-04 Rev A; QST
//! "QMC5883P Datasheet" Doc 13-52-19 Rev C.
//!
//! ```ignore
//! let mut mag = Qmc5883::new(i2c, Variant::Qmc5883p);
//! mag.probe()?;
//! mag.configure()?;
//! let wait = mag.start_measurement()?;
//! delay_us(wait);
//! let m = mag.read_measurement()?; // m.field in nT
//! ```

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, Vector3, WaitUs};

/// QMC5883L register addresses.
pub mod reg_l {
    #![allow(missing_docs)]
    pub const XOUT_L: u8 = 0x00;
    pub const STATUS: u8 = 0x06;
    pub const CONTROL1: u8 = 0x09;
    pub const CONTROL2: u8 = 0x0A;
    pub const SET_RESET_PERIOD: u8 = 0x0B;
    pub const CHIP_ID: u8 = 0x0D;
}

/// QMC5883P register addresses.
pub mod reg_p {
    #![allow(missing_docs)]
    pub const CHIP_ID: u8 = 0x00;
    pub const XOUT_L: u8 = 0x01;
    pub const STATUS: u8 = 0x09;
    pub const CONTROL1: u8 = 0x0A;
    pub const CONTROL2: u8 = 0x0B;
    /// Axis sign definition; the datasheet examples write 0x06.
    pub const AXIS_SIGN: u8 = 0x29;
}

const MODE_STANDBY: u8 = 0b00;
/// QMC5883L continuous / QMC5883P normal mode (periodic at ODR).
const MODE_PERIODIC: u8 = 0b01;
const MODE_P_SINGLE: u8 = 0b10;
const STATUS_DRDY: u8 = 0x01;
const STATUS_OVERFLOW: u8 = 0x02;
const SOFT_RST: u8 = 0x80;
const L_INT_DISABLE: u8 = 0x01;
const P_AXIS_SIGN: u8 = 0x06;
/// Recommended `SET/RESET Period` (QMC5883L).
const L_FBR: u8 = 0x01;

/// Part variant.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// QMC5883L.
    Qmc5883l,
    /// QMC5883P.
    Qmc5883p,
}

impl Variant {
    /// Fixed I2C address.
    pub const fn address(self) -> u8 {
        match self {
            Self::Qmc5883l => 0x0D,
            Self::Qmc5883p => 0x2C,
        }
    }

    /// Expected chip ID.
    pub const fn chip_id(self) -> u8 {
        match self {
            Self::Qmc5883l => 0xFF,
            Self::Qmc5883p => 0x80,
        }
    }

    /// Time after power-on or soft reset before the next command (`PORT`).
    pub const fn reset_us(self) -> WaitUs {
        match self {
            Self::Qmc5883l => 350,
            Self::Qmc5883p => 250,
        }
    }

    const fn status_reg(self) -> u8 {
        match self {
            Self::Qmc5883l => reg_l::STATUS,
            Self::Qmc5883p => reg_p::STATUS,
        }
    }

    const fn data_reg(self) -> u8 {
        match self {
            Self::Qmc5883l => reg_l::XOUT_L,
            Self::Qmc5883p => reg_p::XOUT_L,
        }
    }

    const fn control1_reg(self) -> u8 {
        match self {
            Self::Qmc5883l => reg_l::CONTROL1,
            Self::Qmc5883p => reg_p::CONTROL1,
        }
    }
}

/// Full-scale range.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Range {
    /// ±2 gauss (L: 12000, P: 15000 LSB/gauss).
    G2,
    /// ±8 gauss (L: 3000, P: 3750 LSB/gauss).
    #[default]
    G8,
    /// ±12 gauss, 2500 LSB/gauss (QMC5883P only).
    G12,
    /// ±30 gauss, 1000 LSB/gauss (QMC5883P only).
    G30,
}

impl Range {
    /// Sensitivity in LSB/gauss, `None` if the variant lacks the range.
    pub const fn lsb_per_gauss(self, variant: Variant) -> Option<i32> {
        match (variant, self) {
            (Variant::Qmc5883l, Self::G2) => Some(12_000),
            (Variant::Qmc5883l, Self::G8) => Some(3_000),
            (Variant::Qmc5883l, _) => None,
            (Variant::Qmc5883p, Self::G2) => Some(15_000),
            (Variant::Qmc5883p, Self::G8) => Some(3_750),
            (Variant::Qmc5883p, Self::G12) => Some(2_500),
            (Variant::Qmc5883p, Self::G30) => Some(1_000),
        }
    }

    const fn code(self, variant: Variant) -> u8 {
        match (variant, self) {
            (Variant::Qmc5883l, Self::G2) => 0,
            (Variant::Qmc5883l, _) => 1,
            (Variant::Qmc5883p, Self::G30) => 0,
            (Variant::Qmc5883p, Self::G12) => 1,
            (Variant::Qmc5883p, Self::G8) => 2,
            (Variant::Qmc5883p, Self::G2) => 3,
        }
    }
}

/// Periodic output data rate (`ODR`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DataRate {
    /// 10 Hz (recommended for compassing).
    #[default]
    Hz10 = 0,
    /// 50 Hz.
    Hz50 = 1,
    /// 100 Hz.
    Hz100 = 2,
    /// 200 Hz.
    Hz200 = 3,
}

impl DataRate {
    /// Output period.
    pub const fn period_us(self) -> WaitUs {
        match self {
            Self::Hz10 => 100_000,
            Self::Hz50 => 20_000,
            Self::Hz100 => 10_000,
            Self::Hz200 => 5_000,
        }
    }
}

/// Over-sampling (`OSR` on the L, `OSR1` on the P): more filtering means
/// less noise and more current.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Oversampling {
    /// L: 512, P: 8 (reset value).
    #[default]
    Highest = 0,
    /// L: 256, P: 4.
    High = 1,
    /// L: 128, P: 2.
    Low = 2,
    /// L: 64, P: 1.
    Lowest = 3,
}

impl Oversampling {
    /// QMC5883P `OSR1` ratio.
    pub const fn p_ratio(self) -> u32 {
        8 >> self as u32
    }
}

/// QMC5883P down-sampling filter depth (`OSR2`); ignored on the L.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DownSampling {
    /// 1 (reset value).
    #[default]
    X1 = 0,
    /// 2.
    X2 = 1,
    /// 4.
    X4 = 2,
    /// 8.
    X8 = 3,
}

/// QMC5883P set/reset (offset renewal) mode; ignored on the L.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SetReset {
    /// Set and reset on every measurement (reset value).
    #[default]
    On = 0,
    /// Set only; the offset is not renewed.
    SetOnly = 1,
    /// Off; the offset is not renewed.
    Off = 2,
}

/// Measurement configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Full-scale range (default ±8 gauss).
    pub range: Range,
    /// Periodic rate with `continuous` (default 10 Hz).
    pub data_rate: DataRate,
    /// Over-sampling (default highest).
    pub oversampling: Oversampling,
    /// QMC5883P `OSR2` (default 1).
    pub down_sampling: DownSampling,
    /// QMC5883P set/reset mode (default on).
    pub set_reset: SetReset,
    /// Measure periodically at `data_rate` instead of on request
    /// (default off).
    pub continuous: bool,
    /// QMC5883L: drive data ready on the DRDY pin (default on, reset
    /// value).
    pub drdy_pin: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            range: Range::G8,
            data_rate: DataRate::Hz10,
            oversampling: Oversampling::Highest,
            down_sampling: DownSampling::X1,
            set_reset: SetReset::On,
            continuous: false,
            drdy_pin: true,
        }
    }
}

/// QMC5883P single-measurement estimate per `OSR1 × OSR2` step. The
/// datasheet gives no conversion time; 700 µs matches the 1500 Hz
/// maximum rate at `OSR1` = 1 and the 5 ms self-test wait at `OSR1` = 8.
pub const P_STEP_US: WaitUs = 700;

/// Time from `start_measurement()` to data in on-request mode.
pub fn measurement_us(variant: Variant, config: &Config) -> WaitUs {
    match variant {
        Variant::Qmc5883l => DataRate::Hz200.period_us(),
        Variant::Qmc5883p => {
            P_STEP_US * config.oversampling.p_ratio() * (1 << config.down_sampling as u32)
        }
    }
}

/// `Control 1` value for `mode`. The QMC5883L uses 200 Hz when emulating
/// single measurements.
pub fn control1(variant: Variant, config: &Config, mode: u8) -> u8 {
    let odr = if config.continuous || variant == Variant::Qmc5883p {
        config.data_rate
    } else {
        DataRate::Hz200
    };
    let filter = match variant {
        Variant::Qmc5883l => (config.oversampling as u8) << 6 | config.range.code(variant) << 4,
        Variant::Qmc5883p => (config.down_sampling as u8) << 6 | (config.oversampling as u8) << 4,
    };
    filter | (odr as u8) << 2 | mode
}

/// Register writes for `configure()`: standby/suspend first, then the
/// settings, then the final mode. `None` if the range is unsupported.
pub fn config_writes(variant: Variant, config: &Config) -> Option<[(u8, u8); 4]> {
    config.range.lsb_per_gauss(variant)?;
    let mode = if config.continuous {
        MODE_PERIODIC
    } else {
        MODE_STANDBY
    };
    let c1 = variant.control1_reg();
    Some(match variant {
        Variant::Qmc5883l => [
            (c1, control1(variant, config, MODE_STANDBY)),
            (reg_l::SET_RESET_PERIOD, L_FBR),
            (
                reg_l::CONTROL2,
                if config.drdy_pin { 0 } else { L_INT_DISABLE },
            ),
            (c1, control1(variant, config, mode)),
        ],
        Variant::Qmc5883p => [
            (c1, control1(variant, config, MODE_STANDBY)),
            (reg_p::AXIS_SIGN, P_AXIS_SIGN),
            (
                reg_p::CONTROL2,
                config.range.code(variant) << 2 | config.set_reset as u8,
            ),
            (c1, control1(variant, config, mode)),
        ],
    })
}

/// One magnetometer sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Flux density in nT.
    pub field: Vector3,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::MagneticField(self.field));
    }
}

/// Output count to nT (1 gauss = 100 000 nT), rounded to nearest.
pub const fn raw_to_nt(raw: i16, lsb_per_gauss: i32) -> i32 {
    let num = raw as i64 * 100_000;
    let half = (lsb_per_gauss / 2) as i64;
    let r = if num >= 0 { num + half } else { num - half };
    (r / lsb_per_gauss as i64) as i32
}

/// Decode X, Y, Z (little-endian) to nT.
pub fn decode(b: &[u8; 6], lsb_per_gauss: i32) -> Measurement {
    let axis = |i: usize| raw_to_nt(i16::from_le_bytes([b[i], b[i + 1]]), lsb_per_gauss);
    Measurement {
        field: Vector3 {
            x: axis(0),
            y: axis(2),
            z: axis(4),
        },
    }
}

macro_rules! driver {
    ($doc:literal, $I2c:path, $mode:ident, [$($async:tt)?], [$($aw:tt)*]) => {
        #[doc = $doc]
        #[derive(Debug)]
        pub struct Qmc5883<I2C> {
            i2c: I2C,
            variant: Variant,
            config: Config,
        }

        impl<I2C> Qmc5883<I2C> {
            /// Create a driver at the variant's address; no bus traffic.
            pub fn new(i2c: I2C, variant: Variant) -> Self {
                Self {
                    i2c,
                    variant,
                    config: Config::default(),
                }
            }

            /// Set the configuration used by `configure()`.
            pub fn with_config(mut self, config: Config) -> Self {
                self.config = config;
                self
            }

            /// Part variant.
            pub fn variant(&self) -> Variant {
                self.variant
            }

            /// Current configuration.
            pub fn config(&self) -> &Config {
                &self.config
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.i2c
            }
        }

        impl<I2C: $I2c> Qmc5883<I2C> {
            /// Read consecutive registers.
            pub $($async)? fn read_registers(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
                self.i2c.write_read(self.variant.address(), &[reg], buf) $($aw)* .map_err(Error::bus)
            }

            /// Write one register.
            pub $($async)? fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
                self.i2c.write(self.variant.address(), &[reg, value]) $($aw)* .map_err(Error::bus)
            }

            /// Check the chip ID (QMC5883L: 0xFF, which is a weak check;
            /// QMC5883P: 0x80).
            pub $($async)? fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
                let reg = match self.variant {
                    Variant::Qmc5883l => reg_l::CHIP_ID,
                    Variant::Qmc5883p => reg_p::CHIP_ID,
                };
                let mut id = [0u8];
                self.read_registers(reg, &mut id) $($aw)* ?;
                if id[0] != self.variant.chip_id() {
                    return Err(Error::UnexpectedId(u32::from(id[0])));
                }
                Ok(())
            }

            /// Soft reset to standby/suspend with default registers.
            /// Wait the returned time, then call `configure()`.
            pub $($async)? fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                let reg = match self.variant {
                    Variant::Qmc5883l => reg_l::CONTROL2,
                    Variant::Qmc5883p => reg_p::CONTROL2,
                };
                self.write_register(reg, SOFT_RST) $($aw)* ?;
                Ok(self.variant.reset_us())
            }

            /// Write the configuration. Returns 0 in on-request mode or the
            /// first sample period in continuous mode;
            /// [`Error::Unsupported`] for a range the variant lacks.
            pub $($async)? fn configure(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                let writes = config_writes(self.variant, &self.config).ok_or(Error::Unsupported)?;
                for (r, v) in writes {
                    self.write_register(r, v) $($aw)* ?;
                }
                Ok(if self.config.continuous {
                    self.config.data_rate.period_us()
                } else {
                    0
                })
            }

            /// Store a new configuration and write it.
            pub $($async)? fn set_config(&mut self, config: Config) -> Result<WaitUs, Error<I2C::Error>> {
                self.config = config;
                self.configure() $($aw)*
            }

            /// Standby (L, ≈3 µA) / suspend (P, 22 µA).
            pub $($async)? fn power_down(&mut self) -> Result<(), Error<I2C::Error>> {
                self.set_mode(MODE_STANDBY) $($aw)*
            }

            $($async)? fn set_mode(&mut self, mode: u8) -> Result<(), Error<I2C::Error>> {
                let v = control1(self.variant, &self.config, mode);
                self.write_register(self.variant.control1_reg(), v) $($aw)*
            }
        }

        impl<I2C: $I2c> zigbee_sensor::$mode::Sensor for Qmc5883<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// On request: start one measurement (P: single mode; L:
            /// continuous mode at 200 Hz) and return its duration.
            /// Continuous mode: 0.
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                if self.config.continuous {
                    return Ok(0);
                }
                let mode = match self.variant {
                    Variant::Qmc5883l => MODE_PERIODIC,
                    Variant::Qmc5883p => MODE_P_SINGLE,
                };
                self.set_mode(mode) $($aw)* ?;
                Ok(measurement_us(self.variant, &self.config))
            }

            /// [`Error::NotReady`] until `DRDY`; [`Error::InvalidData`] if
            /// an axis overflowed (choose a larger range). On the
            /// QMC5883L in on-request mode this also returns the part to
            /// standby.
            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let lsb = self.config.range.lsb_per_gauss(self.variant).ok_or(Error::Unsupported)?;
                let mut status = [0u8];
                self.read_registers(self.variant.status_reg(), &mut status) $($aw)* ?;
                if status[0] & STATUS_DRDY == 0 {
                    return Err(Error::NotReady);
                }
                let mut b = [0u8; 6];
                self.read_registers(self.variant.data_reg(), &mut b) $($aw)* ?;
                if self.variant == Variant::Qmc5883l && !self.config.continuous {
                    self.set_mode(MODE_STANDBY) $($aw)* ?;
                }
                if status[0] & STATUS_OVERFLOW != 0 {
                    return Err(Error::InvalidData);
                }
                Ok(decode(&b, lsb))
            }
        }

        impl<I2C: $I2c> zigbee_sensor::$mode::LowPower for Qmc5883<I2C> {
            type BusError = I2C::Error;

            /// [`Self::power_down`].
            $($async)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.power_down() $($aw)*
            }

            /// Restart periodic measurements in continuous mode; nothing
            /// to do on request.
            $($async)? fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                if !self.config.continuous {
                    return Ok(0);
                }
                self.set_mode(MODE_PERIODIC) $($aw)* ?;
                Ok(self.config.data_rate.period_us())
            }
        }
    };
}

driver!("Blocking driver.", embedded_hal::i2c::I2c, blocking, [], []);

/// Async driver.
pub mod asynch {
    use super::*;

    driver!(
        "Async twin of [`super::Qmc5883`].",
        embedded_hal_async::i2c::I2c,
        asynch,
        [async],
        [.await]
    );
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction as I2cTx};
    use std::vec;
    use zigbee_sensor::blocking::{LowPower, Sensor};

    const L: u8 = 0x0D;
    const P: u8 = 0x2C;

    fn rd(a: u8, reg: u8, data: &[u8]) -> I2cTx {
        I2cTx::write_read(a, vec![reg], data.to_vec())
    }
    fn wr(a: u8, reg: u8, value: u8) -> I2cTx {
        I2cTx::write(a, vec![reg, value])
    }

    #[test]
    fn datasheet_sensitivities() {
        let l8 = Range::G8.lsb_per_gauss(Variant::Qmc5883l).unwrap();
        assert_eq!(raw_to_nt(3_000, l8), 100_000);
        assert_eq!(raw_to_nt(-32_768, l8), -1_092_267);
        assert_eq!(raw_to_nt(1, 12_000), 8);
        assert_eq!(raw_to_nt(1_000, 1_000), 100_000);
        assert_eq!(raw_to_nt(-15_000, 15_000), -100_000);
        assert_eq!(Range::G30.lsb_per_gauss(Variant::Qmc5883l), None);
        // X = 3750, Y = -3750, Z = 1 at P ±8 gauss
        let m = decode(&[0xA6, 0x0E, 0x5A, 0xF1, 0x01, 0x00], 3_750);
        assert_eq!(
            m.field,
            Vector3 {
                x: 100_000,
                y: -100_000,
                z: 27
            }
        );
        assert_eq!(measurement_us(Variant::Qmc5883p, &Config::default()), 5_600);
    }

    #[test]
    fn probe_both_variants() {
        let mut i2c = Mock::new(&[
            rd(L, 0x0D, &[0xFF]),
            rd(P, 0x00, &[0x80]),
            rd(P, 0x00, &[0xFF]),
        ]);
        Qmc5883::new(&mut i2c, Variant::Qmc5883l).probe().unwrap();
        let mut p = Qmc5883::new(&mut i2c, Variant::Qmc5883p);
        p.probe().unwrap();
        assert_eq!(p.probe(), Err(Error::UnexpectedId(0xFF)));
        i2c.done();
    }

    #[test]
    fn l_single_measurement_emulated() {
        let mut i2c = Mock::new(&[
            wr(L, 0x09, 0x1C),
            wr(L, 0x0B, 0x01),
            wr(L, 0x0A, 0x00),
            wr(L, 0x09, 0x1C),
            // datasheet example 0x1D: OSR 512, ±8 G, 200 Hz, continuous
            wr(L, 0x09, 0x1D),
            rd(L, 0x06, &[0x00]),
            rd(L, 0x06, &[0x01]),
            rd(L, 0x00, &[0xB8, 0x0B, 0, 0, 0, 0]),
            wr(L, 0x09, 0x1C),
            wr(L, 0x09, 0x1C),
        ]);
        let mut mag = Qmc5883::new(&mut i2c, Variant::Qmc5883l);
        assert_eq!(mag.configure().unwrap(), 0);
        assert_eq!(mag.start_measurement().unwrap(), 5_000);
        assert_eq!(mag.read_measurement(), Err(Error::NotReady));
        assert_eq!(mag.read_measurement().unwrap().field.x, 100_000);
        mag.sleep().unwrap();
        assert_eq!(mag.wake().unwrap(), 0);
        i2c.done();
    }

    #[test]
    fn p_normal_mode_overflow_and_reset() {
        let cfg = Config {
            range: Range::G30,
            data_rate: DataRate::Hz200,
            oversampling: Oversampling::Lowest,
            down_sampling: DownSampling::X8,
            set_reset: SetReset::SetOnly,
            continuous: true,
            drdy_pin: false,
        };
        let mut i2c = Mock::new(&[
            wr(P, 0x0A, 0xFC),
            wr(P, 0x29, 0x06),
            wr(P, 0x0B, 0x01),
            // OSR2 8, OSR1 1, 200 Hz, normal mode
            wr(P, 0x0A, 0xFD),
            rd(P, 0x09, &[0x01]),
            rd(P, 0x01, &[0xE8, 0x03, 0, 0, 0, 0]),
            rd(P, 0x09, &[0x03]),
            rd(P, 0x01, &[0x30, 0x75, 0, 0, 0, 0]),
            wr(P, 0x0A, 0xFC),
            wr(P, 0x0A, 0xFD),
            wr(P, 0x0B, 0x80),
        ]);
        let mut mag = Qmc5883::new(&mut i2c, Variant::Qmc5883p).with_config(cfg);
        assert_eq!(mag.configure().unwrap(), 5_000);
        assert_eq!(mag.start_measurement().unwrap(), 0);
        assert_eq!(mag.read_measurement().unwrap().field.x, 100_000);
        assert_eq!(mag.read_measurement(), Err(Error::InvalidData));
        mag.sleep().unwrap();
        assert_eq!(mag.wake().unwrap(), 5_000);
        assert_eq!(mag.soft_reset().unwrap(), 250);
        i2c.done();
    }

    #[test]
    fn p_single_mode_and_l_range_check() {
        let mut i2c = Mock::new(&[wr(P, 0x0A, 0x02)]);
        let mut p = Qmc5883::new(&mut i2c, Variant::Qmc5883p);
        assert_eq!(p.start_measurement().unwrap(), 5_600);
        i2c.done();

        let cfg = Config {
            range: Range::G12,
            ..Config::default()
        };
        let mut i2c = Mock::new(&[]);
        let mut l = Qmc5883::new(&mut i2c, Variant::Qmc5883l);
        assert_eq!(l.set_config(cfg), Err(Error::Unsupported));
        i2c.done();
    }

    #[test]
    fn async_p_single_measurement() {
        use zigbee_sensor::asynch::Sensor as _;

        let mut i2c = Mock::new(&[
            wr(P, 0x0A, 0x02),
            rd(P, 0x09, &[0x01]),
            rd(P, 0x01, &[0, 0, 0, 0, 0xA6, 0x0E]),
        ]);
        let mut mag = asynch::Qmc5883::new(&mut i2c, Variant::Qmc5883p);
        assert_eq!(block_on(mag.start_measurement()).unwrap(), 5_600);
        assert_eq!(block_on(mag.read_measurement()).unwrap().field.z, 100_000);
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
