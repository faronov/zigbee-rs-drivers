//! Bosch BMM150 3-axis geomagnetic sensor over I2C or SPI.
//!
//! The part powers up in suspend mode (≈1 µA, registers not accessible).
//! [`power_up`](Bmm150::power_up) brings it to sleep mode; every
//! [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//! then triggers one forced-mode measurement, after which the chip
//! returns to sleep mode by itself. Normal mode (2–30 Hz) and the
//! low/high-threshold interrupt on the INT pin are optional.
//!
//! The raw output is temperature compensated on the host with the factory
//! trim registers (integer port of Bosch's SensorAPI, see
//! [`compensation`]) and reported as [`Quantity::MagneticField`] in nT
//! (1/16 µT resolution).
//!
//! Reference: Bosch Sensortec "BMM150 Geomagnetic Sensor" datasheet
//! BST-BMM150-DS001-05 rev. 1.4 (April 2020); BMM150 SensorAPI v2.0.0.
//!
//! ```ignore
//! let mut mag = Bmm150::new_i2c(i2c);
//! delay_us(mag.power_up()?);
//! mag.probe()?;
//! mag.configure()?; // reads the trim registers
//! let wait = mag.start_measurement()?;
//! delay_us(wait);
//! let m = mag.read_measurement()?; // m.field in nT
//! ```

#![no_std]

pub mod compensation;
mod interface;

pub use compensation::Trim;
pub use interface::{AsyncInterface, I2cInterface, Interface, SpiInterface};

use zigbee_sensor::{Error, Quantity, Readings, Vector3, WaitUs};

/// Default I2C address (CSB = SDO = low); 0x11–0x13 with SDO/CSB high.
pub const ADDRESS: u8 = 0x10;

/// `CHIP_ID` value (readable in sleep/active mode only).
pub const CHIP_ID: u8 = 0x32;

/// Register addresses.
pub mod reg {
    #![allow(missing_docs)]
    pub const CHIP_ID: u8 = 0x40;
    pub const DATA_X_LSB: u8 = 0x42;
    pub const INT_STATUS: u8 = 0x4A;
    pub const POWER: u8 = 0x4B;
    pub const OP_MODE: u8 = 0x4C;
    pub const INT_CTRL: u8 = 0x4D;
    pub const INT_AXES: u8 = 0x4E;
    pub const LOW_THRESHOLD: u8 = 0x4F;
    pub const HIGH_THRESHOLD: u8 = 0x50;
    pub const REP_XY: u8 = 0x51;
    pub const REP_Z: u8 = 0x52;
    pub const DIG_X1: u8 = 0x5D;
    pub const DIG_Z4_LSB: u8 = 0x62;
    pub const DIG_Z2_LSB: u8 = 0x68;
}

const POWER_ON: u8 = 0x01;
const SOFT_RESET: u8 = 0x82;
const OP_NORMAL: u8 = 0b00 << 1;
const OP_FORCED: u8 = 0b01 << 1;
const OP_SLEEP: u8 = 0b11 << 1;
const OP_SELF_TEST: u8 = 0x01;
const DRDY: u8 = 0x01;
const INT_EN_ALL_DISABLED: u8 = 0x3F;
const AXES_DEFAULT: u8 = 0x07;
const AXES_DRDY_PIN: u8 = 0x80;
const AXES_INT_PIN: u8 = 0x40;
const AXES_LATCH: u8 = 0x02;

/// Suspend → sleep start-up time.
pub const START_UP_US: WaitUs = 3_000;

/// Soft-reset time (SensorAPI).
pub const SOFT_RESET_US: WaitUs = 1_000;

/// Normal self-test time (SensorAPI).
pub const SELF_TEST_US: WaitUs = 2_000;

/// Repetition setting: noise vs. measurement time and current
/// (datasheet Table 3).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Preset {
    /// nXY = 3, nZ = 3: 1.0/1.4 µT noise, 2.9 ms, 0.17 mA at 10 Hz.
    LowPower,
    /// nXY = 9, nZ = 15: 0.6 µT noise, 9.8 ms, 0.5 mA at 10 Hz.
    #[default]
    Regular,
    /// nXY = 15, nZ = 27: 0.5 µT noise, 16.7 ms, 0.8 mA at 10 Hz.
    Enhanced,
    /// nXY = 47, nZ = 83: 0.3 µT noise, 49.3 ms, 4.9 mA at 20 Hz.
    HighAccuracy,
    /// Raw `REPXY` / `REPZ` register values (nXY = 1 + 2·REPXY,
    /// nZ = 1 + REPZ).
    Custom {
        /// `REPXY` register.
        rep_xy: u8,
        /// `REPZ` register.
        rep_z: u8,
    },
}

impl Preset {
    /// `REPXY` / `REPZ` register values.
    pub const fn registers(self) -> (u8, u8) {
        match self {
            Self::LowPower => (1, 2),
            Self::Regular => (4, 14),
            Self::Enhanced => (7, 26),
            Self::HighAccuracy => (23, 82),
            Self::Custom { rep_xy, rep_z } => (rep_xy, rep_z),
        }
    }

    /// Active measurement time: 145 µs·nXY + 500 µs·nZ + 980 µs.
    pub const fn measurement_us(self) -> WaitUs {
        let (xy, z) = self.registers();
        let n_xy = 1 + 2 * xy as u32;
        let n_z = 1 + z as u32;
        145 * n_xy + 500 * n_z + 980
    }
}

/// Normal-mode output data rate (`Data rate` bits of 0x4C).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DataRate {
    /// 2 Hz.
    Hz2 = 1,
    /// 6 Hz.
    Hz6 = 2,
    /// 8 Hz.
    Hz8 = 3,
    /// 10 Hz (reset value).
    #[default]
    Hz10 = 0,
    /// 15 Hz.
    Hz15 = 4,
    /// 20 Hz.
    Hz20 = 5,
    /// 25 Hz.
    Hz25 = 6,
    /// 30 Hz.
    Hz30 = 7,
}

impl DataRate {
    /// Output period, rounded up.
    pub const fn period_us(self) -> WaitUs {
        match self {
            Self::Hz2 => 500_000,
            Self::Hz6 => 166_667,
            Self::Hz8 => 125_000,
            Self::Hz10 => 100_000,
            Self::Hz15 => 66_667,
            Self::Hz20 => 50_000,
            Self::Hz25 => 40_000,
            Self::Hz30 => 33_334,
        }
    }
}

/// Axis selection for the threshold interrupt.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Axes {
    /// X axis.
    pub x: bool,
    /// Y axis.
    pub y: bool,
    /// Z axis.
    pub z: bool,
}

impl Default for Axes {
    fn default() -> Self {
        Self {
            x: true,
            y: true,
            z: true,
        }
    }
}

impl Axes {
    fn bits(self) -> u8 {
        u8::from(self.x) | u8::from(self.y) << 1 | u8::from(self.z) << 2
    }
}

/// Low/high-threshold interrupt on the INT pin (push-pull, active high).
/// Thresholds are raw register values compared against the
/// uncompensated output: `DATA > high × 16` / `DATA < low × 16`, roughly
/// 6 µT per step. Evaluated after every measurement, so it needs normal
/// mode (or regular forced measurements).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ThresholdInterrupt {
    /// Low-threshold register value; `None` = disabled.
    pub low: Option<u8>,
    /// High-threshold register value; `None` = disabled.
    pub high: Option<u8>,
    /// Axes checked against the thresholds (default all).
    pub axes: Axes,
    /// Keep INT high until [`IntStatus`] is read (default: follow the
    /// condition).
    pub latched: bool,
}

/// Measurement configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// Repetitions (default [`Preset::Regular`]).
    pub preset: Preset,
    /// Run in normal mode at this rate instead of forced (single)
    /// measurements (default `None`).
    pub continuous: Option<DataRate>,
    /// Threshold interrupt (default `None`).
    pub threshold: Option<ThresholdInterrupt>,
    /// Signal data ready on the DRDY pin (active high; default `false`).
    pub drdy_pin: bool,
}

impl Config {
    /// `INT_CTRL` (0x4D) value; enable bits are active low.
    pub fn int_ctrl(&self) -> u8 {
        let Some(t) = self.threshold else {
            return INT_EN_ALL_DISABLED;
        };
        let axes = t.axes.bits();
        let mut v = INT_EN_ALL_DISABLED;
        if t.low.is_some() {
            v &= !axes;
        }
        if t.high.is_some() {
            v &= !(axes << 3);
        }
        v
    }

    /// `INT_AXES` (0x4E) value: all channels on, pin enables, polarity
    /// active high.
    pub fn int_axes(&self) -> u8 {
        let mut v = AXES_DEFAULT & !AXES_LATCH;
        if self.drdy_pin {
            v |= AXES_DRDY_PIN;
        }
        if let Some(t) = self.threshold {
            v |= AXES_INT_PIN;
            if t.latched {
                v |= AXES_LATCH;
            }
        }
        v
    }

    /// `OP_MODE` (0x4C) value after `configure()`: normal mode or sleep.
    pub fn op_mode(&self) -> u8 {
        match self.continuous {
            Some(rate) => (rate as u8) << 3 | OP_NORMAL,
            None => OP_SLEEP,
        }
    }

    /// Wait after `configure()`: first normal-mode sample, or 0.
    /// [`Error::Unsupported`] if the measurement does not fit the ODR.
    pub fn first_sample_us<E>(&self) -> Result<WaitUs, Error<E>> {
        let Some(rate) = self.continuous else {
            return Ok(0);
        };
        let t = self.preset.measurement_us();
        if t > rate.period_us() {
            return Err(Error::Unsupported);
        }
        Ok(rate.period_us() + t)
    }
}

/// Interrupt status register (0x4A); reading it clears latched flags.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IntStatus(pub u8);

impl IntStatus {
    /// Any low-threshold flag (X, Y or Z).
    pub fn low(self) -> bool {
        self.0 & 0x07 != 0
    }

    /// Any high-threshold flag (X, Y or Z).
    pub fn high(self) -> bool {
        self.0 & 0x38 != 0
    }

    /// Overflow flag.
    pub fn overflow(self) -> bool {
        self.0 & 0x40 != 0
    }
}

/// Raw output: X/Y 13-bit, Z 15-bit (signed), RHALL 14-bit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(missing_docs)]
pub struct Raw {
    pub x: i16,
    pub y: i16,
    pub z: i16,
    pub rhall: u16,
}

impl Raw {
    /// Unpack registers 0x42–0x49; `None` if `DRDY` is clear.
    pub fn from_regs(b: &[u8; 8]) -> Option<Self> {
        if b[6] & DRDY == 0 {
            return None;
        }
        let s = |i: usize| i16::from_le_bytes([b[i], b[i + 1]]);
        Some(Self {
            x: s(0) >> 3,
            y: s(2) >> 3,
            z: s(4) >> 1,
            rhall: u16::from_le_bytes([b[6], b[7]]) >> 2,
        })
    }
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

/// 1/16 µT to nT (×62.5), rounded to nearest.
pub const fn sixteenth_ut_to_nt(v: i32) -> i32 {
    let n = v * 125;
    (if n >= 0 { n + 1 } else { n - 1 }) / 2
}

/// Compensate a raw sample. [`Error::InvalidData`] on overflow or
/// missing RHALL/trim data.
pub fn compensate<E>(raw: &Raw, trim: &Trim) -> Result<Measurement, Error<E>> {
    let x = trim.compensate_x(raw.x, raw.rhall);
    let y = trim.compensate_y(raw.y, raw.rhall);
    let z = trim.compensate_z(raw.z, raw.rhall);
    match (x, y, z) {
        (Some(x), Some(y), Some(z)) => Ok(Measurement {
            field: Vector3 {
                x: sixteenth_ut_to_nt(x),
                y: sixteenth_ut_to_nt(y),
                z: sixteenth_ut_to_nt(z),
            },
        }),
        _ => Err(Error::InvalidData),
    }
}

macro_rules! driver {
    ($doc:literal, $Iface:ident, $mode:ident, [$($async:tt)?], [$($aw:tt)*]) => {
        #[doc = $doc]
        #[derive(Debug)]
        pub struct Bmm150<I> {
            iface: I,
            config: Config,
            trim: Option<Trim>,
            configured: bool,
        }

        impl<I> Bmm150<I> {
            /// Create a driver over any register interface; no bus traffic.
            pub fn new(iface: I) -> Self {
                Self {
                    iface,
                    config: Config::default(),
                    trim: None,
                    configured: false,
                }
            }

            /// Set the configuration used by `configure()`.
            pub fn with_config(mut self, config: Config) -> Self {
                self.config = config;
                self
            }

            /// Current configuration.
            pub fn config(&self) -> &Config {
                &self.config
            }

            /// Trim values, once read.
            pub fn trim(&self) -> Option<&Trim> {
                self.trim.as_ref()
            }

            /// Give the register interface back.
            pub fn into_interface(self) -> I {
                self.iface
            }
        }

        impl<I2C> Bmm150<I2cInterface<I2C>> {
            /// Create an I2C driver at [`ADDRESS`] (0x10); use
            /// `Bmm150::new(I2cInterface::new(i2c, addr))` for 0x11–0x13.
            /// No bus traffic.
            pub fn new_i2c(i2c: I2C) -> Self {
                Self::new(I2cInterface::new(i2c, ADDRESS))
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.iface.release()
            }
        }

        impl<SPI> Bmm150<SpiInterface<SPI>> {
            /// Create a 4-wire SPI driver; no bus traffic.
            pub fn new_spi(spi: SPI) -> Self {
                Self::new(SpiInterface::new(spi))
            }

            /// Give the device back.
            pub fn release(self) -> SPI {
                self.iface.release()
            }
        }

        impl<I: $Iface> Bmm150<I> {
            /// Read consecutive registers.
            pub $($async)? fn read_registers(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I::Error>> {
                self.iface.read_regs(reg, buf) $($aw)* .map_err(Error::bus)
            }

            /// Write one register.
            pub $($async)? fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Error<I::Error>> {
                self.iface.write_reg(reg, value) $($aw)* .map_err(Error::bus)
            }

            /// Leave suspend mode (power control bit) for sleep mode;
            /// wait [`START_UP_US`] before the next access. Registers other
            /// than 0x4B are reset, so `configure()` runs again on the next
            /// `start_measurement()`.
            pub $($async)? fn power_up(&mut self) -> Result<WaitUs, Error<I::Error>> {
                self.configured = false;
                self.write_register(reg::POWER, POWER_ON) $($aw)* ?;
                Ok(START_UP_US)
            }

            /// Check `CHIP_ID` (0x32). Reads 0x00 in suspend mode: call
            /// `power_up()` first.
            pub $($async)? fn probe(&mut self) -> Result<(), Error<I::Error>> {
                let mut id = [0u8];
                self.read_registers(reg::CHIP_ID, &mut id) $($aw)* ?;
                if id[0] != CHIP_ID {
                    return Err(Error::UnexpectedId(u32::from(id[0])));
                }
                Ok(())
            }

            /// Soft reset to sleep mode (ignored in suspend mode).
            pub $($async)? fn soft_reset(&mut self) -> Result<WaitUs, Error<I::Error>> {
                self.configured = false;
                self.write_register(reg::POWER, SOFT_RESET | POWER_ON) $($aw)* ?;
                Ok(SOFT_RESET_US)
            }

            /// Read and cache the factory trim registers.
            pub $($async)? fn read_trim(&mut self) -> Result<Trim, Error<I::Error>> {
                let mut a = [0u8; 2];
                let mut b = [0u8; 4];
                let mut c = [0u8; 10];
                self.read_registers(reg::DIG_X1, &mut a) $($aw)* ?;
                self.read_registers(reg::DIG_Z4_LSB, &mut b) $($aw)* ?;
                self.read_registers(reg::DIG_Z2_LSB, &mut c) $($aw)* ?;
                let trim = Trim::from_regs(&a, &b, &c);
                self.trim = Some(trim);
                Ok(trim)
            }

            /// Read the trim registers (first time only), then write
            /// repetitions, thresholds, interrupt setup and the operation
            /// mode (sleep, or normal mode for `continuous`). Returns the
            /// time to the first normal-mode sample, else 0;
            /// [`Error::Unsupported`] if the preset is too slow for the ODR.
            pub $($async)? fn configure(&mut self) -> Result<WaitUs, Error<I::Error>> {
                let c = self.config;
                let wait = c.first_sample_us()?;
                if self.trim.is_none() {
                    self.read_trim() $($aw)* ?;
                }
                let (xy, z) = c.preset.registers();
                let t = c.threshold.unwrap_or_default();
                self.write_register(reg::OP_MODE, OP_SLEEP) $($aw)* ?;
                self.write_register(reg::REP_XY, xy) $($aw)* ?;
                self.write_register(reg::REP_Z, z) $($aw)* ?;
                self.write_register(reg::LOW_THRESHOLD, t.low.unwrap_or(0)) $($aw)* ?;
                self.write_register(reg::HIGH_THRESHOLD, t.high.unwrap_or(0)) $($aw)* ?;
                self.write_register(reg::INT_CTRL, c.int_ctrl()) $($aw)* ?;
                self.write_register(reg::INT_AXES, c.int_axes()) $($aw)* ?;
                if c.continuous.is_some() {
                    self.write_register(reg::OP_MODE, c.op_mode()) $($aw)* ?;
                }
                self.configured = true;
                Ok(wait)
            }

            /// Store a new configuration and write it.
            pub $($async)? fn set_config(&mut self, config: Config) -> Result<WaitUs, Error<I::Error>> {
                self.config = config;
                self.configure() $($aw)*
            }

            /// Read (and clear, if latched) the interrupt status.
            pub $($async)? fn read_interrupt_status(&mut self) -> Result<IntStatus, Error<I::Error>> {
                let mut s = [0u8];
                self.read_registers(reg::INT_STATUS, &mut s) $($aw)* ?;
                Ok(IntStatus(s[0]))
            }

            /// Suspend mode (≈1 µA). All registers except 0x4B are lost;
            /// `wake()` / `power_up()` restores operation.
            pub $($async)? fn power_down(&mut self) -> Result<(), Error<I::Error>> {
                self.configured = false;
                self.write_register(reg::POWER, 0) $($aw)*
            }

            /// Start the normal self-test (from sleep mode). Read the
            /// result with `read_self_test()`; run `configure()` afterwards
            /// for normal mode.
            pub $($async)? fn start_self_test(&mut self) -> Result<WaitUs, Error<I::Error>> {
                self.write_register(reg::OP_MODE, OP_SLEEP | OP_SELF_TEST) $($aw)* ?;
                Ok(SELF_TEST_US)
            }

            /// `true` if the X, Y and Z self-test flags are set;
            /// [`Error::NotReady`] while the self-test bit is still set.
            pub $($async)? fn read_self_test(&mut self) -> Result<bool, Error<I::Error>> {
                let mut op = [0u8];
                self.read_registers(reg::OP_MODE, &mut op) $($aw)* ?;
                if op[0] & OP_SELF_TEST != 0 {
                    return Err(Error::NotReady);
                }
                let mut b = [0u8; 6];
                self.read_registers(reg::DATA_X_LSB, &mut b) $($aw)* ?;
                Ok(b[0] & b[2] & b[4] & 1 != 0)
            }
        }

        impl<I: $Iface> zigbee_sensor::$mode::Sensor for Bmm150<I> {
            type BusError = I::Error;
            type Measurement = Measurement;

            /// Runs `configure()` if needed (first use, after
            /// `power_up()`/`wake()`). Single mode: trigger one forced
            /// measurement and return its active time. Normal mode: the
            /// time to the first sample after (re)configuration, else 0.
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I::Error>> {
                let mut wait = 0;
                if !self.configured {
                    wait = self.configure() $($aw)* ?;
                }
                if self.config.continuous.is_some() {
                    return Ok(wait);
                }
                self.write_register(reg::OP_MODE, OP_FORCED) $($aw)* ?;
                Ok(self.config.preset.measurement_us())
            }

            /// Burst-read 0x42–0x49 and compensate. [`Error::NotReady`]
            /// until `DRDY`; [`Error::InvalidData`] on overflow.
            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<I::Error>> {
                let mut b = [0u8; 8];
                self.read_registers(reg::DATA_X_LSB, &mut b) $($aw)* ?;
                let raw = Raw::from_regs(&b).ok_or(Error::NotReady)?;
                let trim = self.trim.ok_or(Error::NotReady)?;
                compensate(&raw, &trim)
            }
        }

        impl<I: $Iface> zigbee_sensor::$mode::LowPower for Bmm150<I> {
            type BusError = I::Error;

            /// [`Self::power_down`] (suspend mode).
            $($async)? fn sleep(&mut self) -> Result<(), Error<I::Error>> {
                self.power_down() $($aw)*
            }

            /// [`Self::power_up`]; the next `start_measurement()`
            /// rewrites the configuration.
            $($async)? fn wake(&mut self) -> Result<WaitUs, Error<I::Error>> {
                self.power_up() $($aw)*
            }
        }
    };
}

driver!("Blocking driver.", Interface, blocking, [], []);

/// Async driver.
pub mod asynch {
    use super::*;

    driver!(
        "Async twin of [`super::Bmm150`].",
        AsyncInterface,
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
    use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiTx};
    use std::{vec, vec::Vec};
    use zigbee_sensor::blocking::{LowPower, Sensor};

    const A: u8 = ADDRESS;

    fn rd(reg: u8, data: &[u8]) -> I2cTx {
        I2cTx::write_read(A, vec![reg], data.to_vec())
    }
    fn wr(reg: u8, value: u8) -> I2cTx {
        I2cTx::write(A, vec![reg, value])
    }

    // x1 -3, y1 5, z4 12, x2 -20, y2 30, z2 700, z1 21000, xyz1 6600
    // (bit 15 set, must be masked), z3 -50, xy2 -4, xy1 30.
    const T_X1Y1: [u8; 2] = [0xFD, 0x05];
    const T_Z4X2Y2: [u8; 4] = [0x0C, 0x00, 0xEC, 0x1E];
    const T_Z2XY1: [u8; 10] = [0xBC, 0x02, 0x08, 0x52, 0xC8, 0x99, 0xCE, 0xFF, 0xFC, 0x1E];

    // x = -120, y = 200, z = -480, RHALL = 6600, self-test bits and DRDY set.
    const FRAME: [u8; 8] = [0x41, 0xFC, 0x41, 0x06, 0x41, 0xFC, 0x21, 0x67];

    fn trim_reads() -> [I2cTx; 3] {
        [rd(0x5D, &T_X1Y1), rd(0x62, &T_Z4X2Y2), rd(0x68, &T_Z2XY1)]
    }

    fn trim_e() -> Trim {
        Trim::from_regs(&T_X1Y1, &T_Z4X2Y2, &T_Z2XY1)
    }

    /// Golden values from Bosch SensorAPI v2.0.0 fixed-point functions
    /// with the final `/ 16` removed.
    #[test]
    fn compensation_matches_bosch_reference() {
        let d = Trim {
            x2: 26,
            y2: 26,
            z1: 24747,
            z2: 763,
            xy1: 29,
            xy2: -3,
            xyz1: 6615,
            ..Trim::default()
        };
        assert_eq!(d.compensate_x(-120, 6600), Some(-697));
        assert_eq!(d.compensate_y(200, 6600), Some(1162));
        assert_eq!(d.compensate_z(-480, 6600), Some(-2736));
        let e = trim_e();
        assert_eq!(
            e,
            Trim {
                x1: -3,
                y1: 5,
                x2: -20,
                y2: 30,
                z1: 21000,
                z2: 700,
                z3: -50,
                z4: 12,
                xy1: 30,
                xy2: -4,
                xyz1: 6600,
            }
        );
        assert_eq!(e.compensate_x(-120, 6200), Some(-552));
        assert_eq!(e.compensate_y(-120, 7000), Some(-667));
        assert_eq!(e.compensate_z(-480, 6200), Some(-3451));
        assert_eq!(e.compensate_x(4000, 6200), Some(17606));
        assert_eq!(e.compensate_y(-4000, 7000), Some(-23549));
        assert_eq!(e.compensate_z(16000, 6600), Some(32767));
        assert_eq!(e.compensate_z(-16000, 7000), Some(-32767));
        assert_eq!(e.compensate_x(0, 7000), Some(-24));
        assert_eq!(e.compensate_y(0, 7000), Some(40));
        assert_eq!(e.compensate_z(0, 7000), Some(-74));
        // Overflow and missing RHALL.
        assert_eq!(e.compensate_x(-4096, 6600), None);
        assert_eq!(e.compensate_z(-16384, 6600), None);
        assert_eq!(e.compensate_z(100, 0), None);
        assert_eq!(e.compensate_x(-120, 0), e.compensate_x(-120, 6600));
    }

    #[test]
    fn raw_unpack_and_scaling() {
        let raw = Raw::from_regs(&FRAME).unwrap();
        assert_eq!(
            raw,
            Raw {
                x: -120,
                y: 200,
                z: -480,
                rhall: 6600
            }
        );
        let mut nodrdy = FRAME;
        nodrdy[6] &= !1;
        assert_eq!(Raw::from_regs(&nodrdy), None);
        assert_eq!(sixteenth_ut_to_nt(16), 1000);
        assert_eq!(sixteenth_ut_to_nt(-549), -34_313);
        assert_eq!(Preset::LowPower.measurement_us(), 2_915);
        assert_eq!(Preset::Regular.measurement_us(), 9_785);
        assert_eq!(Preset::Enhanced.measurement_us(), 16_655);
        assert_eq!(Preset::HighAccuracy.measurement_us(), 49_295);
    }

    #[test]
    fn power_up_probe_reset() {
        let mut i2c = Mock::new(&[
            wr(0x4B, 0x01),
            rd(0x40, &[0x32]),
            rd(0x40, &[0x00]),
            wr(0x4B, 0x83),
        ]);
        let mut mag = Bmm150::new_i2c(&mut i2c);
        assert_eq!(mag.power_up().unwrap(), 3_000);
        mag.probe().unwrap();
        assert_eq!(mag.probe(), Err(Error::UnexpectedId(0)));
        assert_eq!(mag.soft_reset().unwrap(), 1_000);
        i2c.done();
    }

    #[test]
    fn forced_measurement_and_suspend() {
        let mut ex = Vec::from(trim_reads());
        let cfg = [
            wr(0x4C, 0x06),
            wr(0x51, 4),
            wr(0x52, 14),
            wr(0x4F, 0),
            wr(0x50, 0),
            wr(0x4D, 0x3F),
            wr(0x4E, 0x05),
        ];
        ex.extend(cfg.iter().cloned());
        ex.extend([
            wr(0x4C, 0x02),
            rd(0x42, &[0; 8]),
            rd(0x42, &FRAME),
            wr(0x4B, 0x00),
            wr(0x4B, 0x01),
        ]);
        ex.extend(cfg.iter().cloned());
        ex.push(wr(0x4C, 0x02));
        let mut i2c = Mock::new(&ex);
        let mut mag = Bmm150::new_i2c(&mut i2c);
        assert_eq!(mag.start_measurement().unwrap(), 9_785);
        assert_eq!(mag.read_measurement(), Err(Error::NotReady));
        assert_eq!(
            mag.read_measurement().unwrap().field,
            Vector3 {
                x: -34_313,
                y: 76_688,
                z: -204_375
            }
        );
        mag.sleep().unwrap();
        assert_eq!(mag.wake().unwrap(), 3_000);
        // Trim is cached; configuration is rewritten after suspend.
        assert_eq!(mag.start_measurement().unwrap(), 9_785);
        i2c.done();
    }

    #[test]
    fn spi_normal_mode_with_threshold_interrupt() {
        fn w(reg: u8, value: u8) -> [SpiTx<u8>; 3] {
            [
                SpiTx::transaction_start(),
                SpiTx::write_vec(vec![reg, value]),
                SpiTx::transaction_end(),
            ]
        }
        fn r(reg: u8, data: &[u8]) -> [SpiTx<u8>; 4] {
            [
                SpiTx::transaction_start(),
                SpiTx::write_vec(vec![reg | 0x80]),
                SpiTx::read_vec(data.to_vec()),
                SpiTx::transaction_end(),
            ]
        }
        let mut ex: Vec<SpiTx<u8>> = Vec::new();
        ex.extend(r(0x5D, &T_X1Y1));
        ex.extend(r(0x62, &T_Z4X2Y2));
        ex.extend(r(0x68, &T_Z2XY1));
        for (reg, v) in [
            (0x4C, 0x06),
            (0x51, 1),
            (0x52, 2),
            (0x4F, 0),
            (0x50, 10),
            (0x4D, 0x1F),
            (0x4E, 0x47),
            (0x4C, (DataRate::Hz10 as u8) << 3),
        ] {
            ex.extend(w(reg, v));
        }
        ex.extend(r(0x4A, &[0x20]));
        let cfg = Config {
            preset: Preset::LowPower,
            continuous: Some(DataRate::Hz10),
            threshold: Some(ThresholdInterrupt {
                high: Some(10),
                axes: Axes {
                    x: false,
                    y: false,
                    z: true,
                },
                latched: true,
                ..Default::default()
            }),
            drdy_pin: false,
        };
        let mut spi = SpiMock::new(&ex);
        let mut mag = Bmm150::new_spi(&mut spi).with_config(cfg);
        assert_eq!(mag.configure().unwrap(), 102_915);
        assert_eq!(mag.start_measurement().unwrap(), 0);
        let st = mag.read_interrupt_status().unwrap();
        assert!(st.high() && !st.low() && !st.overflow());
        let too_slow = Config {
            preset: Preset::HighAccuracy,
            continuous: Some(DataRate::Hz30),
            ..Config::default()
        };
        assert_eq!(mag.set_config(too_slow), Err(Error::Unsupported));
        spi.done();
    }

    #[test]
    fn self_test() {
        let mut i2c = Mock::new(&[
            wr(0x4C, 0x07),
            rd(0x4C, &[0x07]),
            rd(0x4C, &[0x06]),
            rd(0x42, &[0x01, 0, 0x01, 0, 0x01, 0]),
            rd(0x4C, &[0x06]),
            rd(0x42, &[0x01, 0, 0x00, 0, 0x01, 0]),
        ]);
        let mut mag = Bmm150::new_i2c(&mut i2c);
        assert_eq!(mag.start_self_test().unwrap(), 2_000);
        assert_eq!(mag.read_self_test(), Err(Error::NotReady));
        assert!(mag.read_self_test().unwrap());
        assert!(!mag.read_self_test().unwrap());
        i2c.done();
    }

    #[test]
    fn async_forced_measurement() {
        use zigbee_sensor::asynch::Sensor as _;

        let mut ex = Vec::from(trim_reads());
        ex.extend([
            wr(0x4C, 0x06),
            wr(0x51, 4),
            wr(0x52, 14),
            wr(0x4F, 0),
            wr(0x50, 0),
            wr(0x4D, 0x3F),
            wr(0x4E, 0x05),
            wr(0x4C, 0x02),
            rd(0x42, &FRAME),
        ]);
        let mut i2c = Mock::new(&ex);
        let mut mag = asynch::Bmm150::new_i2c(&mut i2c);
        assert_eq!(block_on(mag.start_measurement()).unwrap(), 9_785);
        assert_eq!(block_on(mag.read_measurement()).unwrap().field.z, -204_375);
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
