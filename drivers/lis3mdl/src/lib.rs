//! ST LIS3MDL 3-axis magnetometer over I2C or SPI.
//!
//! By default the part stays in power-down (≈1 µA) and every
//! [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//! triggers one single conversion, returning the turn-on time of the selected
//! [`Performance`] mode; the part returns to power-down by itself afterwards.
//! With a [`Threshold`] interrupt enabled the part runs in continuous mode at
//! [`Config::data_rate`] instead, because the interrupt logic only evaluates
//! continuously converted samples.
//!
//! The field is reported as [`Quantity::MagneticField`] in nT
//! (1 gauss = 100 000 nT).
//!
//! References: ST "LIS3MDL" datasheet DocID024204 Rev 4 and application
//! note AN4602 DocID027014 Rev 1.
//!
//! ```ignore
//! let mut mag = Lis3mdl::new_i2c(i2c, ADDRESS_SDO_HIGH);
//! mag.probe()?;
//! mag.configure()?;
//! let wait = mag.start_measurement()?; // single conversion
//! delay_us(wait);
//! let m = mag.read_measurement()?; // m.field in nT
//! ```

#![no_std]

mod interface;

pub use interface::{AsyncInterface, I2cInterface, Interface, SpiInterface};
use zigbee_sensor::{Error, Quantity, Readings, Vector3, WaitUs};

/// I2C address with SDO/SA1 tied to GND.
pub const ADDRESS_SDO_LOW: u8 = 0x1C;
/// I2C address with SDO/SA1 tied to the supply.
pub const ADDRESS_SDO_HIGH: u8 = 0x1E;

/// `WHO_AM_I` value.
pub const WHO_AM_I: u8 = 0x3D;

/// Register addresses.
pub mod reg {
    #![allow(missing_docs)]
    pub const WHO_AM_I: u8 = 0x0F;
    pub const CTRL_REG1: u8 = 0x20;
    pub const CTRL_REG2: u8 = 0x21;
    pub const CTRL_REG3: u8 = 0x22;
    pub const CTRL_REG4: u8 = 0x23;
    pub const CTRL_REG5: u8 = 0x24;
    pub const STATUS: u8 = 0x27;
    pub const OUT_X_L: u8 = 0x28;
    pub const INT_CFG: u8 = 0x30;
    pub const INT_SRC: u8 = 0x31;
    pub const INT_THS_L: u8 = 0x32;
    pub const INT_THS_H: u8 = 0x33;
}

const CTRL_REG2_SOFT_RST: u8 = 0x04;
const CTRL_REG3_CONTINUOUS: u8 = 0x00;
const CTRL_REG3_SINGLE: u8 = 0x01;
const CTRL_REG3_POWER_DOWN: u8 = 0x03;
/// Block data update: output registers are not updated until both bytes of
/// the previous sample have been read.
const CTRL_REG5_BDU: u8 = 0x40;
const STATUS_ZYXDA: u8 = 0x08;
const INT_CFG_XIEN: u8 = 0x80;
const INT_CFG_YIEN: u8 = 0x40;
const INT_CFG_ZIEN: u8 = 0x20;
const INT_CFG_IEA: u8 = 0x04;
/// 1 = not latched.
const INT_CFG_LIR: u8 = 0x02;
const INT_CFG_IEN: u8 = 0x01;
/// `INT_THS` is a 15-bit unsigned value.
const THRESHOLD_MAX: u16 = 0x7FFF;

/// nT per gauss.
pub const NT_PER_GAUSS: i64 = 100_000;

/// X/Y (`OM`) and Z (`OMZ`) operating mode: more internal averaging gives
/// lower noise at the cost of a longer conversion and more current.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Performance {
    /// Low-power (40 µA at 20 Hz continuous).
    LowPower = 0,
    /// Medium performance.
    Medium = 1,
    /// High performance.
    High = 2,
    /// Ultra-high performance (270 µA at 20 Hz continuous).
    #[default]
    UltraHigh = 3,
}

impl Performance {
    /// Time from a single-conversion (or continuous-mode) request until
    /// data is available, rounded up (AN4602 Table 8 turn-on time).
    pub const fn conversion_us(self) -> WaitUs {
        match self {
            Self::LowPower => 1_200,
            Self::Medium => 1_910,
            Self::High => 3_480,
            Self::UltraHigh => 6_650,
        }
    }
}

/// Continuous-mode output data rate (`DO`); only used by the threshold
/// interrupt. `FAST_ODR` rates are not supported because single-conversion
/// mode is only specified up to 80 Hz.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DataRate {
    /// 0.625 Hz.
    Hz0_625 = 0,
    /// 1.25 Hz.
    Hz1_25 = 1,
    /// 2.5 Hz.
    Hz2_5 = 2,
    /// 5 Hz.
    Hz5 = 3,
    /// 10 Hz.
    #[default]
    Hz10 = 4,
    /// 20 Hz.
    Hz20 = 5,
    /// 40 Hz.
    Hz40 = 6,
    /// 80 Hz.
    Hz80 = 7,
}

/// Full scale.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Range {
    /// ±4 gauss (±400 µT), 6842 LSB/gauss.
    #[default]
    Gauss4 = 0,
    /// ±8 gauss, 3421 LSB/gauss.
    Gauss8 = 1,
    /// ±12 gauss, 2281 LSB/gauss.
    Gauss12 = 2,
    /// ±16 gauss, 1711 LSB/gauss.
    Gauss16 = 3,
}

impl Range {
    /// Sensitivity in LSB/gauss (datasheet Table 3).
    pub const fn lsb_per_gauss(self) -> i64 {
        match self {
            Self::Gauss4 => 6842,
            Self::Gauss8 => 3421,
            Self::Gauss12 => 2281,
            Self::Gauss16 => 1711,
        }
    }
}

/// Measurement configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// X/Y/Z operating mode (default ultra-high performance).
    pub performance: Performance,
    /// Full scale (default ±4 gauss).
    pub range: Range,
    /// Continuous rate used while a threshold interrupt is enabled
    /// (default 10 Hz).
    pub data_rate: DataRate,
}

/// Threshold interrupt on the INT pin: fires when the field on an enabled
/// axis exceeds `+threshold` or falls below `-threshold`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Threshold {
    /// Absolute threshold in nT (same for all axes), clamped to the 15-bit
    /// register range.
    pub threshold_nt: u32,
    /// Enable the X axis.
    pub x: bool,
    /// Enable the Y axis.
    pub y: bool,
    /// Enable the Z axis.
    pub z: bool,
    /// INT active high (default) or low.
    pub active_high: bool,
    /// Keep INT asserted until [`Lis3mdl::interrupt_source`] reads
    /// `INT_SRC` (default); otherwise INT follows the condition.
    pub latched: bool,
}

impl Default for Threshold {
    fn default() -> Self {
        Self {
            threshold_nt: 100_000,
            x: true,
            y: true,
            z: true,
            active_high: true,
            latched: true,
        }
    }
}

/// `INT_SRC` contents.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IntSource(pub u8);

impl IntSource {
    /// An interrupt event occurred.
    pub const fn active(self) -> bool {
        self.0 & 0x01 != 0
    }

    /// Internal measurement range overflow.
    pub const fn overflow(self) -> bool {
        self.0 & 0x02 != 0
    }

    /// Positive-side threshold exceeded on X, Y, Z.
    pub const fn positive(self) -> [bool; 3] {
        [self.0 & 0x80 != 0, self.0 & 0x40 != 0, self.0 & 0x20 != 0]
    }

    /// Negative-side threshold exceeded on X, Y, Z.
    pub const fn negative(self) -> [bool; 3] {
        [self.0 & 0x10 != 0, self.0 & 0x08 != 0, self.0 & 0x04 != 0]
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

fn div_round(n: i64, d: i64) -> i64 {
    if n >= 0 {
        (n + d / 2) / d
    } else {
        (n - d / 2) / d
    }
}

/// Raw output count to nT.
pub fn raw_to_nt(raw: i16, range: Range) -> i32 {
    div_round(i64::from(raw) * NT_PER_GAUSS, range.lsb_per_gauss()) as i32
}

/// Threshold in nT to the 15-bit `INT_THS` code.
pub fn threshold_code(threshold_nt: u32, range: Range) -> u16 {
    let code = div_round(
        i64::from(threshold_nt) * range.lsb_per_gauss(),
        NT_PER_GAUSS,
    );
    code.min(i64::from(THRESHOLD_MAX)) as u16
}

/// Decode `OUT_X_L..OUT_Z_H` (little-endian) to nT.
pub fn decode(bytes: &[u8; 6], range: Range) -> Measurement {
    let axis = |i: usize| raw_to_nt(i16::from_le_bytes([bytes[i], bytes[i + 1]]), range);
    Measurement {
        field: Vector3 {
            x: axis(0),
            y: axis(2),
            z: axis(4),
        },
    }
}

/// `INT_CFG` value (0 = disabled).
pub fn int_cfg(threshold: Option<&Threshold>) -> u8 {
    let Some(t) = threshold else { return 0 };
    let mut v = INT_CFG_IEN;
    if t.x {
        v |= INT_CFG_XIEN;
    }
    if t.y {
        v |= INT_CFG_YIEN;
    }
    if t.z {
        v |= INT_CFG_ZIEN;
    }
    if t.active_high {
        v |= INT_CFG_IEA;
    }
    if !t.latched {
        v |= INT_CFG_LIR;
    }
    v
}

/// Register writes for [`Lis3mdl::configure`], ending with the operating
/// mode (power-down, or continuous with a threshold interrupt).
pub fn config_writes(config: &Config, threshold: Option<&Threshold>) -> [(u8, u8); 8] {
    let p = config.performance as u8;
    let ths = threshold.map_or(0, |t| threshold_code(t.threshold_nt, config.range));
    let mode = if threshold.is_some() {
        CTRL_REG3_CONTINUOUS
    } else {
        CTRL_REG3_POWER_DOWN
    };
    [
        (reg::CTRL_REG1, p << 5 | (config.data_rate as u8) << 2),
        (reg::CTRL_REG2, (config.range as u8) << 5),
        (reg::CTRL_REG4, p << 2),
        (reg::CTRL_REG5, CTRL_REG5_BDU),
        (reg::INT_THS_L, ths as u8),
        (reg::INT_THS_H, (ths >> 8) as u8),
        (reg::INT_CFG, int_cfg(threshold)),
        (reg::CTRL_REG3, mode),
    ]
}

macro_rules! driver {
    ($doc:literal, $Iface:ident, $mode:ident, [$($async:tt)?], [$($aw:tt)*]) => {
        #[doc = $doc]
        #[derive(Debug)]
        pub struct Lis3mdl<I> {
            iface: I,
            config: Config,
            threshold: Option<Threshold>,
        }

        impl<I> Lis3mdl<I> {
            /// Create a driver over any register interface; no bus traffic.
            pub fn new(iface: I) -> Self {
                Self {
                    iface,
                    config: Config::default(),
                    threshold: None,
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

            /// Whether the part runs continuously for a threshold interrupt.
            pub fn continuous(&self) -> bool {
                self.threshold.is_some()
            }

            /// Give the register interface back.
            pub fn into_interface(self) -> I {
                self.iface
            }
        }

        impl<I2C> Lis3mdl<I2cInterface<I2C>> {
            /// Create an I2C driver; no bus traffic.
            pub fn new_i2c(i2c: I2C, address: u8) -> Self {
                Self::new(I2cInterface::new(i2c, address))
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.iface.release()
            }
        }

        impl<SPI> Lis3mdl<SpiInterface<SPI>> {
            /// Create a 4-wire SPI driver; no bus traffic.
            pub fn new_spi(spi: SPI) -> Self {
                Self::new(SpiInterface::new(spi))
            }

            /// Give the device back.
            pub fn release(self) -> SPI {
                self.iface.release()
            }
        }

        impl<I: $Iface> Lis3mdl<I> {
            /// Read one register.
            pub $($async)? fn read_register(&mut self, reg: u8) -> Result<u8, Error<I::Error>> {
                let mut b = [0u8];
                self.iface.read_regs(reg, &mut b) $($aw)* .map_err(Error::bus)?;
                Ok(b[0])
            }

            /// Write one register.
            pub $($async)? fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Error<I::Error>> {
                self.iface.write_reg(reg, value) $($aw)* .map_err(Error::bus)
            }

            /// Read `WHO_AM_I` (0x3D).
            pub $($async)? fn probe(&mut self) -> Result<(), Error<I::Error>> {
                let id = self.read_register(reg::WHO_AM_I) $($aw)* ?;
                if id != WHO_AM_I {
                    return Err(Error::UnexpectedId(u32::from(id)));
                }
                Ok(())
            }

            /// Reset the configuration and user registers to their defaults
            /// (power-down). Call `configure()` afterwards.
            pub $($async)? fn soft_reset(&mut self) -> Result<(), Error<I::Error>> {
                self.threshold = None;
                self.write_register(reg::CTRL_REG2, CTRL_REG2_SOFT_RST) $($aw)*
            }

            /// Write the stored configuration and threshold interrupt.
            /// Returns 0 in single-conversion mode, or the time to the first
            /// sample when a threshold interrupt starts continuous mode.
            pub $($async)? fn configure(&mut self) -> Result<WaitUs, Error<I::Error>> {
                for (r, v) in config_writes(&self.config, self.threshold.as_ref()) {
                    self.write_register(r, v) $($aw)* ?;
                }
                Ok(if self.threshold.is_some() {
                    self.config.performance.conversion_us()
                } else {
                    0
                })
            }

            /// Store a new configuration and write it.
            pub $($async)? fn set_config(&mut self, config: Config) -> Result<WaitUs, Error<I::Error>> {
                self.config = config;
                self.configure() $($aw)*
            }

            /// Enable the threshold interrupt and switch to continuous mode
            /// at [`Config::data_rate`].
            pub $($async)? fn enable_threshold(&mut self, threshold: Threshold) -> Result<WaitUs, Error<I::Error>> {
                self.threshold = Some(threshold);
                self.configure() $($aw)*
            }

            /// Disable the threshold interrupt and return to power-down
            /// (single conversions on request).
            pub $($async)? fn disable_threshold(&mut self) -> Result<(), Error<I::Error>> {
                self.threshold = None;
                self.configure() $($aw)* ?;
                Ok(())
            }

            /// Read `INT_SRC`; this also releases a latched INT pin.
            pub $($async)? fn interrupt_source(&mut self) -> Result<IntSource, Error<I::Error>> {
                Ok(IntSource(self.read_register(reg::INT_SRC) $($aw)* ?))
            }

            /// Power-down (≈1 µA); stops a running threshold interrupt
            /// until `wake()` / `configure()`.
            pub $($async)? fn power_down(&mut self) -> Result<(), Error<I::Error>> {
                self.write_register(reg::CTRL_REG3, CTRL_REG3_POWER_DOWN) $($aw)*
            }
        }

        impl<I: $Iface> zigbee_sensor::$mode::Sensor for Lis3mdl<I> {
            type BusError = I::Error;
            type Measurement = Measurement;

            /// Single-conversion mode: trigger one conversion and return
            /// its duration. Continuous mode: 0.
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I::Error>> {
                if self.threshold.is_some() {
                    return Ok(0);
                }
                self.write_register(reg::CTRL_REG3, CTRL_REG3_SINGLE) $($aw)* ?;
                Ok(self.config.performance.conversion_us())
            }

            /// [`Error::NotReady`] until `STATUS.ZYXDA` reports a new sample.
            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<I::Error>> {
                let status = self.read_register(reg::STATUS) $($aw)* ?;
                if status & STATUS_ZYXDA == 0 {
                    return Err(Error::NotReady);
                }
                let mut b = [0u8; 6];
                self.iface.read_regs(reg::OUT_X_L, &mut b) $($aw)* .map_err(Error::bus)?;
                Ok(decode(&b, self.config.range))
            }
        }

        impl<I: $Iface> zigbee_sensor::$mode::LowPower for Lis3mdl<I> {
            type BusError = I::Error;

            /// [`Self::power_down`].
            $($async)? fn sleep(&mut self) -> Result<(), Error<I::Error>> {
                self.power_down() $($aw)*
            }

            /// Restart continuous mode if a threshold interrupt is enabled;
            /// nothing to do in single-conversion mode.
            $($async)? fn wake(&mut self) -> Result<WaitUs, Error<I::Error>> {
                if self.threshold.is_none() {
                    return Ok(0);
                }
                self.write_register(reg::CTRL_REG3, CTRL_REG3_CONTINUOUS) $($aw)* ?;
                Ok(self.config.performance.conversion_us())
            }
        }
    };
}

driver!("Blocking driver.", Interface, blocking, [], []);

/// Async driver.
pub mod asynch {
    use super::*;

    driver!(
        "Async twin of [`super::Lis3mdl`].",
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
    use std::vec;
    use std::vec::Vec;
    use zigbee_sensor::blocking::{LowPower, Sensor};

    const A: u8 = ADDRESS_SDO_HIGH;

    fn rd(reg: u8, data: &[u8]) -> I2cTx {
        I2cTx::write_read(A, vec![reg | 0x80], data.to_vec())
    }
    fn wr(reg: u8, value: u8) -> I2cTx {
        I2cTx::write(A, vec![reg, value])
    }

    #[test]
    fn datasheet_sensitivity_and_threshold() {
        // Table 3: 6842 LSB/gauss at ±4 gauss, 1711 LSB/gauss at ±16 gauss.
        assert_eq!(raw_to_nt(6842, Range::Gauss4), 100_000);
        assert_eq!(raw_to_nt(-1711, Range::Gauss16), -100_000);
        assert_eq!(raw_to_nt(i16::MIN, Range::Gauss16), -1_915_137);
        assert_eq!(raw_to_nt(1, Range::Gauss4), 15);
        // 0.5 gauss at ±4 gauss = 3421 LSB; huge thresholds clamp to 15 bits.
        assert_eq!(threshold_code(50_000, Range::Gauss4), 3421);
        assert_eq!(threshold_code(u32::MAX, Range::Gauss4), 0x7FFF);
        // AN4602 Table 8.
        assert_eq!(Performance::LowPower.conversion_us(), 1_200);
        assert_eq!(Performance::UltraHigh.conversion_us(), 6_650);
    }

    #[test]
    fn probe_who_am_i() {
        let mut i2c = Mock::new(&[rd(reg::WHO_AM_I, &[0x3D]), rd(reg::WHO_AM_I, &[0x33])]);
        let mut mag = Lis3mdl::new_i2c(&mut i2c, A);
        mag.probe().unwrap();
        assert_eq!(mag.probe(), Err(Error::UnexpectedId(0x33)));
        i2c.done();
    }

    #[test]
    fn single_conversion_flow() {
        let mut i2c = Mock::new(&[
            wr(reg::CTRL_REG1, 0x70),
            wr(reg::CTRL_REG2, 0x00),
            wr(reg::CTRL_REG4, 0x0C),
            wr(reg::CTRL_REG5, 0x40),
            wr(reg::INT_THS_L, 0),
            wr(reg::INT_THS_H, 0),
            wr(reg::INT_CFG, 0),
            wr(reg::CTRL_REG3, 0x03),
            wr(reg::CTRL_REG3, 0x01),
            rd(reg::STATUS, &[0x07]),
            rd(reg::STATUS, &[0x0F]),
            // X = +6842, Y = -6842, Z = 0
            rd(reg::OUT_X_L, &[0xBA, 0x1A, 0x46, 0xE5, 0x00, 0x00]),
            wr(reg::CTRL_REG3, 0x03),
        ]);
        let mut mag = Lis3mdl::new_i2c(&mut i2c, A);
        assert_eq!(mag.configure().unwrap(), 0);
        assert_eq!(mag.start_measurement().unwrap(), 6_650);
        assert_eq!(mag.read_measurement(), Err(Error::NotReady));
        let m = mag.read_measurement().unwrap();
        assert_eq!(
            m.field,
            Vector3 {
                x: 100_000,
                y: -100_000,
                z: 0
            }
        );
        mag.sleep().unwrap();
        assert_eq!(mag.wake().unwrap(), 0);
        i2c.done();
    }

    #[test]
    fn threshold_interrupt_runs_continuously() {
        let cfg = Config {
            performance: Performance::LowPower,
            range: Range::Gauss8,
            data_rate: DataRate::Hz0_625,
        };
        let th = Threshold {
            threshold_nt: 200_000,
            z: false,
            latched: false,
            ..Threshold::default()
        };
        let mut i2c = Mock::new(&[
            wr(reg::CTRL_REG1, 0x00),
            wr(reg::CTRL_REG2, 0x20),
            wr(reg::CTRL_REG4, 0x00),
            wr(reg::CTRL_REG5, 0x40),
            // 2 gauss * 3421 = 6842 = 0x1ABA
            wr(reg::INT_THS_L, 0xBA),
            wr(reg::INT_THS_H, 0x1A),
            wr(reg::INT_CFG, 0xC7),
            wr(reg::CTRL_REG3, 0x00),
            rd(reg::INT_SRC, &[0x81]),
            wr(reg::CTRL_REG3, 0x03),
            wr(reg::CTRL_REG3, 0x00),
            wr(reg::CTRL_REG2, 0x04),
        ]);
        let mut mag = Lis3mdl::new_i2c(&mut i2c, A).with_config(cfg);
        assert_eq!(mag.enable_threshold(th).unwrap(), 1_200);
        assert_eq!(mag.start_measurement().unwrap(), 0);
        let src = mag.interrupt_source().unwrap();
        assert!(src.active() && !src.overflow());
        assert_eq!(src.positive(), [true, false, false]);
        mag.sleep().unwrap();
        assert_eq!(mag.wake().unwrap(), 1_200);
        mag.soft_reset().unwrap();
        assert!(!mag.continuous());
        i2c.done();
    }

    #[test]
    fn spi_read_sets_read_and_increment_bits() {
        let mut ex: Vec<SpiTx<u8>> = Vec::new();
        ex.extend([
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![reg::CTRL_REG3, 0x01]),
            SpiTx::transaction_end(),
        ]);
        for (cmd, data) in [(0xE7, vec![0x08]), (0xE8, vec![0, 0, 0, 0, 0x57, 0x06])] {
            ex.extend([
                SpiTx::transaction_start(),
                SpiTx::write_vec(vec![cmd]),
                SpiTx::read_vec(data),
                SpiTx::transaction_end(),
            ]);
        }
        let mut spi = SpiMock::new(&ex);
        let mut mag = Lis3mdl::new_spi(&mut spi).with_config(Config {
            range: Range::Gauss16,
            ..Config::default()
        });
        mag.start_measurement().unwrap();
        // 0x0657 = 1623 LSB at 1711 LSB/gauss = 94 856.8 nT
        assert_eq!(mag.read_measurement().unwrap().field.z, 94_857);
        spi.done();
    }

    #[test]
    fn async_single_conversion() {
        use zigbee_sensor::asynch::Sensor as _;

        let mut i2c = Mock::new(&[
            wr(reg::CTRL_REG3, 0x01),
            rd(reg::STATUS, &[0x08]),
            rd(reg::OUT_X_L, &[0xBA, 0x1A, 0, 0, 0, 0]),
        ]);
        let mut mag = asynch::Lis3mdl::new_i2c(&mut i2c, A);
        assert_eq!(block_on(mag.start_measurement()).unwrap(), 6_650);
        assert_eq!(block_on(mag.read_measurement()).unwrap().field.x, 100_000);
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
