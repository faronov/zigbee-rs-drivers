//! ST LIS2MDL / IIS2MDC 3-axis magnetometer over I2C or SPI.
//!
//! The IIS2MDC (industrial grade) has the same register map and
//! `WHO_AM_I` (0x40) as the LIS2MDL, so one driver covers both.
//!
//! By default the part idles (≈1.5 µA) and every
//! [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//! triggers one single measurement, returning the turn-on time (datasheet
//! Table 12); the part returns to idle by itself. With a [`Threshold`]
//! interrupt enabled it runs in continuous mode at [`Config::data_rate`]
//! instead. Optional offset cancellation (set/reset) and hard-iron offset
//! registers are supported.
//!
//! The field is reported as [`Quantity::MagneticField`] in nT
//! (1.5 mgauss/LSB = 150 nT/LSB, fixed ±50 gauss range).
//!
//! References: ST "LIS2MDL" datasheet DocID030621 Rev 5 (November 2018);
//! IIS2MDC register map cross-checked against ST's `iis2mdc_reg.h`.
//!
//! ```ignore
//! let mut mag = Lis2mdl::new_i2c(i2c);
//! mag.probe()?;
//! mag.configure()?;
//! let wait = mag.start_measurement()?; // single measurement
//! delay_us(wait);
//! let m = mag.read_measurement()?; // m.field in nT
//! ```

#![no_std]

mod interface;

pub use interface::{AsyncInterface, I2cInterface, Interface, SpiInterface};
use zigbee_sensor::{Error, Quantity, Readings, Vector3, WaitUs};

/// Fixed I2C address.
pub const ADDRESS: u8 = 0x1E;

/// `WHO_AM_I` value (LIS2MDL and IIS2MDC).
pub const WHO_AM_I: u8 = 0x40;

/// Register addresses.
pub mod reg {
    #![allow(missing_docs)]
    pub const OFFSET_X_REG_L: u8 = 0x45;
    pub const WHO_AM_I: u8 = 0x4F;
    pub const CFG_REG_A: u8 = 0x60;
    pub const CFG_REG_B: u8 = 0x61;
    pub const CFG_REG_C: u8 = 0x62;
    pub const INT_CTRL_REG: u8 = 0x63;
    pub const INT_SOURCE_REG: u8 = 0x64;
    pub const INT_THS_L_REG: u8 = 0x65;
    pub const INT_THS_H_REG: u8 = 0x66;
    pub const STATUS_REG: u8 = 0x67;
    pub const OUTX_L_REG: u8 = 0x68;
}

/// Must be set for proper operation (datasheet Table 24).
const CFG_A_COMP_TEMP_EN: u8 = 0x80;
const CFG_A_SOFT_RST: u8 = 0x20;
const CFG_A_LP: u8 = 0x10;
const MD_CONTINUOUS: u8 = 0x00;
const MD_SINGLE: u8 = 0x01;
const MD_IDLE: u8 = 0x03;
const CFG_B_OFF_CANC_ONE_SHOT: u8 = 0x10;
const CFG_B_INT_ON_DATAOFF: u8 = 0x08;
const CFG_B_OFF_CANC: u8 = 0x02;
const CFG_B_LPF: u8 = 0x01;
const CFG_C_INT_ON_PIN: u8 = 0x40;
const CFG_C_BDU: u8 = 0x10;
const CFG_C_4WSPI: u8 = 0x04;
const CFG_C_DRDY_ON_PIN: u8 = 0x01;
const INT_XIEN: u8 = 0x80;
const INT_YIEN: u8 = 0x40;
const INT_ZIEN: u8 = 0x20;
const INT_IEA: u8 = 0x04;
const INT_IEL: u8 = 0x02;
const INT_IEN: u8 = 0x01;
const STATUS_ZYXDA: u8 = 0x08;

/// Sensitivity: 1.5 mgauss/LSB = 150 nT/LSB.
pub const NT_PER_LSB: i32 = 150;

/// Power mode (`CFG_REG_A.LP`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Power {
    /// High resolution: 4.5 mgauss RMS noise, 100 µA at 10 Hz.
    #[default]
    HighResolution,
    /// Low power: 9 mgauss RMS noise, 25 µA at 10 Hz.
    LowPower,
}

impl Power {
    /// Turn-on time without offset cancellation (datasheet Table 12).
    pub const fn turn_on_us(self) -> WaitUs {
        match self {
            Self::HighResolution => 9_400,
            Self::LowPower => 6_400,
        }
    }
}

/// Output data rate (`CFG_REG_A.ODR`): continuous-mode rate, and the
/// `1/ODR` added to the turn-on time by offset cancellation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DataRate {
    /// 10 Hz.
    #[default]
    Hz10 = 0,
    /// 20 Hz.
    Hz20 = 1,
    /// 50 Hz.
    Hz50 = 2,
    /// 100 Hz.
    Hz100 = 3,
}

impl DataRate {
    /// Sample period, rounded up.
    pub const fn period_us(self) -> WaitUs {
        match self {
            Self::Hz10 => 100_000,
            Self::Hz20 => 50_000,
            Self::Hz50 => 20_000,
            Self::Hz100 => 10_000,
        }
    }
}

/// Measurement configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// Power mode (default high resolution).
    pub power: Power,
    /// Data rate (default 10 Hz).
    pub data_rate: DataRate,
    /// Digital low-pass filter, bandwidth ODR/4 instead of ODR/2.
    pub low_pass_filter: bool,
    /// Offset cancellation (`OFF_CANC`, plus `OFF_CANC_ONE_SHOT` in single
    /// mode); adds `1/ODR` to the measurement time (default off).
    pub offset_cancellation: bool,
    /// Drive data-ready on the INT/DRDY pin (push-pull; I2C / 3-wire SPI
    /// only).
    pub drdy_on_pin: bool,
}

/// Threshold interrupt: fires when the field on an enabled axis exceeds
/// `+threshold` or falls below `-threshold`. Runs the part continuously.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Threshold {
    /// Absolute threshold in nT (same for all axes), clamped to 16 bits.
    pub threshold_nt: u32,
    /// Enable the X axis.
    pub x: bool,
    /// Enable the Y axis.
    pub y: bool,
    /// Enable the Z axis.
    pub z: bool,
    /// INT active high (default) or low.
    pub active_high: bool,
    /// Keep INT asserted until [`Lis2mdl::interrupt_source`] reads
    /// `INT_SOURCE_REG` (default); otherwise INT is pulsed.
    pub latched: bool,
    /// Drive the interrupt on the INT/DRDY pin (push-pull).
    pub on_pin: bool,
    /// Compare the hard-iron corrected data instead of the raw data.
    pub after_hard_iron: bool,
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
            on_pin: true,
            after_hard_iron: false,
        }
    }
}

/// `INT_SOURCE_REG` contents.
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
    /// Flux density in nT (hard-iron offset already subtracted by the
    /// part if programmed).
    pub field: Vector3,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::MagneticField(self.field));
    }
}

/// Raw output count to nT.
pub const fn raw_to_nt(raw: i16) -> i32 {
    raw as i32 * NT_PER_LSB
}

/// nT to the nearest output count (saturating), for offsets.
pub fn nt_to_raw(nt: i32) -> i16 {
    let half = NT_PER_LSB / 2;
    let q = if nt >= 0 {
        (nt.saturating_add(half)) / NT_PER_LSB
    } else {
        (nt.saturating_sub(half)) / NT_PER_LSB
    };
    q.clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16
}

/// Threshold in nT to the unsigned `INT_THS` code.
pub fn threshold_code(threshold_nt: u32) -> u16 {
    let code = (u64::from(threshold_nt) + 75) / 150;
    code.min(u64::from(u16::MAX)) as u16
}

/// Decode `OUTX_L_REG..OUTZ_H_REG` (little-endian) to nT.
pub fn decode(bytes: &[u8; 6]) -> Measurement {
    let axis = |i: usize| raw_to_nt(i16::from_le_bytes([bytes[i], bytes[i + 1]]));
    Measurement {
        field: Vector3 {
            x: axis(0),
            y: axis(2),
            z: axis(4),
        },
    }
}

/// Time from a measurement request to data (datasheet Table 12).
pub fn measurement_us(config: &Config) -> WaitUs {
    let t = config.power.turn_on_us();
    if config.offset_cancellation {
        t + config.data_rate.period_us()
    } else {
        t
    }
}

/// `CFG_REG_A` value for operating mode `md`.
pub fn cfg_reg_a(config: &Config, md: u8) -> u8 {
    let lp = match config.power {
        Power::HighResolution => 0,
        Power::LowPower => CFG_A_LP,
    };
    CFG_A_COMP_TEMP_EN | lp | (config.data_rate as u8) << 2 | md
}

/// `CFG_REG_B` value.
pub fn cfg_reg_b(config: &Config, threshold: Option<&Threshold>) -> u8 {
    let mut v = 0;
    if config.low_pass_filter {
        v |= CFG_B_LPF;
    }
    if config.offset_cancellation {
        v |= CFG_B_OFF_CANC;
        if threshold.is_none() {
            v |= CFG_B_OFF_CANC_ONE_SHOT;
        }
    }
    if threshold.is_some_and(|t| t.after_hard_iron) {
        v |= CFG_B_INT_ON_DATAOFF;
    }
    v
}

/// `CFG_REG_C` value: BDU always; 4-wire SPI if requested.
pub fn cfg_reg_c(config: &Config, threshold: Option<&Threshold>, four_wire_spi: bool) -> u8 {
    let mut v = CFG_C_BDU;
    if four_wire_spi {
        v |= CFG_C_4WSPI;
    }
    if config.drdy_on_pin {
        v |= CFG_C_DRDY_ON_PIN;
    }
    if threshold.is_some_and(|t| t.on_pin) {
        v |= CFG_C_INT_ON_PIN;
    }
    v
}

/// `INT_CTRL_REG` value (0 = disabled).
pub fn int_ctrl(threshold: Option<&Threshold>) -> u8 {
    let Some(t) = threshold else { return 0 };
    let mut v = INT_IEN;
    if t.x {
        v |= INT_XIEN;
    }
    if t.y {
        v |= INT_YIEN;
    }
    if t.z {
        v |= INT_ZIEN;
    }
    if t.active_high {
        v |= INT_IEA;
    }
    if t.latched {
        v |= INT_IEL;
    }
    v
}

/// Register writes for [`Lis2mdl::configure`], ending with `CFG_REG_A`
/// (idle, or continuous with a threshold interrupt).
pub fn config_writes(
    config: &Config,
    threshold: Option<&Threshold>,
    four_wire_spi: bool,
) -> [(u8, u8); 6] {
    let ths = threshold.map_or(0, |t| threshold_code(t.threshold_nt));
    let md = if threshold.is_some() {
        MD_CONTINUOUS
    } else {
        MD_IDLE
    };
    [
        (reg::CFG_REG_C, cfg_reg_c(config, threshold, four_wire_spi)),
        (reg::CFG_REG_B, cfg_reg_b(config, threshold)),
        (reg::INT_THS_L_REG, ths as u8),
        (reg::INT_THS_H_REG, (ths >> 8) as u8),
        (reg::INT_CTRL_REG, int_ctrl(threshold)),
        (reg::CFG_REG_A, cfg_reg_a(config, md)),
    ]
}

macro_rules! driver {
    ($doc:literal, $Iface:ident, $mode:ident, [$($async:tt)?], [$($aw:tt)*]) => {
        #[doc = $doc]
        #[derive(Debug)]
        pub struct Lis2mdl<I> {
            iface: I,
            config: Config,
            threshold: Option<Threshold>,
            four_wire_spi: bool,
        }

        impl<I> Lis2mdl<I> {
            /// Create a driver over any register interface; no bus traffic.
            pub fn new(iface: I) -> Self {
                Self {
                    iface,
                    config: Config::default(),
                    threshold: None,
                    four_wire_spi: false,
                }
            }

            /// Set the configuration used by `configure()`.
            pub fn with_config(mut self, config: Config) -> Self {
                self.config = config;
                self
            }

            /// Enable 4-wire SPI (`CFG_REG_C.4WSPI`, written before every
            /// read in `probe()` and by `configure()`). Set by
            /// `new_spi()`; the INT/DRDY pin becomes SDO.
            pub fn with_four_wire_spi(mut self, enable: bool) -> Self {
                self.four_wire_spi = enable;
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

        impl<I2C> Lis2mdl<I2cInterface<I2C>> {
            /// Create an I2C driver at [`ADDRESS`]; no bus traffic.
            pub fn new_i2c(i2c: I2C) -> Self {
                Self::new(I2cInterface::new(i2c, ADDRESS))
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.iface.release()
            }
        }

        impl<SPI> Lis2mdl<SpiInterface<SPI>> {
            /// Create a 4-wire SPI driver; no bus traffic.
            pub fn new_spi(spi: SPI) -> Self {
                Self::new(SpiInterface::new(spi)).with_four_wire_spi(true)
            }

            /// Give the device back.
            pub fn release(self) -> SPI {
                self.iface.release()
            }
        }

        impl<I: $Iface> Lis2mdl<I> {
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

            /// Read `WHO_AM_I` (0x40). Over 4-wire SPI, `CFG_REG_C` is
            /// written first so the part drives SDO.
            pub $($async)? fn probe(&mut self) -> Result<(), Error<I::Error>> {
                if self.four_wire_spi {
                    let c = cfg_reg_c(&self.config, self.threshold.as_ref(), true);
                    self.write_register(reg::CFG_REG_C, c) $($aw)* ?;
                }
                let id = self.read_register(reg::WHO_AM_I) $($aw)* ?;
                if id != WHO_AM_I {
                    return Err(Error::UnexpectedId(u32::from(id)));
                }
                Ok(())
            }

            /// Reset the configuration and user registers (back to idle,
            /// 3-wire SPI). Call `configure()` afterwards.
            pub $($async)? fn soft_reset(&mut self) -> Result<(), Error<I::Error>> {
                self.threshold = None;
                self.write_register(reg::CFG_REG_A, CFG_A_SOFT_RST | MD_IDLE) $($aw)*
            }

            /// Write the stored configuration and threshold interrupt.
            /// Returns 0 in single mode, or the time to the first sample
            /// when a threshold interrupt starts continuous mode.
            pub $($async)? fn configure(&mut self) -> Result<WaitUs, Error<I::Error>> {
                let writes = config_writes(&self.config, self.threshold.as_ref(), self.four_wire_spi);
                for (r, v) in writes {
                    self.write_register(r, v) $($aw)* ?;
                }
                Ok(if self.threshold.is_some() {
                    measurement_us(&self.config)
                } else {
                    0
                })
            }

            /// Store a new configuration and write it.
            pub $($async)? fn set_config(&mut self, config: Config) -> Result<WaitUs, Error<I::Error>> {
                self.config = config;
                self.configure() $($aw)*
            }

            /// Program the hard-iron offset (in nT, rounded to 150 nT
            /// steps); the part subtracts it from the output data.
            pub $($async)? fn set_hard_iron_offset(&mut self, offset: Vector3) -> Result<(), Error<I::Error>> {
                let mut r = reg::OFFSET_X_REG_L;
                for v in [offset.x, offset.y, offset.z] {
                    let [lo, hi] = nt_to_raw(v).to_le_bytes();
                    self.write_register(r, lo) $($aw)* ?;
                    self.write_register(r + 1, hi) $($aw)* ?;
                    r += 2;
                }
                Ok(())
            }

            /// Enable the threshold interrupt and switch to continuous mode
            /// at [`Config::data_rate`].
            pub $($async)? fn enable_threshold(&mut self, threshold: Threshold) -> Result<WaitUs, Error<I::Error>> {
                self.threshold = Some(threshold);
                self.configure() $($aw)*
            }

            /// Disable the threshold interrupt and return to idle (single
            /// measurements on request).
            pub $($async)? fn disable_threshold(&mut self) -> Result<(), Error<I::Error>> {
                self.threshold = None;
                self.configure() $($aw)* ?;
                Ok(())
            }

            /// Read `INT_SOURCE_REG`; this also releases a latched INT.
            pub $($async)? fn interrupt_source(&mut self) -> Result<IntSource, Error<I::Error>> {
                Ok(IntSource(self.read_register(reg::INT_SOURCE_REG) $($aw)* ?))
            }

            /// Idle mode (≈1.5 µA); stops a running threshold interrupt
            /// until `wake()` / `configure()`.
            pub $($async)? fn power_down(&mut self) -> Result<(), Error<I::Error>> {
                let a = cfg_reg_a(&self.config, MD_IDLE);
                self.write_register(reg::CFG_REG_A, a) $($aw)*
            }
        }

        impl<I: $Iface> zigbee_sensor::$mode::Sensor for Lis2mdl<I> {
            type BusError = I::Error;
            type Measurement = Measurement;

            /// Single mode: trigger one measurement and return its
            /// duration. Continuous mode: 0.
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I::Error>> {
                if self.threshold.is_some() {
                    return Ok(0);
                }
                let a = cfg_reg_a(&self.config, MD_SINGLE);
                self.write_register(reg::CFG_REG_A, a) $($aw)* ?;
                Ok(measurement_us(&self.config))
            }

            /// [`Error::NotReady`] until `STATUS_REG.ZYXDA` reports a new
            /// sample.
            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<I::Error>> {
                let status = self.read_register(reg::STATUS_REG) $($aw)* ?;
                if status & STATUS_ZYXDA == 0 {
                    return Err(Error::NotReady);
                }
                let mut b = [0u8; 6];
                self.iface.read_regs(reg::OUTX_L_REG, &mut b) $($aw)* .map_err(Error::bus)?;
                Ok(decode(&b))
            }
        }

        impl<I: $Iface> zigbee_sensor::$mode::LowPower for Lis2mdl<I> {
            type BusError = I::Error;

            /// [`Self::power_down`].
            $($async)? fn sleep(&mut self) -> Result<(), Error<I::Error>> {
                self.power_down() $($aw)*
            }

            /// Restart continuous mode if a threshold interrupt is enabled;
            /// nothing to do in single mode.
            $($async)? fn wake(&mut self) -> Result<WaitUs, Error<I::Error>> {
                if self.threshold.is_none() {
                    return Ok(0);
                }
                let a = cfg_reg_a(&self.config, MD_CONTINUOUS);
                self.write_register(reg::CFG_REG_A, a) $($aw)* ?;
                Ok(measurement_us(&self.config))
            }
        }
    };
}

driver!("Blocking driver.", Interface, blocking, [], []);

/// Async driver.
pub mod asynch {
    use super::*;

    driver!(
        "Async twin of [`super::Lis2mdl`].",
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

    const A: u8 = ADDRESS;

    fn rd(reg: u8, data: &[u8]) -> I2cTx {
        I2cTx::write_read(A, vec![reg | 0x80], data.to_vec())
    }
    fn wr(reg: u8, value: u8) -> I2cTx {
        I2cTx::write(A, vec![reg, value])
    }

    #[test]
    fn datasheet_sensitivity_and_timing() {
        // Table 2: 1.5 mgauss/LSB.
        assert_eq!(raw_to_nt(1), 150);
        assert_eq!(raw_to_nt(-1000), -150_000);
        assert_eq!(raw_to_nt(i16::MIN), -4_915_200);
        assert_eq!(nt_to_raw(-150_074), -1000);
        assert_eq!(nt_to_raw(i32::MAX), i16::MAX);
        assert_eq!(threshold_code(50_000), 333);
        assert_eq!(threshold_code(u32::MAX), u16::MAX);
        // Table 12: 9.4 ms HR, 6.4 ms LP, + 1/ODR with offset cancellation.
        let mut cfg = Config::default();
        assert_eq!(measurement_us(&cfg), 9_400);
        cfg.power = Power::LowPower;
        cfg.offset_cancellation = true;
        cfg.data_rate = DataRate::Hz100;
        assert_eq!(measurement_us(&cfg), 16_400);
    }

    #[test]
    fn probe_who_am_i() {
        let mut i2c = Mock::new(&[rd(reg::WHO_AM_I, &[0x40]), rd(reg::WHO_AM_I, &[0x3D])]);
        let mut mag = Lis2mdl::new_i2c(&mut i2c);
        mag.probe().unwrap();
        assert_eq!(mag.probe(), Err(Error::UnexpectedId(0x3D)));
        i2c.done();
    }

    #[test]
    fn single_measurement_with_offset_cancellation() {
        let cfg = Config {
            offset_cancellation: true,
            ..Config::default()
        };
        let mut i2c = Mock::new(&[
            wr(reg::CFG_REG_C, 0x10),
            wr(reg::CFG_REG_B, 0x12),
            wr(reg::INT_THS_L_REG, 0),
            wr(reg::INT_THS_H_REG, 0),
            wr(reg::INT_CTRL_REG, 0),
            wr(reg::CFG_REG_A, 0x83),
            wr(reg::CFG_REG_A, 0x81),
            rd(reg::STATUS_REG, &[0x00]),
            rd(reg::STATUS_REG, &[0x0F]),
            // X = +1000, Y = -1000, Z = 1
            rd(reg::OUTX_L_REG, &[0xE8, 0x03, 0x18, 0xFC, 0x01, 0x00]),
            wr(reg::CFG_REG_A, 0x83),
        ]);
        let mut mag = Lis2mdl::new_i2c(&mut i2c).with_config(cfg);
        assert_eq!(mag.configure().unwrap(), 0);
        assert_eq!(mag.start_measurement().unwrap(), 109_400);
        assert_eq!(mag.read_measurement(), Err(Error::NotReady));
        assert_eq!(
            mag.read_measurement().unwrap().field,
            Vector3 {
                x: 150_000,
                y: -150_000,
                z: 150
            }
        );
        mag.sleep().unwrap();
        assert_eq!(mag.wake().unwrap(), 0);
        i2c.done();
    }

    #[test]
    fn threshold_interrupt_and_hard_iron() {
        let cfg = Config {
            power: Power::LowPower,
            data_rate: DataRate::Hz20,
            ..Config::default()
        };
        let th = Threshold {
            threshold_nt: 300_000,
            y: false,
            active_high: false,
            latched: false,
            after_hard_iron: true,
            ..Threshold::default()
        };
        let mut i2c = Mock::new(&[
            // -150 nT -> -1 = 0xFFFF; 300 nT -> 2; 0
            wr(reg::OFFSET_X_REG_L, 0xFF),
            wr(reg::OFFSET_X_REG_L + 1, 0xFF),
            wr(reg::OFFSET_X_REG_L + 2, 0x02),
            wr(reg::OFFSET_X_REG_L + 3, 0x00),
            wr(reg::OFFSET_X_REG_L + 4, 0x00),
            wr(reg::OFFSET_X_REG_L + 5, 0x00),
            wr(reg::CFG_REG_C, 0x50),
            wr(reg::CFG_REG_B, 0x08),
            // 300 000 nT / 150 = 2000 = 0x07D0
            wr(reg::INT_THS_L_REG, 0xD0),
            wr(reg::INT_THS_H_REG, 0x07),
            wr(reg::INT_CTRL_REG, 0xA1),
            wr(reg::CFG_REG_A, 0x94),
            rd(reg::INT_SOURCE_REG, &[0x09]),
            wr(reg::CFG_REG_A, 0x97),
            wr(reg::CFG_REG_A, 0x94),
            wr(reg::CFG_REG_C, 0x10),
            wr(reg::CFG_REG_B, 0x00),
            wr(reg::INT_THS_L_REG, 0),
            wr(reg::INT_THS_H_REG, 0),
            wr(reg::INT_CTRL_REG, 0),
            wr(reg::CFG_REG_A, 0x97),
        ]);
        let mut mag = Lis2mdl::new_i2c(&mut i2c).with_config(cfg);
        mag.set_hard_iron_offset(Vector3 {
            x: -150,
            y: 300,
            z: 0,
        })
        .unwrap();
        assert_eq!(mag.enable_threshold(th).unwrap(), 6_400);
        assert_eq!(mag.start_measurement().unwrap(), 0);
        let src = mag.interrupt_source().unwrap();
        assert!(src.active());
        assert_eq!(src.negative(), [false, true, false]);
        mag.sleep().unwrap();
        assert_eq!(mag.wake().unwrap(), 6_400);
        mag.disable_threshold().unwrap();
        assert!(!mag.continuous());
        i2c.done();
    }

    #[test]
    fn spi_enables_four_wire_before_reads() {
        let mut ex: Vec<SpiTx<u8>> = Vec::new();
        let w = |r: u8, v: u8| {
            [
                SpiTx::transaction_start(),
                SpiTx::write_vec(vec![r, v]),
                SpiTx::transaction_end(),
            ]
        };
        let r = |cmd: u8, data: Vec<u8>| {
            [
                SpiTx::transaction_start(),
                SpiTx::write_vec(vec![cmd]),
                SpiTx::read_vec(data),
                SpiTx::transaction_end(),
            ]
        };
        ex.extend(w(reg::CFG_REG_C, 0x14));
        ex.extend(r(0xCF, vec![0x40]));
        ex.extend(w(reg::CFG_REG_A, 0x81));
        ex.extend(r(0xE7, vec![0x08]));
        ex.extend(r(0xE8, vec![0, 0, 0, 0, 0x64, 0x00]));
        ex.extend(w(reg::CFG_REG_A, 0x20 | 0x03));
        let mut spi = SpiMock::new(&ex);
        let mut mag = Lis2mdl::new_spi(&mut spi);
        mag.probe().unwrap();
        assert_eq!(mag.start_measurement().unwrap(), 9_400);
        assert_eq!(mag.read_measurement().unwrap().field.z, 15_000);
        mag.soft_reset().unwrap();
        spi.done();
    }

    #[test]
    fn async_single_measurement() {
        use zigbee_sensor::asynch::Sensor as _;

        let mut i2c = Mock::new(&[
            wr(reg::CFG_REG_A, 0x81),
            rd(reg::STATUS_REG, &[0x08]),
            rd(reg::OUTX_L_REG, &[0xE8, 0x03, 0, 0, 0, 0]),
        ]);
        let mut mag = asynch::Lis2mdl::new_i2c(&mut i2c);
        assert_eq!(block_on(mag.start_measurement()).unwrap(), 9_400);
        assert_eq!(block_on(mag.read_measurement()).unwrap().field.x, 150_000);
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
