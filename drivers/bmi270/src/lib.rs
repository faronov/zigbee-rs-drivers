//! Bosch BMI270 accelerometer and gyroscope over I2C or SPI.
//!
//! After power-on or soft reset the BMI270 must receive Bosch's 8 KB
//! configuration file before it can be used. With the default
//! `config-blob` cargo feature the file is bundled ([`CONFIG_FILE`],
//! BSD-3-Clause, see `src/config_file.rs`); without it pass the file to
//! [`Bmi270::with_config_file`].
//!
//! The driver configures the part in steps, each returning a [`WaitUs`]
//! (continue with [`Bmi270::continue_configure`] until it returns 0):
//!
//! 1. [`Bmi270::configure`]: soft reset (2 ms) unless the file is already
//!    loaded; then advanced power save off (450 µs);
//! 2. upload the configuration file in bursts of
//!    [`Bmi270::with_burst_len`] bytes and complete the load (20 ms);
//! 3. check `INTERNAL_STATUS`, write the sensor, any-motion and power
//!    registers and return the sensor start-up time (accelerometer 2 ms,
//!    gyroscope 45 ms);
//! 4. returns 0.
//!
//! `start_measurement()` performs the next step while configuration is
//! pending; `read_measurement()` returns [`Error::NotReady`] until it has
//! finished and `STATUS` reports new accelerometer data.
//!
//! Reference: Bosch Sensortec BST-BMI270-DS000-08 "BMI270 Datasheet";
//! feature-register defaults and the configuration file from Bosch's
//! BMI270 SensorAPI v2.86.1.
//!
//! ```ignore
//! let mut imu = Bmi270::new_i2c(i2c, ADDRESS_SDO_LOW);
//! imu.probe()?;
//! let mut wait = imu.configure()?;
//! while wait != 0 {
//!     delay_us(wait);
//!     wait = imu.continue_configure()?;
//! }
//! let m = imu.read_measurement()?; // NotReady until new data
//! ```

#![no_std]

#[cfg(feature = "config-blob")]
mod config_file;
mod interface;

#[cfg(feature = "config-blob")]
pub use config_file::CONFIG_FILE;
pub use interface::{AsyncInterface, I2cInterface, Interface, SpiInterface};
use zigbee_sensor::{Error, Quantity, Readings, Vector3, WaitUs};

/// I2C address with SDO low.
pub const ADDRESS_SDO_LOW: u8 = 0x68;
/// I2C address with SDO high.
pub const ADDRESS_SDO_HIGH: u8 = 0x69;

/// `CHIP_ID` value.
pub const CHIP_ID: u8 = 0x24;

/// Register addresses.
pub mod reg {
    #![allow(missing_docs)]
    pub const CHIP_ID: u8 = 0x00;
    pub const STATUS: u8 = 0x03;
    pub const DATA_ACC: u8 = 0x0C;
    pub const INT_STATUS_0: u8 = 0x1C;
    pub const INTERNAL_STATUS: u8 = 0x21;
    pub const FEAT_PAGE: u8 = 0x2F;
    /// `ANYMO_1` on feature page 1.
    pub const ANYMO_1: u8 = 0x3C;
    pub const ACC_CONF: u8 = 0x40;
    pub const ACC_RANGE: u8 = 0x41;
    pub const GYR_CONF: u8 = 0x42;
    pub const GYR_RANGE: u8 = 0x43;
    pub const INT1_IO_CTRL: u8 = 0x53;
    pub const INT_LATCH: u8 = 0x55;
    pub const INT1_MAP_FEAT: u8 = 0x56;
    pub const INIT_CTRL: u8 = 0x59;
    pub const INIT_ADDR_0: u8 = 0x5B;
    pub const INIT_ADDR_1: u8 = 0x5C;
    pub const INIT_DATA: u8 = 0x5E;
    pub const PWR_CONF: u8 = 0x7C;
    pub const PWR_CTRL: u8 = 0x7D;
    pub const CMD: u8 = 0x7E;
}

const CMD_SOFTRESET: u8 = 0xB6;
const STATUS_DRDY_ACC: u8 = 0x80;
const ANY_MOTION_OUT: u8 = 0x40;
const INTERNAL_STATUS_INIT_OK: u8 = 0x01;
const PWR_CONF_APS: u8 = 0x01;
const PWR_CTRL_GYR: u8 = 0x02;
const PWR_CTRL_ACC: u8 = 0x04;
/// INT1 output enabled, push-pull, active high.
const INT1_IO_OUTPUT_HIGH: u8 = 0x0A;
const FILTER_PERF: u8 = 0x80;
const BWP_NORMAL: u8 = 0x02;
/// `ANYMO_1`: all three axes selected.
const ANYMO_XYZ: u16 = 0xE000;
/// `ANYMO_2`: `out_conf` = bit 6 (reset value), feature enable.
const ANYMO_OUT_CONF: u16 = 0x7 << 11;
const ANYMO_ENABLE: u16 = 0x8000;
/// `ANYMO_1` / `ANYMO_2` reset values (duration 100 ms, 83 mg, disabled).
const ANYMO_DEFAULT: [u16; 2] = [0xE005, 0x38AA];

/// After soft reset or power-on, before the first access.
pub const SOFTRESET_WAIT_US: WaitUs = 2_000;
/// After disabling advanced power save.
pub const APS_OFF_WAIT_US: WaitUs = 450;
/// After `INIT_CTRL` = 1 until `INTERNAL_STATUS` reports `init_ok`.
pub const INIT_WAIT_US: WaitUs = 20_000;
/// Accelerometer suspend → normal.
pub const ACCEL_START_US: WaitUs = 2_000;
/// Gyroscope suspend → normal.
pub const GYRO_START_US: WaitUs = 45_000;

/// Accelerometer full scale.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AccelRange {
    /// ±2 g, 16384 LSB/g.
    #[default]
    G2,
    /// ±4 g.
    G4,
    /// ±8 g.
    G8,
    /// ±16 g.
    G16,
}

impl AccelRange {
    /// Full scale in mg.
    pub const fn full_scale_mg(self) -> i32 {
        2_000 << (self as u8)
    }
}

/// Gyroscope full scale.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GyroRange {
    /// ±2000 dps, 16.384 LSB/dps.
    #[default]
    Dps2000,
    /// ±1000 dps.
    Dps1000,
    /// ±500 dps.
    Dps500,
    /// ±250 dps.
    Dps250,
    /// ±125 dps.
    Dps125,
}

impl GyroRange {
    /// Full scale in dps.
    pub const fn full_scale_dps(self) -> i32 {
        2_000 >> (self as u8)
    }
}

/// Accelerometer output data rate (`acc_odr` = 25 Hz · 2^(code - 6)).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AccelOdr {
    /// 0.78 Hz (low-power mode only).
    Hz0_78 = 1,
    /// 1.5 Hz (low-power mode only).
    Hz1_5,
    /// 3.1 Hz (low-power mode only).
    Hz3_1,
    /// 6.25 Hz (low-power mode only).
    Hz6_25,
    /// 12.5 Hz.
    Hz12_5,
    /// 25 Hz.
    Hz25,
    /// 50 Hz.
    Hz50,
    /// 100 Hz.
    Hz100,
    /// 200 Hz.
    Hz200,
    /// 400 Hz.
    Hz400,
    /// 800 Hz.
    Hz800,
    /// 1600 Hz.
    Hz1600,
}

/// Gyroscope output data rate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GyroOdr {
    /// 25 Hz.
    #[default]
    Hz25 = 6,
    /// 50 Hz.
    Hz50,
    /// 100 Hz.
    Hz100,
    /// 200 Hz.
    Hz200,
    /// 400 Hz.
    Hz400,
    /// 800 Hz.
    Hz800,
    /// 1600 Hz.
    Hz1600,
    /// 3200 Hz.
    Hz3200,
}

/// Samples averaged per output in accelerometer low-power mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Averaging {
    /// No averaging (lowest current).
    #[default]
    X1,
    /// 2 samples.
    X2,
    /// 4 samples.
    X4,
    /// 8 samples.
    X8,
    /// 16 samples.
    X16,
    /// 32 samples.
    X32,
    /// 64 samples.
    X64,
    /// 128 samples.
    X128,
}

/// Accelerometer power mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AccelMode {
    /// Performance filter, advanced power save off, 210 µA; ODR
    /// 12.5–1600 Hz.
    #[default]
    Normal,
    /// Power-optimised under-sampling with advanced power save, down to
    /// 4 µA; gyroscope must be off.
    LowPower(Averaging),
}

/// Gyroscope settings (normal mode, 600 µA with the accelerometer at
/// 685 µA).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct GyroConfig {
    /// Full scale.
    pub range: GyroRange,
    /// Output data rate.
    pub odr: GyroOdr,
}

/// Device configuration, applied by [`Bmi270::configure`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Accelerometer full scale.
    pub accel_range: AccelRange,
    /// Accelerometer output data rate.
    pub accel_odr: AccelOdr,
    /// Accelerometer power mode.
    pub accel_mode: AccelMode,
    /// Gyroscope settings; `None` keeps it disabled.
    pub gyro: Option<GyroConfig>,
}

impl Default for Config {
    /// ±2 g, 25 Hz, normal mode, gyroscope off.
    fn default() -> Self {
        Self {
            accel_range: AccelRange::G2,
            accel_odr: AccelOdr::Hz25,
            accel_mode: AccelMode::Normal,
            gyro: None,
        }
    }
}

/// Any-motion feature settings (INT1, all axes).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WakeOnMotion {
    /// Slope threshold in mg, 1 g / 2048 per LSB, clamped to 1..=2047 LSB
    /// (≈ 1 g).
    pub threshold_mg: u32,
    /// Time above the threshold in ms, 20 ms (one 50 Hz sample) per LSB,
    /// up to 163 s.
    pub duration_ms: u32,
    /// Hold INT1 until `INT_STATUS_0` is read (else INT1 follows the
    /// motion condition).
    pub latched: bool,
}

impl Default for WakeOnMotion {
    fn default() -> Self {
        Self {
            threshold_mg: 83,
            duration_ms: 100,
            latched: true,
        }
    }
}

/// One IMU sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Acceleration in milli-g.
    pub accel: Vector3,
    /// Angular rate in milli-degrees per second, if the gyroscope is on.
    pub gyro: Option<Vector3>,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Acceleration(self.accel));
        if let Some(g) = self.gyro {
            f(Quantity::AngularRate(g));
        }
    }
}

/// Up to eight register writes computed without bus traffic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RegWrites {
    buf: [(u8, u8); 8],
    len: usize,
}

impl RegWrites {
    const fn new() -> Self {
        Self {
            buf: [(0, 0); 8],
            len: 0,
        }
    }

    fn push(&mut self, reg: u8, value: u8) {
        self.buf[self.len] = (reg, value);
        self.len += 1;
    }

    /// The `(register, value)` pairs in write order.
    pub fn as_slice(&self) -> &[(u8, u8)] {
        &self.buf[..self.len]
    }
}

/// Register writes for the final configuration step: `pre`, then the
/// `ANYMO_1`/`ANYMO_2` burst at [`reg::ANYMO_1`] (feature page 1), then
/// `post` (interrupt and power registers, power configuration last).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigPlan {
    /// Sensor configuration and feature page select.
    pub pre: RegWrites,
    /// `ANYMO_1` and `ANYMO_2`, little-endian.
    pub any_motion: [u8; 4],
    /// Interrupt pin, power control and power configuration.
    pub post: RegWrites,
}

/// Raw accelerometer count to mg (full scale / 32768).
pub fn accel_mg(raw: i16, range: AccelRange) -> i32 {
    let n = i32::from(raw) * range.full_scale_mg();
    (n + (1 << 14)) >> 15
}

/// Raw gyroscope count to mdps (full scale / 32768, i.e. 16.384 LSB/dps
/// at ±2000 dps).
pub fn gyro_mdps(raw: i16, range: GyroRange) -> i32 {
    let n = i64::from(raw) * i64::from(range.full_scale_dps()) * 1_000;
    ((n + (1 << 14)) >> 15) as i32
}

/// Decode three little-endian i16 words (X, Y, Z).
pub fn decode_axes(bytes: &[u8]) -> [i16; 3] {
    [
        i16::from_le_bytes([bytes[0], bytes[1]]),
        i16::from_le_bytes([bytes[2], bytes[3]]),
        i16::from_le_bytes([bytes[4], bytes[5]]),
    ]
}

fn vector(raw: [i16; 3], f: impl Fn(i16) -> i32) -> Vector3 {
    Vector3 {
        x: f(raw[0]),
        y: f(raw[1]),
        z: f(raw[2]),
    }
}

/// Decode the 12-byte block at `DATA_8` (accelerometer, then gyroscope).
pub fn decode_block(bytes: &[u8; 12], accel: AccelRange, gyro: Option<GyroRange>) -> Measurement {
    Measurement {
        accel: vector(decode_axes(&bytes[..6]), |r| accel_mg(r, accel)),
        gyro: gyro.map(|range| vector(decode_axes(&bytes[6..]), |r| gyro_mdps(r, range))),
    }
}

/// `ANYMO_1` / `ANYMO_2` words for `wom`, or the disabled defaults.
pub fn any_motion_words(wom: Option<&WakeOnMotion>) -> [u16; 2] {
    match wom {
        None => ANYMO_DEFAULT,
        Some(w) => {
            let duration = (w.duration_ms + 10) / 20;
            let threshold = (w.threshold_mg.saturating_mul(2048) + 500) / 1000;
            [
                ANYMO_XYZ | duration.min(0x1FFF) as u16,
                ANYMO_ENABLE | ANYMO_OUT_CONF | threshold.clamp(1, 0x7FF) as u16,
            ]
        }
    }
}

/// Register writes that apply `config` once the configuration file is
/// loaded. `None` if the ODR is below 12.5 Hz in normal mode, the
/// gyroscope is on in low-power mode, or any-motion is requested in
/// low-power mode below 50 Hz (feature engine minimum).
pub fn config_plan(config: &Config, wom: Option<&WakeOnMotion>) -> Option<ConfigPlan> {
    let odr = config.accel_odr as u8;
    let (acc_conf, low_power) = match config.accel_mode {
        AccelMode::Normal if odr < AccelOdr::Hz12_5 as u8 => return None,
        AccelMode::Normal => (FILTER_PERF | BWP_NORMAL << 4 | odr, false),
        AccelMode::LowPower(_) if config.gyro.is_some() => return None,
        AccelMode::LowPower(_) if wom.is_some() && odr < AccelOdr::Hz50 as u8 => return None,
        AccelMode::LowPower(avg) => ((avg as u8) << 4 | odr, true),
    };
    let mut pre = RegWrites::new();
    pre.push(reg::ACC_CONF, acc_conf);
    pre.push(reg::ACC_RANGE, config.accel_range as u8);
    let mut pwr_ctrl = PWR_CTRL_ACC;
    if let Some(g) = config.gyro {
        pre.push(reg::GYR_CONF, FILTER_PERF | BWP_NORMAL << 4 | g.odr as u8);
        pre.push(reg::GYR_RANGE, g.range as u8);
        pwr_ctrl |= PWR_CTRL_GYR;
    }
    pre.push(reg::FEAT_PAGE, 1);
    let [w1, w2] = any_motion_words(wom);
    let [a, b] = w1.to_le_bytes();
    let [c, d] = w2.to_le_bytes();
    let mut post = RegWrites::new();
    post.push(
        reg::INT1_IO_CTRL,
        if wom.is_some() {
            INT1_IO_OUTPUT_HIGH
        } else {
            0
        },
    );
    post.push(reg::INT_LATCH, u8::from(wom.is_some_and(|w| w.latched)));
    post.push(
        reg::INT1_MAP_FEAT,
        if wom.is_some() { ANY_MOTION_OUT } else { 0 },
    );
    post.push(reg::PWR_CTRL, pwr_ctrl);
    post.push(reg::PWR_CONF, if low_power { PWR_CONF_APS } else { 0 });
    Some(ConfigPlan {
        pre,
        any_motion: [a, b, c, d],
        post,
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Idle,
    ApsOff,
    Upload,
    CheckInit,
    Registers,
}

macro_rules! driver {
    ($doc:literal, $Iface:ident, $mode:ident, [$($async:tt)?], [$($aw:tt)*]) => {
        #[doc = $doc]
        #[derive(Debug)]
        pub struct Bmi270<I> {
            iface: I,
            spi: bool,
            config_file: &'static [u8],
            burst_len: usize,
            loaded: bool,
            low_power: bool,
            config: Config,
            wom: Option<WakeOnMotion>,
            step: Step,
        }

        impl<I> Bmi270<I> {
            /// Create a driver over any register interface; no bus traffic.
            pub fn new(iface: I) -> Self {
                Self {
                    iface,
                    spi: false,
                    #[cfg(feature = "config-blob")]
                    config_file: &CONFIG_FILE,
                    #[cfg(not(feature = "config-blob"))]
                    config_file: &[],
                    burst_len: 256,
                    loaded: false,
                    // Unknown state: power_down() writes PWR_CONF too.
                    low_power: false,
                    config: Config::default(),
                    wom: None,
                    step: Step::Idle,
                }
            }

            /// Use `file` as the configuration file (required without the
            /// `config-blob` feature).
            pub fn with_config_file(mut self, file: &'static [u8]) -> Self {
                self.config_file = file;
                self
            }

            /// Maximum bytes per configuration-file burst write (default
            /// 256, rounded down to an even number, minimum 2).
            pub fn with_burst_len(mut self, len: usize) -> Self {
                self.burst_len = (len & !1).max(2);
                self
            }

            /// Set the configuration used by `configure()` / `wake()`.
            pub fn with_config(mut self, config: Config) -> Self {
                self.config = config;
                self
            }

            /// Current configuration.
            pub fn config(&self) -> &Config {
                &self.config
            }

            /// Whether a configuration sequence is still in progress.
            pub fn configuring(&self) -> bool {
                self.step != Step::Idle
            }

            /// Give the register interface back.
            pub fn into_interface(self) -> I {
                self.iface
            }
        }

        impl<I2C> Bmi270<I2cInterface<I2C>> {
            /// Create an I2C driver; no bus traffic.
            pub fn new_i2c(i2c: I2C, address: u8) -> Self {
                Self::new(I2cInterface::new(i2c, address))
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.iface.release()
            }
        }

        impl<SPI> Bmi270<SpiInterface<SPI>> {
            /// Create an SPI driver; no bus traffic. The driver performs
            /// the dummy read that selects SPI after power-up and reset.
            pub fn new_spi(spi: SPI) -> Self {
                let mut s = Self::new(SpiInterface::new(spi));
                s.spi = true;
                s
            }

            /// Give the device back.
            pub fn release(self) -> SPI {
                self.iface.release()
            }
        }

        impl<I: $Iface> Bmi270<I> {
            /// Read one register.
            pub $($async)? fn read_register(&mut self, reg: u8) -> Result<u8, Error<I::Error>> {
                let mut b = [0u8];
                self.iface.read_regs(reg, &mut b) $($aw)* .map_err(Error::bus)?;
                Ok(b[0])
            }

            /// Write one register. With advanced power save on (after
            /// reset, in low-power and suspend mode) allow 450 µs before
            /// the next write.
            pub $($async)? fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Error<I::Error>> {
                self.iface.write_reg(reg, value) $($aw)* .map_err(Error::bus)
            }

            $($async)? fn write_all(&mut self, writes: &RegWrites) -> Result<(), Error<I::Error>> {
                for &(r, v) in writes.as_slice() {
                    self.write_register(r, v) $($aw)* ?;
                }
                Ok(())
            }

            $($async)? fn select_spi(&mut self) -> Result<(), Error<I::Error>> {
                if self.spi {
                    self.read_register(reg::CHIP_ID) $($aw)* ?;
                }
                Ok(())
            }

            /// Read `CHIP_ID` (0x24).
            pub $($async)? fn probe(&mut self) -> Result<(), Error<I::Error>> {
                self.select_spi() $($aw)* ?;
                let id = self.read_register(reg::CHIP_ID) $($aw)* ?;
                if id != CHIP_ID {
                    return Err(Error::UnexpectedId(u32::from(id)));
                }
                Ok(())
            }

            /// Soft reset; the configuration file must be uploaded again
            /// (done by `configure()`).
            pub $($async)? fn reset(&mut self) -> Result<WaitUs, Error<I::Error>> {
                self.select_spi() $($aw)* ?;
                self.write_register(reg::CMD, CMD_SOFTRESET) $($aw)* ?;
                self.loaded = false;
                self.low_power = true;
                self.step = Step::Idle;
                Ok(SOFTRESET_WAIT_US)
            }

            /// Start applying the stored configuration (and the any-motion
            /// feature, if enabled). Soft-resets and uploads the
            /// configuration file the first time. Call
            /// [`Self::continue_configure`] after each returned wait until
            /// it returns 0. [`Error::Unsupported`] if the configuration is
            /// invalid or no configuration file is set.
            pub $($async)? fn configure(&mut self) -> Result<WaitUs, Error<I::Error>> {
                config_plan(&self.config, self.wom.as_ref()).ok_or(Error::Unsupported)?;
                if self.config_file.is_empty() || self.config_file.len() % 2 != 0 {
                    return Err(Error::Unsupported);
                }
                if self.loaded {
                    self.write_register(reg::PWR_CONF, 0) $($aw)* ?;
                    self.low_power = false;
                    self.step = Step::Registers;
                    Ok(APS_OFF_WAIT_US)
                } else {
                    let wait = self.reset() $($aw)* ?;
                    self.step = Step::ApsOff;
                    Ok(wait)
                }
            }

            /// Perform the next configuration step; returns the time to
            /// wait before the next call, or 0 when configuration is done.
            /// [`Error::InvalidData`] if the configuration file was
            /// rejected.
            pub $($async)? fn continue_configure(&mut self) -> Result<WaitUs, Error<I::Error>> {
                match self.step {
                    Step::Idle => Ok(0),
                    Step::ApsOff => {
                        self.select_spi() $($aw)* ?;
                        self.write_register(reg::PWR_CONF, 0) $($aw)* ?;
                        self.low_power = false;
                        self.step = Step::Upload;
                        Ok(APS_OFF_WAIT_US)
                    }
                    Step::Upload => {
                        self.write_register(reg::INIT_CTRL, 0) $($aw)* ?;
                        let file = self.config_file;
                        let mut offset = 0;
                        for chunk in file.chunks(self.burst_len) {
                            let word = offset / 2;
                            self.write_register(reg::INIT_ADDR_0, (word & 0x0F) as u8) $($aw)* ?;
                            self.write_register(reg::INIT_ADDR_1, (word >> 4) as u8) $($aw)* ?;
                            self.iface.write_regs(reg::INIT_DATA, chunk) $($aw)* .map_err(Error::bus)?;
                            offset += chunk.len();
                        }
                        self.write_register(reg::INIT_CTRL, 1) $($aw)* ?;
                        self.step = Step::CheckInit;
                        Ok(INIT_WAIT_US)
                    }
                    Step::CheckInit => {
                        let status = self.read_register(reg::INTERNAL_STATUS) $($aw)* ?;
                        if status & 0x0F != INTERNAL_STATUS_INIT_OK {
                            self.step = Step::Idle;
                            return Err(Error::InvalidData);
                        }
                        self.loaded = true;
                        self.step = Step::Registers;
                        self.continue_configure_registers() $($aw)*
                    }
                    Step::Registers => self.continue_configure_registers() $($aw)*,
                }
            }

            $($async)? fn continue_configure_registers(&mut self) -> Result<WaitUs, Error<I::Error>> {
                let plan = config_plan(&self.config, self.wom.as_ref()).ok_or(Error::Unsupported)?;
                self.write_all(&plan.pre) $($aw)* ?;
                self.iface.write_regs(reg::ANYMO_1, &plan.any_motion) $($aw)* .map_err(Error::bus)?;
                self.write_all(&plan.post) $($aw)* ?;
                self.low_power = matches!(self.config.accel_mode, AccelMode::LowPower(_));
                self.step = Step::Idle;
                Ok(if self.config.gyro.is_some() {
                    GYRO_START_US
                } else {
                    ACCEL_START_US
                })
            }

            /// Store a new configuration and start applying it.
            pub $($async)? fn set_config(&mut self, config: Config) -> Result<WaitUs, Error<I::Error>> {
                self.config = config;
                self.configure() $($aw)*
            }

            /// Suspend mode (3.5 µA): sensors off, advanced power save on;
            /// the configuration file and settings are kept. Allow 450 µs
            /// before the next write.
            pub $($async)? fn power_down(&mut self) -> Result<(), Error<I::Error>> {
                self.write_register(reg::PWR_CTRL, 0) $($aw)* ?;
                if !self.low_power {
                    self.write_register(reg::PWR_CONF, PWR_CONF_APS) $($aw)* ?;
                    self.low_power = true;
                }
                self.step = Step::Idle;
                Ok(())
            }

            /// Enable the any-motion feature on INT1 (all axes) and
            /// restart configuration (see [`Self::configure`]). Combine
            /// with [`AccelMode::LowPower`] (ODR ≥ 50 Hz) for a µA-range
            /// wake-on-motion sensor. While enabled, `read_measurement()`
            /// returns the latest sample without waiting for new data.
            pub $($async)? fn enable_wake_on_motion(&mut self, wom: WakeOnMotion) -> Result<WaitUs, Error<I::Error>> {
                self.wom = Some(wom);
                self.configure() $($aw)*
            }

            /// Disable the any-motion feature and restart configuration.
            pub $($async)? fn disable_wake_on_motion(&mut self) -> Result<WaitUs, Error<I::Error>> {
                self.wom = None;
                self.configure() $($aw)*
            }

            /// Whether any-motion was detected since the last call; reads
            /// (and thereby clears) `INT_STATUS_0`, releasing a latched
            /// INT1.
            pub $($async)? fn motion_detected(&mut self) -> Result<bool, Error<I::Error>> {
                let s = self.read_register(reg::INT_STATUS_0) $($aw)* ?;
                Ok(s & ANY_MOTION_OUT != 0)
            }
        }

        impl<I: $Iface> zigbee_sensor::$mode::Sensor for Bmi270<I> {
            type BusError = I::Error;
            type Measurement = Measurement;

            /// Free-running: returns 0, or performs the next pending
            /// configuration step and returns its wait.
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I::Error>> {
                self.continue_configure() $($aw)*
            }

            /// [`Error::NotReady`] while configuring and until `STATUS`
            /// reports new accelerometer data (not checked while
            /// wake-on-motion is enabled).
            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<I::Error>> {
                if self.step != Step::Idle {
                    return Err(Error::NotReady);
                }
                if self.wom.is_none() {
                    let status = self.read_register(reg::STATUS) $($aw)* ?;
                    if status & STATUS_DRDY_ACC == 0 {
                        return Err(Error::NotReady);
                    }
                }
                let mut b = [0u8; 12];
                let gyro = self.config.gyro.map(|g| g.range);
                let len = if gyro.is_some() { 12 } else { 6 };
                self.iface.read_regs(reg::DATA_ACC, &mut b[..len]) $($aw)* .map_err(Error::bus)?;
                Ok(decode_block(&b, self.config.accel_range, gyro))
            }
        }

        impl<I: $Iface> zigbee_sensor::$mode::LowPower for Bmi270<I> {
            type BusError = I::Error;

            /// Suspend mode.
            $($async)? fn sleep(&mut self) -> Result<(), Error<I::Error>> {
                self.power_down() $($aw)*
            }

            /// Start re-applying the stored configuration; finish with
            /// `continue_configure()` or `start_measurement()`.
            $($async)? fn wake(&mut self) -> Result<WaitUs, Error<I::Error>> {
                self.configure() $($aw)*
            }
        }
    };
}

driver!("Blocking driver.", Interface, blocking, [], []);

/// Async driver.
pub mod asynch {
    use super::*;

    driver!(
        "Async twin of [`super::Bmi270`].",
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

    const A: u8 = ADDRESS_SDO_LOW;
    static FILE: [u8; 6] = [1, 2, 3, 4, 5, 6];

    fn rd(reg: u8, data: &[u8]) -> I2cTx {
        I2cTx::write_read(A, vec![reg], data.to_vec())
    }
    fn wr(reg: u8, value: u8) -> I2cTx {
        I2cTx::write(A, vec![reg, value])
    }
    fn burst(reg: u8, data: &[u8]) -> [I2cTx; 4] {
        [
            I2cTx::transaction_start(A),
            I2cTx::write(A, vec![reg]),
            I2cTx::write(A, data.to_vec()),
            I2cTx::transaction_end(A),
        ]
    }
    fn plan(p: &ConfigPlan) -> Vec<I2cTx> {
        let mut v: Vec<I2cTx> = p.pre.as_slice().iter().map(|&(r, x)| wr(r, x)).collect();
        v.extend(burst(reg::ANYMO_1, &p.any_motion));
        v.extend(p.post.as_slice().iter().map(|&(r, x)| wr(r, x)));
        v
    }

    #[test]
    fn datasheet_sensitivities() {
        // 16384 LSB/g at ±2 g, 2048 LSB/g at ±16 g; 16.384 LSB/dps at
        // ±2000 dps, 262.144 LSB/dps at ±125 dps.
        assert_eq!(accel_mg(16_384, AccelRange::G2), 1_000);
        assert_eq!(accel_mg(-2_048, AccelRange::G16), -1_000);
        assert_eq!(gyro_mdps(16_384, GyroRange::Dps2000), 1_000_000);
        assert_eq!(gyro_mdps(-262, GyroRange::Dps125), -999);
        // ANYMO defaults: 0xAA = 83 mg, duration 5 = 100 ms.
        let w = any_motion_words(Some(&WakeOnMotion::default()));
        assert_eq!(w, [0xE005, 0x8000 | 0x3800 | 0xAA]);
        assert_eq!(any_motion_words(None), [0xE005, 0x38AA]);
    }

    #[test]
    fn probe_chip_id() {
        let mut i2c = Mock::new(&[rd(reg::CHIP_ID, &[0x24]), rd(reg::CHIP_ID, &[0xD1])]);
        let mut imu = Bmi270::new_i2c(&mut i2c, A);
        imu.probe().unwrap();
        assert_eq!(imu.probe(), Err(Error::UnexpectedId(0xD1)));
        i2c.done();
    }

    #[test]
    fn init_upload_in_chunks_then_read() {
        let p = config_plan(&Config::default(), None).unwrap();
        assert_eq!(
            p.pre.as_slice(),
            &[
                (reg::ACC_CONF, 0xA6),
                (reg::ACC_RANGE, 0),
                (reg::FEAT_PAGE, 1)
            ]
        );
        assert_eq!(
            p.post.as_slice(),
            &[
                (reg::INT1_IO_CTRL, 0),
                (reg::INT_LATCH, 0),
                (reg::INT1_MAP_FEAT, 0),
                (reg::PWR_CTRL, 0x04),
                (reg::PWR_CONF, 0),
            ]
        );
        let mut ex = vec![
            wr(reg::CMD, 0xB6),
            wr(reg::PWR_CONF, 0),
            wr(reg::INIT_CTRL, 0),
            wr(reg::INIT_ADDR_0, 0),
            wr(reg::INIT_ADDR_1, 0),
        ];
        ex.extend(burst(reg::INIT_DATA, &FILE[..4]));
        ex.extend([wr(reg::INIT_ADDR_0, 2), wr(reg::INIT_ADDR_1, 0)]);
        ex.extend(burst(reg::INIT_DATA, &FILE[4..]));
        ex.push(wr(reg::INIT_CTRL, 1));
        ex.push(rd(reg::INTERNAL_STATUS, &[0x01]));
        ex.extend(plan(&p));
        ex.push(rd(reg::STATUS, &[0x80]));
        ex.push(rd(reg::DATA_ACC, &[0x00, 0x40, 0x00, 0xC0, 0, 0]));
        // Second configure skips the upload.
        ex.push(wr(reg::PWR_CONF, 0));
        ex.extend(plan(&p));
        let mut i2c = Mock::new(&ex);
        let mut imu = Bmi270::new_i2c(&mut i2c, A)
            .with_config_file(&FILE)
            .with_burst_len(5);
        assert_eq!(imu.configure().unwrap(), SOFTRESET_WAIT_US);
        assert_eq!(imu.continue_configure().unwrap(), APS_OFF_WAIT_US);
        assert_eq!(imu.read_measurement(), Err(Error::NotReady));
        assert_eq!(imu.continue_configure().unwrap(), INIT_WAIT_US);
        assert_eq!(imu.start_measurement().unwrap(), ACCEL_START_US);
        assert!(!imu.configuring());
        let m = imu.read_measurement().unwrap();
        assert_eq!(
            m.accel,
            Vector3 {
                x: 1_000,
                y: -1_000,
                z: 0
            }
        );
        assert_eq!(imu.wake().unwrap(), APS_OFF_WAIT_US);
        assert_eq!(imu.continue_configure().unwrap(), ACCEL_START_US);
        assert_eq!(imu.continue_configure().unwrap(), 0);
        i2c.done();
    }

    #[test]
    fn init_failure_and_invalid_configs() {
        let mut ex = vec![wr(reg::CMD, 0xB6), wr(reg::PWR_CONF, 0)];
        ex.extend([
            wr(reg::INIT_CTRL, 0),
            wr(reg::INIT_ADDR_0, 0),
            wr(reg::INIT_ADDR_1, 0),
        ]);
        ex.extend(burst(reg::INIT_DATA, &FILE));
        ex.extend([wr(reg::INIT_CTRL, 1), rd(reg::INTERNAL_STATUS, &[0x02])]);
        let mut i2c = Mock::new(&ex);
        let mut imu = Bmi270::new_i2c(&mut i2c, A).with_config_file(&FILE);
        let mut wait = imu.configure().unwrap();
        while wait != INIT_WAIT_US {
            wait = imu.continue_configure().unwrap();
        }
        assert_eq!(imu.continue_configure(), Err(Error::InvalidData));
        assert!(!imu.configuring());
        let mut imu = imu.with_config_file(&[]);
        assert_eq!(imu.configure(), Err(Error::Unsupported));
        i2c.done();

        let lp = Config {
            accel_odr: AccelOdr::Hz25,
            accel_mode: AccelMode::LowPower(Averaging::X2),
            ..Config::default()
        };
        assert!(config_plan(&lp, None).is_some());
        assert!(config_plan(&lp, Some(&WakeOnMotion::default())).is_none());
        let lp_gyro = Config {
            gyro: Some(GyroConfig::default()),
            ..lp
        };
        assert!(config_plan(&lp_gyro, None).is_none());
        let slow = Config {
            accel_odr: AccelOdr::Hz6_25,
            ..Config::default()
        };
        assert!(config_plan(&slow, None).is_none());
    }

    #[test]
    fn low_power_any_motion_plan_and_suspend() {
        let cfg = Config {
            accel_range: AccelRange::G4,
            accel_odr: AccelOdr::Hz50,
            accel_mode: AccelMode::LowPower(Averaging::X2),
            gyro: None,
        };
        let wom = WakeOnMotion {
            threshold_mg: 50,
            duration_ms: 40,
            latched: true,
        };
        let p = config_plan(&cfg, Some(&wom)).unwrap();
        assert_eq!(
            p.pre.as_slice(),
            &[
                (reg::ACC_CONF, 0x17),
                (reg::ACC_RANGE, 1),
                (reg::FEAT_PAGE, 1)
            ]
        );
        // 50 mg → 102 LSB; 40 ms → 2 samples.
        assert_eq!(p.any_motion, [0x02, 0xE0, 102, 0xB8]);
        assert_eq!(
            p.post.as_slice(),
            &[
                (reg::INT1_IO_CTRL, 0x0A),
                (reg::INT_LATCH, 1),
                (reg::INT1_MAP_FEAT, 0x40),
                (reg::PWR_CTRL, 0x04),
                (reg::PWR_CONF, 0x01),
            ]
        );

        // Already loaded: APS off, registers, then motion and suspend (APS
        // is already on, so only PWR_CTRL is written).
        let mut ex = vec![wr(reg::CMD, 0xB6), wr(reg::PWR_CONF, 0)];
        ex.extend([
            wr(reg::INIT_CTRL, 0),
            wr(reg::INIT_ADDR_0, 0),
            wr(reg::INIT_ADDR_1, 0),
        ]);
        ex.extend(burst(reg::INIT_DATA, &FILE));
        ex.extend([wr(reg::INIT_CTRL, 1), rd(reg::INTERNAL_STATUS, &[0x01])]);
        ex.extend(plan(&p));
        ex.extend([
            rd(reg::DATA_ACC, &[0, 0, 0, 0, 0x00, 0x20]),
            rd(reg::INT_STATUS_0, &[0x40]),
            wr(reg::PWR_CTRL, 0),
        ]);
        let mut i2c = Mock::new(&ex);
        let mut imu = Bmi270::new_i2c(&mut i2c, A)
            .with_config_file(&FILE)
            .with_config(cfg);
        let mut wait = imu.enable_wake_on_motion(wom).unwrap();
        while wait != 0 {
            wait = imu.continue_configure().unwrap();
        }
        assert_eq!(imu.read_measurement().unwrap().accel.z, 1_000);
        assert!(imu.motion_detected().unwrap());
        imu.sleep().unwrap();
        i2c.done();
    }

    #[test]
    fn spi_dummy_byte_and_gyro_block() {
        let cfg = Config {
            gyro: Some(GyroConfig {
                range: GyroRange::Dps2000,
                odr: GyroOdr::Hz100,
            }),
            ..Config::default()
        };
        let mut block = vec![0x00, 0x40, 0, 0, 0, 0];
        block.extend_from_slice(&[0x00, 0x40, 0x00, 0xC0, 0, 0]);
        let r = |reg: u8, data: Vec<u8>| {
            [
                SpiTx::transaction_start(),
                SpiTx::write_vec(vec![reg | 0x80]),
                SpiTx::read_vec(vec![0xFF]),
                SpiTx::read_vec(data),
                SpiTx::transaction_end(),
            ]
        };
        let mut ex = Vec::new();
        ex.extend(r(reg::CHIP_ID, vec![0]));
        ex.extend(r(reg::CHIP_ID, vec![0x24]));
        ex.extend(r(reg::STATUS, vec![0xC0]));
        ex.extend(r(reg::DATA_ACC, block));
        let mut spi = SpiMock::new(&ex);
        let mut imu = Bmi270::new_spi(&mut spi).with_config(cfg);
        imu.probe().unwrap();
        let m = imu.read_measurement().unwrap();
        assert_eq!(m.accel.x, 1_000);
        assert_eq!(
            m.gyro,
            Some(Vector3 {
                x: 1_000_000,
                y: -1_000_000,
                z: 0
            })
        );
        spi.done();

        let p = config_plan(&cfg, None).unwrap();
        assert_eq!(
            &p.pre.as_slice()[2..4],
            &[(reg::GYR_CONF, 0xA8), (reg::GYR_RANGE, 0)]
        );
        assert_eq!(p.post.as_slice()[3], (reg::PWR_CTRL, 0x06));
    }

    #[test]
    fn async_motion_and_suspend() {
        use zigbee_sensor::asynch::LowPower as _;

        let mut i2c = Mock::new(&[
            rd(reg::INT_STATUS_0, &[0x00]),
            wr(reg::PWR_CTRL, 0),
            wr(reg::PWR_CONF, 0x01),
        ]);
        let mut imu = asynch::Bmi270::new_i2c(&mut i2c, A);
        // State unknown: PWR_CONF is written as well.
        assert!(!block_on(imu.motion_detected()).unwrap());
        block_on(imu.sleep()).unwrap();
        i2c.done();

        let mut i2c = Mock::new(&[wr(reg::CMD, 0xB6)]);
        let mut imu = asynch::Bmi270::new_i2c(&mut i2c, A).with_config_file(&FILE);
        assert_eq!(block_on(imu.configure()).unwrap(), SOFTRESET_WAIT_US);
        i2c.done();
    }

    #[cfg(feature = "config-blob")]
    #[test]
    fn bundled_file_is_8k() {
        assert_eq!(CONFIG_FILE.len(), 8_192);
        assert_eq!(&CONFIG_FILE[..4], &[0xC8, 0x2E, 0x00, 0x2E]);
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
