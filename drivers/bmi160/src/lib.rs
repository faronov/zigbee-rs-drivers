//! Bosch BMI160 accelerometer and gyroscope over I2C or SPI.
//!
//! The BMI160 comes out of power-on and soft reset with both sensors in
//! suspend mode. Power modes are changed with commands written to `CMD`;
//! a command occupies `CMD` for up to 3.8 ms (accelerometer) or 80 ms
//! (gyroscope) and further commands are dropped meanwhile. While all
//! sensors are in suspend or low-power mode every register write also
//! needs ~400 µs before the next one. The driver therefore configures the
//! part in steps, each returning a [`WaitUs`]:
//!
//! 1. [`Bmi160::configure`]: soft reset (1 ms);
//! 2. [`Bmi160::continue_configure`]: accelerometer to normal mode (4.1 ms);
//! 3. if the gyroscope is enabled: gyroscope to normal mode (80 ms);
//! 4. configuration and interrupt registers (accelerometer in normal mode,
//!    so no write gaps are needed), then the low-power command if
//!    [`AccelMode::LowPower`] is selected; returns 0.
//!
//! `start_measurement()` performs the next step while configuration is
//! pending and returns its wait; `read_measurement()` returns
//! [`Error::NotReady`] until configuration has finished and `STATUS`
//! reports new accelerometer data.
//!
//! Reference: Bosch Sensortec BST-BMI160-DS000-09 "BMI160 Data sheet",
//! Revision 1.0, November 2020.
//!
//! ```ignore
//! let mut imu = Bmi160::new_i2c(i2c, ADDRESS_SDO_LOW);
//! imu.probe()?;
//! let mut wait = imu.configure()?;
//! while wait != 0 {
//!     delay_us(wait);
//!     wait = imu.continue_configure()?;
//! }
//! let m = imu.read_measurement()?; // NotReady until new data
//! ```

#![no_std]

mod interface;

pub use interface::{AsyncInterface, I2cInterface, Interface, SpiInterface};
use zigbee_sensor::{Error, Quantity, Readings, Vector3, WaitUs};

/// I2C address with SDO low.
pub const ADDRESS_SDO_LOW: u8 = 0x68;
/// I2C address with SDO high.
pub const ADDRESS_SDO_HIGH: u8 = 0x69;

/// `CHIP_ID` value.
pub const CHIP_ID: u8 = 0xD1;

/// Register addresses.
pub mod reg {
    #![allow(missing_docs)]
    pub const CHIP_ID: u8 = 0x00;
    pub const ERR_REG: u8 = 0x02;
    pub const PMU_STATUS: u8 = 0x03;
    pub const DATA_GYR: u8 = 0x0C;
    pub const DATA_ACC: u8 = 0x12;
    pub const STATUS: u8 = 0x1B;
    pub const INT_STATUS0: u8 = 0x1C;
    pub const ACC_CONF: u8 = 0x40;
    pub const ACC_RANGE: u8 = 0x41;
    pub const GYR_CONF: u8 = 0x42;
    pub const GYR_RANGE: u8 = 0x43;
    pub const INT_EN0: u8 = 0x50;
    pub const INT_OUT_CTRL: u8 = 0x53;
    pub const INT_LATCH: u8 = 0x54;
    pub const INT_MAP0: u8 = 0x55;
    pub const INT_MOTION0: u8 = 0x5F;
    pub const INT_MOTION1: u8 = 0x60;
    pub const CMD: u8 = 0x7E;
    /// Reserved; read once to switch the interface to SPI.
    pub const SPI_SELECT: u8 = 0x7F;
}

/// `CMD` values.
pub mod cmd {
    #![allow(missing_docs)]
    pub const ACC_SUSPEND: u8 = 0x10;
    pub const ACC_NORMAL: u8 = 0x11;
    pub const ACC_LOW_POWER: u8 = 0x12;
    pub const GYR_SUSPEND: u8 = 0x14;
    pub const GYR_NORMAL: u8 = 0x15;
    pub const INT_RESET: u8 = 0xB1;
    pub const SOFTRESET: u8 = 0xB6;
}

const STATUS_DRDY_ACC: u8 = 0x80;
const INT0_ANYM: u8 = 0x04;
const INT_EN0_ANYM_XYZ: u8 = 0x07;
const INT_MAP0_INT1_ANYM: u8 = 0x04;
/// INT1 output enabled, push-pull, active high.
const INT_OUT_CTRL_INT1: u8 = 0x0A;
const INT_LATCH_LATCHED: u8 = 0x0F;
const BWP_NORMAL: u8 = 0x02;
const ACC_US: u8 = 0x80;

/// Soft reset: 300 µs system start-up (Table 24 note), rounded up.
pub const SOFTRESET_WAIT_US: WaitUs = 1_000;
/// Accelerometer suspend → normal: 3.8 ms max + 0.3 ms with all sensors
/// suspended.
pub const ACCEL_START_US: WaitUs = 4_100;
/// Gyroscope suspend → normal: 80 ms max.
pub const GYRO_START_US: WaitUs = 80_000;

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
    /// `ACC_RANGE` code.
    const fn code(self) -> u8 {
        match self {
            Self::G2 => 0x03,
            Self::G4 => 0x05,
            Self::G8 => 0x08,
            Self::G16 => 0x0C,
        }
    }

    /// Full scale in mg.
    pub const fn full_scale_mg(self) -> i32 {
        2_000 << (self as u8)
    }
}

/// Gyroscope full scale.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GyroRange {
    /// ±2000 dps, 16.4 LSB/dps.
    #[default]
    Dps2000,
    /// ±1000 dps, 32.8 LSB/dps.
    Dps1000,
    /// ±500 dps, 65.6 LSB/dps.
    Dps500,
    /// ±250 dps, 131.2 LSB/dps.
    Dps250,
    /// ±125 dps, 262.4 LSB/dps.
    Dps125,
}

impl GyroRange {
    /// Datasheet sensitivity in 0.1 LSB per dps.
    const fn tenth_lsb_per_dps(self) -> i32 {
        164 << (self as u8)
    }
}

/// Accelerometer output data rate (`acc_odr` = 100 Hz / 2^(8 - code)).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AccelOdr {
    /// 0.78 Hz (low-power mode only).
    Hz0_78 = 1,
    /// 1.56 Hz (low-power mode only).
    Hz1_56,
    /// 3.12 Hz (low-power mode only).
    Hz3_12,
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

/// Gyroscope output data rate (`gyr_odr`, normal filter mode).
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

/// Number of samples averaged per output in accelerometer low-power mode
/// (`acc_bwp` with `acc_us` = 1).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Averaging {
    /// 1 sample (lowest current).
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
    /// Continuous sampling, 180 µA; ODR 12.5–1600 Hz.
    #[default]
    Normal,
    /// Duty-cycled under-sampling (e.g. 4 µA at 1.56 Hz, 14 µA at 25 Hz
    /// with no averaging); gyroscope must be off.
    LowPower(Averaging),
}

/// Gyroscope settings.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct GyroConfig {
    /// Full scale.
    pub range: GyroRange,
    /// Output data rate.
    pub odr: GyroOdr,
}

/// Device configuration, applied by [`Bmi160::configure`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Accelerometer full scale.
    pub accel_range: AccelRange,
    /// Accelerometer output data rate.
    pub accel_odr: AccelOdr,
    /// Accelerometer power mode.
    pub accel_mode: AccelMode,
    /// Gyroscope settings; `None` keeps it suspended.
    pub gyro: Option<GyroConfig>,
}

impl Default for Config {
    /// ±2 g, 25 Hz, normal mode, gyroscope suspended.
    fn default() -> Self {
        Self {
            accel_range: AccelRange::G2,
            accel_odr: AccelOdr::Hz25,
            accel_mode: AccelMode::Normal,
            gyro: None,
        }
    }
}

/// Any-motion interrupt settings (INT1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WakeOnMotion {
    /// Sample-to-sample slope threshold in mg; full scale / 512 per LSB
    /// (3.91 mg at ±2 g), clamped to 1..=255 LSB.
    pub threshold_mg: u32,
    /// Consecutive slope samples above the threshold, 1..=4.
    pub samples: u8,
    /// Hold INT1 until [`Bmi160::motion_detected`] resets it (else INT1
    /// follows the motion condition).
    pub latched: bool,
}

impl Default for WakeOnMotion {
    fn default() -> Self {
        Self {
            threshold_mg: 64,
            samples: 1,
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

/// Up to twelve register writes computed without bus traffic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RegWrites {
    buf: [(u8, u8); 12],
    len: usize,
}

impl RegWrites {
    const fn new() -> Self {
        Self {
            buf: [(0, 0); 12],
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

fn div_round(n: i32, d: i32) -> i32 {
    if n >= 0 {
        (n + d / 2) / d
    } else {
        (n - d / 2) / d
    }
}

/// Raw accelerometer count to mg (full scale / 32768).
pub fn accel_mg(raw: i16, range: AccelRange) -> i32 {
    let n = i32::from(raw) * range.full_scale_mg();
    (n + (1 << 14)) >> 15
}

/// Raw gyroscope count to mdps using the datasheet sensitivity.
pub fn gyro_mdps(raw: i16, range: GyroRange) -> i32 {
    div_round(i32::from(raw) * 10_000, range.tenth_lsb_per_dps())
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

/// Decode the 12-byte block at `DATA_GYR` (gyroscope, then accelerometer).
pub fn decode_block(bytes: &[u8; 12], accel: AccelRange, gyro: Option<GyroRange>) -> Measurement {
    Measurement {
        accel: vector(decode_axes(&bytes[6..]), |r| accel_mg(r, accel)),
        gyro: gyro.map(|range| vector(decode_axes(&bytes[..6]), |r| gyro_mdps(r, range))),
    }
}

/// Any-motion threshold code (`int_anym_th`) for `threshold_mg`.
pub fn any_motion_threshold(threshold_mg: u32, range: AccelRange) -> u8 {
    let fs = range.full_scale_mg() as u32;
    let code = (threshold_mg.saturating_mul(512) + fs / 2) / fs;
    code.clamp(1, 255) as u8
}

/// Register writes for the final configuration step (accelerometer in
/// normal mode). `None` if the ODR is below 12.5 Hz in normal mode or the
/// gyroscope is on together with accelerometer low-power mode.
pub fn config_writes(config: &Config, wom: Option<&WakeOnMotion>) -> Option<RegWrites> {
    let odr = config.accel_odr as u8;
    let acc_conf = match config.accel_mode {
        AccelMode::Normal if odr < AccelOdr::Hz12_5 as u8 => return None,
        AccelMode::Normal => BWP_NORMAL << 4 | odr,
        AccelMode::LowPower(_) if config.gyro.is_some() => return None,
        AccelMode::LowPower(avg) => ACC_US | (avg as u8) << 4 | odr,
    };
    let mut w = RegWrites::new();
    w.push(reg::ACC_CONF, acc_conf);
    w.push(reg::ACC_RANGE, config.accel_range.code());
    if let Some(g) = config.gyro {
        w.push(reg::GYR_CONF, BWP_NORMAL << 4 | g.odr as u8);
        w.push(reg::GYR_RANGE, g.range as u8);
    }
    if let Some(wom) = wom {
        w.push(reg::INT_OUT_CTRL, INT_OUT_CTRL_INT1);
        w.push(
            reg::INT_LATCH,
            if wom.latched { INT_LATCH_LATCHED } else { 0 },
        );
        w.push(reg::INT_MOTION0, wom.samples.clamp(1, 4) - 1);
        w.push(
            reg::INT_MOTION1,
            any_motion_threshold(wom.threshold_mg, config.accel_range),
        );
        w.push(reg::INT_MAP0, INT_MAP0_INT1_ANYM);
        w.push(reg::INT_EN0, INT_EN0_ANYM_XYZ);
    }
    if let AccelMode::LowPower(_) = config.accel_mode {
        w.push(reg::CMD, cmd::ACC_LOW_POWER);
    }
    Some(w)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Idle,
    AccelNormal,
    GyroNormal,
    Registers,
}

macro_rules! driver {
    ($doc:literal, $Iface:ident, $mode:ident, [$($async:tt)?], [$($aw:tt)*]) => {
        #[doc = $doc]
        #[derive(Debug)]
        pub struct Bmi160<I> {
            iface: I,
            spi: bool,
            config: Config,
            wom: Option<WakeOnMotion>,
            step: Step,
        }

        impl<I> Bmi160<I> {
            /// Create a driver over any register interface; no bus traffic.
            pub fn new(iface: I) -> Self {
                Self {
                    iface,
                    spi: false,
                    config: Config::default(),
                    wom: None,
                    step: Step::Idle,
                }
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

        impl<I2C> Bmi160<I2cInterface<I2C>> {
            /// Create an I2C driver; no bus traffic.
            pub fn new_i2c(i2c: I2C, address: u8) -> Self {
                Self::new(I2cInterface::new(i2c, address))
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.iface.release()
            }
        }

        impl<SPI> Bmi160<SpiInterface<SPI>> {
            /// Create an SPI driver; no bus traffic. The driver performs
            /// the dummy read of register 0x7F that switches the part to
            /// SPI after power-up and soft reset.
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

        impl<I: $Iface> Bmi160<I> {
            /// Read one register.
            pub $($async)? fn read_register(&mut self, reg: u8) -> Result<u8, Error<I::Error>> {
                let mut b = [0u8];
                self.iface.read_regs(reg, &mut b) $($aw)* .map_err(Error::bus)?;
                Ok(b[0])
            }

            /// Write one register. While all sensors are suspended or in
            /// low-power mode, allow 400 µs before the next write.
            pub $($async)? fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Error<I::Error>> {
                self.iface.write_reg(reg, value) $($aw)* .map_err(Error::bus)
            }

            $($async)? fn select_spi(&mut self) -> Result<(), Error<I::Error>> {
                if self.spi {
                    self.read_register(reg::SPI_SELECT) $($aw)* ?;
                }
                Ok(())
            }

            /// Read `CHIP_ID` (0xD1).
            pub $($async)? fn probe(&mut self) -> Result<(), Error<I::Error>> {
                self.select_spi() $($aw)* ?;
                let id = self.read_register(reg::CHIP_ID) $($aw)* ?;
                if id != CHIP_ID {
                    return Err(Error::UnexpectedId(u32::from(id)));
                }
                Ok(())
            }

            /// Soft reset; both sensors are suspended afterwards (3 µA)
            /// and all settings are lost.
            pub $($async)? fn reset(&mut self) -> Result<WaitUs, Error<I::Error>> {
                self.select_spi() $($aw)* ?;
                self.write_register(reg::CMD, cmd::SOFTRESET) $($aw)* ?;
                self.step = Step::Idle;
                Ok(SOFTRESET_WAIT_US)
            }

            /// Start applying the stored configuration (and the any-motion
            /// interrupt, if enabled) from a soft reset. Call
            /// [`Self::continue_configure`] after each returned wait until
            /// it returns 0.
            pub $($async)? fn configure(&mut self) -> Result<WaitUs, Error<I::Error>> {
                config_writes(&self.config, self.wom.as_ref()).ok_or(Error::Unsupported)?;
                let wait = self.reset() $($aw)* ?;
                self.step = Step::AccelNormal;
                Ok(wait)
            }

            /// Perform the next configuration step; returns the time to
            /// wait before the next call, or 0 when configuration is done.
            pub $($async)? fn continue_configure(&mut self) -> Result<WaitUs, Error<I::Error>> {
                match self.step {
                    Step::Idle => Ok(0),
                    Step::AccelNormal => {
                        self.select_spi() $($aw)* ?;
                        self.write_register(reg::CMD, cmd::ACC_NORMAL) $($aw)* ?;
                        self.step = if self.config.gyro.is_some() {
                            Step::GyroNormal
                        } else {
                            Step::Registers
                        };
                        Ok(ACCEL_START_US)
                    }
                    Step::GyroNormal => {
                        self.write_register(reg::CMD, cmd::GYR_NORMAL) $($aw)* ?;
                        self.step = Step::Registers;
                        Ok(GYRO_START_US)
                    }
                    Step::Registers => {
                        let writes = config_writes(&self.config, self.wom.as_ref()).ok_or(Error::Unsupported)?;
                        for &(r, v) in writes.as_slice() {
                            self.write_register(r, v) $($aw)* ?;
                        }
                        self.step = Step::Idle;
                        Ok(0)
                    }
                }
            }

            /// Store a new configuration and start applying it.
            pub $($async)? fn set_config(&mut self, config: Config) -> Result<WaitUs, Error<I::Error>> {
                self.config = config;
                self.configure() $($aw)*
            }

            /// Suspend both sensors (3 µA) by soft reset; settings are
            /// re-applied by `configure()` / `wake()`. Allow 1 ms before
            /// the next access.
            pub $($async)? fn power_down(&mut self) -> Result<(), Error<I::Error>> {
                self.reset() $($aw)* .map(|_| ())
            }

            /// Enable the any-motion interrupt on INT1 (all axes) and
            /// restart configuration (see [`Self::configure`]). Combine
            /// with [`AccelMode::LowPower`] for a µA-range wake-on-motion
            /// sensor. While enabled, `read_measurement()` returns the
            /// latest sample without waiting for new data.
            pub $($async)? fn enable_wake_on_motion(&mut self, wom: WakeOnMotion) -> Result<WaitUs, Error<I::Error>> {
                self.wom = Some(wom);
                self.configure() $($aw)*
            }

            /// Disable the any-motion interrupt and restart configuration.
            pub $($async)? fn disable_wake_on_motion(&mut self) -> Result<WaitUs, Error<I::Error>> {
                self.wom = None;
                self.configure() $($aw)*
            }

            /// Whether any-motion was detected since the last call. Reads
            /// `INT_STATUS[0]` and, in latched mode, releases INT1 with the
            /// `int_reset` command.
            pub $($async)? fn motion_detected(&mut self) -> Result<bool, Error<I::Error>> {
                let s = self.read_register(reg::INT_STATUS0) $($aw)* ?;
                let motion = s & INT0_ANYM != 0;
                if motion && self.wom.is_some_and(|w| w.latched) {
                    self.write_register(reg::CMD, cmd::INT_RESET) $($aw)* ?;
                }
                Ok(motion)
            }
        }

        impl<I: $Iface> zigbee_sensor::$mode::Sensor for Bmi160<I> {
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
                if gyro.is_some() {
                    self.iface.read_regs(reg::DATA_GYR, &mut b) $($aw)* .map_err(Error::bus)?;
                } else {
                    self.iface.read_regs(reg::DATA_ACC, &mut b[6..]) $($aw)* .map_err(Error::bus)?;
                }
                Ok(decode_block(&b, self.config.accel_range, gyro))
            }
        }

        impl<I: $Iface> zigbee_sensor::$mode::LowPower for Bmi160<I> {
            type BusError = I::Error;

            /// Soft reset to suspend mode.
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
        "Async twin of [`super::Bmi160`].",
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

    fn rd(reg: u8, data: &[u8]) -> I2cTx {
        I2cTx::write_read(A, vec![reg], data.to_vec())
    }
    fn wr(reg: u8, value: u8) -> I2cTx {
        I2cTx::write(A, vec![reg, value])
    }
    fn writes(w: &RegWrites) -> Vec<I2cTx> {
        w.as_slice().iter().map(|&(r, v)| wr(r, v)).collect()
    }

    #[test]
    fn datasheet_sensitivities() {
        // Table 1: 16384 LSB/g at ±2 g, 2048 LSB/g at ±16 g; 16.4 LSB/dps
        // at ±2000 dps, 262.4 LSB/dps at ±125 dps.
        assert_eq!(accel_mg(16_384, AccelRange::G2), 1_000);
        assert_eq!(accel_mg(-2_048, AccelRange::G16), -1_000);
        assert_eq!(gyro_mdps(164, GyroRange::Dps2000), 10_000);
        assert_eq!(gyro_mdps(-656, GyroRange::Dps500), -10_000);
        assert_eq!(gyro_mdps(2_624, GyroRange::Dps125), 10_000);
        // INT_MOTION[1]: 3.91 mg/LSB at ±2 g, 31.25 mg/LSB at ±16 g.
        assert_eq!(any_motion_threshold(70, AccelRange::G2), 18);
        assert_eq!(any_motion_threshold(250, AccelRange::G16), 8);
        assert_eq!(any_motion_threshold(0, AccelRange::G2), 1);
        assert_eq!(any_motion_threshold(10_000, AccelRange::G2), 255);
    }

    #[test]
    fn probe_chip_id() {
        let mut i2c = Mock::new(&[rd(reg::CHIP_ID, &[0xD1]), rd(reg::CHIP_ID, &[0xD8])]);
        let mut imu = Bmi160::new_i2c(&mut i2c, A);
        imu.probe().unwrap();
        assert_eq!(imu.probe(), Err(Error::UnexpectedId(0xD8)));
        i2c.done();
    }

    #[test]
    fn configure_default_then_read() {
        let mut i2c = Mock::new(&[
            wr(reg::CMD, cmd::SOFTRESET),
            wr(reg::CMD, cmd::ACC_NORMAL),
            wr(reg::ACC_CONF, 0x26),
            wr(reg::ACC_RANGE, 0x03),
            rd(reg::STATUS, &[0x00]),
            rd(reg::STATUS, &[0x80]),
            rd(reg::DATA_ACC, &[0x00, 0x40, 0x00, 0xC0, 0, 0]),
        ]);
        let mut imu = Bmi160::new_i2c(&mut i2c, A);
        assert_eq!(imu.configure().unwrap(), SOFTRESET_WAIT_US);
        assert_eq!(imu.read_measurement(), Err(Error::NotReady));
        assert_eq!(imu.continue_configure().unwrap(), ACCEL_START_US);
        assert_eq!(imu.start_measurement().unwrap(), 0);
        assert!(!imu.configuring());
        assert_eq!(imu.start_measurement().unwrap(), 0);
        assert_eq!(imu.read_measurement(), Err(Error::NotReady));
        let m = imu.read_measurement().unwrap();
        assert_eq!(
            m.accel,
            Vector3 {
                x: 1_000,
                y: -1_000,
                z: 0
            }
        );
        assert_eq!(m.gyro, None);
        i2c.done();
    }

    #[test]
    fn low_power_any_motion_writes() {
        let cfg = Config {
            accel_range: AccelRange::G4,
            accel_odr: AccelOdr::Hz6_25,
            accel_mode: AccelMode::LowPower(Averaging::X2),
            gyro: None,
        };
        let wom = WakeOnMotion {
            threshold_mg: 78,
            samples: 2,
            latched: true,
        };
        let w = config_writes(&cfg, Some(&wom)).unwrap();
        assert_eq!(
            w.as_slice(),
            &[
                (reg::ACC_CONF, 0x94),
                (reg::ACC_RANGE, 0x05),
                (reg::INT_OUT_CTRL, 0x0A),
                (reg::INT_LATCH, 0x0F),
                (reg::INT_MOTION0, 0x01),
                (reg::INT_MOTION1, 10),
                (reg::INT_MAP0, 0x04),
                (reg::INT_EN0, 0x07),
                (reg::CMD, 0x12),
            ]
        );
        let with_gyro = Config {
            gyro: Some(GyroConfig::default()),
            ..cfg
        };
        assert!(config_writes(&with_gyro, None).is_none());
        let slow_normal = Config {
            accel_mode: AccelMode::Normal,
            ..cfg
        };
        assert!(config_writes(&slow_normal, None).is_none());
        let mut i2c = Mock::new(&[]);
        let mut imu = Bmi160::new_i2c(&mut i2c, A).with_config(slow_normal);
        assert_eq!(imu.configure(), Err(Error::Unsupported));
        i2c.done();
    }

    #[test]
    fn wake_on_motion_flow() {
        let cfg = Config {
            accel_odr: AccelOdr::Hz12_5,
            accel_mode: AccelMode::LowPower(Averaging::X1),
            ..Config::default()
        };
        let wom = WakeOnMotion::default();
        let mut ex = vec![wr(reg::CMD, cmd::SOFTRESET), wr(reg::CMD, cmd::ACC_NORMAL)];
        ex.extend(writes(&config_writes(&cfg, Some(&wom)).unwrap()));
        ex.extend([
            rd(reg::DATA_ACC, &[0, 0, 0, 0, 0x00, 0x40]),
            rd(reg::INT_STATUS0, &[0x04]),
            wr(reg::CMD, cmd::INT_RESET),
            rd(reg::INT_STATUS0, &[0x00]),
            wr(reg::CMD, cmd::SOFTRESET),
        ]);
        let mut i2c = Mock::new(&ex);
        let mut imu = Bmi160::new_i2c(&mut i2c, A).with_config(cfg);
        let mut wait = imu.enable_wake_on_motion(wom).unwrap();
        while wait != 0 {
            wait = imu.continue_configure().unwrap();
        }
        assert_eq!(imu.read_measurement().unwrap().accel.z, 1_000);
        assert!(imu.motion_detected().unwrap());
        assert!(!imu.motion_detected().unwrap());
        imu.sleep().unwrap();
        i2c.done();
    }

    #[test]
    fn spi_gyro_sequence_and_block() {
        let cfg = Config {
            gyro: Some(GyroConfig {
                range: GyroRange::Dps2000,
                odr: GyroOdr::Hz100,
            }),
            ..Config::default()
        };
        let w = |r: u8, v: u8| {
            [
                SpiTx::transaction_start(),
                SpiTx::write_vec(vec![r, v]),
                SpiTx::transaction_end(),
            ]
        };
        let r = |reg: u8, data: Vec<u8>| {
            [
                SpiTx::transaction_start(),
                SpiTx::write_vec(vec![reg | 0x80]),
                SpiTx::read_vec(data),
                SpiTx::transaction_end(),
            ]
        };
        let mut block = vec![0xA4, 0x00, 0x5C, 0xFF, 0, 0];
        block.extend_from_slice(&[0x00, 0x40, 0, 0, 0, 0]);
        let mut ex = Vec::new();
        ex.extend(r(reg::SPI_SELECT, vec![0]));
        ex.extend(w(reg::CMD, cmd::SOFTRESET));
        ex.extend(r(reg::SPI_SELECT, vec![0]));
        ex.extend(w(reg::CMD, cmd::ACC_NORMAL));
        ex.extend(w(reg::CMD, cmd::GYR_NORMAL));
        ex.extend(w(reg::ACC_CONF, 0x26));
        ex.extend(w(reg::ACC_RANGE, 0x03));
        ex.extend(w(reg::GYR_CONF, 0x28));
        ex.extend(w(reg::GYR_RANGE, 0x00));
        ex.extend(r(reg::STATUS, vec![0xC0]));
        ex.extend(r(reg::DATA_GYR, block));
        let mut spi = SpiMock::new(&ex);
        let mut imu = Bmi160::new_spi(&mut spi).with_config(cfg);
        assert_eq!(imu.wake().unwrap(), SOFTRESET_WAIT_US);
        assert_eq!(imu.start_measurement().unwrap(), ACCEL_START_US);
        assert_eq!(imu.start_measurement().unwrap(), GYRO_START_US);
        assert_eq!(imu.start_measurement().unwrap(), 0);
        let m = imu.read_measurement().unwrap();
        assert_eq!(m.accel.x, 1_000);
        assert_eq!(
            m.gyro,
            Some(Vector3 {
                x: 10_000,
                y: -10_000,
                z: 0
            })
        );
        spi.done();
    }

    #[test]
    fn async_configure_and_read() {
        use zigbee_sensor::asynch::Sensor as _;

        let mut ex = vec![wr(reg::CMD, cmd::SOFTRESET), wr(reg::CMD, cmd::ACC_NORMAL)];
        ex.extend(writes(&config_writes(&Config::default(), None).unwrap()));
        ex.push(rd(reg::STATUS, &[0x80]));
        ex.push(rd(reg::DATA_ACC, &[0, 0, 0, 0, 0x00, 0x40]));
        let mut i2c = Mock::new(&ex);
        let mut imu = asynch::Bmi160::new_i2c(&mut i2c, A);
        assert_eq!(block_on(imu.configure()).unwrap(), SOFTRESET_WAIT_US);
        assert_eq!(block_on(imu.start_measurement()).unwrap(), ACCEL_START_US);
        assert_eq!(block_on(imu.start_measurement()).unwrap(), 0);
        assert_eq!(block_on(imu.read_measurement()).unwrap().accel.z, 1_000);
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
