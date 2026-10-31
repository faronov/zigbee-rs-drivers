//! InvenSense MPU-6050 / MPU-6000, MPU-6500 and MPU-9250 (accelerometer and
//! gyroscope only) over I2C or SPI.
//!
//! [`Variant`] is detected from `WHO_AM_I`; the parts differ in the
//! accelerometer low-power ("cycle") mode and the motion interrupt:
//!
//! * MPU-6050/6000: cycle mode with `LP_WAKE_CTRL` (1.25/5/20/40 Hz) and the
//!   motion-detection engine (`MOT_THR` 1 mg/LSB, `MOT_DUR` 1 ms/LSB,
//!   accelerometer high-pass filter at 5 Hz);
//! * MPU-6500/9250: cycle mode with `LP_ACCEL_ODR` (0.24–500 Hz) and
//!   Wake-on-Motion (`WOM_THR` 4 mg/LSB, compare to previous sample).
//!
//! The AK8963 magnetometer inside the MPU-9250 is not used.
//!
//! The device is free-running: [`Mpu6050::configure`] wakes it,
//! `start_measurement()` returns 0 and `read_measurement()` returns
//! [`Error::NotReady`] until `INT_STATUS` reports new data (the data-ready
//! interrupt is enabled for that, so INT pulses once per sample unless
//! wake-on-motion is enabled). The gyroscope is off by default.
//!
//! References:
//! * InvenSense PS-MPU-6000A-00 "MPU-6000 and MPU-6050 Product
//!   Specification", Rev 3.1 (motion detection §8.3);
//! * InvenSense RM-MPU-6000A-00 "MPU-6000 and MPU-6050 Register Map and
//!   Descriptions", Rev 4.0 (`MOT_DUR` 0x20 and `ACCEL_HPF` from the earlier
//!   register-map revision);
//! * InvenSense PS-MPU-9250A-01 "MPU-9250 Product Specification", Rev 1.0
//!   (Wake-on-Motion flow, Figure 3) and RM-MPU-9250A-00 "Register Map and
//!   Descriptions", Rev 1.4 (identical to the MPU-6500 map).
//!
//! ```ignore
//! let mut imu = Mpu6050::new_i2c(i2c, ADDRESS_AD0_LOW);
//! imu.probe()?;
//! let wait = imu.configure()?;
//! let m = imu.read_measurement()?; // NotReady until new data
//! ```

#![no_std]

mod interface;

pub use interface::{AsyncInterface, I2cInterface, Interface, SpiInterface};
use zigbee_sensor::{Error, Quantity, Readings, Vector3, WaitUs};

/// I2C address with AD0 low.
pub const ADDRESS_AD0_LOW: u8 = 0x68;
/// I2C address with AD0 high.
pub const ADDRESS_AD0_HIGH: u8 = 0x69;

/// Register addresses.
pub mod reg {
    #![allow(missing_docs)]
    pub const SMPLRT_DIV: u8 = 0x19;
    pub const CONFIG: u8 = 0x1A;
    pub const GYRO_CONFIG: u8 = 0x1B;
    pub const ACCEL_CONFIG: u8 = 0x1C;
    /// MPU-6500/9250.
    pub const ACCEL_CONFIG2: u8 = 0x1D;
    /// MPU-6500/9250.
    pub const LP_ACCEL_ODR: u8 = 0x1E;
    /// `MOT_THR` on MPU-6050, `WOM_THR` on MPU-6500/9250.
    pub const MOT_THR: u8 = 0x1F;
    /// MPU-6050.
    pub const MOT_DUR: u8 = 0x20;
    pub const INT_PIN_CFG: u8 = 0x37;
    pub const INT_ENABLE: u8 = 0x38;
    pub const INT_STATUS: u8 = 0x3A;
    pub const ACCEL_XOUT_H: u8 = 0x3B;
    /// `MOT_DETECT_CTRL` on MPU-6050, `ACCEL_INTEL_CTRL` on MPU-6500/9250.
    pub const MOT_DETECT_CTRL: u8 = 0x69;
    pub const USER_CTRL: u8 = 0x6A;
    pub const PWR_MGMT_1: u8 = 0x6B;
    pub const PWR_MGMT_2: u8 = 0x6C;
    pub const WHO_AM_I: u8 = 0x75;
}

const PWR1_RESET: u8 = 0x80;
const PWR1_SLEEP: u8 = 0x40;
const PWR1_CYCLE: u8 = 0x20;
const PWR1_TEMP_DIS: u8 = 0x08;
const PWR1_CLKSEL_PLL: u8 = 0x01;
const PWR2_STBY_GYRO: u8 = 0x07;
const INT_DATA_RDY: u8 = 0x01;
const INT_MOTION: u8 = 0x40;
const INT_PIN_LATCH: u8 = 0x20;
const ACCEL_HPF_5HZ: u8 = 0x01;
const ACCEL_INTEL_EN_MODE: u8 = 0xC0;
const USER_CTRL_I2C_IF_DIS: u8 = 0x10;

/// Wait after `DEVICE_RESET` (start-up time for register read/write, max).
pub const RESET_WAIT_US: WaitUs = 100_000;
/// Gyroscope start-up time from sleep (MPU-9250 35 ms, MPU-6050 30 ms).
pub const GYRO_START_US: WaitUs = 35_000;
/// Accelerometer start-up time from sleep (MPU-9250: 20 ms).
pub const ACCEL_START_US: WaitUs = 20_000;

/// Part, selected from `WHO_AM_I`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Variant {
    /// MPU-6050 / MPU-6000 (`WHO_AM_I` = 0x68).
    #[default]
    Mpu6050,
    /// MPU-6500 (0x70).
    Mpu6500,
    /// MPU-9250 (0x71); register-compatible with the MPU-6500.
    Mpu9250,
}

impl Variant {
    /// Map a `WHO_AM_I` value.
    pub const fn from_id(id: u8) -> Option<Self> {
        match id {
            0x68 => Some(Self::Mpu6050),
            0x70 => Some(Self::Mpu6500),
            0x71 => Some(Self::Mpu9250),
            _ => None,
        }
    }

    const fn is_6500(self) -> bool {
        !matches!(self, Self::Mpu6050)
    }
}

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
    const fn bits(self) -> u8 {
        self as u8
    }

    /// Full scale in mg.
    pub const fn full_scale_mg(self) -> i32 {
        2_000 << (self as u8)
    }
}

/// Gyroscope full scale.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GyroRange {
    /// ±250 dps, 131 LSB/dps.
    Dps250,
    /// ±500 dps, 65.5 LSB/dps.
    Dps500,
    /// ±1000 dps, 32.8 LSB/dps.
    Dps1000,
    /// ±2000 dps, 16.4 LSB/dps.
    #[default]
    Dps2000,
}

impl GyroRange {
    const fn bits(self) -> u8 {
        self as u8
    }

    /// Datasheet sensitivity in 0.1 LSB per dps.
    const fn tenth_lsb_per_dps(self) -> i32 {
        match self {
            Self::Dps250 => 1310,
            Self::Dps500 => 655,
            Self::Dps1000 => 328,
            Self::Dps2000 => 164,
        }
    }
}

/// Digital low-pass filter (`DLPF_CFG` / `A_DLPF_CFG`); internal sample
/// rate 1 kHz for all settings.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Dlpf {
    /// ~184 Hz gyro / 184–218 Hz accel.
    Hz184 = 1,
    /// ~94 Hz.
    Hz94 = 2,
    /// ~44 Hz.
    #[default]
    Hz44 = 3,
    /// ~21 Hz.
    Hz21 = 4,
    /// ~10 Hz.
    Hz10 = 5,
    /// ~5 Hz.
    Hz5 = 6,
}

/// Accelerometer-only low-power (cycle mode) wake-up rate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CycleRate {
    /// 0.24 Hz (MPU-6500/9250).
    Hz0_24,
    /// 0.49 Hz (MPU-6500/9250).
    Hz0_49,
    /// 0.98 Hz (MPU-6500/9250), 8.4 µA on MPU-9250.
    Hz0_98,
    /// 1.25 Hz (MPU-6050), 10 µA.
    Hz1_25,
    /// 1.95 Hz (MPU-6500/9250).
    Hz1_95,
    /// 3.91 Hz (MPU-6500/9250).
    Hz3_91,
    /// 5 Hz (MPU-6050), 20 µA.
    Hz5,
    /// 7.81 Hz (MPU-6500/9250).
    Hz7_81,
    /// 15.63 Hz (MPU-6500/9250).
    Hz15_63,
    /// 20 Hz (MPU-6050), 60 µA.
    Hz20,
    /// 31.25 Hz (MPU-6500/9250), 19.8 µA on MPU-9250.
    Hz31_25,
    /// 40 Hz (MPU-6050), 110 µA.
    Hz40,
    /// 62.5 Hz (MPU-6500/9250).
    Hz62_5,
    /// 125 Hz (MPU-6500/9250).
    Hz125,
    /// 250 Hz (MPU-6500/9250).
    Hz250,
    /// 500 Hz (MPU-6500/9250).
    Hz500,
}

impl CycleRate {
    /// `LP_WAKE_CTRL` (MPU-6050) or `Lposc_clksel` (MPU-6500/9250) code.
    pub const fn code(self, variant: Variant) -> Option<u8> {
        if variant.is_6500() {
            match self {
                Self::Hz0_24 => Some(0),
                Self::Hz0_49 => Some(1),
                Self::Hz0_98 => Some(2),
                Self::Hz1_95 => Some(3),
                Self::Hz3_91 => Some(4),
                Self::Hz7_81 => Some(5),
                Self::Hz15_63 => Some(6),
                Self::Hz31_25 => Some(7),
                Self::Hz62_5 => Some(8),
                Self::Hz125 => Some(9),
                Self::Hz250 => Some(10),
                Self::Hz500 => Some(11),
                _ => None,
            }
        } else {
            match self {
                Self::Hz1_25 => Some(0),
                Self::Hz5 => Some(1),
                Self::Hz20 => Some(2),
                Self::Hz40 => Some(3),
                _ => None,
            }
        }
    }
}

/// Sampling mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// Continuous sampling at 1 kHz / (1 + `sample_rate_div`).
    Normal {
        /// `SMPLRT_DIV`.
        sample_rate_div: u8,
        /// Low-pass filter.
        dlpf: Dlpf,
    },
    /// Accelerometer-only cycle mode; the gyroscope must be off.
    LowPowerAccel(CycleRate),
}

/// Device configuration, applied by [`Mpu6050::configure`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Accelerometer full scale.
    pub accel_range: AccelRange,
    /// Gyroscope full scale; `None` keeps the gyroscope in standby.
    pub gyro: Option<GyroRange>,
    /// Sampling mode.
    pub mode: Mode,
}

impl Default for Config {
    /// ±2 g, 25 Hz (divider 39), 44 Hz low-pass, gyroscope off.
    fn default() -> Self {
        Self {
            accel_range: AccelRange::G2,
            gyro: None,
            mode: Mode::Normal {
                sample_rate_div: 39,
                dlpf: Dlpf::Hz44,
            },
        }
    }
}

/// Motion interrupt settings.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WakeOnMotion {
    /// Threshold in mg: 1 mg/LSB on MPU-6050 (high-passed at 5 Hz),
    /// 4 mg/LSB on MPU-6500/9250 (difference to the previous sample).
    /// Clamped to 1..=255 LSB.
    pub threshold_mg: u32,
    /// MPU-6050 only: motion duration in ms (`MOT_DUR`, 1 ms/LSB).
    pub duration_ms: u8,
    /// Hold INT until `INT_STATUS` is read (else 50 µs pulses).
    pub latched: bool,
}

impl Default for WakeOnMotion {
    fn default() -> Self {
        Self {
            threshold_mg: 64,
            duration_ms: 1,
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

/// Up to ten register writes computed without bus traffic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RegWrites {
    buf: [(u8, u8); 10],
    len: usize,
}

impl RegWrites {
    const fn new() -> Self {
        Self {
            buf: [(0, 0); 10],
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

/// Decode three big-endian i16 words (X, Y, Z).
pub fn decode_axes(bytes: &[u8]) -> [i16; 3] {
    [
        i16::from_be_bytes([bytes[0], bytes[1]]),
        i16::from_be_bytes([bytes[2], bytes[3]]),
        i16::from_be_bytes([bytes[4], bytes[5]]),
    ]
}

/// Decode the 14-byte block at `ACCEL_XOUT_H` (accel, temperature, gyro).
pub fn decode_block(bytes: &[u8; 14], accel: AccelRange, gyro: Option<GyroRange>) -> Measurement {
    let a = decode_axes(&bytes[..6]);
    Measurement {
        accel: Vector3 {
            x: accel_mg(a[0], accel),
            y: accel_mg(a[1], accel),
            z: accel_mg(a[2], accel),
        },
        gyro: gyro.map(|range| {
            let g = decode_axes(&bytes[8..]);
            Vector3 {
                x: gyro_mdps(g[0], range),
                y: gyro_mdps(g[1], range),
                z: gyro_mdps(g[2], range),
            }
        }),
    }
}

/// `PWR_MGMT_1` value for running (non-cycle) operation.
fn pwr1_running(variant: Variant, config: &Config) -> u8 {
    // The MPU-6050 PLL needs a running gyroscope; the MPU-6500 CLKSEL=1
    // falls back to the internal oscillator by itself.
    let clk = if config.gyro.is_some() || variant.is_6500() {
        PWR1_CLKSEL_PLL
    } else {
        0
    };
    clk | PWR1_TEMP_DIS
}

fn pwr1_final(variant: Variant, config: &Config) -> u8 {
    match config.mode {
        Mode::LowPowerAccel(_) => PWR1_CYCLE | PWR1_TEMP_DIS,
        Mode::Normal { .. } => pwr1_running(variant, config),
    }
}

/// Register writes that apply `config`; `None` if the mode is not
/// available on `variant` or the gyroscope is on in cycle mode.
pub fn config_writes(variant: Variant, config: &Config, wom: bool) -> Option<RegWrites> {
    let mut w = RegWrites::new();
    let stby = if config.gyro.is_some() {
        0
    } else {
        PWR2_STBY_GYRO
    };
    let (div, dlpf, cycle) = match config.mode {
        Mode::Normal {
            sample_rate_div,
            dlpf,
        } => (sample_rate_div, dlpf as u8, None),
        Mode::LowPowerAccel(rate) => {
            if config.gyro.is_some() {
                return None;
            }
            (0, Dlpf::Hz184 as u8, Some(rate.code(variant)?))
        }
    };
    w.push(reg::PWR_MGMT_1, pwr1_running(variant, config));
    let pwr2 = match cycle {
        Some(code) if !variant.is_6500() => code << 6 | stby,
        _ => stby,
    };
    w.push(reg::PWR_MGMT_2, pwr2);
    w.push(reg::SMPLRT_DIV, div);
    w.push(reg::CONFIG, dlpf);
    w.push(
        reg::GYRO_CONFIG,
        config.gyro.unwrap_or_default().bits() << 3,
    );
    if variant.is_6500() {
        w.push(reg::ACCEL_CONFIG, config.accel_range.bits() << 3);
        w.push(reg::ACCEL_CONFIG2, dlpf);
        if let Some(code) = cycle {
            w.push(reg::LP_ACCEL_ODR, code);
        }
    } else {
        // ACCEL_HPF only feeds the motion detector, not the output registers.
        w.push(
            reg::ACCEL_CONFIG,
            config.accel_range.bits() << 3 | ACCEL_HPF_5HZ,
        );
    }
    w.push(reg::INT_ENABLE, if wom { INT_MOTION } else { INT_DATA_RDY });
    w.push(reg::PWR_MGMT_1, pwr1_final(variant, config));
    Some(w)
}

/// Time until the first valid sample after [`config_writes`].
pub fn startup_wait_us(config: &Config) -> WaitUs {
    if config.gyro.is_some() {
        GYRO_START_US
    } else {
        ACCEL_START_US
    }
}

/// Register writes that enable the motion interrupt (Wake-on-Motion flow
/// of PS-MPU-9250A-01 Figure 3; MPU-6050 PS §8.3). Cycle mode, if
/// configured, is re-entered last.
pub fn wake_on_motion_writes(variant: Variant, config: &Config, wom: &WakeOnMotion) -> RegWrites {
    let mut w = RegWrites::new();
    w.push(reg::PWR_MGMT_1, pwr1_running(variant, config));
    w.push(
        reg::INT_PIN_CFG,
        if wom.latched { INT_PIN_LATCH } else { 0 },
    );
    w.push(reg::INT_ENABLE, INT_MOTION);
    if variant.is_6500() {
        w.push(reg::MOT_DETECT_CTRL, ACCEL_INTEL_EN_MODE);
        let code = (wom.threshold_mg + 2) / 4;
        w.push(reg::MOT_THR, code.clamp(1, 255) as u8);
    } else {
        w.push(reg::MOT_THR, wom.threshold_mg.clamp(1, 255) as u8);
        w.push(reg::MOT_DUR, wom.duration_ms.max(1));
    }
    w.push(reg::PWR_MGMT_1, pwr1_final(variant, config));
    w
}

/// Register writes that disable the motion interrupt and restore
/// data-ready.
pub fn wake_on_motion_off_writes(variant: Variant) -> RegWrites {
    let mut w = RegWrites::new();
    w.push(reg::INT_ENABLE, INT_DATA_RDY);
    w.push(reg::INT_PIN_CFG, 0);
    if variant.is_6500() {
        w.push(reg::MOT_DETECT_CTRL, 0);
    }
    w.push(reg::MOT_THR, 0);
    w
}

macro_rules! driver {
    ($doc:literal, $Iface:ident, $mode:ident, [$($async:tt)?], [$($aw:tt)*]) => {
        #[doc = $doc]
        #[derive(Debug)]
        pub struct Mpu6050<I> {
            iface: I,
            variant: Variant,
            config: Config,
            wom: bool,
            motion: bool,
        }

        impl<I> Mpu6050<I> {
            /// Create a driver over any register interface; no bus traffic.
            pub fn new(iface: I) -> Self {
                Self {
                    iface,
                    variant: Variant::Mpu6050,
                    config: Config::default(),
                    wom: false,
                    motion: false,
                }
            }

            /// Set the configuration used by `configure()` / `wake()`.
            pub fn with_config(mut self, config: Config) -> Self {
                self.config = config;
                self
            }

            /// Set the variant without probing.
            pub fn with_variant(mut self, variant: Variant) -> Self {
                self.variant = variant;
                self
            }

            /// Current configuration.
            pub fn config(&self) -> &Config {
                &self.config
            }

            /// Detected / selected variant.
            pub fn variant(&self) -> Variant {
                self.variant
            }

            /// Give the register interface back.
            pub fn into_interface(self) -> I {
                self.iface
            }
        }

        impl<I2C> Mpu6050<I2cInterface<I2C>> {
            /// Create an I2C driver; no bus traffic.
            pub fn new_i2c(i2c: I2C, address: u8) -> Self {
                Self::new(I2cInterface::new(i2c, address))
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.iface.release()
            }
        }

        impl<SPI> Mpu6050<SpiInterface<SPI>> {
            /// Create an SPI driver (MPU-6000/6500/9250); no bus traffic.
            pub fn new_spi(spi: SPI) -> Self {
                Self::new(SpiInterface::new(spi))
            }

            /// Give the device back.
            pub fn release(self) -> SPI {
                self.iface.release()
            }
        }

        impl<I: $Iface> Mpu6050<I> {
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

            $($async)? fn write_all(&mut self, writes: &RegWrites) -> Result<(), Error<I::Error>> {
                for &(r, v) in writes.as_slice() {
                    self.write_register(r, v) $($aw)* ?;
                }
                Ok(())
            }

            $($async)? fn read_int_status(&mut self) -> Result<u8, Error<I::Error>> {
                let s = self.read_register(reg::INT_STATUS) $($aw)* ?;
                self.motion |= s & INT_MOTION != 0;
                Ok(s)
            }

            /// Read `WHO_AM_I` and select the [`Variant`].
            pub $($async)? fn probe(&mut self) -> Result<Variant, Error<I::Error>> {
                let id = self.read_register(reg::WHO_AM_I) $($aw)* ?;
                self.variant = Variant::from_id(id).ok_or(Error::UnexpectedId(u32::from(id)))?;
                Ok(self.variant)
            }

            /// `DEVICE_RESET`; the part comes back in sleep mode.
            pub $($async)? fn reset(&mut self) -> Result<WaitUs, Error<I::Error>> {
                self.write_register(reg::PWR_MGMT_1, PWR1_RESET) $($aw)* ?;
                self.wom = false;
                Ok(RESET_WAIT_US)
            }

            /// SPI only (MPU-6000/6500/9250): disable the I2C slave
            /// interface, as recommended when the part shares an SPI bus.
            pub $($async)? fn disable_i2c(&mut self) -> Result<(), Error<I::Error>> {
                self.write_register(reg::USER_CTRL, USER_CTRL_I2C_IF_DIS) $($aw)*
            }

            /// Apply the stored configuration and start sampling. Returns
            /// the time until the first valid sample.
            pub $($async)? fn configure(&mut self) -> Result<WaitUs, Error<I::Error>> {
                let writes = config_writes(self.variant, &self.config, self.wom).ok_or(Error::Unsupported)?;
                self.write_all(&writes) $($aw)* ?;
                Ok(startup_wait_us(&self.config))
            }

            /// Store a new configuration and apply it.
            pub $($async)? fn set_config(&mut self, config: Config) -> Result<WaitUs, Error<I::Error>> {
                self.config = config;
                self.configure() $($aw)*
            }

            /// Sleep mode (MPU-6050 5 µA, MPU-9250 8 µA); motion detection
            /// stops too.
            pub $($async)? fn power_down(&mut self) -> Result<(), Error<I::Error>> {
                self.write_register(reg::PWR_MGMT_1, PWR1_SLEEP | PWR1_TEMP_DIS) $($aw)*
            }

            /// Enable the motion interrupt on INT. Combine with
            /// [`Mode::LowPowerAccel`] for a µA-range wake-on-motion
            /// sensor. While enabled, data-ready is not signalled and
            /// `read_measurement()` returns the latest sample without
            /// waiting for new data.
            pub $($async)? fn enable_wake_on_motion(&mut self, wom: WakeOnMotion) -> Result<(), Error<I::Error>> {
                let writes = wake_on_motion_writes(self.variant, &self.config, &wom);
                self.write_all(&writes) $($aw)* ?;
                self.wom = true;
                Ok(())
            }

            /// Disable the motion interrupt and re-enable data-ready.
            pub $($async)? fn disable_wake_on_motion(&mut self) -> Result<(), Error<I::Error>> {
                self.write_all(&wake_on_motion_off_writes(self.variant)) $($aw)* ?;
                self.wom = false;
                Ok(())
            }

            /// Whether motion was detected since the last call. Reads (and
            /// thereby clears) `INT_STATUS`; motion flags seen by
            /// `read_measurement()` are remembered.
            pub $($async)? fn motion_detected(&mut self) -> Result<bool, Error<I::Error>> {
                self.read_int_status() $($aw)* ?;
                Ok(core::mem::take(&mut self.motion))
            }
        }

        impl<I: $Iface> zigbee_sensor::$mode::Sensor for Mpu6050<I> {
            type BusError = I::Error;
            type Measurement = Measurement;

            /// Free-running: returns 0.
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I::Error>> {
                Ok(0)
            }

            /// [`Error::NotReady`] until `INT_STATUS` reports new data
            /// (not checked while wake-on-motion is enabled).
            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<I::Error>> {
                let status = self.read_int_status() $($aw)* ?;
                if !self.wom && status & INT_DATA_RDY == 0 {
                    return Err(Error::NotReady);
                }
                let mut b = [0u8; 14];
                let len = if self.config.gyro.is_some() { 14 } else { 6 };
                self.iface.read_regs(reg::ACCEL_XOUT_H, &mut b[..len]) $($aw)* .map_err(Error::bus)?;
                Ok(decode_block(&b, self.config.accel_range, self.config.gyro))
            }
        }

        impl<I: $Iface> zigbee_sensor::$mode::LowPower for Mpu6050<I> {
            type BusError = I::Error;

            /// Sleep mode.
            $($async)? fn sleep(&mut self) -> Result<(), Error<I::Error>> {
                self.power_down() $($aw)*
            }

            /// Re-apply the stored configuration.
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
        "Async twin of [`super::Mpu6050`].",
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

    const A: u8 = ADDRESS_AD0_LOW;

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
        // PS-MPU-6000A §6.1/6.2: 16384 LSB/g at ±2 g, 2048 LSB/g at ±16 g,
        // 131 LSB/dps at ±250 dps, 16.4 LSB/dps at ±2000 dps.
        assert_eq!(accel_mg(16_384, AccelRange::G2), 1_000);
        assert_eq!(accel_mg(-16_384, AccelRange::G2), -1_000);
        assert_eq!(accel_mg(2_048, AccelRange::G16), 1_000);
        assert_eq!(gyro_mdps(131, GyroRange::Dps250), 1_000);
        assert_eq!(gyro_mdps(-164, GyroRange::Dps2000), -10_000);
        assert_eq!(gyro_mdps(655, GyroRange::Dps500), 10_000);
    }

    #[test]
    fn probe_variants() {
        for (id, v) in [
            (0x68, Variant::Mpu6050),
            (0x70, Variant::Mpu6500),
            (0x71, Variant::Mpu9250),
        ] {
            let mut i2c = Mock::new(&[rd(reg::WHO_AM_I, &[id])]);
            let mut imu = Mpu6050::new_i2c(&mut i2c, A);
            assert_eq!(imu.probe().unwrap(), v);
            i2c.done();
        }
        let mut i2c = Mock::new(&[rd(reg::WHO_AM_I, &[0x98])]);
        let mut imu = Mpu6050::new_i2c(&mut i2c, A);
        assert_eq!(imu.probe(), Err(Error::UnexpectedId(0x98)));
        i2c.done();
    }

    #[test]
    fn configure_default_mpu6050() {
        let mut i2c = Mock::new(&[
            wr(reg::PWR_MGMT_1, 0x08),
            wr(reg::PWR_MGMT_2, 0x07),
            wr(reg::SMPLRT_DIV, 39),
            wr(reg::CONFIG, 3),
            wr(reg::GYRO_CONFIG, 0x18),
            wr(reg::ACCEL_CONFIG, 0x01),
            wr(reg::INT_ENABLE, 0x01),
            wr(reg::PWR_MGMT_1, 0x08),
        ]);
        let mut imu = Mpu6050::new_i2c(&mut i2c, A);
        assert_eq!(imu.configure().unwrap(), ACCEL_START_US);
        i2c.done();
    }

    #[test]
    fn cycle_mode_per_variant() {
        let cfg = Config {
            accel_range: AccelRange::G4,
            gyro: None,
            mode: Mode::LowPowerAccel(CycleRate::Hz5),
        };
        let w = config_writes(Variant::Mpu6050, &cfg, false).unwrap();
        assert_eq!(
            w.as_slice(),
            &[
                (reg::PWR_MGMT_1, 0x08),
                (reg::PWR_MGMT_2, 0x47),
                (reg::SMPLRT_DIV, 0),
                (reg::CONFIG, 1),
                (reg::GYRO_CONFIG, 0x18),
                (reg::ACCEL_CONFIG, 0x09),
                (reg::INT_ENABLE, 0x01),
                (reg::PWR_MGMT_1, 0x28),
            ]
        );
        assert!(config_writes(Variant::Mpu6500, &cfg, false).is_none());

        let cfg = Config {
            mode: Mode::LowPowerAccel(CycleRate::Hz0_98),
            ..cfg
        };
        let w = config_writes(Variant::Mpu9250, &cfg, true).unwrap();
        assert_eq!(
            w.as_slice(),
            &[
                (reg::PWR_MGMT_1, 0x09),
                (reg::PWR_MGMT_2, 0x07),
                (reg::SMPLRT_DIV, 0),
                (reg::CONFIG, 1),
                (reg::GYRO_CONFIG, 0x18),
                (reg::ACCEL_CONFIG, 0x08),
                (reg::ACCEL_CONFIG2, 1),
                (reg::LP_ACCEL_ODR, 2),
                (reg::INT_ENABLE, 0x40),
                (reg::PWR_MGMT_1, 0x28),
            ]
        );
        let with_gyro = Config {
            gyro: Some(GyroRange::Dps250),
            ..cfg
        };
        assert!(config_writes(Variant::Mpu9250, &with_gyro, false).is_none());
    }

    #[test]
    fn wake_on_motion_mpu6500_flow() {
        let cfg = Config {
            mode: Mode::LowPowerAccel(CycleRate::Hz31_25),
            ..Config::default()
        };
        let wom = WakeOnMotion {
            threshold_mg: 100,
            duration_ms: 5,
            latched: true,
        };
        let mut i2c = Mock::new(&[
            wr(reg::PWR_MGMT_1, 0x09),
            wr(reg::INT_PIN_CFG, 0x20),
            wr(reg::INT_ENABLE, 0x40),
            wr(reg::MOT_DETECT_CTRL, 0xC0),
            wr(reg::MOT_THR, 25),
            wr(reg::PWR_MGMT_1, 0x28),
            // read_measurement: no data-ready check while WOM is on, but the
            // motion flag is remembered.
            rd(reg::INT_STATUS, &[0x40]),
            rd(reg::ACCEL_XOUT_H, &[0x40, 0x00, 0xC0, 0x00, 0, 0]),
            rd(reg::INT_STATUS, &[0x00]),
            rd(reg::INT_STATUS, &[0x00]),
        ]);
        let mut imu = Mpu6050::new_i2c(&mut i2c, A)
            .with_variant(Variant::Mpu6500)
            .with_config(cfg);
        imu.enable_wake_on_motion(wom).unwrap();
        let m = imu.read_measurement().unwrap();
        assert_eq!(
            m.accel,
            Vector3 {
                x: 1_000,
                y: -1_000,
                z: 0
            }
        );
        assert!(imu.motion_detected().unwrap());
        assert!(!imu.motion_detected().unwrap());
        i2c.done();

        let w = wake_on_motion_writes(Variant::Mpu6050, &Config::default(), &wom);
        assert_eq!(
            &w.as_slice()[3..5],
            &[(reg::MOT_THR, 100), (reg::MOT_DUR, 5)]
        );
    }

    #[test]
    fn not_ready_then_gyro_block_over_spi() {
        let cfg = Config {
            gyro: Some(GyroRange::Dps2000),
            ..Config::default()
        };
        let mut block = vec![0x40, 0x00, 0, 0, 0, 0, 0x12, 0x34];
        block.extend_from_slice(&[0x00, 0xA4, 0xFF, 0x5C, 0, 0]);
        let mut spi = SpiMock::new(&[
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![0xBA]),
            SpiTx::read_vec(vec![0x00]),
            SpiTx::transaction_end(),
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![0xBA]),
            SpiTx::read_vec(vec![0x01]),
            SpiTx::transaction_end(),
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![0xBB]),
            SpiTx::read_vec(block),
            SpiTx::transaction_end(),
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![0x6B, 0x48]),
            SpiTx::transaction_end(),
        ]);
        let mut imu = Mpu6050::new_spi(&mut spi).with_config(cfg);
        assert_eq!(imu.start_measurement().unwrap(), 0);
        assert_eq!(imu.read_measurement(), Err(Error::NotReady));
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
        imu.sleep().unwrap();
        spi.done();
    }

    #[test]
    fn async_configure_and_read() {
        use zigbee_sensor::asynch::Sensor as _;

        let mut ex = writes(&config_writes(Variant::Mpu6050, &Config::default(), false).unwrap());
        ex.push(rd(reg::INT_STATUS, &[0x01]));
        ex.push(rd(reg::ACCEL_XOUT_H, &[0, 0, 0, 0, 0x40, 0x00]));
        let mut i2c = Mock::new(&ex);
        let mut imu = asynch::Mpu6050::new_i2c(&mut i2c, A);
        assert_eq!(block_on(imu.configure()).unwrap(), ACCEL_START_US);
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
