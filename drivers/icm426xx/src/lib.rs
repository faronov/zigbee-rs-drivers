//! TDK InvenSense ICM-42605, ICM-42688-P and ICM-42670-P 6-axis IMUs over
//! I2C or SPI.
//!
//! [`Variant`] is detected from `WHO_AM_I`. The ICM-42605 and ICM-42688-P
//! share one register map (bank 0 plus `REG_BANK_SEL` banks 1–4, the
//! Wake-on-Motion thresholds live in bank 4). The ICM-42670-P has a
//! different bank-0 layout and reaches its WoM thresholds through the
//! indirect MREG1 window (`BLK_SEL_W` / `MADDR_W` / `M_W`), which needs
//! 10 µs of bus silence after every access; [`Variant::regs`] holds the
//! per-map addresses.
//!
//! The device is free-running: [`Icm426xx::configure`] starts the
//! accelerometer (low-power mode by default) and `read_measurement()`
//! returns [`Error::NotReady`] until the data-ready status bit is set. The
//! gyroscope is off by default.
//!
//! Wake-on-Motion is armed in steps because the datasheet flows contain
//! waits: call [`Icm426xx::enable_wake_on_motion`], then wait the
//! returned time and call [`Icm426xx::continue_wake_on_motion`] until it
//! returns 0.
//!
//! References:
//! * TDK DS-000292 "ICM-42605 Datasheet", Rev 1.2 (§8.7 WoM programming);
//! * TDK DS-000347 "ICM-42688-P Datasheet", Rev 1.8;
//! * TDK DS-000451 "ICM-42670-P Datasheet", Rev 1.0 (§13 MREG access).
//!
//! ```ignore
//! let mut imu = Icm426xx::new_i2c(i2c, ADDRESS_AP_AD0_LOW);
//! imu.probe()?;
//! let wait = imu.configure()?;
//! let m = imu.read_measurement()?; // NotReady until new data
//! ```

#![no_std]

mod interface;

pub use interface::{AsyncInterface, I2cInterface, Interface, SpiInterface};
use zigbee_sensor::{Error, Quantity, Readings, Vector3, WaitUs};

/// I2C address with AP_AD0 low.
pub const ADDRESS_AP_AD0_LOW: u8 = 0x68;
/// I2C address with AP_AD0 high.
pub const ADDRESS_AP_AD0_HIGH: u8 = 0x69;

/// `WHO_AM_I` register (0x75 in all maps).
pub const WHO_AM_I: u8 = 0x75;

/// Wait after a soft reset ("wait 1 ms ... before attempting any other
/// register access").
pub const RESET_WAIT_US: WaitUs = 1_000;
/// Accelerometer start-up time from sleep to valid data (typical, all parts).
pub const ACCEL_START_US: WaitUs = 10_000;
/// Gyroscope start-up time (typical, all parts).
pub const GYRO_START_US: WaitUs = 30_000;
/// Bus silence required after an ICM-42670-P MREG access.
pub const MREG_WAIT_US: WaitUs = 10;
/// Poll interval while waiting for ICM-42670-P `MCLK_RDY`.
pub const MCLK_POLL_US: WaitUs = 100;

const PWR_TEMP_DIS: u8 = 0x20;
const PWR_IDLE: u8 = 0x10;
const PWR_GYRO_LN: u8 = 0x0C;
const PWR_ACCEL_LP: u8 = 0x02;
const PWR_ACCEL_LN: u8 = 0x03;
const INT1_PUSH_PULL_HIGH: u8 = 0x03;
const INT1_LATCHED: u8 = 0x04;
const INT_SRC0_DRDY: u8 = 0x08;
const INT_SRC1_WOM_XYZ: u8 = 0x07;
const INT_STATUS2_WOM: u8 = 0x07;
const INT_CONFIG1_FAST_ODR: u8 = 0x60;
const SMD_MODE_WOM: u8 = 0x01;
const WOM_EN: u8 = 0x01;
const MCLK_RDY: u8 = 0x08;

/// Register addresses of one register map.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Regs {
    /// Soft-reset register and bit (`DEVICE_CONFIG` / `SIGNAL_PATH_RESET`).
    pub reset: (u8, u8),
    /// `INT_CONFIG`.
    pub int_config: u8,
    /// `ACCEL_DATA_X1`; the gyroscope block follows directly.
    pub accel_data: u8,
    /// Data-ready status register and bit.
    pub drdy_status: (u8, u8),
    /// `INT_STATUS2` (WoM flags in bits 2:0).
    pub int_status2: u8,
    /// `PWR_MGMT0`.
    pub pwr_mgmt0: u8,
    /// `GYRO_CONFIG0`.
    pub gyro_config0: u8,
    /// `ACCEL_CONFIG0`.
    pub accel_config0: u8,
    /// `INT_SOURCE0`.
    pub int_source0: u8,
    /// `INT_SOURCE1`.
    pub int_source1: u8,
    /// `ACCEL_WOM_X_THR` (Y, Z follow) in bank 4 / MREG1.
    pub wom_x_thr: u8,
}

/// ICM-42605 / ICM-42688-P bank-0-only registers.
pub mod reg426 {
    #![allow(missing_docs)]
    pub const INT_CONFIG1: u8 = 0x64;
    pub const SMD_CONFIG: u8 = 0x57;
    pub const REG_BANK_SEL: u8 = 0x76;
}

/// ICM-42670-P-only registers.
pub mod reg4267 {
    #![allow(missing_docs)]
    pub const MCLK_RDY: u8 = 0x00;
    pub const WOM_CONFIG: u8 = 0x27;
    pub const BLK_SEL_W: u8 = 0x79;
    pub const MADDR_W: u8 = 0x7A;
    pub const M_W: u8 = 0x7B;
}

const REGS_426: Regs = Regs {
    reset: (0x11, 0x01),
    int_config: 0x14,
    accel_data: 0x1F,
    drdy_status: (0x2D, 0x08),
    int_status2: 0x37,
    pwr_mgmt0: 0x4E,
    gyro_config0: 0x4F,
    accel_config0: 0x50,
    int_source0: 0x65,
    int_source1: 0x66,
    wom_x_thr: 0x4A,
};

const REGS_4267: Regs = Regs {
    reset: (0x02, 0x10),
    int_config: 0x06,
    accel_data: 0x0B,
    drdy_status: (0x39, 0x01),
    int_status2: 0x3B,
    pwr_mgmt0: 0x1F,
    gyro_config0: 0x20,
    accel_config0: 0x21,
    int_source0: 0x2B,
    int_source1: 0x2C,
    wom_x_thr: 0x4B,
};

/// Part, selected from `WHO_AM_I`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Variant {
    /// ICM-42605 (`WHO_AM_I` = 0x42).
    Icm42605,
    /// ICM-42688-P (0x47).
    #[default]
    Icm42688P,
    /// ICM-42670-P (0x67).
    Icm42670P,
}

impl Variant {
    /// Map a `WHO_AM_I` value.
    pub const fn from_id(id: u8) -> Option<Self> {
        match id {
            0x42 => Some(Self::Icm42605),
            0x47 => Some(Self::Icm42688P),
            0x67 => Some(Self::Icm42670P),
            _ => None,
        }
    }

    const fn is_4267(self) -> bool {
        matches!(self, Self::Icm42670P)
    }

    /// Register map of this part.
    pub const fn regs(self) -> &'static Regs {
        if self.is_4267() {
            &REGS_4267
        } else {
            &REGS_426
        }
    }
}

/// Accelerometer full scale.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AccelRange {
    /// ±2 g.
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
    /// `ACCEL_FS_SEL` / `ACCEL_UI_FS_SEL` code (same in all maps).
    const fn bits(self) -> u8 {
        3 - self as u8
    }

    /// Full scale in mg.
    pub const fn full_scale_mg(self) -> i32 {
        2_000 << (self as u8)
    }
}

/// Gyroscope full scale.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GyroRange {
    /// ±125 dps, 262 LSB/dps (ICM-42605 / ICM-42688-P only).
    Dps125,
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
    /// `GYRO_FS_SEL` code (000 = ±2000 dps).
    const fn bits(self) -> u8 {
        4 - self as u8
    }

    /// Datasheet sensitivity in 0.1 LSB per dps.
    const fn tenth_lsb_per_dps(self) -> i32 {
        match self {
            Self::Dps125 => 2620,
            Self::Dps250 => 1310,
            Self::Dps500 => 655,
            Self::Dps1000 => 328,
            Self::Dps2000 => 164,
        }
    }
}

/// Output data rate (shared by accelerometer and gyroscope).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Odr {
    /// 1.5625 Hz (accelerometer low-power only).
    Hz1_5625,
    /// 3.125 Hz (accelerometer low-power only).
    Hz3_125,
    /// 6.25 Hz (accelerometer low-power only).
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
    /// 400 Hz (ICM-42670-P).
    Hz400,
    /// 500 Hz (ICM-42605 / ICM-42688-P).
    Hz500,
    /// 800 Hz (ICM-42670-P, low-noise only).
    Hz800,
    /// 1 kHz (ICM-42605 / ICM-42688-P, low-noise only).
    Hz1000,
    /// 1.6 kHz (ICM-42670-P, low-noise only).
    Hz1600,
    /// 2 kHz (ICM-42605 / ICM-42688-P, low-noise only).
    Hz2000,
    /// 4 kHz (ICM-42605 / ICM-42688-P, low-noise only).
    Hz4000,
    /// 8 kHz (ICM-42605 / ICM-42688-P, low-noise only).
    Hz8000,
}

impl Odr {
    /// `ACCEL_ODR` / `GYRO_ODR` code, or `None` if the rate is not
    /// available on `variant` in the given mode (`low_power` = accelerometer
    /// LP mode; the gyroscope uses the low-noise codes).
    pub const fn code(self, variant: Variant, low_power: bool) -> Option<u8> {
        let (code, lp, ln) = if variant.is_4267() {
            match self {
                Self::Hz1600 => (5, false, true),
                Self::Hz800 => (6, false, true),
                Self::Hz400 => (7, true, true),
                Self::Hz200 => (8, true, true),
                Self::Hz100 => (9, true, true),
                Self::Hz50 => (10, true, true),
                Self::Hz25 => (11, true, true),
                Self::Hz12_5 => (12, true, true),
                Self::Hz6_25 => (13, true, false),
                Self::Hz3_125 => (14, true, false),
                Self::Hz1_5625 => (15, true, false),
                _ => return None,
            }
        } else {
            match self {
                Self::Hz8000 => (3, false, true),
                Self::Hz4000 => (4, false, true),
                Self::Hz2000 => (5, false, true),
                Self::Hz1000 => (6, false, true),
                Self::Hz200 => (7, true, true),
                Self::Hz100 => (8, true, true),
                Self::Hz50 => (9, true, true),
                Self::Hz25 => (10, true, true),
                Self::Hz12_5 => (11, true, true),
                Self::Hz6_25 => (12, true, false),
                Self::Hz3_125 => (13, true, false),
                Self::Hz1_5625 => (14, true, false),
                Self::Hz500 => (15, true, true),
                _ => return None,
            }
        };
        if (low_power && lp) || (!low_power && ln) {
            Some(code)
        } else {
            None
        }
    }
}

/// Accelerometer power mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AccelMode {
    /// Duty-cycled low-power mode on the wake-up oscillator.
    #[default]
    LowPower,
    /// Low-noise mode.
    LowNoise,
}

/// Device configuration, applied by [`Icm426xx::configure`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Accelerometer full scale.
    pub accel_range: AccelRange,
    /// Accelerometer power mode.
    pub accel_mode: AccelMode,
    /// Output data rate of both sensors.
    pub odr: Odr,
    /// Gyroscope full scale; `None` turns the gyroscope off.
    pub gyro: Option<GyroRange>,
}

impl Default for Config {
    /// ±2 g, 25 Hz, accelerometer low-power mode, gyroscope off.
    fn default() -> Self {
        Self {
            accel_range: AccelRange::G2,
            accel_mode: AccelMode::LowPower,
            odr: Odr::Hz25,
            gyro: None,
        }
    }
}

/// Wake-on-Motion settings.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WakeOnMotion {
    /// Threshold in mg on every axis, 1 g / 256 per LSB, clamped to
    /// 1..=255 LSB (~4–996 mg).
    pub threshold_mg: u32,
    /// Compare with the previous sample (`true`) or with the sample taken
    /// when WoM was enabled (`false`).
    pub compare_previous: bool,
    /// ICM-42670-P only: over-threshold events before INT asserts (1..=4).
    pub events: u8,
    /// Hold INT1 until `INT_STATUS2` is read (else pulsed).
    pub latched: bool,
}

impl Default for WakeOnMotion {
    fn default() -> Self {
        Self {
            threshold_mg: 64,
            compare_previous: true,
            events: 1,
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

fn div_round(n: i32, d: i32) -> i32 {
    if n >= 0 {
        (n + d / 2) / d
    } else {
        (n - d / 2) / d
    }
}

/// Raw accelerometer count to mg (full scale / 32768).
pub fn accel_mg(raw: i16, range: AccelRange) -> i32 {
    (i32::from(raw) * range.full_scale_mg() + (1 << 14)) >> 15
}

/// Raw gyroscope count to mdps using the datasheet sensitivity.
pub fn gyro_mdps(raw: i16, range: GyroRange) -> i32 {
    div_round(i32::from(raw) * 10_000, range.tenth_lsb_per_dps())
}

fn axes(bytes: &[u8], f: impl Fn(i16) -> i32) -> Vector3 {
    let w = |i: usize| f(i16::from_be_bytes([bytes[i], bytes[i + 1]]));
    Vector3 {
        x: w(0),
        y: w(2),
        z: w(4),
    }
}

/// Decode the 12-byte big-endian accelerometer + gyroscope block.
pub fn decode_block(bytes: &[u8; 12], accel: AccelRange, gyro: Option<GyroRange>) -> Measurement {
    Measurement {
        accel: axes(&bytes[..6], |r| accel_mg(r, accel)),
        gyro: gyro.map(|range| axes(&bytes[6..], |r| gyro_mdps(r, range))),
    }
}

/// `PWR_MGMT0` value that runs `config`.
pub fn pwr_mgmt0(variant: Variant, config: &Config) -> u8 {
    let accel = match config.accel_mode {
        AccelMode::LowPower => PWR_ACCEL_LP,
        AccelMode::LowNoise => PWR_ACCEL_LN,
    };
    let gyro = if config.gyro.is_some() {
        PWR_GYRO_LN
    } else {
        0
    };
    let temp = if variant.is_4267() { 0 } else { PWR_TEMP_DIS };
    temp | gyro | accel
}

/// `PWR_MGMT0` value for sleep (both sensors off, RC oscillator off).
pub const fn pwr_mgmt0_off(variant: Variant) -> u8 {
    if variant.is_4267() { 0 } else { PWR_TEMP_DIS }
}

const fn int_config(latched: bool) -> u8 {
    if latched {
        INT1_PUSH_PULL_HIGH | INT1_LATCHED
    } else {
        INT1_PUSH_PULL_HIGH
    }
}

/// Register writes that apply `config` (`PWR_MGMT0` last). INT1 is
/// push-pull active-high; data-ready is routed to it unless `wom` is set.
/// `None` if the ODR or a range is not available in that mode / variant.
pub fn config_writes(variant: Variant, config: &Config, wom: bool) -> Option<RegWrites> {
    let r = variant.regs();
    let lp = matches!(config.accel_mode, AccelMode::LowPower);
    let accel_odr = config.odr.code(variant, lp)?;
    let mut w = RegWrites::new();
    if !variant.is_4267() {
        let fast = matches!(config.odr, Odr::Hz4000 | Odr::Hz8000);
        w.push(
            reg426::INT_CONFIG1,
            if fast { INT_CONFIG1_FAST_ODR } else { 0 },
        );
    }
    if !wom {
        w.push(r.int_config, int_config(false));
    }
    if let Some(range) = config.gyro {
        if variant.is_4267() && matches!(range, GyroRange::Dps125) {
            return None;
        }
        let gyro_odr = config.odr.code(variant, false)?;
        w.push(r.gyro_config0, range.bits() << 5 | gyro_odr);
    }
    w.push(r.accel_config0, config.accel_range.bits() << 5 | accel_odr);
    w.push(r.int_source0, if wom { 0 } else { INT_SRC0_DRDY });
    w.push(r.pwr_mgmt0, pwr_mgmt0(variant, config));
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

/// WoM threshold code (1 g / 256 per LSB).
pub fn wom_threshold_code(mg: u32) -> u8 {
    let code = (mg.min(1_000) * 256 + 500) / 1_000;
    code.clamp(1, 255) as u8
}

/// Step `step` of the Wake-on-Motion arming sequence: the writes and the
/// wait before the next step (0 after the last step), or `None` when the
/// sequence is complete.
///
/// ICM-42605 / ICM-42688-P (datasheet §8.7): thresholds in bank 4, wait
/// 1 ms, route WoM to INT1, wait 50 ms, then `SMD_CONFIG` with
/// `SMD_MODE` = 01 (WoM). ICM-42670-P: RC oscillator on (`IDLE`), one
/// MREG1 write per axis with 10 µs of silence, then `WOM_CONFIG`.
pub fn wom_step(
    variant: Variant,
    config: &Config,
    wom: &WakeOnMotion,
    step: u8,
) -> Option<(RegWrites, WaitUs)> {
    let r = variant.regs();
    let thr = wom_threshold_code(wom.threshold_mg);
    let mode = u8::from(wom.compare_previous);
    let mut w = RegWrites::new();
    let wait = if variant.is_4267() {
        match step {
            0 => {
                w.push(r.int_config, int_config(wom.latched));
                w.push(r.int_source0, 0);
                w.push(r.pwr_mgmt0, pwr_mgmt0(variant, config) | PWR_IDLE);
                MREG_WAIT_US
            }
            1..=3 => {
                w.push(reg4267::BLK_SEL_W, 0);
                w.push(reg4267::MADDR_W, r.wom_x_thr + step - 1);
                w.push(reg4267::M_W, thr);
                MREG_WAIT_US
            }
            4 => {
                let cfg = (wom.events.clamp(1, 4) - 1) << 3 | mode << 1;
                w.push(reg4267::WOM_CONFIG, cfg);
                w.push(r.int_source1, INT_SRC1_WOM_XYZ);
                w.push(reg4267::WOM_CONFIG, cfg | WOM_EN);
                w.push(r.pwr_mgmt0, pwr_mgmt0(variant, config));
                0
            }
            _ => return None,
        }
    } else {
        match step {
            0 => {
                w.push(reg426::REG_BANK_SEL, 4);
                w.push(r.wom_x_thr, thr);
                w.push(r.wom_x_thr + 1, thr);
                w.push(r.wom_x_thr + 2, thr);
                w.push(reg426::REG_BANK_SEL, 0);
                w.push(r.int_config, int_config(wom.latched));
                w.push(r.int_source0, 0);
                1_000
            }
            1 => {
                w.push(r.int_source1, INT_SRC1_WOM_XYZ);
                50_000
            }
            2 => {
                w.push(reg426::SMD_CONFIG, mode << 2 | SMD_MODE_WOM);
                0
            }
            _ => return None,
        }
    };
    Some((w, wait))
}

/// Register writes that disable WoM and route data-ready to INT1 again.
pub fn wom_off_writes(variant: Variant) -> RegWrites {
    let r = variant.regs();
    let mut w = RegWrites::new();
    if variant.is_4267() {
        w.push(reg4267::WOM_CONFIG, 0);
    } else {
        w.push(reg426::SMD_CONFIG, 0);
    }
    w.push(r.int_source1, 0);
    w.push(r.int_config, int_config(false));
    w.push(r.int_source0, INT_SRC0_DRDY);
    w
}

macro_rules! driver {
    ($doc:literal, $Iface:ident, $mode:ident, [$($async:tt)?], [$($aw:tt)*]) => {
        #[doc = $doc]
        #[derive(Debug)]
        pub struct Icm426xx<I> {
            iface: I,
            variant: Variant,
            config: Config,
            wom: Option<WakeOnMotion>,
            wom_step: u8,
        }

        impl<I> Icm426xx<I> {
            /// Create a driver over any register interface; no bus traffic.
            pub fn new(iface: I) -> Self {
                Self {
                    iface,
                    variant: Variant::default(),
                    config: Config::default(),
                    wom: None,
                    wom_step: 0,
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

        impl<I2C> Icm426xx<I2cInterface<I2C>> {
            /// Create an I2C driver; no bus traffic.
            pub fn new_i2c(i2c: I2C, address: u8) -> Self {
                Self::new(I2cInterface::new(i2c, address))
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.iface.release()
            }
        }

        impl<SPI> Icm426xx<SpiInterface<SPI>> {
            /// Create an SPI driver (mode 0 or 3); no bus traffic.
            pub fn new_spi(spi: SPI) -> Self {
                Self::new(SpiInterface::new(spi))
            }

            /// Give the device back.
            pub fn release(self) -> SPI {
                self.iface.release()
            }
        }

        impl<I: $Iface> Icm426xx<I> {
            /// Read one bank-0 register.
            pub $($async)? fn read_register(&mut self, reg: u8) -> Result<u8, Error<I::Error>> {
                let mut b = [0u8];
                self.iface.read_regs(reg, &mut b) $($aw)* .map_err(Error::bus)?;
                Ok(b[0])
            }

            /// Write one bank-0 register.
            pub $($async)? fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Error<I::Error>> {
                self.iface.write_reg(reg, value) $($aw)* .map_err(Error::bus)
            }

            $($async)? fn write_all(&mut self, writes: &RegWrites) -> Result<(), Error<I::Error>> {
                for &(r, v) in writes.as_slice() {
                    self.write_register(r, v) $($aw)* ?;
                }
                Ok(())
            }

            /// Read `WHO_AM_I` and select the [`Variant`].
            pub $($async)? fn probe(&mut self) -> Result<Variant, Error<I::Error>> {
                let id = self.read_register(WHO_AM_I) $($aw)* ?;
                self.variant = Variant::from_id(id).ok_or(Error::UnexpectedId(u32::from(id)))?;
                Ok(self.variant)
            }

            /// Soft reset; the part comes back in sleep mode with register
            /// bank 0 selected.
            pub $($async)? fn reset(&mut self) -> Result<WaitUs, Error<I::Error>> {
                let (reg, bit) = self.variant.regs().reset;
                self.write_register(reg, bit) $($aw)* ?;
                self.wom = None;
                self.wom_step = 0;
                Ok(RESET_WAIT_US)
            }

            /// Apply the stored configuration and start sampling. Returns
            /// the time until the first valid sample.
            pub $($async)? fn configure(&mut self) -> Result<WaitUs, Error<I::Error>> {
                let writes = config_writes(self.variant, &self.config, self.wom.is_some())
                    .ok_or(Error::Unsupported)?;
                self.write_all(&writes) $($aw)* ?;
                Ok(startup_wait_us(&self.config))
            }

            /// Store a new configuration and apply it.
            pub $($async)? fn set_config(&mut self, config: Config) -> Result<WaitUs, Error<I::Error>> {
                self.config = config;
                self.configure() $($aw)*
            }

            /// Sleep mode: both sensors and the RC oscillator off (7.5 µA
            /// ICM-42605/42688-P, 3.5 µA ICM-42670-P). Stops WoM too.
            pub $($async)? fn power_down(&mut self) -> Result<(), Error<I::Error>> {
                let r = self.variant.regs();
                self.write_register(r.pwr_mgmt0, pwr_mgmt0_off(self.variant)) $($aw)*
            }

            /// Start arming Wake-on-Motion on INT1 (all axes, OR-ed). The
            /// accelerometer must be running (call `configure()` first,
            /// ideally in [`AccelMode::LowPower`]). Returns the time to
            /// wait before [`Self::continue_wake_on_motion`]; 0 means armed.
            ///
            /// While WoM is enabled data-ready is not routed to INT1 and
            /// `read_measurement()` returns the latest sample without
            /// waiting for new data.
            pub $($async)? fn enable_wake_on_motion(&mut self, wom: WakeOnMotion) -> Result<WaitUs, Error<I::Error>> {
                self.wom = Some(wom);
                self.wom_step = 0;
                self.continue_wake_on_motion() $($aw)*
            }

            /// Next step of the WoM arming sequence; returns the time to
            /// wait before calling again, or 0 once armed.
            pub $($async)? fn continue_wake_on_motion(&mut self) -> Result<WaitUs, Error<I::Error>> {
                let Some(wom) = self.wom else {
                    return Ok(0);
                };
                if self.variant.is_4267() && self.wom_step == 1 {
                    let ready = self.read_register(reg4267::MCLK_RDY) $($aw)* ?;
                    if ready & MCLK_RDY == 0 {
                        return Ok(MCLK_POLL_US);
                    }
                }
                match wom_step(self.variant, &self.config, &wom, self.wom_step) {
                    Some((writes, wait)) => {
                        self.write_all(&writes) $($aw)* ?;
                        self.wom_step += 1;
                        Ok(wait)
                    }
                    None => Ok(0),
                }
            }

            /// Disable Wake-on-Motion and route data-ready to INT1 again.
            pub $($async)? fn disable_wake_on_motion(&mut self) -> Result<(), Error<I::Error>> {
                self.write_all(&wom_off_writes(self.variant)) $($aw)* ?;
                self.wom = None;
                self.wom_step = 0;
                Ok(())
            }

            /// Whether WoM fired since the last call (reads and thereby
            /// clears `INT_STATUS2`, releasing a latched INT1).
            pub $($async)? fn motion_detected(&mut self) -> Result<bool, Error<I::Error>> {
                let s = self.read_register(self.variant.regs().int_status2) $($aw)* ?;
                Ok(s & INT_STATUS2_WOM != 0)
            }
        }

        impl<I: $Iface> zigbee_sensor::$mode::Sensor for Icm426xx<I> {
            type BusError = I::Error;
            type Measurement = Measurement;

            /// Free-running: returns 0.
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I::Error>> {
                Ok(0)
            }

            /// [`Error::NotReady`] until the data-ready bit is set (not
            /// checked while WoM is enabled).
            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<I::Error>> {
                let r = self.variant.regs();
                if self.wom.is_none() {
                    let (reg, bit) = r.drdy_status;
                    if self.read_register(reg) $($aw)* ? & bit == 0 {
                        return Err(Error::NotReady);
                    }
                }
                let mut b = [0u8; 12];
                let len = if self.config.gyro.is_some() { 12 } else { 6 };
                self.iface.read_regs(r.accel_data, &mut b[..len]) $($aw)* .map_err(Error::bus)?;
                Ok(decode_block(&b, self.config.accel_range, self.config.gyro))
            }
        }

        impl<I: $Iface> zigbee_sensor::$mode::LowPower for Icm426xx<I> {
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
        "Async twin of [`super::Icm426xx`].",
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

    const A: u8 = ADDRESS_AP_AD0_LOW;

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
        // DS-000347 Table 1/2: 2048 LSB/g at ±16 g, 16384 LSB/g at ±2 g,
        // 16.4 LSB/dps at ±2000 dps, 262 LSB/dps at ±125 dps.
        assert_eq!(accel_mg(2_048, AccelRange::G16), 1_000);
        assert_eq!(accel_mg(-16_384, AccelRange::G2), -1_000);
        assert_eq!(gyro_mdps(164, GyroRange::Dps2000), 10_000);
        assert_eq!(gyro_mdps(-262, GyroRange::Dps125), -1_000);
        assert_eq!(wom_threshold_code(98 * 1000 / 256), 98);
        assert_eq!(wom_threshold_code(0), 1);
        assert_eq!(wom_threshold_code(5_000), 255);
    }

    #[test]
    fn probe_variants() {
        for (id, v) in [
            (0x42, Variant::Icm42605),
            (0x47, Variant::Icm42688P),
            (0x67, Variant::Icm42670P),
        ] {
            let mut i2c = Mock::new(&[rd(WHO_AM_I, &[id])]);
            let mut imu = Icm426xx::new_i2c(&mut i2c, A);
            assert_eq!(imu.probe().unwrap(), v);
            i2c.done();
        }
        let mut i2c = Mock::new(&[rd(WHO_AM_I, &[0x6F])]);
        let mut imu = Icm426xx::new_i2c(&mut i2c, A);
        assert_eq!(imu.probe(), Err(Error::UnexpectedId(0x6F)));
        i2c.done();
    }

    #[test]
    fn odr_codes_per_variant() {
        use Variant::*;
        assert_eq!(Odr::Hz25.code(Icm42688P, true), Some(10));
        assert_eq!(Odr::Hz25.code(Icm42670P, true), Some(11));
        assert_eq!(Odr::Hz1_5625.code(Icm42605, true), Some(14));
        assert_eq!(Odr::Hz1_5625.code(Icm42670P, true), Some(15));
        assert_eq!(Odr::Hz1_5625.code(Icm42688P, false), None);
        assert_eq!(Odr::Hz1000.code(Icm42688P, true), None);
        assert_eq!(Odr::Hz400.code(Icm42688P, false), None);
        assert_eq!(Odr::Hz1600.code(Icm42670P, false), Some(5));
        let gyro125 = Config {
            gyro: Some(GyroRange::Dps125),
            accel_mode: AccelMode::LowNoise,
            ..Config::default()
        };
        assert!(config_writes(Icm42688P, &gyro125, false).is_some());
        assert!(config_writes(Icm42670P, &gyro125, false).is_none());
        let lp_gyro = Config {
            odr: Odr::Hz6_25,
            gyro: Some(GyroRange::Dps250),
            ..Config::default()
        };
        assert!(config_writes(Icm42688P, &lp_gyro, false).is_none());
    }

    #[test]
    fn configure_default_icm42688() {
        let mut i2c = Mock::new(&[
            wr(reg426::INT_CONFIG1, 0x00),
            wr(0x14, 0x03),
            wr(0x50, 0x6A),
            wr(0x65, 0x08),
            wr(0x4E, 0x22),
        ]);
        let mut imu = Icm426xx::new_i2c(&mut i2c, A);
        assert_eq!(imu.configure().unwrap(), ACCEL_START_US);
        i2c.done();
    }

    #[test]
    fn wake_on_motion_icm42688_flow() {
        let wom = WakeOnMotion {
            threshold_mg: 383,
            ..WakeOnMotion::default()
        };
        let mut i2c = Mock::new(&[
            wr(0x76, 4),
            wr(0x4A, 98),
            wr(0x4B, 98),
            wr(0x4C, 98),
            wr(0x76, 0),
            wr(0x14, 0x07),
            wr(0x65, 0x00),
            wr(0x66, 0x07),
            wr(0x57, 0x05),
            // read_measurement skips the data-ready check while WoM is on.
            rd(0x1F, &[0x40, 0x00, 0, 0, 0, 0]),
            rd(0x37, &[0x02]),
            wr(0x57, 0x00),
            wr(0x66, 0x00),
            wr(0x14, 0x03),
            wr(0x65, 0x08),
        ]);
        let mut imu = Icm426xx::new_i2c(&mut i2c, A);
        assert_eq!(imu.enable_wake_on_motion(wom).unwrap(), 1_000);
        assert_eq!(imu.continue_wake_on_motion().unwrap(), 50_000);
        assert_eq!(imu.continue_wake_on_motion().unwrap(), 0);
        assert_eq!(imu.continue_wake_on_motion().unwrap(), 0);
        assert_eq!(imu.read_measurement().unwrap().accel.x, 1_000);
        assert!(imu.motion_detected().unwrap());
        imu.disable_wake_on_motion().unwrap();
        i2c.done();
    }

    #[test]
    fn wake_on_motion_icm42670_mreg() {
        let wom = WakeOnMotion {
            threshold_mg: 100,
            events: 2,
            latched: false,
            ..WakeOnMotion::default()
        };
        let mut ex = vec![
            wr(0x06, 0x03),
            wr(0x2B, 0x00),
            wr(0x1F, 0x12),
            rd(0x00, &[0x00]),
            rd(0x00, &[0x08]),
        ];
        for addr in [0x4B, 0x4C, 0x4D] {
            ex.extend([wr(0x79, 0), wr(0x7A, addr), wr(0x7B, 26)]);
        }
        ex.extend([
            wr(0x27, 0x0A),
            wr(0x2C, 0x07),
            wr(0x27, 0x0B),
            wr(0x1F, 0x02),
        ]);
        let mut i2c = Mock::new(&ex);
        let mut imu = Icm426xx::new_i2c(&mut i2c, A).with_variant(Variant::Icm42670P);
        assert_eq!(imu.enable_wake_on_motion(wom).unwrap(), MREG_WAIT_US);
        assert_eq!(imu.continue_wake_on_motion().unwrap(), MCLK_POLL_US);
        for _ in 0..3 {
            assert_eq!(imu.continue_wake_on_motion().unwrap(), MREG_WAIT_US);
        }
        assert_eq!(imu.continue_wake_on_motion().unwrap(), 0);
        i2c.done();
    }

    #[test]
    fn icm42670_not_ready_then_gyro_over_spi() {
        let cfg = Config {
            accel_mode: AccelMode::LowNoise,
            gyro: Some(GyroRange::Dps2000),
            odr: Odr::Hz100,
            ..Config::default()
        };
        let w = config_writes(Variant::Icm42670P, &cfg, false).unwrap();
        assert_eq!(
            w.as_slice(),
            &[
                (0x06, 0x03),
                (0x20, 0x09),
                (0x21, 0x69),
                (0x2B, 0x08),
                (0x1F, 0x0F)
            ]
        );
        let block = vec![0, 0, 0xC0, 0x00, 0, 0, 0x00, 0xA4, 0, 0, 0xFF, 0x5C];
        let mut spi = SpiMock::new(&[
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![0xB9]),
            SpiTx::read_vec(vec![0x00]),
            SpiTx::transaction_end(),
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![0xB9]),
            SpiTx::read_vec(vec![0x01]),
            SpiTx::transaction_end(),
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![0x8B]),
            SpiTx::read_vec(block),
            SpiTx::transaction_end(),
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![0x1F, 0x00]),
            SpiTx::transaction_end(),
        ]);
        let mut imu = Icm426xx::new_spi(&mut spi)
            .with_variant(Variant::Icm42670P)
            .with_config(cfg);
        assert_eq!(imu.start_measurement().unwrap(), 0);
        assert_eq!(imu.read_measurement(), Err(Error::NotReady));
        let m = imu.read_measurement().unwrap();
        assert_eq!(m.accel.y, -1_000);
        assert_eq!(
            m.gyro,
            Some(Vector3 {
                x: 10_000,
                y: 0,
                z: -10_000
            })
        );
        imu.sleep().unwrap();
        spi.done();
    }

    #[test]
    fn async_configure_and_read() {
        use zigbee_sensor::asynch::Sensor as _;

        let mut ex = vec![wr(0x11, 0x01)];
        ex.extend(writes(
            &config_writes(Variant::Icm42605, &Config::default(), false).unwrap(),
        ));
        ex.push(rd(0x2D, &[0x08]));
        ex.push(rd(0x1F, &[0, 0, 0, 0, 0x40, 0x00]));
        let mut i2c = Mock::new(&ex);
        let mut imu = asynch::Icm426xx::new_i2c(&mut i2c, A).with_variant(Variant::Icm42605);
        assert_eq!(block_on(imu.reset()).unwrap(), RESET_WAIT_US);
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
