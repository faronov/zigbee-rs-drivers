//! ST LSM6DS3 / LSM6DS33 / LSM6DSL / LSM6DS3TR-C / LSM6DSO / LSM6DSOX
//! 6-axis IMU (accelerometer + gyroscope) over I2C or SPI.
//!
//! The parts share the register subset used here (CTRL1_XL..CTRL7_G,
//! STATUS_REG, output registers, wake-up engine); [`Variant`] covers the
//! few differences (interrupt-enable bit, 1.6 Hz ODR). LSM6DSO and LSM6DSOX
//! (and LSM6DSL / LSM6DS3TR-C) return the same `WHO_AM_I` and are treated
//! identically.
//!
//! The device is free-running: [`Lsm6dso::configure`] starts the
//! accelerometer (and the gyroscope if enabled), `start_measurement()`
//! returns 0 and `read_measurement()` returns [`Error::NotReady`] until
//! STATUS_REG reports a new accelerometer sample. The gyroscope is off by
//! default (it draws ~0.5 mA); the accelerometer defaults to 26 Hz
//! low-power mode (~10 µA class), suitable for vibration / orientation on a
//! sleepy end device. [`Lsm6dso::enable_wake_on_motion`] routes the
//! wake-up (slope threshold) interrupt to INT1 or INT2 so the MCU can sleep
//! until something moves.
//!
//! References:
//! * ST DocID026899 "LSM6DS3 datasheet", Rev 4 (2015);
//! * ST DocID027423 "LSM6DS33 datasheet", Rev 4 (2015);
//! * ST DS11884 "LSM6DSL datasheet", Rev 3; DS12140 "LSM6DSO datasheet",
//!   Rev 2; DS12814 "LSM6DSOX datasheet", Rev 3 (register maps
//!   cross-checked against ST's register headers).
//!
//! ```ignore
//! let mut imu = Lsm6dso::new_i2c(i2c, ADDRESS_SA0_HIGH);
//! imu.probe()?;
//! let wait = imu.configure()?;
//! // ... wait, then periodically:
//! let m = imu.read_measurement()?; // NotReady until a new sample exists
//! ```

#![no_std]

mod interface;

pub use interface::{AsyncInterface, I2cInterface, Interface, SpiInterface};
use zigbee_sensor::{Error, Quantity, Readings, Vector3, WaitUs};

/// I2C address with SA0/SDO low.
pub const ADDRESS_SA0_LOW: u8 = 0x6A;
/// I2C address with SA0/SDO high.
pub const ADDRESS_SA0_HIGH: u8 = 0x6B;

/// Register addresses.
pub mod reg {
    #![allow(missing_docs)]
    pub const WHO_AM_I: u8 = 0x0F;
    pub const CTRL1_XL: u8 = 0x10;
    pub const CTRL2_G: u8 = 0x11;
    pub const CTRL3_C: u8 = 0x12;
    pub const CTRL6_C: u8 = 0x15;
    pub const CTRL7_G: u8 = 0x16;
    pub const WAKE_UP_SRC: u8 = 0x1B;
    pub const STATUS_REG: u8 = 0x1E;
    pub const OUTX_L_G: u8 = 0x22;
    pub const OUTX_L_XL: u8 = 0x28;
    /// LSM6DSO/DSOX only (latch bit).
    pub const TAP_CFG0: u8 = 0x56;
    /// TAP_CFG on LSM6DS3/LSM6DSL, TAP_CFG2 on LSM6DSO/DSOX.
    pub const TAP_CFG: u8 = 0x58;
    pub const WAKE_UP_THS: u8 = 0x5B;
    pub const WAKE_UP_DUR: u8 = 0x5C;
    pub const MD1_CFG: u8 = 0x5E;
    pub const MD2_CFG: u8 = 0x5F;
}

const CTRL3_BOOT_SW_RESET: u8 = 0x01;
const CTRL3_BDU_IF_INC: u8 = 0x44;
const CTRL6_XL_HM_MODE: u8 = 0x10;
const CTRL7_G_HM_MODE: u8 = 0x80;
const STATUS_XLDA: u8 = 0x01;
const MD_INT_WU: u8 = 0x20;
const TAP_CFG_INTERRUPTS_ENABLE: u8 = 0x80;
const TAP_CFG_LIR: u8 = 0x01;

/// Wait after a software reset. The datasheets give no figure; this covers
/// the 10 ms memory boot that follows.
pub const RESET_WAIT_US: WaitUs = 10_000;
/// Conservative gyroscope turn-on time from power-down.
pub const GYRO_TURN_ON_US: WaitUs = 70_000;

/// Part family, selected from `WHO_AM_I`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Variant {
    /// LSM6DS3 / LSM6DS33 (`WHO_AM_I` = 0x69).
    Ds3,
    /// LSM6DSL / LSM6DS3TR-C (`WHO_AM_I` = 0x6A).
    Dsl,
    /// LSM6DSO / LSM6DSOX (`WHO_AM_I` = 0x6C).
    #[default]
    Dso,
}

impl Variant {
    /// Map a `WHO_AM_I` value.
    pub const fn from_id(id: u8) -> Option<Self> {
        match id {
            0x69 => Some(Self::Ds3),
            0x6A => Some(Self::Dsl),
            0x6C => Some(Self::Dso),
            _ => None,
        }
    }
}

/// Output data rate, shared by accelerometer and gyroscope.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Odr {
    /// 1.6 Hz, accelerometer low-power mode only, not on LSM6DS3.
    Hz1_6,
    /// 12.5 Hz.
    Hz12_5,
    /// 26 Hz.
    Hz26,
    /// 52 Hz.
    Hz52,
    /// 104 Hz.
    Hz104,
    /// 208 Hz.
    Hz208,
    /// 416 Hz.
    Hz416,
    /// 833 Hz.
    Hz833,
    /// 1.66 kHz.
    Hz1660,
}

impl Odr {
    /// `ODR[3:0]` field value.
    pub const fn bits(self) -> u8 {
        match self {
            Self::Hz1_6 => 0b1011,
            Self::Hz12_5 => 1,
            Self::Hz26 => 2,
            Self::Hz52 => 3,
            Self::Hz104 => 4,
            Self::Hz208 => 5,
            Self::Hz416 => 6,
            Self::Hz833 => 7,
            Self::Hz1660 => 8,
        }
    }

    /// Sample period in µs (rounded up).
    pub const fn period_us(self) -> u32 {
        match self {
            Self::Hz1_6 => 625_000,
            Self::Hz12_5 => 80_000,
            Self::Hz26 => 38_462,
            Self::Hz52 => 19_231,
            Self::Hz104 => 9_616,
            Self::Hz208 => 4_808,
            Self::Hz416 => 2_404,
            Self::Hz833 => 1_201,
            Self::Hz1660 => 603,
        }
    }
}

/// Accelerometer full scale.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AccelRange {
    /// ±2 g, 0.061 mg/LSB.
    #[default]
    G2,
    /// ±4 g, 0.122 mg/LSB.
    G4,
    /// ±8 g, 0.244 mg/LSB.
    G8,
    /// ±16 g, 0.488 mg/LSB.
    G16,
}

impl AccelRange {
    /// `FS_XL[1:0]` field value.
    pub const fn bits(self) -> u8 {
        match self {
            Self::G2 => 0b00,
            Self::G16 => 0b01,
            Self::G4 => 0b10,
            Self::G8 => 0b11,
        }
    }

    /// Full scale in mg.
    pub const fn full_scale_mg(self) -> u32 {
        match self {
            Self::G2 => 2_000,
            Self::G4 => 4_000,
            Self::G8 => 8_000,
            Self::G16 => 16_000,
        }
    }

    /// Datasheet sensitivity in µg/LSB.
    pub const fn ug_per_lsb(self) -> i32 {
        match self {
            Self::G2 => 61,
            Self::G4 => 122,
            Self::G8 => 244,
            Self::G16 => 488,
        }
    }
}

/// Gyroscope full scale.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GyroRange {
    /// ±125 dps, 4.375 mdps/LSB.
    Dps125,
    /// ±250 dps, 8.75 mdps/LSB.
    Dps250,
    /// ±500 dps, 17.5 mdps/LSB.
    Dps500,
    /// ±1000 dps, 35 mdps/LSB.
    Dps1000,
    /// ±2000 dps, 70 mdps/LSB.
    #[default]
    Dps2000,
}

impl GyroRange {
    /// `FS_G[1:0]` and `FS_125` bits of `CTRL2_G`.
    pub const fn bits(self) -> u8 {
        match self {
            Self::Dps125 => 0b0010,
            Self::Dps250 => 0b0000,
            Self::Dps500 => 0b0100,
            Self::Dps1000 => 0b1000,
            Self::Dps2000 => 0b1100,
        }
    }

    /// Datasheet sensitivity in units of 1/8 mdps per LSB.
    const fn eighth_mdps_per_lsb(self) -> i32 {
        match self {
            Self::Dps125 => 35,
            Self::Dps250 => 70,
            Self::Dps500 => 140,
            Self::Dps1000 => 280,
            Self::Dps2000 => 560,
        }
    }
}

/// Accelerometer / gyroscope power mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PowerMode {
    /// High-performance mode (lowest noise, highest current).
    HighPerformance,
    /// Low-power mode at ODR ≤ 52 Hz, normal mode at 104/208 Hz
    /// (`XL_HM_MODE` / `G_HM_MODE` = 1). Above 208 Hz the part always runs
    /// in high-performance mode.
    #[default]
    LowPower,
}

/// Gyroscope settings; the gyroscope is off when the config holds `None`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GyroConfig {
    /// Full scale.
    pub range: GyroRange,
    /// Output data rate.
    pub odr: Odr,
    /// Power mode.
    pub mode: PowerMode,
}

/// Device configuration, applied by [`Lsm6dso::configure`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Accelerometer full scale.
    pub accel_range: AccelRange,
    /// Accelerometer output data rate.
    pub accel_odr: Odr,
    /// Accelerometer power mode.
    pub accel_mode: PowerMode,
    /// Gyroscope, off by default.
    pub gyro: Option<GyroConfig>,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            accel_range: AccelRange::G2,
            accel_odr: Odr::Hz26,
            accel_mode: PowerMode::LowPower,
            gyro: None,
        }
    }
}

/// Interrupt output pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum IntPin {
    /// INT1.
    #[default]
    Int1,
    /// INT2.
    Int2,
}

/// Wake-up (any-motion) interrupt settings.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WakeOnMotion {
    /// High-pass (slope) threshold in mg; resolution is full scale / 64
    /// (31.25 mg at ±2 g). Clamped to 1..=63 LSB.
    pub threshold_mg: u32,
    /// Samples above threshold before the event fires (0..=3 ODR periods).
    pub duration: u8,
    /// Output pin.
    pub pin: IntPin,
    /// Latch the interrupt until [`Lsm6dso::read_wake_up_source`] is called.
    pub latched: bool,
}

impl Default for WakeOnMotion {
    fn default() -> Self {
        Self {
            threshold_mg: 63,
            duration: 0,
            pin: IntPin::Int1,
            latched: true,
        }
    }
}

/// WAKE_UP_SRC contents.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WakeUpSource(pub u8);

impl WakeUpSource {
    /// A wake-up event was detected.
    pub const fn wake_up(self) -> bool {
        self.0 & 0x08 != 0
    }
    /// Wake-up on the X axis.
    pub const fn x(self) -> bool {
        self.0 & 0x04 != 0
    }
    /// Wake-up on the Y axis.
    pub const fn y(self) -> bool {
        self.0 & 0x02 != 0
    }
    /// Wake-up on the Z axis.
    pub const fn z(self) -> bool {
        self.0 & 0x01 != 0
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

/// Up to six register writes computed without bus traffic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RegWrites {
    buf: [(u8, u8); 6],
    len: usize,
}

impl RegWrites {
    const fn new() -> Self {
        Self {
            buf: [(0, 0); 6],
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

/// Raw accelerometer count to mg using the datasheet sensitivity.
pub fn accel_mg(raw: i16, range: AccelRange) -> i32 {
    div_round(i32::from(raw) * range.ug_per_lsb(), 1_000)
}

/// Raw gyroscope count to mdps using the datasheet sensitivity.
pub fn gyro_mdps(raw: i16, range: GyroRange) -> i32 {
    div_round(i32::from(raw) * range.eighth_mdps_per_lsb(), 8)
}

/// Decode three little-endian i16 words (X, Y, Z).
pub fn decode_axes(bytes: &[u8]) -> [i16; 3] {
    [
        i16::from_le_bytes([bytes[0], bytes[1]]),
        i16::from_le_bytes([bytes[2], bytes[3]]),
        i16::from_le_bytes([bytes[4], bytes[5]]),
    ]
}

fn scale<R: Copy>(raw: [i16; 3], range: R, f: fn(i16, R) -> i32) -> Vector3 {
    Vector3 {
        x: f(raw[0], range),
        y: f(raw[1], range),
        z: f(raw[2], range),
    }
}

/// Decode the 12-byte gyro+accel block starting at OUTX_L_G.
pub fn decode_gyro_accel(bytes: &[u8; 12], accel: AccelRange, gyro: GyroRange) -> Measurement {
    Measurement {
        gyro: Some(scale(decode_axes(&bytes[..6]), gyro, gyro_mdps)),
        accel: scale(decode_axes(&bytes[6..]), accel, accel_mg),
    }
}

/// Register writes that apply `config` (BDU + auto-increment, power modes,
/// ranges, ODRs); `None` if the variant cannot run that configuration.
pub fn config_writes(variant: Variant, config: &Config) -> Option<RegWrites> {
    let hm = |mode: PowerMode, bit: u8| {
        if mode == PowerMode::LowPower { bit } else { 0 }
    };
    if config.accel_odr == Odr::Hz1_6
        && (variant == Variant::Ds3 || config.accel_mode != PowerMode::LowPower)
    {
        return None;
    }
    let (ctrl2, ctrl7) = match config.gyro {
        Some(g) if g.odr == Odr::Hz1_6 => return None,
        Some(g) => (
            g.odr.bits() << 4 | g.range.bits(),
            hm(g.mode, CTRL7_G_HM_MODE),
        ),
        None => (0, 0),
    };
    let mut w = RegWrites::new();
    w.push(reg::CTRL3_C, CTRL3_BDU_IF_INC);
    w.push(reg::CTRL6_C, hm(config.accel_mode, CTRL6_XL_HM_MODE));
    w.push(reg::CTRL7_G, ctrl7);
    w.push(
        reg::CTRL1_XL,
        config.accel_odr.bits() << 4 | config.accel_range.bits() << 2,
    );
    w.push(reg::CTRL2_G, ctrl2);
    Some(w)
}

/// Time until the first valid sample after [`config_writes`]: two
/// accelerometer periods, or the gyroscope turn-on time.
pub fn startup_wait_us(config: &Config) -> WaitUs {
    let accel = 2 * config.accel_odr.period_us();
    match config.gyro {
        Some(_) if accel < GYRO_TURN_ON_US => GYRO_TURN_ON_US,
        _ => accel,
    }
}

/// WK_THS code for a threshold in mg (1 LSB = full scale / 64).
pub fn wake_threshold_code(threshold_mg: u32, range: AccelRange) -> u8 {
    let lsb = range.full_scale_mg() / 64;
    (threshold_mg.saturating_add(lsb / 2) / lsb).clamp(1, 63) as u8
}

/// Register writes that enable the wake-up interrupt.
pub fn wake_on_motion_writes(variant: Variant, range: AccelRange, wom: &WakeOnMotion) -> RegWrites {
    let lir = if wom.latched { TAP_CFG_LIR } else { 0 };
    let mut w = RegWrites::new();
    w.push(reg::WAKE_UP_DUR, (wom.duration.min(3)) << 5);
    w.push(
        reg::WAKE_UP_THS,
        wake_threshold_code(wom.threshold_mg, range),
    );
    match variant {
        Variant::Ds3 => w.push(reg::TAP_CFG, lir),
        Variant::Dsl => w.push(reg::TAP_CFG, TAP_CFG_INTERRUPTS_ENABLE | lir),
        Variant::Dso => {
            w.push(reg::TAP_CFG0, lir);
            w.push(reg::TAP_CFG, TAP_CFG_INTERRUPTS_ENABLE);
        }
    }
    let (on, off) = match wom.pin {
        IntPin::Int1 => (reg::MD1_CFG, reg::MD2_CFG),
        IntPin::Int2 => (reg::MD2_CFG, reg::MD1_CFG),
    };
    w.push(off, 0);
    w.push(on, MD_INT_WU);
    w
}

/// Register writes that disable the wake-up interrupt.
pub fn wake_on_motion_off_writes() -> RegWrites {
    let mut w = RegWrites::new();
    w.push(reg::MD1_CFG, 0);
    w.push(reg::MD2_CFG, 0);
    w.push(reg::TAP_CFG, 0);
    w.push(reg::WAKE_UP_THS, 0);
    w
}

macro_rules! driver {
    ($doc:literal, $Iface:ident, $mode:ident, [$($async:tt)?], [$($aw:tt)*]) => {
        #[doc = $doc]
        #[derive(Debug)]
        pub struct Lsm6dso<I> {
            iface: I,
            variant: Variant,
            config: Config,
        }

        impl<I> Lsm6dso<I> {
            /// Create a driver over any register interface; no bus traffic.
            pub fn new(iface: I) -> Self {
                Self {
                    iface,
                    variant: Variant::Dso,
                    config: Config::default(),
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

        impl<I2C> Lsm6dso<I2cInterface<I2C>> {
            /// Create an I2C driver; no bus traffic.
            pub fn new_i2c(i2c: I2C, address: u8) -> Self {
                Self::new(I2cInterface::new(i2c, address))
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.iface.release()
            }
        }

        impl<SPI> Lsm6dso<SpiInterface<SPI>> {
            /// Create an SPI driver; no bus traffic.
            pub fn new_spi(spi: SPI) -> Self {
                Self::new(SpiInterface::new(spi))
            }

            /// Give the device back.
            pub fn release(self) -> SPI {
                self.iface.release()
            }
        }

        impl<I: $Iface> Lsm6dso<I> {
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

            /// Read `WHO_AM_I` and select the [`Variant`].
            pub $($async)? fn probe(&mut self) -> Result<Variant, Error<I::Error>> {
                let id = self.read_register(reg::WHO_AM_I) $($aw)* ?;
                self.variant = Variant::from_id(id).ok_or(Error::UnexpectedId(u32::from(id)))?;
                Ok(self.variant)
            }

            /// Software reset; registers return to defaults (power-down).
            pub $($async)? fn reset(&mut self) -> Result<WaitUs, Error<I::Error>> {
                self.write_register(reg::CTRL3_C, CTRL3_BOOT_SW_RESET) $($aw)* ?;
                Ok(RESET_WAIT_US)
            }

            /// Apply the stored configuration and start sampling. Returns the
            /// time until the first valid sample.
            pub $($async)? fn configure(&mut self) -> Result<WaitUs, Error<I::Error>> {
                let writes = config_writes(self.variant, &self.config).ok_or(Error::Unsupported)?;
                self.write_all(&writes) $($aw)* ?;
                Ok(startup_wait_us(&self.config))
            }

            /// Store a new configuration and apply it.
            pub $($async)? fn set_config(&mut self, config: Config) -> Result<WaitUs, Error<I::Error>> {
                self.config = config;
                self.configure() $($aw)*
            }

            /// Power down accelerometer and gyroscope (~3 µA). Motion
            /// detection stops too.
            pub $($async)? fn power_down(&mut self) -> Result<(), Error<I::Error>> {
                self.write_register(reg::CTRL1_XL, 0) $($aw)* ?;
                self.write_register(reg::CTRL2_G, 0) $($aw)*
            }

            /// Route the wake-up (any-motion) interrupt to a pin. The
            /// accelerometer must be running; a low-power ODR such as 26 Hz
            /// keeps the current in the µA range.
            pub $($async)? fn enable_wake_on_motion(&mut self, wom: WakeOnMotion) -> Result<(), Error<I::Error>> {
                let writes = wake_on_motion_writes(self.variant, self.config.accel_range, &wom);
                self.write_all(&writes) $($aw)*
            }

            /// Disable the wake-up interrupt.
            pub $($async)? fn disable_wake_on_motion(&mut self) -> Result<(), Error<I::Error>> {
                self.write_all(&wake_on_motion_off_writes()) $($aw)*
            }

            /// Read (and, when latched, clear) the wake-up source.
            pub $($async)? fn read_wake_up_source(&mut self) -> Result<WakeUpSource, Error<I::Error>> {
                Ok(WakeUpSource(self.read_register(reg::WAKE_UP_SRC) $($aw)* ?))
            }
        }

        impl<I: $Iface> zigbee_sensor::$mode::Sensor for Lsm6dso<I> {
            type BusError = I::Error;
            type Measurement = Measurement;

            /// Free-running: returns 0.
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I::Error>> {
                Ok(0)
            }

            /// [`Error::NotReady`] until a new accelerometer sample exists.
            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<I::Error>> {
                let status = self.read_register(reg::STATUS_REG) $($aw)* ?;
                if status & STATUS_XLDA == 0 {
                    return Err(Error::NotReady);
                }
                let accel_range = self.config.accel_range;
                match self.config.gyro {
                    Some(g) => {
                        let mut b = [0u8; 12];
                        self.iface.read_regs(reg::OUTX_L_G, &mut b) $($aw)* .map_err(Error::bus)?;
                        Ok(decode_gyro_accel(&b, accel_range, g.range))
                    }
                    None => {
                        let mut b = [0u8; 6];
                        self.iface.read_regs(reg::OUTX_L_XL, &mut b) $($aw)* .map_err(Error::bus)?;
                        Ok(Measurement {
                            accel: scale(decode_axes(&b), accel_range, accel_mg),
                            gyro: None,
                        })
                    }
                }
            }
        }

        impl<I: $Iface> zigbee_sensor::$mode::LowPower for Lsm6dso<I> {
            type BusError = I::Error;

            /// Power-down mode.
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
        "Async twin of [`super::Lsm6dso`].",
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
    use zigbee_sensor::blocking::{LowPower, Sensor};

    const A: u8 = ADDRESS_SA0_HIGH;

    fn rd(reg: u8, data: &[u8]) -> I2cTx {
        I2cTx::write_read(A, vec![reg], data.to_vec())
    }
    fn wr(reg: u8, value: u8) -> I2cTx {
        I2cTx::write(A, vec![reg, value])
    }

    #[test]
    fn datasheet_sensitivities() {
        // Table 3 (mechanical characteristics): 0.061 mg/LSB at ±2 g,
        // 0.488 mg/LSB at ±16 g, 4.375 mdps/LSB at ±125 dps, 70 mdps/LSB at ±2000 dps.
        assert_eq!(accel_mg(16_393, AccelRange::G2), 1_000);
        assert_eq!(accel_mg(-16_393, AccelRange::G2), -1_000);
        assert_eq!(accel_mg(2_049, AccelRange::G16), 1_000);
        assert_eq!(gyro_mdps(1_000, GyroRange::Dps2000), 70_000);
        assert_eq!(gyro_mdps(-8, GyroRange::Dps125), -35);
        assert_eq!(gyro_mdps(i16::MAX, GyroRange::Dps2000), 2_293_690);
    }

    #[test]
    fn threshold_code() {
        assert_eq!(wake_threshold_code(63, AccelRange::G2), 2);
        assert_eq!(wake_threshold_code(0, AccelRange::G2), 1);
        assert_eq!(wake_threshold_code(10_000, AccelRange::G2), 63);
        assert_eq!(wake_threshold_code(500, AccelRange::G16), 2);
    }

    #[test]
    fn probe_identifies_variants() {
        for (id, v) in [
            (0x69, Variant::Ds3),
            (0x6A, Variant::Dsl),
            (0x6C, Variant::Dso),
        ] {
            let mut i2c = Mock::new(&[rd(reg::WHO_AM_I, &[id])]);
            let mut imu = Lsm6dso::new_i2c(&mut i2c, A);
            assert_eq!(imu.probe().unwrap(), v);
            i2c.done();
        }
        let mut i2c = Mock::new(&[rd(reg::WHO_AM_I, &[0x6B])]);
        let mut imu = Lsm6dso::new_i2c(&mut i2c, A);
        assert_eq!(imu.probe(), Err(Error::UnexpectedId(0x6B)));
        i2c.done();
    }

    #[test]
    fn configure_accel_only_low_power() {
        let mut i2c = Mock::new(&[
            wr(reg::CTRL3_C, 0x44),
            wr(reg::CTRL6_C, 0x10),
            wr(reg::CTRL7_G, 0x00),
            wr(reg::CTRL1_XL, 0x20),
            wr(reg::CTRL2_G, 0x00),
        ]);
        let mut imu = Lsm6dso::new_i2c(&mut i2c, A);
        assert_eq!(imu.configure().unwrap(), 2 * 38_462);
        i2c.done();
    }

    #[test]
    fn configure_with_gyro_and_unsupported_odr() {
        let config = Config {
            accel_range: AccelRange::G8,
            accel_odr: Odr::Hz104,
            accel_mode: PowerMode::HighPerformance,
            gyro: Some(GyroConfig {
                range: GyroRange::Dps500,
                odr: Odr::Hz52,
                mode: PowerMode::LowPower,
            }),
        };
        let mut i2c = Mock::new(&[
            wr(reg::CTRL3_C, 0x44),
            wr(reg::CTRL6_C, 0x00),
            wr(reg::CTRL7_G, 0x80),
            wr(reg::CTRL1_XL, 0x4C),
            wr(reg::CTRL2_G, 0x34),
        ]);
        let mut imu = Lsm6dso::new_i2c(&mut i2c, A).with_config(config);
        assert_eq!(imu.configure().unwrap(), GYRO_TURN_ON_US);
        i2c.done();

        let slow = Config {
            accel_odr: Odr::Hz1_6,
            ..Config::default()
        };
        let mut i2c = Mock::new(&[]);
        let mut imu = Lsm6dso::new_i2c(&mut i2c, A)
            .with_variant(Variant::Ds3)
            .with_config(slow);
        assert_eq!(imu.configure(), Err(Error::Unsupported));
        i2c.done();
    }

    #[test]
    fn read_accel_only_and_not_ready() {
        let mut i2c = Mock::new(&[
            rd(reg::STATUS_REG, &[0x00]),
            rd(reg::STATUS_REG, &[0x05]),
            rd(reg::OUTX_L_XL, &[0x09, 0x40, 0xF7, 0xBF, 0, 0]),
        ]);
        let mut imu = Lsm6dso::new_i2c(&mut i2c, A);
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
    fn read_gyro_and_accel_over_spi() {
        let config = Config {
            gyro: Some(GyroConfig {
                range: GyroRange::Dps2000,
                odr: Odr::Hz104,
                mode: PowerMode::HighPerformance,
            }),
            ..Config::default()
        };
        let mut data = vec![0xE8, 0x03, 0x18, 0xFC, 0, 0];
        data.extend_from_slice(&[0, 0, 0, 0, 0x09, 0x40]);
        let mut spi = SpiMock::new(&[
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![0x9E]),
            SpiTx::read_vec(vec![0x03]),
            SpiTx::transaction_end(),
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![0xA2]),
            SpiTx::read_vec(data),
            SpiTx::transaction_end(),
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![0x10, 0x00]),
            SpiTx::transaction_end(),
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![0x11, 0x00]),
            SpiTx::transaction_end(),
        ]);
        let mut imu = Lsm6dso::new_spi(&mut spi).with_config(config);
        let m = imu.read_measurement().unwrap();
        assert_eq!(
            m.gyro,
            Some(Vector3 {
                x: 70_000,
                y: -70_000,
                z: 0
            })
        );
        assert_eq!(
            m.accel,
            Vector3 {
                x: 0,
                y: 0,
                z: 1_000
            }
        );
        imu.sleep().unwrap();
        spi.done();
    }

    #[test]
    fn wake_on_motion_per_variant() {
        let wom = WakeOnMotion {
            threshold_mg: 125,
            duration: 1,
            pin: IntPin::Int1,
            latched: true,
        };
        let mut i2c = Mock::new(&[
            wr(reg::WAKE_UP_DUR, 0x20),
            wr(reg::WAKE_UP_THS, 4),
            wr(reg::TAP_CFG0, 0x01),
            wr(reg::TAP_CFG, 0x80),
            wr(reg::MD2_CFG, 0),
            wr(reg::MD1_CFG, 0x20),
            rd(reg::WAKE_UP_SRC, &[0x0C]),
        ]);
        let mut imu = Lsm6dso::new_i2c(&mut i2c, A);
        imu.enable_wake_on_motion(wom).unwrap();
        let src = imu.read_wake_up_source().unwrap();
        assert!(src.wake_up() && src.x() && !src.y() && !src.z());
        i2c.done();

        let dsl = wake_on_motion_writes(Variant::Dsl, AccelRange::G2, &wom);
        assert_eq!(dsl.as_slice()[2], (reg::TAP_CFG, 0x81));
        let ds3 = wake_on_motion_writes(
            Variant::Ds3,
            AccelRange::G2,
            &WakeOnMotion {
                pin: IntPin::Int2,
                ..wom
            },
        );
        assert_eq!(
            ds3.as_slice(),
            &[
                (reg::WAKE_UP_DUR, 0x20),
                (reg::WAKE_UP_THS, 4),
                (reg::TAP_CFG, 0x01),
                (reg::MD1_CFG, 0),
                (reg::MD2_CFG, 0x20),
            ]
        );
    }

    #[test]
    fn async_read_and_reset() {
        use zigbee_sensor::asynch::Sensor as _;

        let mut i2c = Mock::new(&[
            wr(reg::CTRL3_C, 0x01),
            rd(reg::STATUS_REG, &[0x01]),
            rd(reg::OUTX_L_XL, &[0, 0, 0, 0, 0x09, 0x40]),
        ]);
        let mut imu = asynch::Lsm6dso::new_i2c(&mut i2c, A);
        assert_eq!(block_on(imu.reset()).unwrap(), RESET_WAIT_US);
        let m = block_on(imu.read_measurement()).unwrap();
        assert_eq!(m.accel.z, 1_000);
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
