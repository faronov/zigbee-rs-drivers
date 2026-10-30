//! QST QMI8658A / QMI8658C accelerometer and gyroscope over I2C or SPI.
//!
//! Both parts share `WHO_AM_I` (0x05) and the data registers but differ in
//! the `CTRL9` command handshake, the interrupt pin enables and timing, so
//! the [`Variant`] is chosen by the caller (default [`Variant::A`]):
//!
//! | | QMI8658A | QMI8658C |
//! |---|---|---|
//! | `CTRL9` done flag | `STATUSINT` bit 7, cleared by writing `CTRL_CMD_ACK` | `STATUS1` bit 0, cleared on read |
//! | INT pin enables | `CTRL1` bits 3/4 (pins are high-Z by default) | always driven |
//! | System turn-on (reset, power-down exit) | 15 ms | 1.75 s |
//! | Gyroscope wake-up | 150 ms + 3/ODR | 60 ms + 3/ODR |
//!
//! The first [`Qmi8658::configure`] soft-resets the part and returns the
//! system turn-on time; later calls only rewrite the registers. When the
//! wake-on-motion setting changes, a `CTRL9` command is executed and
//! [`Qmi8658::continue_configure`] polls its completion every
//! [`CMD_POLL_US`]. `start_measurement()` performs the next pending step;
//! `read_measurement()` returns [`Error::NotReady`] until configuration has
//! finished and `STATUS0` reports new accelerometer data.
//!
//! References: QST "QMI8658A Datasheet" Rev A (13-52-25, 2022) and
//! "QMI8658C Datasheet" Rev 0.6 (January 2021).
//!
//! ```ignore
//! let mut imu = Qmi8658::new_i2c(i2c, ADDRESS_SA0_HIGH);
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

/// I2C address with SA0 high or unconnected (internal pull-up).
pub const ADDRESS_SA0_HIGH: u8 = 0x6A;
/// I2C address with SA0 pulled low.
pub const ADDRESS_SA0_LOW: u8 = 0x6B;

/// `WHO_AM_I` value (both variants).
pub const WHO_AM_I: u8 = 0x05;

/// Register addresses.
pub mod reg {
    #![allow(missing_docs)]
    pub const WHO_AM_I: u8 = 0x00;
    pub const REVISION_ID: u8 = 0x01;
    pub const CTRL1: u8 = 0x02;
    pub const CTRL2: u8 = 0x03;
    pub const CTRL3: u8 = 0x04;
    pub const CTRL5: u8 = 0x06;
    pub const CTRL7: u8 = 0x08;
    pub const CTRL8: u8 = 0x09;
    pub const CTRL9: u8 = 0x0A;
    pub const CAL1_L: u8 = 0x0B;
    pub const CAL1_H: u8 = 0x0C;
    pub const STATUSINT: u8 = 0x2D;
    pub const STATUS0: u8 = 0x2E;
    pub const STATUS1: u8 = 0x2F;
    pub const AX_L: u8 = 0x35;
    pub const GX_L: u8 = 0x3B;
    pub const RESET: u8 = 0x60;
}

/// `CTRL9` command values.
pub mod cmd {
    #![allow(missing_docs)]
    /// QMI8658A: acknowledge a finished command.
    pub const ACK: u8 = 0x00;
    pub const WRITE_WOM_SETTING: u8 = 0x08;
}

const RESET_VALUE: u8 = 0xB0;
/// `ADDR_AI` = 1 (burst reads), `BE` = 0 (little-endian), 4-wire SPI.
const CTRL1_BASE: u8 = 0x40;
const CTRL1_INT1_EN: u8 = 0x08;
const CTRL1_INT2_EN: u8 = 0x10;
const CTRL1_SENSOR_DISABLE: u8 = 0x01;
/// QMI8658A: `CTRL9` handshake via `STATUSINT` bit 7 instead of INT1.
const CTRL8_HANDSHAKE_STATUSINT: u8 = 0x80;
const CTRL7_AEN: u8 = 0x01;
const CTRL7_GEN: u8 = 0x02;
const STATUSINT_CMD_DONE: u8 = 0x80;
const STATUS1_CMD_DONE: u8 = 0x01;
const STATUS1_WOM: u8 = 0x04;
const STATUS0_ADA: u8 = 0x01;

/// Interval between `CTRL9` completion polls.
pub const CMD_POLL_US: WaitUs = 1_000;
/// Polls before a `CTRL9` command is reported as [`Error::Timeout`].
pub const CMD_MAX_POLLS: u8 = 50;
/// Accelerometer wake-up time t2 (both variants).
pub const ACCEL_WAKE_US: WaitUs = 3_000;

/// Part variant.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Variant {
    /// QMI8658A (`REVISION_ID` 0x7C).
    #[default]
    A,
    /// QMI8658C.
    C,
}

impl Variant {
    /// System turn-on time after soft reset or power-down.
    pub const fn turn_on_us(self) -> WaitUs {
        match self {
            Self::A => 15_000,
            Self::C => 1_750_000,
        }
    }

    /// Gyroscope wake-up time t1.
    pub const fn gyro_wake_us(self) -> WaitUs {
        match self {
            Self::A => 150_000,
            Self::C => 60_000,
        }
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
    /// Full scale in mg.
    pub const fn full_scale_mg(self) -> i32 {
        2_000 << (self as u8)
    }
}

/// Gyroscope full scale (`gFS`); sensitivity is 2048 LSB/dps at ±16 dps
/// and halves with each step.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GyroRange {
    /// ±16 dps.
    Dps16,
    /// ±32 dps.
    Dps32,
    /// ±64 dps.
    Dps64,
    /// ±128 dps.
    Dps128,
    /// ±256 dps.
    Dps256,
    /// ±512 dps.
    Dps512,
    /// ±1024 dps.
    Dps1024,
    /// ±2048 dps, 16 LSB/dps.
    #[default]
    Dps2048,
}

impl GyroRange {
    /// Full scale in dps.
    pub const fn full_scale_dps(self) -> i32 {
        16 << (self as u8)
    }
}

/// Accelerometer output data rate (`aODR`). With the gyroscope on, the
/// QMI8658A derives the rate from the gyroscope drive frequency (e.g.
/// 28.025 Hz instead of 31.25 Hz); 8000–2000 Hz are not available on the
/// QMI8658A with the accelerometer alone.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AccelOdr {
    /// 8000 Hz.
    Hz8000 = 0,
    /// 4000 Hz.
    Hz4000,
    /// 2000 Hz.
    Hz2000,
    /// 1000 Hz.
    Hz1000,
    /// 500 Hz.
    Hz500,
    /// 250 Hz.
    Hz250,
    /// 125 Hz.
    Hz125,
    /// 62.5 Hz.
    Hz62_5,
    /// 31.25 Hz, 132 µA.
    #[default]
    Hz31_25,
    /// Low-power mode, 128 Hz, 55 µA (gyroscope must be off).
    LowPower128 = 12,
    /// Low-power mode, 21 Hz, 42 µA.
    LowPower21,
    /// Low-power mode, 11 Hz, 35 µA.
    LowPower11,
    /// Low-power mode, 3 Hz, 30 µA.
    LowPower3,
}

impl AccelOdr {
    /// Whether this is a low-power (duty-cycled) rate.
    pub const fn is_low_power(self) -> bool {
        self as u8 >= Self::LowPower128 as u8
    }

    /// Sample period in µs, rounded up.
    pub const fn period_us(self) -> WaitUs {
        match self {
            Self::LowPower128 => 7_813,
            Self::LowPower21 => 47_620,
            Self::LowPower11 => 90_910,
            Self::LowPower3 => 333_334,
            normal => 125 << (normal as u8),
        }
    }
}

/// Gyroscope output data rate (`gODR`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GyroOdr {
    /// 8000 Hz.
    Hz8000 = 0,
    /// 4000 Hz.
    Hz4000,
    /// 2000 Hz.
    Hz2000,
    /// 1000 Hz.
    Hz1000,
    /// 500 Hz.
    Hz500,
    /// 250 Hz.
    Hz250,
    /// 125 Hz.
    Hz125,
    /// 62.5 Hz.
    Hz62_5,
    /// 31.25 Hz.
    #[default]
    Hz31_25,
}

impl GyroOdr {
    /// Sample period in µs (nominal rate).
    pub const fn period_us(self) -> WaitUs {
        125 << (self as u8)
    }
}

/// Gyroscope settings.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct GyroConfig {
    /// Full scale.
    pub range: GyroRange,
    /// Output data rate.
    pub odr: GyroOdr,
}

/// Device configuration, applied by [`Qmi8658::configure`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// Accelerometer full scale.
    pub accel_range: AccelRange,
    /// Accelerometer output data rate; low-power rates need the gyroscope
    /// off.
    pub accel_odr: AccelOdr,
    /// Gyroscope settings; `None` keeps it disabled.
    pub gyro: Option<GyroConfig>,
}

/// Interrupt pin used for wake-on-motion.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum IntPin {
    /// INT1 (on the QMI8658C also pulsed by `CTRL9` command completion).
    #[default]
    Int1,
    /// INT2 (data-ready pin outside wake-on-motion).
    Int2,
}

/// Wake-on-motion settings. In WoM mode only the accelerometer runs and no
/// samples are produced; the selected pin toggles on every motion event.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WakeOnMotion {
    /// Slope threshold in mg (1 mg/LSB), clamped to 1..=255.
    pub threshold_mg: u8,
    /// Interrupt pin.
    pub pin: IntPin,
    /// Pin level before the first event.
    pub initial_high: bool,
    /// Accelerometer samples ignored after enabling, 0..=63.
    pub blanking_samples: u8,
}

impl Default for WakeOnMotion {
    fn default() -> Self {
        Self {
            threshold_mg: 64,
            pin: IntPin::Int1,
            initial_high: false,
            blanking_samples: 4,
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

/// Raw accelerometer count to mg (full scale / 32768).
pub fn accel_mg(raw: i16, range: AccelRange) -> i32 {
    let n = i32::from(raw) * range.full_scale_mg();
    (n + (1 << 14)) >> 15
}

/// Raw gyroscope count to mdps (full scale / 32768).
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

/// Decode the 12-byte block at `AX_L` (accelerometer, then gyroscope).
pub fn decode_block(bytes: &[u8; 12], accel: AccelRange, gyro: Option<GyroRange>) -> Measurement {
    Measurement {
        accel: vector(decode_axes(&bytes[..6]), |r| accel_mg(r, accel)),
        gyro: gyro.map(|range| vector(decode_axes(&bytes[6..]), |r| gyro_mdps(r, range))),
    }
}

/// `CTRL1` value: burst reads, little-endian, plus the INT pin enable for
/// wake-on-motion on the QMI8658A.
pub fn ctrl1(variant: Variant, wom: Option<&WakeOnMotion>) -> u8 {
    match (variant, wom.map(|w| w.pin)) {
        (Variant::A, Some(IntPin::Int1)) => CTRL1_BASE | CTRL1_INT1_EN,
        (Variant::A, Some(IntPin::Int2)) => CTRL1_BASE | CTRL1_INT2_EN,
        _ => CTRL1_BASE,
    }
}

/// Register writes that disable the sensors and set range/ODR. `None` if
/// a low-power accelerometer rate or wake-on-motion is combined with the
/// gyroscope.
pub fn setup_writes(
    variant: Variant,
    config: &Config,
    wom: Option<&WakeOnMotion>,
) -> Option<RegWrites> {
    if config.gyro.is_some() && (config.accel_odr.is_low_power() || wom.is_some()) {
        return None;
    }
    let mut w = RegWrites::new();
    w.push(reg::CTRL7, 0);
    w.push(reg::CTRL1, ctrl1(variant, wom));
    if variant == Variant::A {
        w.push(reg::CTRL8, CTRL8_HANDSHAKE_STATUSINT);
    }
    w.push(
        reg::CTRL2,
        (config.accel_range as u8) << 4 | config.accel_odr as u8,
    );
    if let Some(g) = config.gyro {
        w.push(reg::CTRL3, (g.range as u8) << 4 | g.odr as u8);
    }
    Some(w)
}

/// `CAL1_L`, `CAL1_H` and `CTRL9` writes for `CTRL_CMD_WRITE_WOM_SETTING`;
/// `None` writes threshold 0, which leaves WoM mode.
pub fn wom_writes(wom: Option<&WakeOnMotion>) -> [(u8, u8); 3] {
    let (th, cal1_h) = match wom {
        Some(w) => {
            let sel = match (w.pin, w.initial_high) {
                (IntPin::Int1, false) => 0b00,
                (IntPin::Int2, false) => 0b01,
                (IntPin::Int1, true) => 0b10,
                (IntPin::Int2, true) => 0b11,
            };
            (w.threshold_mg.max(1), sel << 6 | w.blanking_samples.min(63))
        }
        None => (0, 0),
    };
    [
        (reg::CAL1_L, th),
        (reg::CAL1_H, cal1_h),
        (reg::CTRL9, cmd::WRITE_WOM_SETTING),
    ]
}

/// `CTRL7` sensor enables.
pub fn ctrl7(config: &Config) -> u8 {
    if config.gyro.is_some() {
        CTRL7_AEN | CTRL7_GEN
    } else {
        CTRL7_AEN
    }
}

/// Time from enabling the sensors to settled data: wake-up plus 3/ODR.
pub fn start_wait_us(variant: Variant, config: &Config) -> WaitUs {
    let accel = ACCEL_WAKE_US + 3 * config.accel_odr.period_us();
    match config.gyro {
        Some(g) => accel.max(variant.gyro_wake_us() + 3 * g.odr.period_us()),
        None => accel,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Idle,
    Setup,
    CmdDone,
}

macro_rules! driver {
    ($doc:literal, $Iface:ident, $mode:ident, [$($async:tt)?], [$($aw:tt)*]) => {
        #[doc = $doc]
        #[derive(Debug)]
        pub struct Qmi8658<I> {
            iface: I,
            variant: Variant,
            config: Config,
            wom: Option<WakeOnMotion>,
            /// WoM setting currently programmed in the device.
            wom_active: bool,
            /// Soft reset done since creation or power-down.
            initialized: bool,
            step: Step,
            polls: u8,
        }

        impl<I> Qmi8658<I> {
            /// Create a driver over any register interface; no bus traffic.
            pub fn new(iface: I) -> Self {
                Self {
                    iface,
                    variant: Variant::A,
                    config: Config::default(),
                    wom: None,
                    wom_active: false,
                    initialized: false,
                    step: Step::Idle,
                    polls: 0,
                }
            }

            /// Select the part variant (default [`Variant::A`]).
            pub fn with_variant(mut self, variant: Variant) -> Self {
                self.variant = variant;
                self
            }

            /// Set the configuration used by `configure()` / `wake()`.
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

            /// Whether a configuration sequence is still in progress.
            pub fn configuring(&self) -> bool {
                self.step != Step::Idle
            }

            /// Give the register interface back.
            pub fn into_interface(self) -> I {
                self.iface
            }
        }

        impl<I2C> Qmi8658<I2cInterface<I2C>> {
            /// Create an I2C driver; no bus traffic.
            pub fn new_i2c(i2c: I2C, address: u8) -> Self {
                Self::new(I2cInterface::new(i2c, address))
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.iface.release()
            }
        }

        impl<SPI> Qmi8658<SpiInterface<SPI>> {
            /// Create a 4-wire SPI driver; no bus traffic.
            pub fn new_spi(spi: SPI) -> Self {
                Self::new(SpiInterface::new(spi))
            }

            /// Give the device back.
            pub fn release(self) -> SPI {
                self.iface.release()
            }
        }

        impl<I: $Iface> Qmi8658<I> {
            /// Read one register.
            pub $($async)? fn read_register(&mut self, reg: u8) -> Result<u8, Error<I::Error>> {
                let mut b = [0u8];
                self.iface.read_regs(reg, &mut b) $($aw)* .map_err(Error::bus)?;
                Ok(b[0])
            }

            /// Write one register (configuration registers do not support
            /// burst writes).
            pub $($async)? fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Error<I::Error>> {
                self.iface.write_reg(reg, value) $($aw)* .map_err(Error::bus)
            }

            $($async)? fn write_all(&mut self, writes: &[(u8, u8)]) -> Result<(), Error<I::Error>> {
                for &(r, v) in writes {
                    self.write_register(r, v) $($aw)* ?;
                }
                Ok(())
            }

            /// Read `WHO_AM_I` (0x05).
            pub $($async)? fn probe(&mut self) -> Result<(), Error<I::Error>> {
                let id = self.read_register(reg::WHO_AM_I) $($aw)* ?;
                if id != WHO_AM_I {
                    return Err(Error::UnexpectedId(u32::from(id)));
                }
                Ok(())
            }

            /// Read `REVISION_ID` (0x7C on the QMI8658A).
            pub $($async)? fn revision(&mut self) -> Result<u8, Error<I::Error>> {
                self.read_register(reg::REVISION_ID) $($aw)*
            }

            /// Soft reset to the power-on default state (sensors off,
            /// 15 µA); returns the system turn-on time.
            pub $($async)? fn reset(&mut self) -> Result<WaitUs, Error<I::Error>> {
                self.write_register(reg::RESET, RESET_VALUE) $($aw)* ?;
                self.initialized = true;
                self.wom_active = false;
                self.step = Step::Idle;
                Ok(self.variant.turn_on_us())
            }

            /// Start applying the stored configuration (and the
            /// wake-on-motion setting). The first call after creation or
            /// [`Self::power_down`] soft-resets the part. Call
            /// [`Self::continue_configure`] after each returned wait until
            /// it returns 0.
            pub $($async)? fn configure(&mut self) -> Result<WaitUs, Error<I::Error>> {
                setup_writes(self.variant, &self.config, self.wom.as_ref()).ok_or(Error::Unsupported)?;
                if !self.initialized {
                    let wait = self.reset() $($aw)* ?;
                    self.step = Step::Setup;
                    return Ok(wait);
                }
                self.step = Step::Setup;
                self.continue_configure() $($aw)*
            }

            $($async)? fn enable_sensors(&mut self) -> Result<WaitUs, Error<I::Error>> {
                self.write_register(reg::CTRL7, ctrl7(&self.config)) $($aw)* ?;
                self.step = Step::Idle;
                Ok(start_wait_us(self.variant, &self.config))
            }

            /// Perform the next configuration step; returns the time to
            /// wait before the next call, or 0 when configuration is done.
            /// [`Error::Timeout`] if a `CTRL9` command does not complete
            /// within [`CMD_MAX_POLLS`] polls.
            pub $($async)? fn continue_configure(&mut self) -> Result<WaitUs, Error<I::Error>> {
                match self.step {
                    Step::Idle => Ok(0),
                    Step::Setup => {
                        let writes = setup_writes(self.variant, &self.config, self.wom.as_ref())
                            .ok_or(Error::Unsupported)?;
                        self.write_all(writes.as_slice()) $($aw)* ?;
                        if self.wom.is_none() && !self.wom_active {
                            return self.enable_sensors() $($aw)*;
                        }
                        self.write_all(&wom_writes(self.wom.as_ref())) $($aw)* ?;
                        self.polls = 0;
                        self.step = Step::CmdDone;
                        Ok(CMD_POLL_US)
                    }
                    Step::CmdDone => {
                        let done = match self.variant {
                            Variant::A => {
                                self.read_register(reg::STATUSINT) $($aw)* ? & STATUSINT_CMD_DONE != 0
                            }
                            Variant::C => {
                                self.read_register(reg::STATUS1) $($aw)* ? & STATUS1_CMD_DONE != 0
                            }
                        };
                        if !done {
                            self.polls += 1;
                            if self.polls >= CMD_MAX_POLLS {
                                self.step = Step::Idle;
                                return Err(Error::Timeout);
                            }
                            return Ok(CMD_POLL_US);
                        }
                        if self.variant == Variant::A {
                            self.write_register(reg::CTRL9, cmd::ACK) $($aw)* ?;
                        }
                        self.wom_active = self.wom.is_some();
                        self.enable_sensors() $($aw)*
                    }
                }
            }

            /// Store a new configuration and start applying it.
            pub $($async)? fn set_config(&mut self, config: Config) -> Result<WaitUs, Error<I::Error>> {
                self.config = config;
                self.configure() $($aw)*
            }

            /// Low-power state (sensors off, slow clock, 8 µA); settings and
            /// the fast restart are kept. Wake with `configure()`.
            pub $($async)? fn standby(&mut self) -> Result<(), Error<I::Error>> {
                self.step = Step::Idle;
                self.write_register(reg::CTRL7, 0) $($aw)* ?;
                let ctrl2 = (self.config.accel_range as u8) << 4 | AccelOdr::LowPower3 as u8;
                self.write_register(reg::CTRL2, ctrl2) $($aw)*
            }

            /// Power-down (oscillator off, 6 µA). Leaving it takes the
            /// system turn-on time, so the next `configure()` starts with a
            /// soft reset (1.75 s on the QMI8658C).
            pub $($async)? fn power_down(&mut self) -> Result<(), Error<I::Error>> {
                self.step = Step::Idle;
                self.write_register(reg::CTRL7, 0) $($aw)* ?;
                self.write_register(reg::CTRL1, CTRL1_BASE | CTRL1_SENSOR_DISABLE) $($aw)* ?;
                self.initialized = false;
                Ok(())
            }

            /// Enable wake-on-motion (accelerometer only, gyroscope must be
            /// off) and restart configuration (see [`Self::configure`]).
            /// Use a low-power accelerometer rate for µA-range operation.
            pub $($async)? fn enable_wake_on_motion(&mut self, wom: WakeOnMotion) -> Result<WaitUs, Error<I::Error>> {
                self.wom = Some(wom);
                self.configure() $($aw)*
            }

            /// Leave wake-on-motion mode and restart configuration.
            pub $($async)? fn disable_wake_on_motion(&mut self) -> Result<WaitUs, Error<I::Error>> {
                self.wom = None;
                self.configure() $($aw)*
            }

            /// Whether wake-on-motion fired since the last call. Reads
            /// `STATUS1`, which clears the flag and returns the pin to its
            /// initial level.
            pub $($async)? fn motion_detected(&mut self) -> Result<bool, Error<I::Error>> {
                let s = self.read_register(reg::STATUS1) $($aw)* ?;
                Ok(s & STATUS1_WOM != 0)
            }
        }

        impl<I: $Iface> zigbee_sensor::$mode::Sensor for Qmi8658<I> {
            type BusError = I::Error;
            type Measurement = Measurement;

            /// Free-running: returns 0, or performs the next pending
            /// configuration step and returns its wait.
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I::Error>> {
                self.continue_configure() $($aw)*
            }

            /// [`Error::NotReady`] while configuring, in wake-on-motion mode
            /// (no samples are produced) and until `STATUS0` reports new
            /// accelerometer data.
            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<I::Error>> {
                if self.step != Step::Idle || self.wom_active {
                    return Err(Error::NotReady);
                }
                let status = self.read_register(reg::STATUS0) $($aw)* ?;
                if status & STATUS0_ADA == 0 {
                    return Err(Error::NotReady);
                }
                let mut b = [0u8; 12];
                let gyro = self.config.gyro.map(|g| g.range);
                let len = if gyro.is_some() { 12 } else { 6 };
                self.iface.read_regs(reg::AX_L, &mut b[..len]) $($aw)* .map_err(Error::bus)?;
                Ok(decode_block(&b, self.config.accel_range, gyro))
            }
        }

        impl<I: $Iface> zigbee_sensor::$mode::LowPower for Qmi8658<I> {
            type BusError = I::Error;

            /// [`Self::standby`] (8 µA, fast restart).
            $($async)? fn sleep(&mut self) -> Result<(), Error<I::Error>> {
                self.standby() $($aw)*
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
        "Async twin of [`super::Qmi8658`].",
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

    const A: u8 = ADDRESS_SA0_HIGH;

    fn rd(reg: u8, data: &[u8]) -> I2cTx {
        I2cTx::write_read(A, vec![reg], data.to_vec())
    }
    fn wr(reg: u8, value: u8) -> I2cTx {
        I2cTx::write(A, vec![reg, value])
    }

    #[test]
    fn datasheet_sensitivities_and_waits() {
        // Tables 7/8: 16384 LSB/g at ±2 g, 2048 LSB/g at ±16 g; 2048 LSB/dps
        // at ±16 dps, 16 LSB/dps at ±2048 dps.
        assert_eq!(accel_mg(16_384, AccelRange::G2), 1_000);
        assert_eq!(accel_mg(-2_048, AccelRange::G16), -1_000);
        assert_eq!(gyro_mdps(2_048, GyroRange::Dps16), 1_000);
        assert_eq!(gyro_mdps(-16, GyroRange::Dps2048), -1_000);
        assert_eq!(gyro_mdps(i16::MIN, GyroRange::Dps2048), -2_048_000);
        // Accel 3 ms + 3/ODR; gyro 150 ms (A) / 60 ms (C) + 3/ODR.
        let cfg = Config::default();
        assert_eq!(start_wait_us(Variant::A, &cfg), 99_000);
        let cfg = Config {
            accel_odr: AccelOdr::Hz1000,
            gyro: Some(GyroConfig {
                range: GyroRange::Dps512,
                odr: GyroOdr::Hz1000,
            }),
            ..cfg
        };
        assert_eq!(start_wait_us(Variant::A, &cfg), 153_000);
        assert_eq!(start_wait_us(Variant::C, &cfg), 63_000);
        assert_eq!(AccelOdr::LowPower3.period_us(), 333_334);
    }

    #[test]
    fn probe_who_am_i() {
        let mut i2c = Mock::new(&[
            rd(reg::WHO_AM_I, &[0x05]),
            rd(reg::WHO_AM_I, &[0x6A]),
            rd(reg::REVISION_ID, &[0x7C]),
        ]);
        let mut imu = Qmi8658::new_i2c(&mut i2c, A);
        imu.probe().unwrap();
        assert_eq!(imu.probe(), Err(Error::UnexpectedId(0x6A)));
        assert_eq!(imu.revision().unwrap(), 0x7C);
        i2c.done();
    }

    #[test]
    fn configure_default_then_read_and_reconfigure() {
        let mut i2c = Mock::new(&[
            wr(reg::RESET, 0xB0),
            wr(reg::CTRL7, 0x00),
            wr(reg::CTRL1, 0x40),
            wr(reg::CTRL8, 0x80),
            wr(reg::CTRL2, 0x08),
            wr(reg::CTRL7, 0x01),
            rd(reg::STATUS0, &[0x00]),
            rd(reg::STATUS0, &[0x01]),
            rd(reg::AX_L, &[0x00, 0x40, 0x00, 0xC0, 0, 0]),
            // Second configure: no reset.
            wr(reg::CTRL7, 0x00),
            wr(reg::CTRL1, 0x40),
            wr(reg::CTRL8, 0x80),
            wr(reg::CTRL2, 0x1D),
            wr(reg::CTRL7, 0x01),
        ]);
        let mut imu = Qmi8658::new_i2c(&mut i2c, A);
        assert_eq!(imu.configure().unwrap(), 15_000);
        assert_eq!(imu.read_measurement(), Err(Error::NotReady));
        assert_eq!(imu.start_measurement().unwrap(), 99_000);
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
        let lp = Config {
            accel_range: AccelRange::G4,
            accel_odr: AccelOdr::LowPower21,
            gyro: None,
        };
        assert_eq!(imu.set_config(lp).unwrap(), 3_000 + 3 * 47_620);
        assert!(!imu.configuring());
        i2c.done();
    }

    #[test]
    fn unsupported_combinations() {
        let gyro = Some(GyroConfig::default());
        let lp_gyro = Config {
            accel_odr: AccelOdr::LowPower128,
            gyro,
            ..Config::default()
        };
        assert!(setup_writes(Variant::A, &lp_gyro, None).is_none());
        let imu6 = Config {
            gyro,
            ..Config::default()
        };
        assert!(setup_writes(Variant::A, &imu6, None).is_some());
        assert!(setup_writes(Variant::A, &imu6, Some(&WakeOnMotion::default())).is_none());
        let mut i2c = Mock::new(&[]);
        let mut imu = Qmi8658::new_i2c(&mut i2c, A).with_config(lp_gyro);
        assert_eq!(imu.configure(), Err(Error::Unsupported));
        i2c.done();
    }

    #[test]
    fn wake_on_motion_a() {
        let cfg = Config {
            accel_range: AccelRange::G4,
            accel_odr: AccelOdr::LowPower21,
            gyro: None,
        };
        let wom = WakeOnMotion {
            threshold_mg: 100,
            pin: IntPin::Int2,
            initial_high: true,
            blanking_samples: 2,
        };
        let mut i2c = Mock::new(&[
            wr(reg::RESET, 0xB0),
            wr(reg::CTRL7, 0x00),
            wr(reg::CTRL1, 0x50),
            wr(reg::CTRL8, 0x80),
            wr(reg::CTRL2, 0x1D),
            wr(reg::CAL1_L, 100),
            wr(reg::CAL1_H, 0xC2),
            wr(reg::CTRL9, 0x08),
            rd(reg::STATUSINT, &[0x00]),
            rd(reg::STATUSINT, &[0x80]),
            wr(reg::CTRL9, 0x00),
            wr(reg::CTRL7, 0x01),
            rd(reg::STATUS1, &[0x04]),
            rd(reg::STATUS1, &[0x00]),
            // Sleep: sensors off, low-power clock.
            wr(reg::CTRL7, 0x00),
            wr(reg::CTRL2, 0x1F),
        ]);
        let mut imu = Qmi8658::new_i2c(&mut i2c, A).with_config(cfg);
        assert_eq!(imu.enable_wake_on_motion(wom).unwrap(), 15_000);
        assert_eq!(imu.continue_configure().unwrap(), CMD_POLL_US);
        assert_eq!(imu.continue_configure().unwrap(), CMD_POLL_US);
        assert_eq!(imu.continue_configure().unwrap(), 3_000 + 3 * 47_620);
        assert_eq!(imu.read_measurement(), Err(Error::NotReady));
        assert!(imu.motion_detected().unwrap());
        assert!(!imu.motion_detected().unwrap());
        imu.sleep().unwrap();
        i2c.done();
    }

    #[test]
    fn variant_c_wom_disable_and_timeout() {
        let cfg = Config {
            accel_odr: AccelOdr::LowPower3,
            ..Config::default()
        };
        let mut ex = vec![
            wr(reg::RESET, 0xB0),
            wr(reg::CTRL7, 0x00),
            wr(reg::CTRL1, 0x40),
            wr(reg::CTRL2, 0x0F),
            wr(reg::CAL1_L, 64),
            wr(reg::CAL1_H, 0x04),
            wr(reg::CTRL9, 0x08),
            rd(reg::STATUS1, &[0x01]),
            wr(reg::CTRL7, 0x01),
            // Disable: threshold 0 command, no ACK on the C.
            wr(reg::CTRL7, 0x00),
            wr(reg::CTRL1, 0x40),
            wr(reg::CTRL2, 0x0F),
            wr(reg::CAL1_L, 0),
            wr(reg::CAL1_H, 0),
            wr(reg::CTRL9, 0x08),
        ];
        ex.extend((0..CMD_MAX_POLLS).map(|_| rd(reg::STATUS1, &[0x00])));
        let mut i2c = Mock::new(&ex);
        let mut imu = Qmi8658::new_i2c(&mut i2c, A)
            .with_variant(Variant::C)
            .with_config(cfg);
        assert_eq!(
            imu.enable_wake_on_motion(WakeOnMotion::default()).unwrap(),
            1_750_000
        );
        assert_eq!(imu.continue_configure().unwrap(), CMD_POLL_US);
        assert_eq!(imu.continue_configure().unwrap(), 3_000 + 3 * 333_334);
        assert_eq!(imu.disable_wake_on_motion().unwrap(), CMD_POLL_US);
        for _ in 1..CMD_MAX_POLLS {
            assert_eq!(imu.continue_configure().unwrap(), CMD_POLL_US);
        }
        assert_eq!(imu.continue_configure(), Err(Error::Timeout));
        assert!(!imu.configuring());
        i2c.done();
    }

    #[test]
    fn spi_six_axis_block_and_power_down() {
        let cfg = Config {
            accel_odr: AccelOdr::Hz125,
            gyro: Some(GyroConfig {
                range: GyroRange::Dps2048,
                odr: GyroOdr::Hz125,
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
        let mut block = vec![0x00, 0x40, 0, 0, 0, 0];
        block.extend_from_slice(&[0xA0, 0x00, 0x60, 0xFF, 0, 0]);
        let mut ex = Vec::new();
        ex.extend(w(reg::RESET, 0xB0));
        ex.extend(w(reg::CTRL7, 0x00));
        ex.extend(w(reg::CTRL1, 0x40));
        ex.extend(w(reg::CTRL8, 0x80));
        ex.extend(w(reg::CTRL2, 0x06));
        ex.extend(w(reg::CTRL3, 0x76));
        ex.extend(w(reg::CTRL7, 0x03));
        ex.extend(r(reg::STATUS0, vec![0x03]));
        ex.extend(r(reg::AX_L, block));
        ex.extend(w(reg::CTRL7, 0x00));
        ex.extend(w(reg::CTRL1, 0x41));
        ex.extend(w(reg::RESET, 0xB0));
        let mut spi = SpiMock::new(&ex);
        let mut imu = Qmi8658::new_spi(&mut spi).with_config(cfg);
        assert_eq!(imu.wake().unwrap(), 15_000);
        assert_eq!(imu.start_measurement().unwrap(), 150_000 + 3 * 8_000);
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
        imu.power_down().unwrap();
        assert_eq!(imu.configure().unwrap(), 15_000);
        spi.done();
    }

    #[test]
    fn async_configure_and_read() {
        use zigbee_sensor::asynch::Sensor as _;

        let mut ex = vec![wr(reg::RESET, 0xB0)];
        let setup = setup_writes(Variant::A, &Config::default(), None).unwrap();
        ex.extend(setup.as_slice().iter().map(|&(r, v)| wr(r, v)));
        ex.push(wr(reg::CTRL7, 0x01));
        ex.push(rd(reg::STATUS0, &[0x01]));
        ex.push(rd(reg::AX_L, &[0, 0, 0, 0, 0x00, 0x40]));
        let mut i2c = Mock::new(&ex);
        let mut imu = asynch::Qmi8658::new_i2c(&mut i2c, A);
        assert_eq!(block_on(imu.configure()).unwrap(), 15_000);
        assert_eq!(block_on(imu.start_measurement()).unwrap(), 99_000);
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
