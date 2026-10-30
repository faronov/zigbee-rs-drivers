//! ST LIS2DH12 / LIS2DH / LIS3DH and Silan SC7A20 three-axis accelerometers
//! (I2C or SPI).
//!
//! The parts are free-running: once [`Config`] is applied they convert at the
//! selected output data rate (ODR). [`Sensor::start_measurement`] powers the
//! part up if it was put to sleep and returns the turn-on time, otherwise `0`;
//! [`Sensor::read_measurement`] returns [`Error::NotReady`] until a fresh
//! sample is available (`STATUS_REG.ZYXDA`).
//!
//! For a sleepy Zigbee end device, keep the part in [`Mode::LowPower`] at a
//! low ODR (2 µA at 1 Hz, 3 µA at 10 Hz) with [`WakeUpConfig`] routed to INT1
//! or INT2, and sleep the MCU until the pin fires. [`LowPower::sleep`] is a
//! full power-down (0.5 µA) and stops the motion detector.
//!
//! WHO_AM_I is `0x33` for every ST part (LIS2DH12, LIS2DH, LIS3DH) and `0x11`
//! for the SC7A20, whose ±16 g sensitivity and threshold steps differ; see
//! [`Variant`].
//!
//! References:
//! * ST "LIS2DH12 datasheet", DocID025056 Rev 6 (2017).
//! * ST "LIS3DH datasheet", Doc ID 17530 Rev 1 (2010).
//! * Silan "SC7A20 说明书" (datasheet) v0.4.
//!
//! [`Sensor::start_measurement`]: zigbee_sensor::blocking::Sensor::start_measurement
//! [`Sensor::read_measurement`]: zigbee_sensor::blocking::Sensor::read_measurement
//! [`LowPower::sleep`]: zigbee_sensor::blocking::LowPower::sleep
//!
//! ```ignore
//! let mut acc = Lis2dh::new_i2c(i2c, ADDRESS_SA0_LOW);
//! acc.probe()?;
//! acc.configure(Config::default())?;
//! acc.configure_wake_up(&WakeUpConfig { threshold_mg: 80, ..Default::default() })?;
//! // ... sleep until INT1, then:
//! let src = acc.read_interrupt_source(IntPin::Int1)?; // clears the latch
//! ```

#![no_std]

mod interface;

pub use interface::{AsyncInterface, I2cInterface, Interface, SpiInterface};
use zigbee_sensor::{Error, Quantity, Readings, Vector3, WaitUs};

/// I2C address with SA0/SDO low.
pub const ADDRESS_SA0_LOW: u8 = 0x18;
/// I2C address with SA0/SDO high (the pin has an internal pull-up).
pub const ADDRESS_SA0_HIGH: u8 = 0x19;

/// WHO_AM_I value of LIS2DH12, LIS2DH and LIS3DH.
pub const WHO_AM_I_ST: u8 = 0x33;
/// WHO_AM_I value of the SC7A20.
pub const WHO_AM_I_SC7A20: u8 = 0x11;

const REG_WHO_AM_I: u8 = 0x0F;
const REG_CTRL1: u8 = 0x20;
const REG_CTRL2: u8 = 0x21;
const REG_CTRL3: u8 = 0x22;
const REG_CTRL4: u8 = 0x23;
const REG_CTRL5: u8 = 0x24;
const REG_CTRL6: u8 = 0x25;
const REG_REFERENCE: u8 = 0x26;
const REG_STATUS: u8 = 0x27;
const REG_INT1_CFG: u8 = 0x30;
const REG_INT2_CFG: u8 = 0x34;

const CTRL1_XYZ: u8 = 0x07;
const CTRL1_LPEN: u8 = 0x08;
const CTRL4_BDU: u8 = 0x80;
const CTRL4_HR: u8 = 0x08;
const CTRL6_ACTIVE_LOW: u8 = 0x02;
const STATUS_ZYXDA: u8 = 0x08;
const CFG_XYZ_HIGH_OR: u8 = 0x2A;

/// Part family, selected by WHO_AM_I.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Variant {
    /// ST LIS2DH12 / LIS2DH / LIS3DH (WHO_AM_I `0x33`).
    #[default]
    St,
    /// Silan SC7A20 (WHO_AM_I `0x11`): 8 mg/digit at ±16 g, threshold steps
    /// 16/32/64/128 mg.
    Sc7a20,
}

impl Variant {
    /// Map a WHO_AM_I value to a variant.
    pub const fn from_id(id: u8) -> Option<Self> {
        match id {
            WHO_AM_I_ST => Some(Self::St),
            WHO_AM_I_SC7A20 => Some(Self::Sc7a20),
            _ => None,
        }
    }

    /// Sensitivity in high-resolution (12-bit) mode, mg/digit
    /// (LIS2DH12 Table 4, SC7A20 mechanical characteristics).
    pub const fn hr_sensitivity_mg(self, range: Range) -> i32 {
        match (range, self) {
            (Range::G2, _) => 1,
            (Range::G4, _) => 2,
            (Range::G8, _) => 4,
            (Range::G16, Self::St) => 12,
            (Range::G16, Self::Sc7a20) => 8,
        }
    }

    /// INTx_THS step in mg (LIS2DH12 Table 59, SC7A20 §12.13).
    pub const fn threshold_lsb_mg(self, range: Range) -> u16 {
        match (range, self) {
            (Range::G2, _) => 16,
            (Range::G4, _) => 32,
            (Range::G8, Self::St) => 62,
            (Range::G8, Self::Sc7a20) => 64,
            (Range::G16, Self::St) => 186,
            (Range::G16, Self::Sc7a20) => 128,
        }
    }

    /// CTRL_REG6 bit routing interrupt generator 2 to the INT2 pin
    /// (ST: `I2_IA2`, bit 5; SC7A20: `I2_AOI2`, bit 6).
    const fn int2_route(self) -> u8 {
        match self {
            Self::St => 0x20,
            Self::Sc7a20 => 0x40,
        }
    }
}

/// Full-scale range.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Range {
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

impl Range {
    const fn bits(self) -> u8 {
        self as u8
    }
}

/// Operating mode: resolution versus current.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Mode {
    /// 8-bit data, lowest current (2 µA at 1 Hz, 3 µA at 10 Hz on LIS2DH12).
    #[default]
    LowPower,
    /// 10-bit data.
    Normal,
    /// 12-bit data, 7/ODR turn-on time.
    HighResolution,
}

impl Mode {
    const fn bits(self) -> u32 {
        match self {
            Self::LowPower => 8,
            Self::Normal => 10,
            Self::HighResolution => 12,
        }
    }
}

/// Output data rate (CTRL_REG1 ODR\[3:0\]).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DataRate {
    /// 1 Hz.
    Hz1 = 1,
    /// 10 Hz.
    #[default]
    Hz10 = 2,
    /// 25 Hz.
    Hz25 = 3,
    /// 50 Hz.
    Hz50 = 4,
    /// 100 Hz.
    Hz100 = 5,
    /// 200 Hz.
    Hz200 = 6,
    /// 400 Hz.
    Hz400 = 7,
    /// 1.62 kHz (SC7A20: 1.6 kHz); low-power mode only.
    Hz1620 = 8,
    /// 1.344 kHz in normal/high-resolution mode, 5.376 kHz in low-power mode
    /// (SC7A20: 1.25 / 5 kHz).
    Hz1344 = 9,
}

impl DataRate {
    /// Sample period in µs (rounded up, covering the slower SC7A20 rates).
    pub const fn period_us(self, mode: Mode) -> u32 {
        match (self, mode) {
            (Self::Hz1, _) => 1_000_000,
            (Self::Hz10, _) => 100_000,
            (Self::Hz25, _) => 40_000,
            (Self::Hz50, _) => 20_000,
            (Self::Hz100, _) => 10_000,
            (Self::Hz200, _) => 5_000,
            (Self::Hz400, _) => 2_500,
            (Self::Hz1620, _) => 625,
            (Self::Hz1344, Mode::LowPower) => 200,
            (Self::Hz1344, _) => 800,
        }
    }
}

/// Measurement configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// Full-scale range.
    pub range: Range,
    /// Resolution / power mode.
    pub mode: Mode,
    /// Output data rate.
    pub rate: DataRate,
    /// INT1/INT2 active low (default active high).
    pub int_active_low: bool,
}

/// Time from power-down to the first valid sample (LIS2DH12 Table 10/11):
/// 1/ODR plus 1 ms (low-power) or 1.6 ms (normal), 7/ODR in high resolution.
pub const fn turn_on_us(mode: Mode, rate: DataRate) -> WaitUs {
    let period = rate.period_us(mode);
    match mode {
        Mode::LowPower => period + 1_000,
        Mode::Normal => period + 1_600,
        Mode::HighResolution => 7 * period,
    }
}

/// Interrupt output pin. Each pin uses its own interrupt generator
/// (INT1 → IA1 at 30h–33h, INT2 → IA2 at 34h–37h).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum IntPin {
    /// INT1 pin, generator 1.
    #[default]
    Int1,
    /// INT2 pin, generator 2.
    Int2,
}

impl IntPin {
    const fn base(self) -> u8 {
        match self {
            Self::Int1 => REG_INT1_CFG,
            Self::Int2 => REG_INT2_CFG,
        }
    }

    /// CTRL_REG2 HP_IA1 / HP_IA2.
    const fn high_pass_bit(self) -> u8 {
        match self {
            Self::Int1 => 0x01,
            Self::Int2 => 0x02,
        }
    }

    /// CTRL_REG5 LIR_INT1 / LIR_INT2.
    const fn latch_bit(self) -> u8 {
        match self {
            Self::Int1 => 0x08,
            Self::Int2 => 0x02,
        }
    }

    /// CTRL_REG5 D4D_INT1 / D4D_INT2.
    const fn d4d_bit(self) -> u8 {
        match self {
            Self::Int1 => 0x04,
            Self::Int2 => 0x01,
        }
    }
}

/// Wake-up (motion) interrupt: any axis above the threshold for `duration`
/// samples (OR of X/Y/Z high events).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WakeUpConfig {
    /// Threshold in mg; rounded to the range-dependent step
    /// ([`Variant::threshold_lsb_mg`]), at most 127 steps.
    pub threshold_mg: u16,
    /// Minimum event duration in samples (1/ODR each), 0..=127.
    pub duration: u8,
    /// Output pin (and generator).
    pub pin: IntPin,
    /// Latch the interrupt until [`Lis2dh::read_interrupt_source`].
    pub latch: bool,
    /// Apply the high-pass filter so gravity / static tilt is ignored
    /// (recommended for motion and vibration detection).
    pub high_pass: bool,
}

impl Default for WakeUpConfig {
    fn default() -> Self {
        Self {
            threshold_mg: 64,
            duration: 0,
            pin: IntPin::Int1,
            latch: true,
            high_pass: true,
        }
    }
}

/// 6D orientation interrupt mode (INTx_CFG AOI/6D).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum OrientationMode {
    /// Interrupt when the part moves into a known orientation (AOI-6D = 01).
    #[default]
    Movement,
    /// Interrupt asserted while the part stays in a known orientation
    /// (AOI-6D = 11).
    Position,
}

/// 6D/4D orientation interrupt (tilt, door, garage door).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OrientationConfig {
    /// Threshold in mg (an axis counts as up/down above it); typical 600 mg.
    pub threshold_mg: u16,
    /// Minimum duration in samples (1/ODR each), 0..=127.
    pub duration: u8,
    /// Output pin (and generator).
    pub pin: IntPin,
    /// Latch the interrupt until [`Lis2dh::read_interrupt_source`].
    pub latch: bool,
    /// Ignore the Z axis (4D portrait/landscape).
    pub four_d: bool,
    /// Movement or position recognition.
    pub mode: OrientationMode,
}

impl Default for OrientationConfig {
    fn default() -> Self {
        Self {
            threshold_mg: 600,
            duration: 0,
            pin: IntPin::Int1,
            latch: true,
            four_d: false,
            mode: OrientationMode::Movement,
        }
    }
}

/// INT1_SRC / INT2_SRC.
///
/// In 6D mode the high/low bits tell which axis points up/down.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct InterruptSource(pub u8);

impl InterruptSource {
    /// One or more events occurred (IA).
    pub const fn active(self) -> bool {
        self.0 & 0x40 != 0
    }
    /// Z high event.
    pub const fn z_high(self) -> bool {
        self.0 & 0x20 != 0
    }
    /// Z low event.
    pub const fn z_low(self) -> bool {
        self.0 & 0x10 != 0
    }
    /// Y high event.
    pub const fn y_high(self) -> bool {
        self.0 & 0x08 != 0
    }
    /// Y low event.
    pub const fn y_low(self) -> bool {
        self.0 & 0x04 != 0
    }
    /// X high event.
    pub const fn x_high(self) -> bool {
        self.0 & 0x02 != 0
    }
    /// X low event.
    pub const fn x_low(self) -> bool {
        self.0 & 0x01 != 0
    }
}

/// One acceleration sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Acceleration in milli-g.
    pub acceleration: Vector3,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Acceleration(self.acceleration));
    }
}

/// Convert one left-justified 16-bit output word to milli-g.
pub const fn raw_to_mg(raw: i16, variant: Variant, range: Range, mode: Mode) -> i32 {
    let bits = mode.bits();
    ((raw as i32) >> (16 - bits)) * (variant.hr_sensitivity_mg(range) << (12 - bits))
}

/// Decode OUT_X_L..OUT_Z_H (little endian, left-justified) to milli-g.
pub fn decode(frame: &[u8; 6], variant: Variant, range: Range, mode: Mode) -> Vector3 {
    let axis = |i: usize| {
        raw_to_mg(
            i16::from_le_bytes([frame[i], frame[i + 1]]),
            variant,
            range,
            mode,
        )
    };
    Vector3 {
        x: axis(0),
        y: axis(2),
        z: axis(4),
    }
}

/// INTx_THS value for a threshold in mg (rounded to nearest, 0..=127).
pub fn threshold_reg(threshold_mg: u16, variant: Variant, range: Range) -> u8 {
    let lsb = variant.threshold_lsb_mg(range);
    let steps = (u32::from(threshold_mg) + u32::from(lsb / 2)) / u32::from(lsb);
    steps.min(127) as u8
}

const fn ctrl1(config: &Config) -> u8 {
    ((config.rate as u8) << 4)
        | if matches!(config.mode, Mode::LowPower) {
            CTRL1_LPEN
        } else {
            0
        }
        | CTRL1_XYZ
}

const fn ctrl4(config: &Config) -> u8 {
    CTRL4_BDU
        | (config.range.bits() << 4)
        | if matches!(config.mode, Mode::HighResolution) {
            CTRL4_HR
        } else {
            0
        }
}

macro_rules! driver {
    ($Iface:ident, $Sensor:path, $LowPower:path, [$($async:tt)?], [$($await:tt)*]) => {
        /// LIS2DH12 / LIS3DH / SC7A20 driver.
        pub struct Lis2dh<IFACE> {
            iface: IFACE,
            variant: Variant,
            config: Config,
            ctrl2: u8,
            ctrl3: u8,
            ctrl5: u8,
            ctrl6: u8,
            active: bool,
        }

        impl<IFACE> Lis2dh<IFACE> {
            /// Create a driver over a register interface; no bus traffic.
            /// Assumes the ST variant until [`Self::probe`] or
            /// [`Self::with_variant`].
            pub const fn new(iface: IFACE) -> Self {
                Self {
                    iface,
                    variant: Variant::St,
                    config: Config {
                        range: Range::G2,
                        mode: Mode::LowPower,
                        rate: DataRate::Hz10,
                        int_active_low: false,
                    },
                    ctrl2: 0,
                    ctrl3: 0,
                    ctrl5: 0,
                    ctrl6: 0,
                    active: false,
                }
            }

            /// Select the variant without reading WHO_AM_I.
            pub const fn with_variant(mut self, variant: Variant) -> Self {
                self.variant = variant;
                self
            }

            /// Detected or selected variant.
            pub const fn variant(&self) -> Variant {
                self.variant
            }

            /// Configuration last applied with [`Self::configure`].
            pub const fn config(&self) -> Config {
                self.config
            }

            /// Give the interface back.
            pub fn release(self) -> IFACE {
                self.iface
            }
        }

        impl<I2C> Lis2dh<I2cInterface<I2C>> {
            /// Driver on an I2C bus ([`ADDRESS_SA0_LOW`] or [`ADDRESS_SA0_HIGH`]).
            pub const fn new_i2c(i2c: I2C, address: u8) -> Self {
                Self::new(I2cInterface::new(i2c, address))
            }
        }

        impl<SPI> Lis2dh<SpiInterface<SPI>> {
            /// Driver on an SPI device (mode 0 or 3, up to 10 MHz).
            pub const fn new_spi(spi: SPI) -> Self {
                Self::new(SpiInterface::new(spi))
            }
        }

        impl<IFACE: $Iface> Lis2dh<IFACE> {
            $($async)? fn read_reg(&mut self, reg: u8) -> Result<u8, Error<IFACE::Error>> {
                let mut buf = [0u8];
                self.iface.read_regs(reg, &mut buf)$($await)*.map_err(Error::bus)?;
                Ok(buf[0])
            }

            $($async)? fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error<IFACE::Error>> {
                self.iface.write_reg(reg, value)$($await)*.map_err(Error::bus)
            }

            /// Read WHO_AM_I and select the matching [`Variant`].
            pub $($async)? fn probe(&mut self) -> Result<Variant, Error<IFACE::Error>> {
                let id = self.read_reg(REG_WHO_AM_I)$($await)*?;
                let variant = Variant::from_id(id).ok_or(Error::UnexpectedId(u32::from(id)))?;
                self.variant = variant;
                Ok(variant)
            }

            /// Apply range, mode and data rate and start converting. Returns
            /// the time until the first sample ([`turn_on_us`]).
            ///
            /// [`DataRate::Hz1620`] requires [`Mode::LowPower`]
            /// ([`Error::Unsupported`] otherwise).
            pub $($async)? fn configure(&mut self, config: Config) -> Result<WaitUs, Error<IFACE::Error>> {
                if matches!(config.rate, DataRate::Hz1620) && !matches!(config.mode, Mode::LowPower) {
                    return Err(Error::Unsupported);
                }
                self.config = config;
                self.ctrl6 = (self.ctrl6 & !CTRL6_ACTIVE_LOW)
                    | if config.int_active_low { CTRL6_ACTIVE_LOW } else { 0 };
                self.write_reg(REG_CTRL4, ctrl4(&config))$($await)*?;
                self.write_reg(REG_CTRL6, self.ctrl6)$($await)*?;
                self.power_up()$($await)*
            }

            $($async)? fn power_up(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
                if matches!(self.config.mode, Mode::HighResolution) {
                    // Datasheet note to Table 10: reset the filter block by
                    // reading REFERENCE before leaving power-down in HR mode.
                    self.read_reg(REG_REFERENCE)$($await)*?;
                }
                self.write_reg(REG_CTRL1, ctrl1(&self.config))$($await)*?;
                self.active = true;
                Ok(turn_on_us(self.config.mode, self.config.rate))
            }

            /// Enable the wake-up (motion) interrupt on `config.pin`. Replaces
            /// any orientation interrupt on the same pin.
            pub $($async)? fn configure_wake_up(&mut self, config: &WakeUpConfig) -> Result<(), Error<IFACE::Error>> {
                let ths = threshold_reg(config.threshold_mg, self.variant, self.config.range);
                self.write_generator(
                    config.pin,
                    CFG_XYZ_HIGH_OR,
                    ths,
                    config.duration,
                    config.latch,
                    config.high_pass,
                    false,
                )$($await)*
            }

            /// Enable the 6D/4D orientation interrupt on `config.pin`.
            /// Replaces any wake-up interrupt on the same pin.
            pub $($async)? fn configure_orientation(&mut self, config: &OrientationConfig) -> Result<(), Error<IFACE::Error>> {
                let axes = if config.four_d { 0x0F } else { 0x3F };
                let mode = match config.mode {
                    OrientationMode::Movement => 0x40,
                    OrientationMode::Position => 0xC0,
                };
                let ths = threshold_reg(config.threshold_mg, self.variant, self.config.range);
                self.write_generator(
                    config.pin,
                    mode | axes,
                    ths,
                    config.duration,
                    config.latch,
                    false,
                    config.four_d,
                )$($await)*
            }

            #[allow(clippy::too_many_arguments)]
            $($async)? fn write_generator(
                &mut self,
                pin: IntPin,
                cfg: u8,
                ths: u8,
                duration: u8,
                latch: bool,
                high_pass: bool,
                four_d: bool,
            ) -> Result<(), Error<IFACE::Error>> {
                let base = pin.base();
                self.ctrl2 = (self.ctrl2 & !pin.high_pass_bit())
                    | if high_pass { pin.high_pass_bit() } else { 0 };
                self.ctrl5 = (self.ctrl5 & !(pin.latch_bit() | pin.d4d_bit()))
                    | if latch { pin.latch_bit() } else { 0 }
                    | if four_d { pin.d4d_bit() } else { 0 };
                self.write_reg(REG_CTRL2, self.ctrl2)$($await)*?;
                self.write_reg(base + 2, ths)$($await)*?;
                self.write_reg(base + 3, duration & 0x7F)$($await)*?;
                self.write_reg(REG_CTRL5, self.ctrl5)$($await)*?;
                if high_pass {
                    // HPM = 00: reading REFERENCE resets the filter to the
                    // current acceleration.
                    self.read_reg(REG_REFERENCE)$($await)*?;
                }
                self.write_reg(base, cfg)$($await)*?;
                self.route(pin, true)$($await)*
            }

            $($async)? fn route(&mut self, pin: IntPin, enable: bool) -> Result<(), Error<IFACE::Error>> {
                match pin {
                    IntPin::Int1 => {
                        // I1_IA1
                        self.ctrl3 = (self.ctrl3 & !0x40) | if enable { 0x40 } else { 0 };
                        self.write_reg(REG_CTRL3, self.ctrl3)$($await)*
                    }
                    IntPin::Int2 => {
                        let bit = self.variant.int2_route();
                        self.ctrl6 = (self.ctrl6 & !bit) | if enable { bit } else { 0 };
                        self.write_reg(REG_CTRL6, self.ctrl6)$($await)*
                    }
                }
            }

            /// Disable the wake-up / orientation interrupt on `pin`.
            pub $($async)? fn disable_interrupt(&mut self, pin: IntPin) -> Result<(), Error<IFACE::Error>> {
                self.write_reg(pin.base(), 0)$($await)*?;
                self.route(pin, false)$($await)*?;
                self.ctrl2 &= !pin.high_pass_bit();
                self.ctrl5 &= !(pin.latch_bit() | pin.d4d_bit());
                self.write_reg(REG_CTRL2, self.ctrl2)$($await)*?;
                self.write_reg(REG_CTRL5, self.ctrl5)$($await)*
            }

            /// Read INT1_SRC / INT2_SRC. Clears a latched interrupt and
            /// releases the pin.
            pub $($async)? fn read_interrupt_source(&mut self, pin: IntPin) -> Result<InterruptSource, Error<IFACE::Error>> {
                Ok(InterruptSource(self.read_reg(pin.base() + 1)$($await)*?))
            }
        }

        impl<IFACE: $Iface> $Sensor for Lis2dh<IFACE> {
            type BusError = IFACE::Error;
            type Measurement = Measurement;

            /// Power up if asleep (returns the turn-on time), else `0`.
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
                if self.active {
                    Ok(0)
                } else {
                    self.power_up()$($await)*
                }
            }

            /// Reads STATUS_REG and the output registers in one burst.
            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<IFACE::Error>> {
                let mut buf = [0u8; 7];
                self.iface.read_regs(REG_STATUS, &mut buf)$($await)*.map_err(Error::bus)?;
                if buf[0] & STATUS_ZYXDA == 0 {
                    return Err(Error::NotReady);
                }
                let mut frame = [0u8; 6];
                frame.copy_from_slice(&buf[1..]);
                Ok(Measurement {
                    acceleration: decode(&frame, self.variant, self.config.range, self.config.mode),
                })
            }
        }

        impl<IFACE: $Iface> $LowPower for Lis2dh<IFACE> {
            type BusError = IFACE::Error;

            /// Power-down mode (ODR = 0, 0.5 µA); configuration is kept but
            /// interrupts stop.
            $($async)? fn sleep(&mut self) -> Result<(), Error<IFACE::Error>> {
                self.write_reg(REG_CTRL1, CTRL1_XYZ)$($await)*?;
                self.active = false;
                Ok(())
            }

            /// Restore the configured ODR; returns the turn-on time.
            $($async)? fn wake(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
                self.power_up()$($await)*
            }
        }
    };
}

driver!(
    Interface,
    zigbee_sensor::blocking::Sensor,
    zigbee_sensor::blocking::LowPower,
    [],
    []
);

/// Async driver.
pub mod asynch {
    use super::*;

    driver!(
        AsyncInterface,
        zigbee_sensor::asynch::Sensor,
        zigbee_sensor::asynch::LowPower,
        [async],
        [.await]
    );
}

#[cfg(test)]
mod tests;
