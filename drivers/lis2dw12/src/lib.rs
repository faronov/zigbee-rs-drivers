//! ST LIS2DW12 ultra-low-power three-axis accelerometer (I2C or SPI).
//!
//! Two conversion schemes are supported (see [`Conversion`]):
//!
//! * [`Conversion::OnDemand`] (default): single data conversion on demand
//!   triggered over the bus. [`Sensor::start_measurement`] sets
//!   `CTRL3.SLP_MODE_1` and returns the turn-on time of the selected
//!   low-power mode (1.2–3.55 ms); the part converts once and powers down
//!   again by itself. [`Sensor::read_measurement`] returns
//!   [`Error::NotReady`] until `STATUS.DRDY` is set.
//! * [`Conversion::Continuous`]: free-running at the selected output data
//!   rate. Required for the wake-up, sleep-change and 6D interrupts, which
//!   only run while the part converts.
//!
//! For a sleepy Zigbee end device, run continuously in
//! [`PowerMode::LowPower1`] at 12.5 Hz (1 µA) or 1.6 Hz (0.38 µA) with
//! [`WakeUpConfig`] routed to INT1 and sleep the MCU until the pin fires.
//! [`LowPower::sleep`] is a full power-down (50 nA) and stops the detector.
//!
//! Reference: ST "LIS2DW12 datasheet", DocID029682 Rev 4 (2017).
//!
//! [`Sensor::start_measurement`]: zigbee_sensor::blocking::Sensor::start_measurement
//! [`Sensor::read_measurement`]: zigbee_sensor::blocking::Sensor::read_measurement
//! [`LowPower::sleep`]: zigbee_sensor::blocking::LowPower::sleep
//!
//! ```ignore
//! let mut acc = Lis2dw12::new_i2c(i2c, ADDRESS_SA0_LOW);
//! acc.probe()?;
//! acc.configure(Config { conversion: Conversion::Continuous, ..Default::default() })?;
//! acc.configure_wake_up(&WakeUpConfig { threshold_mg: 63, ..Default::default() })?;
//! // ... sleep until INT1, then:
//! let src = acc.read_interrupt_source()?; // clears latched events
//! ```

#![no_std]

mod interface;

pub use interface::{AsyncInterface, I2cInterface, Interface, SpiInterface};
use zigbee_sensor::{Error, Quantity, Readings, Vector3, WaitUs};

/// I2C address with SA0 low.
pub const ADDRESS_SA0_LOW: u8 = 0x18;
/// I2C address with SA0 high.
pub const ADDRESS_SA0_HIGH: u8 = 0x19;

/// WHO_AM_I value.
pub const WHO_AM_I: u8 = 0x44;

const REG_WHO_AM_I: u8 = 0x0F;
const REG_CTRL1: u8 = 0x20;
const REG_CTRL2: u8 = 0x21;
const REG_CTRL3: u8 = 0x22;
const REG_CTRL4: u8 = 0x23;
const REG_CTRL5: u8 = 0x24;
const REG_CTRL6: u8 = 0x25;
const REG_STATUS: u8 = 0x27;
const REG_TAP_THS_X: u8 = 0x30;
const REG_WAKE_UP_THS: u8 = 0x34;
const REG_WAKE_UP_DUR: u8 = 0x35;
const REG_WAKE_UP_SRC: u8 = 0x38;
const REG_CTRL7: u8 = 0x3F;

const CTRL2_BDU: u8 = 0x08;
const CTRL2_IF_ADD_INC: u8 = 0x04;
const CTRL3_LIR: u8 = 0x10;
const CTRL3_H_LACTIVE: u8 = 0x08;
const CTRL3_SLP_MODE_SEL: u8 = 0x02;
const CTRL3_SLP_MODE_1: u8 = 0x01;
const CTRL4_INT1_6D: u8 = 0x80;
const CTRL4_INT1_WU: u8 = 0x20;
const CTRL5_INT2_SLEEP_CHG: u8 = 0x40;
const CTRL6_LOW_NOISE: u8 = 0x04;
const CTRL7_INTERRUPTS_ENABLE: u8 = 0x20;
const WAKE_UP_THS_SLEEP_ON: u8 = 0x40;
const STATUS_DRDY: u8 = 0x01;

/// Full-scale range (CTRL6 FS\[1:0\]).
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

    /// Full scale in mg.
    pub const fn full_scale_mg(self) -> u32 {
        2000 << (self as u32)
    }
}

/// Power mode (CTRL1 MODE / LP_MODE). Current figures at 12.5 Hz
/// (datasheet Table 11).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PowerMode {
    /// Low-power mode 1: 12-bit, 1 µA, 4.5 mg RMS noise.
    #[default]
    LowPower1,
    /// Low-power mode 2: 14-bit, 1.6 µA.
    LowPower2,
    /// Low-power mode 3: 14-bit, 2.5 µA.
    LowPower3,
    /// Low-power mode 4: 14-bit, 4 µA, 1.3 mg RMS noise.
    LowPower4,
    /// High-performance: 14-bit, 90 µA, up to 1.6 kHz. Continuous only.
    HighPerformance,
}

impl PowerMode {
    const fn lp_bits(self) -> u8 {
        match self {
            Self::LowPower1 | Self::HighPerformance => 0,
            Self::LowPower2 => 1,
            Self::LowPower3 => 2,
            Self::LowPower4 => 3,
        }
    }

    /// Output resolution in bits.
    pub const fn resolution_bits(self) -> u32 {
        match self {
            Self::LowPower1 => 12,
            _ => 14,
        }
    }

    /// Single-conversion turn-on time T_on (datasheet §3.2.2, typical),
    /// µs. High-performance mode has no on-demand conversion.
    pub const fn on_demand_us(self) -> WaitUs {
        match self {
            Self::LowPower1 => 1_200,
            Self::LowPower2 => 1_700,
            Self::LowPower3 => 2_300,
            Self::LowPower4 | Self::HighPerformance => 3_550,
        }
    }
}

/// Output data rate (CTRL1 ODR\[3:0\]). Low-power modes are capped at
/// 200 Hz.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DataRate {
    /// 1.6 Hz in low-power modes, 12.5 Hz in high-performance mode.
    Hz1_6 = 1,
    /// 12.5 Hz.
    #[default]
    Hz12_5 = 2,
    /// 25 Hz.
    Hz25 = 3,
    /// 50 Hz.
    Hz50 = 4,
    /// 100 Hz.
    Hz100 = 5,
    /// 200 Hz.
    Hz200 = 6,
    /// 400 Hz (200 Hz in low-power modes).
    Hz400 = 7,
    /// 800 Hz (200 Hz in low-power modes).
    Hz800 = 8,
    /// 1.6 kHz (200 Hz in low-power modes).
    Hz1600 = 9,
}

impl DataRate {
    /// Sample period in µs, rounded up.
    pub const fn period_us(self, mode: PowerMode) -> u32 {
        let hp = matches!(mode, PowerMode::HighPerformance);
        match self {
            Self::Hz1_6 if hp => 80_000,
            Self::Hz1_6 => 625_000,
            Self::Hz12_5 => 80_000,
            Self::Hz25 => 40_000,
            Self::Hz50 => 20_000,
            Self::Hz100 => 10_000,
            Self::Hz200 => 5_000,
            _ if !hp => 5_000,
            Self::Hz400 => 2_500,
            Self::Hz800 => 1_250,
            Self::Hz1600 => 625,
        }
    }
}

/// How samples are produced.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Conversion {
    /// Single data conversion on demand (MODE = 10, SLP_MODE_SEL = 1):
    /// one sample per [`Sensor::start_measurement`], power-down otherwise.
    /// Low-power modes only. No motion detection between conversions.
    ///
    /// [`Sensor::start_measurement`]: zigbee_sensor::blocking::Sensor::start_measurement
    #[default]
    OnDemand,
    /// Free-running at [`Config::rate`].
    Continuous,
}

/// Measurement configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// Full-scale range.
    pub range: Range,
    /// Power mode / resolution.
    pub mode: PowerMode,
    /// Output data rate (also sets the filter bandwidth in on-demand mode).
    pub rate: DataRate,
    /// On-demand or continuous conversion.
    pub conversion: Conversion,
    /// CTRL6.LOW_NOISE: lower noise for a little more current.
    pub low_noise: bool,
    /// INT1/INT2 active low (default active high).
    pub int_active_low: bool,
}

/// Time from power-down to the first continuous sample. Not specified in
/// the datasheet: this driver waits the single-conversion T_on plus one
/// sample period (low-power modes) or two sample periods (high-performance,
/// Table 14 discards up to two samples).
pub const fn turn_on_us(mode: PowerMode, rate: DataRate) -> WaitUs {
    let period = rate.period_us(mode);
    match mode {
        PowerMode::HighPerformance => 2 * period,
        _ => mode.on_demand_us() + period,
    }
}

/// Interrupt output pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum IntPin {
    /// INT1: wake-up event (`INT1_WU`) and 6D (`INT1_6D`).
    #[default]
    Int1,
    /// INT2: activity/inactivity change (`INT2_SLEEP_CHG`); requires
    /// [`WakeUpConfig::auto_sleep`].
    Int2,
}

/// Wake-up (motion) interrupt: any axis of the high-pass filtered
/// acceleration above the threshold for `duration` samples.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WakeUpConfig {
    /// Threshold in mg; 1 LSB = full scale / 64 (31.25 mg at ±2 g),
    /// at most 63 steps.
    pub threshold_mg: u16,
    /// Minimum event duration in samples (1/ODR each), 0..=3.
    pub duration: u8,
    /// [`IntPin::Int1`]: every wake-up event. [`IntPin::Int2`]: only the
    /// sleep ↔ wake transitions of [`Self::auto_sleep`].
    pub pin: IntPin,
    /// Latch interrupts until [`Lis2dw12::read_interrupt_source`]
    /// (`CTRL3.LIR`, shared by all interrupt functions).
    pub latch: bool,
    /// Activity/inactivity: after `512 × n / ODR` without motion
    /// (n = 0..=15, 0 means 16/ODR) the part drops to 12.5 Hz in the
    /// selected low-power mode and returns to the configured ODR on the next
    /// wake-up event. `None` disables it.
    pub auto_sleep: Option<u8>,
}

impl Default for WakeUpConfig {
    fn default() -> Self {
        Self {
            threshold_mg: 63,
            duration: 0,
            pin: IntPin::Int1,
            latch: true,
            auto_sleep: None,
        }
    }
}

/// 6D threshold angle (TAP_THS_X 6D_THS\[1:0\]).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum OrientationThreshold {
    /// 80°.
    Deg80,
    /// 70°.
    Deg70,
    /// 60°.
    #[default]
    Deg60,
    /// 50°.
    Deg50,
}

/// 6D/4D orientation interrupt on INT1 (portrait/landscape/face up/down).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct OrientationConfig {
    /// Angle threshold.
    pub threshold: OrientationThreshold,
    /// Ignore the Z axis (4D portrait/landscape only).
    pub four_d: bool,
}

/// WAKE_UP_SRC, TAP_SRC, SIXD_SRC and ALL_INT_SRC, read in one burst.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct InterruptSource {
    /// WAKE_UP_SRC (38h).
    pub wake_up_src: u8,
    /// TAP_SRC (39h).
    pub tap_src: u8,
    /// SIXD_SRC (3Ah): ZH ZL YH YL XH XL in bits 5..0.
    pub sixd_src: u8,
    /// ALL_INT_SRC (3Bh).
    pub all_int_src: u8,
}

impl InterruptSource {
    /// Wake-up event (WU_IA).
    pub const fn wake_up(self) -> bool {
        self.wake_up_src & 0x08 != 0
    }
    /// Wake-up on X.
    pub const fn x_wake_up(self) -> bool {
        self.wake_up_src & 0x04 != 0
    }
    /// Wake-up on Y.
    pub const fn y_wake_up(self) -> bool {
        self.wake_up_src & 0x02 != 0
    }
    /// Wake-up on Z.
    pub const fn z_wake_up(self) -> bool {
        self.wake_up_src & 0x01 != 0
    }
    /// The part is in the inactivity (sleep) state.
    pub const fn sleep_state(self) -> bool {
        self.wake_up_src & 0x10 != 0
    }
    /// Activity ↔ inactivity transition (SLEEP_CHANGE_IA).
    pub const fn sleep_change(self) -> bool {
        self.all_int_src & 0x20 != 0
    }
    /// Orientation change (6D_IA).
    pub const fn orientation_change(self) -> bool {
        self.sixd_src & 0x40 != 0
    }
    /// Free-fall event (FF_IA).
    pub const fn free_fall(self) -> bool {
        self.wake_up_src & 0x20 != 0
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

/// Convert one left-justified 16-bit output word to milli-g
/// (0.244 mg/digit at ±2 g with 14 bits, 0.976 mg/digit with 12 bits).
pub const fn raw_to_mg(raw: i16, range: Range, mode: PowerMode) -> i32 {
    let bits = mode.resolution_bits();
    let counts = (raw as i32) >> (16 - bits);
    // full scale (mg) / 2^(bits-1) per digit.
    counts * (range.full_scale_mg() as i32) / (1 << (bits - 1))
}

/// Decode OUT_X_L..OUT_Z_H to milli-g.
pub fn decode(frame: &[u8; 6], range: Range, mode: PowerMode) -> Vector3 {
    let axis = |i: usize| raw_to_mg(i16::from_le_bytes([frame[i], frame[i + 1]]), range, mode);
    Vector3 {
        x: axis(0),
        y: axis(2),
        z: axis(4),
    }
}

/// WK_THS value for a threshold in mg (1 LSB = FS/64, rounded, 0..=63).
pub fn threshold_reg(threshold_mg: u16, range: Range) -> u8 {
    let fs = range.full_scale_mg();
    let steps = (u32::from(threshold_mg) * 64 + fs / 2) / fs;
    steps.min(63) as u8
}

const fn ctrl1(config: &Config) -> u8 {
    let mode = match (config.conversion, config.mode) {
        (_, PowerMode::HighPerformance) => 0x04,
        (Conversion::OnDemand, _) => 0x08,
        (Conversion::Continuous, _) => 0x00,
    };
    ((config.rate as u8) << 4) | mode | config.mode.lp_bits()
}

macro_rules! driver {
    ($Iface:ident, $Sensor:path, $LowPower:path, [$($async:tt)?], [$($await:tt)*]) => {
        /// LIS2DW12 driver.
        pub struct Lis2dw12<IFACE> {
            iface: IFACE,
            config: Config,
            ctrl3: u8,
            ctrl4: u8,
            ctrl5: u8,
            active: bool,
        }

        impl<IFACE> Lis2dw12<IFACE> {
            /// Create a driver over a register interface; no bus traffic.
            pub const fn new(iface: IFACE) -> Self {
                Self {
                    iface,
                    config: Config {
                        range: Range::G2,
                        mode: PowerMode::LowPower1,
                        rate: DataRate::Hz12_5,
                        conversion: Conversion::OnDemand,
                        low_noise: false,
                        int_active_low: false,
                    },
                    ctrl3: CTRL3_SLP_MODE_SEL,
                    ctrl4: 0,
                    ctrl5: 0,
                    active: false,
                }
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

        impl<I2C> Lis2dw12<I2cInterface<I2C>> {
            /// Driver on an I2C bus ([`ADDRESS_SA0_LOW`] or [`ADDRESS_SA0_HIGH`]).
            pub const fn new_i2c(i2c: I2C, address: u8) -> Self {
                Self::new(I2cInterface::new(i2c, address))
            }
        }

        impl<SPI> Lis2dw12<SpiInterface<SPI>> {
            /// Driver on an SPI device (mode 0 or 3, up to 10 MHz).
            pub const fn new_spi(spi: SPI) -> Self {
                Self::new(SpiInterface::new(spi))
            }
        }

        impl<IFACE: $Iface> Lis2dw12<IFACE> {
            $($async)? fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error<IFACE::Error>> {
                self.iface.write_reg(reg, value)$($await)*.map_err(Error::bus)
            }

            /// Check WHO_AM_I.
            pub $($async)? fn probe(&mut self) -> Result<(), Error<IFACE::Error>> {
                let mut id = [0u8];
                self.iface.read_regs(REG_WHO_AM_I, &mut id)$($await)*.map_err(Error::bus)?;
                if id[0] != WHO_AM_I {
                    return Err(Error::UnexpectedId(u32::from(id[0])));
                }
                Ok(())
            }

            /// Apply the configuration. In continuous mode the part starts
            /// converting and the turn-on time is returned; in on-demand mode
            /// it stays powered down and `0` is returned.
            ///
            /// [`Conversion::OnDemand`] with [`PowerMode::HighPerformance`]
            /// is [`Error::Unsupported`].
            pub $($async)? fn configure(&mut self, config: Config) -> Result<WaitUs, Error<IFACE::Error>> {
                if matches!(config.conversion, Conversion::OnDemand)
                    && matches!(config.mode, PowerMode::HighPerformance)
                {
                    return Err(Error::Unsupported);
                }
                self.config = config;
                self.ctrl3 = (self.ctrl3 & CTRL3_LIR)
                    | if config.int_active_low { CTRL3_H_LACTIVE } else { 0 }
                    | match config.conversion {
                        Conversion::OnDemand => CTRL3_SLP_MODE_SEL,
                        Conversion::Continuous => 0,
                    };
                let ctrl6 = (config.range.bits() << 4)
                    | if config.low_noise { CTRL6_LOW_NOISE } else { 0 };
                self.write_reg(REG_CTRL2, CTRL2_BDU | CTRL2_IF_ADD_INC)$($await)*?;
                self.write_reg(REG_CTRL6, ctrl6)$($await)*?;
                self.write_reg(REG_CTRL3, self.ctrl3)$($await)*?;
                self.power_up()$($await)*
            }

            $($async)? fn power_up(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
                self.write_reg(REG_CTRL1, ctrl1(&self.config))$($await)*?;
                self.active = true;
                Ok(match self.config.conversion {
                    Conversion::OnDemand => 0,
                    Conversion::Continuous => turn_on_us(self.config.mode, self.config.rate),
                })
            }

            $($async)? fn write_latch(&mut self, latch: bool) -> Result<(), Error<IFACE::Error>> {
                self.ctrl3 = (self.ctrl3 & !CTRL3_LIR) | if latch { CTRL3_LIR } else { 0 };
                self.write_reg(REG_CTRL3, self.ctrl3)$($await)*
            }

            $($async)? fn write_routing(&mut self) -> Result<(), Error<IFACE::Error>> {
                self.write_reg(REG_CTRL4, self.ctrl4)$($await)*?;
                self.write_reg(REG_CTRL5, self.ctrl5)$($await)*?;
                let ctrl7 = if self.ctrl4 | self.ctrl5 != 0 { CTRL7_INTERRUPTS_ENABLE } else { 0 };
                self.write_reg(REG_CTRL7, ctrl7)$($await)*
            }

            /// Enable the wake-up interrupt (and optionally activity/inactivity).
            /// Needs [`Conversion::Continuous`].
            ///
            /// [`IntPin::Int2`] without [`WakeUpConfig::auto_sleep`] is
            /// [`Error::Unsupported`]: the wake-up event itself can only be
            /// routed to INT1.
            pub $($async)? fn configure_wake_up(&mut self, config: &WakeUpConfig) -> Result<(), Error<IFACE::Error>> {
                if matches!(config.pin, IntPin::Int2) && config.auto_sleep.is_none() {
                    return Err(Error::Unsupported);
                }
                let ths = threshold_reg(config.threshold_mg, self.config.range)
                    | if config.auto_sleep.is_some() { WAKE_UP_THS_SLEEP_ON } else { 0 };
                let dur = ((config.duration.min(3)) << 5) | (config.auto_sleep.unwrap_or(0) & 0x0F);
                self.write_reg(REG_WAKE_UP_DUR, dur)$($await)*?;
                self.write_reg(REG_WAKE_UP_THS, ths)$($await)*?;
                self.write_latch(config.latch)$($await)*?;
                match config.pin {
                    IntPin::Int1 => self.ctrl4 |= CTRL4_INT1_WU,
                    IntPin::Int2 => self.ctrl5 |= CTRL5_INT2_SLEEP_CHG,
                }
                self.write_routing()$($await)*
            }

            /// Enable the 6D/4D orientation interrupt on INT1. Needs
            /// [`Conversion::Continuous`]. Latching follows the last
            /// [`WakeUpConfig::latch`] (`CTRL3.LIR`).
            pub $($async)? fn configure_orientation(&mut self, config: &OrientationConfig) -> Result<(), Error<IFACE::Error>> {
                let ths = ((config.threshold as u8) << 5) | if config.four_d { 0x80 } else { 0 };
                self.write_reg(REG_TAP_THS_X, ths)$($await)*?;
                self.ctrl4 |= CTRL4_INT1_6D;
                self.write_routing()$($await)*
            }

            /// Remove every interrupt routed to `pin`; when neither pin has
            /// a source left, `CTRL7.INTERRUPTS_ENABLE` is cleared too.
            /// Thresholds and auto-sleep settings are kept.
            pub $($async)? fn disable_interrupt(&mut self, pin: IntPin) -> Result<(), Error<IFACE::Error>> {
                match pin {
                    IntPin::Int1 => self.ctrl4 = 0,
                    IntPin::Int2 => self.ctrl5 = 0,
                }
                self.write_routing()$($await)*
            }

            /// Read the event sources in one burst. Reading ALL_INT_SRC
            /// clears every latched event and releases the pins.
            pub $($async)? fn read_interrupt_source(&mut self) -> Result<InterruptSource, Error<IFACE::Error>> {
                let mut buf = [0u8; 4];
                self.iface.read_regs(REG_WAKE_UP_SRC, &mut buf)$($await)*.map_err(Error::bus)?;
                Ok(InterruptSource {
                    wake_up_src: buf[0],
                    tap_src: buf[1],
                    sixd_src: buf[2],
                    all_int_src: buf[3],
                })
            }
        }

        impl<IFACE: $Iface> $Sensor for Lis2dw12<IFACE> {
            type BusError = IFACE::Error;
            type Measurement = Measurement;

            /// On demand: trigger one conversion and return T_on. Continuous:
            /// power up if asleep (returns the turn-on time), else `0`.
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
                if !self.active {
                    let wait = self.power_up()$($await)*?;
                    if matches!(self.config.conversion, Conversion::Continuous) {
                        return Ok(wait);
                    }
                }
                match self.config.conversion {
                    Conversion::Continuous => Ok(0),
                    Conversion::OnDemand => {
                        self.write_reg(REG_CTRL3, self.ctrl3 | CTRL3_SLP_MODE_1)$($await)*?;
                        Ok(self.config.mode.on_demand_us())
                    }
                }
            }

            /// Reads STATUS and the output registers in one burst.
            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<IFACE::Error>> {
                let mut buf = [0u8; 7];
                self.iface.read_regs(REG_STATUS, &mut buf)$($await)*.map_err(Error::bus)?;
                if buf[0] & STATUS_DRDY == 0 {
                    return Err(Error::NotReady);
                }
                let mut frame = [0u8; 6];
                frame.copy_from_slice(&buf[1..]);
                Ok(Measurement {
                    acceleration: decode(&frame, self.config.range, self.config.mode),
                })
            }
        }

        impl<IFACE: $Iface> $LowPower for Lis2dw12<IFACE> {
            type BusError = IFACE::Error;

            /// Power-down (ODR = 0, 50 nA); configuration is kept but
            /// interrupts and on-demand triggers stop.
            $($async)? fn sleep(&mut self) -> Result<(), Error<IFACE::Error>> {
                self.write_reg(REG_CTRL1, 0)$($await)*?;
                self.active = false;
                Ok(())
            }

            /// Restore CTRL1; returns the continuous turn-on time or `0` in
            /// on-demand mode.
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
