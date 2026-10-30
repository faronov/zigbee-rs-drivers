//! Analog Devices ADXL367 nanopower three-axis accelerometer (I2C or SPI).
//!
//! The part converts continuously ([`PowerMode`]: 0.89 µA normal at
//! 100 Hz, 180 nA wake-up mode); there is no single-shot mode.
//! [`Sensor::start_measurement`] leaves standby and
//! [`Sensor::read_measurement`] returns [`Error::NotReady`] until
//! `STATUS.DATA_READY` is set. [`LowPower::sleep`] is standby (40 nA).
//!
//! For a sleepy Zigbee end device use referenced activity detection
//! ([`WakeUpConfig`]) in [`PowerMode::WakeUp`] (1.5–12 samples/s), or in
//! normal mode with inactivity, [`Linking::Loop`] and
//! [`WakeUpConfig::auto_sleep`] so the part drops to wake-up mode by itself
//! when still. Single/double tap ([`TapConfig`]) is also available.
//!
//! Registers 0x00–0x2D may only be changed in standby (datasheet
//! "Operating Modes"); the driver drops to standby and resumes around
//! those writes.
//!
//! Reference: Analog Devices "ADXL367 Data Sheet" Rev. A.
//!
//! [`Sensor::start_measurement`]: zigbee_sensor::blocking::Sensor::start_measurement
//! [`Sensor::read_measurement`]: zigbee_sensor::blocking::Sensor::read_measurement
//! [`LowPower::sleep`]: zigbee_sensor::blocking::LowPower::sleep
//!
//! ```ignore
//! let mut acc = Adxl367::new_i2c(i2c, ADDRESS_ASEL_HIGH);
//! acc.probe()?;
//! acc.configure(Config { mode: PowerMode::WakeUp, ..Default::default() })?;
//! acc.configure_wake_up(&WakeUpConfig { threshold_mg: 250, ..Default::default() })?;
//! // ... sleep until INT1, then:
//! let src = acc.read_interrupt_source()?; // reading STATUS clears ACT/INACT
//! ```

#![no_std]

mod interface;

pub use interface::{AsyncInterface, I2cInterface, Interface, SpiInterface};
use zigbee_sensor::{Error, Quantity, Readings, Vector3, WaitUs};

/// I2C address with ASEL high.
pub const ADDRESS_ASEL_HIGH: u8 = 0x53;
/// I2C address with ASEL grounded.
pub const ADDRESS_ASEL_LOW: u8 = 0x1D;

/// DEVID_AD, DEVID_MST and PART_ID.
pub const DEVICE_ID: [u8; 3] = [0xAD, 0x1D, 0xF7];

const REG_DEVID_AD: u8 = 0x00;
const REG_STATUS: u8 = 0x0B;
const REG_THRESH_ACT_H: u8 = 0x20;
const REG_THRESH_ACT_L: u8 = 0x21;
const REG_TIME_ACT: u8 = 0x22;
const REG_THRESH_INACT_H: u8 = 0x23;
const REG_THRESH_INACT_L: u8 = 0x24;
const REG_TIME_INACT_H: u8 = 0x25;
const REG_TIME_INACT_L: u8 = 0x26;
const REG_ACT_INACT_CTL: u8 = 0x27;
const REG_INTMAP1_LOWER: u8 = 0x2A;
const REG_INTMAP2_LOWER: u8 = 0x2B;
const REG_FILTER_CTL: u8 = 0x2C;
const REG_POWER_CTL: u8 = 0x2D;
const REG_TAP_THRESH: u8 = 0x2F;
const REG_TAP_DUR: u8 = 0x30;
const REG_TAP_LATENT: u8 = 0x31;
const REG_TAP_WINDOW: u8 = 0x32;
const REG_TIMER_CTL: u8 = 0x39;
const REG_INTMAP1_UPPER: u8 = 0x3A;
const REG_INTMAP2_UPPER: u8 = 0x3B;
const REG_AXIS_MASK: u8 = 0x43;
const REG_STATUS2: u8 = 0x45;

const ACT_EN: u8 = 0x01;
const ACT_REF: u8 = 0x02;
const INACT_EN: u8 = 0x04;
const INACT_REF: u8 = 0x08;
const FILTER_I2C_HS: u8 = 0x20;
const POWER_CTL_LOW_NOISE: u8 = 0x10;
const POWER_CTL_WAKEUP: u8 = 0x08;
const POWER_CTL_AUTOSLEEP: u8 = 0x04;
const POWER_CTL_MEASURE: u8 = 0x02;
const INTMAP_INT_LOW: u8 = 0x80;
const INTMAP_AWAKE: u8 = 0x40;
const INTMAP_INACT: u8 = 0x20;
const INTMAP_ACT: u8 = 0x10;
const INTMAP_TAP_TWO: u8 = 0x02;
const INTMAP_TAP_ONE: u8 = 0x01;

const STATUS_ERR_USER_REGS: u8 = 0x80;
const STATUS_AWAKE: u8 = 0x40;
const STATUS_INACT: u8 = 0x20;
const STATUS_ACT: u8 = 0x10;
const STATUS_DATA_READY: u8 = 0x01;
const STATUS2_TAP_TWO: u8 = 0x02;
const STATUS2_TAP_ONE: u8 = 0x01;

/// Measurement range (FILTER_CTL RANGE). 14-bit data: 0.25/0.5/1 mg/LSB.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Range {
    /// ±2 g, 0.25 mg/LSB.
    #[default]
    G2,
    /// ±4 g, 0.5 mg/LSB.
    G4,
    /// ±8 g, 1 mg/LSB.
    G8,
}

impl Range {
    /// LSB per mg is `4 >> shift`.
    const fn shift(self) -> u32 {
        self as u32
    }
}

/// Output data rate (FILTER_CTL ODR, filter at ODR/2). Ignored in
/// [`PowerMode::WakeUp`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DataRate {
    /// 12.5 Hz.
    Hz12_5,
    /// 25 Hz.
    Hz25,
    /// 50 Hz.
    Hz50,
    /// 100 Hz (0.89 µA in normal mode, reset default).
    #[default]
    Hz100,
    /// 200 Hz.
    Hz200,
    /// 400 Hz.
    Hz400,
}

/// Power / noise trade-off (POWER_CTL NOISE and WAKEUP).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PowerMode {
    /// Normal (low power) operation, 0.89 µA at 100 Hz.
    #[default]
    Normal,
    /// Low noise, 1.77 µA at 100 Hz.
    LowNoise,
    /// Wake-up mode: a few samples per second ([`Config::wake_up_rate`]),
    /// 180 nA at 1.5 samples/s. The activity timer is ignored.
    WakeUp,
}

/// Sample interval in wake-up mode (TIMER_CTL WAKEUP_RATE).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum WakeUpRate {
    /// 80 ms (about 12 samples/s, reset default).
    #[default]
    Ms80,
    /// 160 ms (about 6 samples/s).
    Ms160,
    /// 320 ms (about 3 samples/s).
    Ms320,
    /// 640 ms (about 1.5 samples/s, lowest current).
    Ms640,
}

impl WakeUpRate {
    /// Sample interval in µs.
    pub const fn period_us(self) -> u32 {
        80_000 << (self as u32)
    }
}

/// Measurement configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Measurement range.
    pub range: Range,
    /// Output data rate (measurement mode).
    pub rate: DataRate,
    /// Power / noise mode.
    pub mode: PowerMode,
    /// Sample interval in [`PowerMode::WakeUp`] and during autosleep.
    pub wake_up_rate: WakeUpRate,
    /// INT1/INT2 active low (INTMAPx_LOWER.INT_LOW, default active high).
    pub int_active_low: bool,
}

impl Config {
    /// ±2 g, 100 Hz, normal mode, 80 ms wake-up rate, active-high
    /// interrupts.
    pub const DEFAULT: Self = Self {
        range: Range::G2,
        rate: DataRate::Hz100,
        mode: PowerMode::Normal,
        wake_up_rate: WakeUpRate::Ms80,
        int_active_low: false,
    };
}

impl Default for Config {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Settling time after entering measurement mode (datasheet "Operating
/// Modes": 100 ms; Table 1 gives 31 ms typical at 100 Hz).
pub const SETTLE_US: u32 = 100_000;

/// Measurement command to valid data. In wake-up mode the first sample
/// takes one interval plus up to 10 ms extra (datasheet "Wake-Up Mode").
pub const fn turn_on_us(mode: PowerMode, wake_up_rate: WakeUpRate) -> WaitUs {
    match mode {
        PowerMode::WakeUp => {
            let first = wake_up_rate.period_us() + 10_000;
            if first > SETTLE_US { first } else { SETTLE_US }
        }
        _ => SETTLE_US,
    }
}

/// Interrupt output pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum IntPin {
    /// INT1 (INTMAP1_LOWER/UPPER).
    #[default]
    Int1,
    /// INT2 (INTMAP2_LOWER/UPPER).
    Int2,
}

impl IntPin {
    const fn index(self) -> usize {
        self as usize
    }
}

/// How activity and inactivity interact (ACT_INACT_CTL LINKLOOP).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Linking {
    /// Both functions run concurrently; each event is latched until
    /// `read_interrupt_source`.
    #[default]
    Default,
    /// Alternate activity → inactivity → activity; each event is latched
    /// until `read_interrupt_source`.
    Linked,
    /// Alternate like `Linked` but acknowledged internally (not latched).
    /// Pair with [`WakeUpConfig::awake_output`] for a motion switch.
    Loop,
}

/// Inactivity detection: every axis within the threshold for `samples`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InactivityConfig {
    /// Threshold in mg (13-bit, 0.25/0.5/1 mg/LSB by range).
    pub threshold_mg: u16,
    /// Duration in samples (1/ODR each), 16-bit.
    pub samples: u16,
    /// Referenced (relative) instead of absolute comparison.
    pub referenced: bool,
}

impl Default for InactivityConfig {
    fn default() -> Self {
        Self {
            threshold_mg: 150,
            samples: 30,
            referenced: true,
        }
    }
}

/// Activity (motion) interrupt, optionally with inactivity and autosleep.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WakeUpConfig {
    /// Activity threshold in mg (13-bit, 0.25/0.5/1 mg/LSB by range).
    pub threshold_mg: u16,
    /// Consecutive samples above the threshold (TIME_ACT). Ignored in
    /// [`PowerMode::WakeUp`].
    pub duration: u8,
    /// Referenced (relative to the orientation when detection starts)
    /// instead of absolute comparison; removes the static 1 g.
    pub referenced: bool,
    /// Output pin.
    pub pin: IntPin,
    /// Inactivity detection; `None` disables it.
    pub inactivity: Option<InactivityConfig>,
    /// Activity/inactivity linking. `Linked` and `Loop` need `inactivity`.
    pub linking: Linking,
    /// POWER_CTL.AUTOSLEEP: drop to wake-up mode on inactivity and return to
    /// measurement mode on activity. Needs `Linked` or `Loop`.
    pub auto_sleep: bool,
    /// Route the AWAKE state (level, high while moving) to the pin instead
    /// of the ACT/INACT events.
    pub awake_output: bool,
}

impl Default for WakeUpConfig {
    fn default() -> Self {
        Self {
            threshold_mg: 250,
            duration: 0,
            referenced: true,
            pin: IntPin::Int1,
            inactivity: None,
            linking: Linking::Default,
            auto_sleep: false,
            awake_output: false,
        }
    }
}

/// Axis watched by tap detection (AXIS_MASK TAP_AXIS).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum TapAxis {
    /// X axis.
    X,
    /// Y axis.
    Y,
    /// Z axis.
    #[default]
    Z,
}

/// Single / double tap detection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TapConfig {
    /// Threshold in mg (62.5 mg/LSB, at most 15.9 g).
    pub threshold_mg: u16,
    /// Maximum time above the threshold, µs (625 µs/LSB).
    pub duration_us: u32,
    /// Double tap: quiet time after the first tap, µs (1.25 ms/LSB).
    pub latency_us: u32,
    /// Double tap: window for the second tap after `latency_us`, µs
    /// (1.25 ms/LSB).
    pub window_us: u32,
    /// Axis to watch.
    pub axis: TapAxis,
    /// Also detect (and route) double taps.
    pub double: bool,
    /// Output pin.
    pub pin: IntPin,
}

impl Default for TapConfig {
    fn default() -> Self {
        Self {
            threshold_mg: 3000,
            duration_us: 20_000,
            latency_us: 100_000,
            window_us: 250_000,
            axis: TapAxis::Z,
            double: false,
            pin: IntPin::Int1,
        }
    }
}

/// STATUS and STATUS2 registers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct InterruptSource {
    /// STATUS (0Bh), including ACT/INACT events consumed by earlier
    /// [`Sensor::read_measurement`] calls.
    ///
    /// [`Sensor::read_measurement`]: zigbee_sensor::blocking::Sensor::read_measurement
    pub status: u8,
    /// STATUS2 (45h).
    pub status2: u8,
}

impl InterruptSource {
    /// Activity event.
    pub const fn activity(self) -> bool {
        self.status & STATUS_ACT != 0
    }
    /// Inactivity event.
    pub const fn inactivity(self) -> bool {
        self.status & STATUS_INACT != 0
    }
    /// AWAKE state (only meaningful in linked/loop mode).
    pub const fn awake(self) -> bool {
        self.status & STATUS_AWAKE != 0
    }
    /// ERR_USER_REGS: an SEU upset the configuration, or the part has not
    /// been configured since reset.
    pub const fn config_error(self) -> bool {
        self.status & STATUS_ERR_USER_REGS != 0
    }
    /// Single tap.
    pub const fn single_tap(self) -> bool {
        self.status2 & STATUS2_TAP_ONE != 0
    }
    /// Double tap.
    pub const fn double_tap(self) -> bool {
        self.status2 & STATUS2_TAP_TWO != 0
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

/// Convert one 14-bit sample to mg (0.25/0.5/1 mg/LSB, truncated toward
/// zero).
pub const fn raw_to_mg(raw: i16, range: Range) -> i32 {
    ((raw as i32) << range.shift()) / 4
}

/// Decode XDATA_H..ZDATA_L (big-endian, 14 bits left-justified) to
/// milli-g.
pub fn decode(frame: &[u8; 6], range: Range) -> Vector3 {
    let axis = |i: usize| raw_to_mg(i16::from_be_bytes([frame[i], frame[i + 1]]) >> 2, range);
    Vector3 {
        x: axis(0),
        y: axis(2),
        z: axis(4),
    }
}

/// 13-bit THRESH_ACT / THRESH_INACT code for a threshold in mg (rounded,
/// at most 8191).
pub fn threshold_reg(threshold_mg: u16, range: Range) -> u16 {
    let shift = range.shift();
    let half = (1u32 << shift) >> 1;
    (((u32::from(threshold_mg) << 2) + half) >> shift).min(0x1FFF) as u16
}

/// Split a 13-bit threshold into the `_H` (bits 12:6) and `_L` (bits 5:0
/// in 7:2) register values.
pub const fn threshold_bytes(code: u16) -> [u8; 2] {
    [((code >> 6) & 0x7F) as u8, ((code & 0x3F) << 2) as u8]
}

fn scale(value: u32, step: u32) -> u8 {
    ((value + step / 2) / step).min(255) as u8
}

const fn pin_bits(map: u8, active_low: bool) -> u8 {
    (map & !INTMAP_INT_LOW) | if active_low { INTMAP_INT_LOW } else { 0 }
}

macro_rules! driver {
    ($Iface:path, $Sensor:path, $LowPower:path, [$($async:tt)?], [$($await:tt)*]) => {
        /// ADXL367 driver over a register [`Interface`].
        pub struct Adxl367<IFACE> {
            iface: IFACE,
            config: Config,
            power_ctl: u8,
            intmap: [u8; 2],
            intmap_upper: [u8; 2],
            pending: u8,
            active: bool,
        }

        impl<IFACE> Adxl367<IFACE> {
            /// Create a driver over a register interface; no bus traffic.
            pub const fn new(iface: IFACE) -> Self {
                Self {
                    iface,
                    config: Config::DEFAULT,
                    power_ctl: 0,
                    intmap: [0; 2],
                    intmap_upper: [0; 2],
                    pending: 0,
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

        impl<I2C> Adxl367<I2cInterface<I2C>> {
            /// Driver on an I2C bus ([`ADDRESS_ASEL_HIGH`] or
            /// [`ADDRESS_ASEL_LOW`]).
            pub const fn new_i2c(i2c: I2C, address: u8) -> Self {
                Self::new(I2cInterface::new(i2c, address))
            }
        }

        impl<SPI> Adxl367<SpiInterface<SPI>> {
            /// Driver on an SPI device (mode 0, 1–8 MHz).
            pub const fn new_spi(spi: SPI) -> Self {
                Self::new(SpiInterface::new(spi))
            }
        }

        impl<IFACE: $Iface> Adxl367<IFACE> {
            $($async)? fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<IFACE::Error>> {
                self.iface.read_regs(reg, buf)$($await)*.map_err(Error::bus)
            }

            $($async)? fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error<IFACE::Error>> {
                self.iface.write_reg(reg, value)$($await)*.map_err(Error::bus)
            }

            /// Check DEVID_AD, DEVID_MST and PART_ID. On mismatch the three
            /// bytes are reported as `0x00AD1DF7`-style
            /// [`Error::UnexpectedId`].
            pub $($async)? fn probe(&mut self) -> Result<(), Error<IFACE::Error>> {
                let mut id = [0u8; 3];
                self.read_regs(REG_DEVID_AD, &mut id)$($await)*?;
                if id != DEVICE_ID {
                    return Err(Error::UnexpectedId(u32::from_be_bytes([0, id[0], id[1], id[2]])));
                }
                Ok(())
            }

            $($async)? fn pause(&mut self) -> Result<(), Error<IFACE::Error>> {
                if self.active {
                    self.write_reg(REG_POWER_CTL, self.power_ctl)$($await)*?;
                }
                Ok(())
            }

            $($async)? fn resume(&mut self) -> Result<(), Error<IFACE::Error>> {
                if self.active {
                    self.write_reg(REG_POWER_CTL, self.power_ctl | POWER_CTL_MEASURE)$($await)*?;
                }
                Ok(())
            }

            $($async)? fn write_intmaps(&mut self) -> Result<(), Error<IFACE::Error>> {
                let low = self.config.int_active_low;
                self.intmap = [pin_bits(self.intmap[0], low), pin_bits(self.intmap[1], low)];
                self.write_reg(REG_INTMAP1_LOWER, self.intmap[0])$($await)*?;
                self.write_reg(REG_INTMAP2_LOWER, self.intmap[1])$($await)*
            }

            /// Apply the configuration and start measuring; returns the
            /// settling time (see [`turn_on_us`]).
            pub $($async)? fn configure(&mut self, config: Config) -> Result<WaitUs, Error<IFACE::Error>> {
                self.pause()$($await)*?;
                self.config = config;
                let filter = ((config.range as u8) << 6) | FILTER_I2C_HS | config.rate as u8;
                self.write_reg(REG_FILTER_CTL, filter)$($await)*?;
                self.write_intmaps()$($await)*?;
                self.write_reg(REG_TIMER_CTL, (config.wake_up_rate as u8) << 6)$($await)*?;
                self.power_ctl = (self.power_ctl & POWER_CTL_AUTOSLEEP)
                    | match config.mode {
                        PowerMode::Normal => 0,
                        PowerMode::LowNoise => POWER_CTL_LOW_NOISE,
                        PowerMode::WakeUp => POWER_CTL_WAKEUP,
                    };
                self.power_up()$($await)*
            }

            $($async)? fn power_up(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
                self.write_reg(REG_POWER_CTL, self.power_ctl | POWER_CTL_MEASURE)$($await)*?;
                self.active = true;
                Ok(turn_on_us(self.config.mode, self.config.wake_up_rate))
            }

            /// Configure activity (and optionally inactivity / autosleep)
            /// detection and route it to `config.pin`. Done in standby; the
            /// part resumes measuring afterwards if it was.
            ///
            /// `Linked`/`Loop` without `inactivity`, or `auto_sleep` with
            /// [`Linking::Default`], is [`Error::Unsupported`].
            pub $($async)? fn configure_wake_up(&mut self, config: &WakeUpConfig) -> Result<(), Error<IFACE::Error>> {
                let linked = !matches!(config.linking, Linking::Default);
                if (linked && config.inactivity.is_none()) || (config.auto_sleep && !linked) {
                    return Err(Error::Unsupported);
                }
                self.pause()$($await)*?;
                let range = self.config.range;
                let [h, l] = threshold_bytes(threshold_reg(config.threshold_mg, range));
                self.write_reg(REG_THRESH_ACT_H, h)$($await)*?;
                self.write_reg(REG_THRESH_ACT_L, l)$($await)*?;
                self.write_reg(REG_TIME_ACT, config.duration)$($await)*?;
                let mut ctl = ACT_EN | if config.referenced { ACT_REF } else { 0 };
                let mut events = INTMAP_ACT;
                if let Some(inact) = config.inactivity {
                    let [h, l] = threshold_bytes(threshold_reg(inact.threshold_mg, range));
                    self.write_reg(REG_THRESH_INACT_H, h)$($await)*?;
                    self.write_reg(REG_THRESH_INACT_L, l)$($await)*?;
                    let [h, l] = inact.samples.to_be_bytes();
                    self.write_reg(REG_TIME_INACT_H, h)$($await)*?;
                    self.write_reg(REG_TIME_INACT_L, l)$($await)*?;
                    ctl |= INACT_EN | if inact.referenced { INACT_REF } else { 0 };
                    events |= INTMAP_INACT;
                }
                ctl |= match config.linking {
                    Linking::Default => 0,
                    Linking::Linked => 0x10,
                    Linking::Loop => 0x30,
                };
                self.write_reg(REG_ACT_INACT_CTL, ctl)$($await)*?;
                if config.awake_output {
                    events = INTMAP_AWAKE;
                }
                let all = INTMAP_AWAKE | INTMAP_ACT | INTMAP_INACT;
                let on = config.pin.index();
                self.intmap[on] = (self.intmap[on] & !all) | events;
                self.intmap[1 - on] &= !all;
                self.write_intmaps()$($await)*?;
                self.power_ctl = (self.power_ctl & !POWER_CTL_AUTOSLEEP)
                    | if config.auto_sleep { POWER_CTL_AUTOSLEEP } else { 0 };
                self.resume()$($await)*
            }

            /// Configure single (and optionally double) tap detection on
            /// one axis and route it to `config.pin`. These registers can
            /// be written while measuring.
            pub $($async)? fn configure_tap(&mut self, config: &TapConfig) -> Result<(), Error<IFACE::Error>> {
                let thresh = scale(u32::from(config.threshold_mg) * 2, 125).max(1);
                self.write_reg(REG_TAP_THRESH, thresh)$($await)*?;
                self.write_reg(REG_TAP_DUR, scale(config.duration_us, 625))$($await)*?;
                let (latent, window) = if config.double {
                    (scale(config.latency_us, 1250).max(1), scale(config.window_us, 1250))
                } else {
                    (0, 0)
                };
                self.write_reg(REG_TAP_LATENT, latent)$($await)*?;
                self.write_reg(REG_TAP_WINDOW, window)$($await)*?;
                self.write_reg(REG_AXIS_MASK, (config.axis as u8) << 4)$($await)*?;
                let events = INTMAP_TAP_ONE | if config.double { INTMAP_TAP_TWO } else { 0 };
                let on = config.pin.index();
                self.intmap_upper[on] = events;
                self.intmap_upper[1 - on] = 0;
                self.write_reg(REG_INTMAP1_UPPER, self.intmap_upper[0])$($await)*?;
                self.write_reg(REG_INTMAP2_UPPER, self.intmap_upper[1])$($await)*
            }

            /// Turn tap detection off (TAP_THRESH = 0) and unmap it.
            pub $($async)? fn disable_tap(&mut self) -> Result<(), Error<IFACE::Error>> {
                self.write_reg(REG_TAP_THRESH, 0)$($await)*?;
                self.intmap_upper = [0; 2];
                self.write_reg(REG_INTMAP1_UPPER, 0)$($await)*?;
                self.write_reg(REG_INTMAP2_UPPER, 0)$($await)*
            }

            /// Unmap every function from `pin` (it goes high-impedance).
            /// Detection settings are kept.
            pub $($async)? fn disable_interrupt(&mut self, pin: IntPin) -> Result<(), Error<IFACE::Error>> {
                let (i, lower, upper) = match pin {
                    IntPin::Int1 => (0, REG_INTMAP1_LOWER, REG_INTMAP1_UPPER),
                    IntPin::Int2 => (1, REG_INTMAP2_LOWER, REG_INTMAP2_UPPER),
                };
                self.pause()$($await)*?;
                self.intmap[i] &= INTMAP_INT_LOW;
                self.write_reg(lower, self.intmap[i])$($await)*?;
                self.resume()$($await)*?;
                self.intmap_upper[i] = 0;
                self.write_reg(upper, 0)$($await)*
            }

            /// Read STATUS (clears latched ACT/INACT events) and STATUS2
            /// (tap). Events already seen by [`Sensor::read_measurement`]
            /// are merged in.
            ///
            /// [`Sensor::read_measurement`]: zigbee_sensor::blocking::Sensor::read_measurement
            pub $($async)? fn read_interrupt_source(&mut self) -> Result<InterruptSource, Error<IFACE::Error>> {
                let mut status = [0u8];
                self.read_regs(REG_STATUS, &mut status)$($await)*?;
                let mut status2 = [0u8];
                self.read_regs(REG_STATUS2, &mut status2)$($await)*?;
                let status = status[0] | self.pending;
                self.pending = 0;
                Ok(InterruptSource { status, status2: status2[0] })
            }
        }

        impl<IFACE: $Iface> $Sensor for Adxl367<IFACE> {
            type BusError = IFACE::Error;
            type Measurement = Measurement;

            /// Leave standby if needed (returns [`turn_on_us`]), else `0`.
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
                if self.active {
                    return Ok(0);
                }
                self.power_up()$($await)*
            }

            /// Reads STATUS, FIFO_ENTRIES and XDATA..ZDATA in one burst.
            /// ACT/INACT events cleared by that STATUS read are kept for
            /// [`Adxl367::read_interrupt_source`].
            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<IFACE::Error>> {
                let mut buf = [0u8; 9];
                self.read_regs(REG_STATUS, &mut buf)$($await)*?;
                self.pending |= buf[0] & (STATUS_ACT | STATUS_INACT);
                if buf[0] & STATUS_DATA_READY == 0 {
                    return Err(Error::NotReady);
                }
                let mut frame = [0u8; 6];
                frame.copy_from_slice(&buf[3..]);
                Ok(Measurement { acceleration: decode(&frame, self.config.range) })
            }
        }

        impl<IFACE: $Iface> $LowPower for Adxl367<IFACE> {
            type BusError = IFACE::Error;

            /// Standby (40 nA). Pending events are kept but no new ones are
            /// generated; use [`PowerMode::WakeUp`] to wait for motion.
            $($async)? fn sleep(&mut self) -> Result<(), Error<IFACE::Error>> {
                self.write_reg(REG_POWER_CTL, self.power_ctl)$($await)*?;
                self.active = false;
                Ok(())
            }

            /// Resume measuring; returns [`turn_on_us`].
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
