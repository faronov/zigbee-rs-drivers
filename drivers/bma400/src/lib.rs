//! Bosch BMA400 ultra-low-power three-axis accelerometer (I2C or SPI).
//!
//! The part converts continuously; there is no single-shot mode.
//! [`PowerMode::LowPower`] samples at a fixed 25 Hz for 0.85–1.35 µA and
//! [`PowerMode::Normal`] at 12.5–800 Hz for 3.5–14.5 µA.
//! [`Sensor::start_measurement`] leaves sleep (returns 2/ODR) and
//! [`Sensor::read_measurement`] returns [`Error::NotReady`] until
//! `STATUS.drdy_stat` is set. [`LowPower::sleep`] is sleep mode (160 nA).
//!
//! For a sleepy Zigbee end device run the part in low-power mode with the
//! wake-up interrupt ([`WakeUpConfig`]), which is the only motion function
//! evaluated in low-power mode. Optionally let the part switch itself to
//! normal mode on motion ([`AutoWakeUp`]) and back to low-power mode on
//! inactivity ([`AutoLowPower`] with a generic interrupt,
//! [`GenericConfig`]). The generic activity/inactivity interrupts run only
//! in normal mode.
//!
//! Reference: Bosch Sensortec "BMA400 Data sheet" rev. 2.3
//! (BST-BMA400-DS000-14).
//!
//! [`Sensor::start_measurement`]: zigbee_sensor::blocking::Sensor::start_measurement
//! [`Sensor::read_measurement`]: zigbee_sensor::blocking::Sensor::read_measurement
//! [`LowPower::sleep`]: zigbee_sensor::blocking::LowPower::sleep
//!
//! ```ignore
//! let mut acc = Bma400::new_i2c(i2c, ADDRESS_SDO_LOW);
//! acc.probe()?;
//! acc.configure(Config { mode: PowerMode::LowPower, ..Default::default() })?;
//! acc.configure_wake_up(&WakeUpConfig { threshold_mg: 125, ..Default::default() })?;
//! // ... sleep until INT1, then:
//! let src = acc.read_interrupt_source()?; // clears latched interrupts
//! ```

#![no_std]

mod interface;

pub use interface::{AsyncInterface, I2cInterface, Interface, SpiInterface};
use zigbee_sensor::{Error, Quantity, Readings, Vector3, WaitUs};

/// I2C address with SDO to GND.
pub const ADDRESS_SDO_LOW: u8 = 0x14;
/// I2C address with SDO to VDDIO.
pub const ADDRESS_SDO_HIGH: u8 = 0x15;

/// CHIPID register value.
pub const CHIP_ID: u8 = 0x90;

const REG_CHIPID: u8 = 0x00;
const REG_STATUS: u8 = 0x03;
const REG_INT_STAT0: u8 = 0x0E;
const REG_ACC_CONFIG0: u8 = 0x19;
const REG_ACC_CONFIG1: u8 = 0x1A;
const REG_INT_CONFIG0: u8 = 0x1F;
const REG_INT_CONFIG1: u8 = 0x20;
const REG_INT1_MAP: u8 = 0x21;
const REG_INT2_MAP: u8 = 0x22;
const REG_INT12_IO_CTRL: u8 = 0x24;
const REG_AUTOLOWPOW_0: u8 = 0x2A;
const REG_AUTOLOWPOW_1: u8 = 0x2B;
const REG_AUTOWAKEUP_0: u8 = 0x2C;
const REG_AUTOWAKEUP_1: u8 = 0x2D;
const REG_WKUP_INT_CONFIG0: u8 = 0x2F;
const REG_WKUP_INT_CONFIG1: u8 = 0x30;
const REG_GEN1INT_CONFIG0: u8 = 0x3F;
const REG_GEN2INT_CONFIG0: u8 = 0x4A;

const STATUS_DRDY: u8 = 0x80;
const INT_CONFIG1_LATCH: u8 = 0x80;
const INT12_IO_ACTIVE_HIGH: u8 = 0x22;
const AUTOWAKEUP_TIMEOUT: u8 = 0x04;
const AUTOWAKEUP_WKUP_INT: u8 = 0x02;
const GEN_DATA_SRC_FILT2: u8 = 0x10;

const INT_WKUP: u8 = 0x01;
const INT_ORIENTCH: u8 = 0x02;
const INT_GEN1: u8 = 0x04;
const INT_GEN2: u8 = 0x08;
const INT_IENG_OVERRUN: u8 = 0x10;
const INT_DRDY: u8 = 0x80;

/// Measurement range (ACC_CONFIG1 acc_range). 12-bit data.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Range {
    /// ±2 g, 1024 LSB/g.
    #[default]
    G2,
    /// ±4 g, 512 LSB/g.
    G4,
    /// ±8 g, 256 LSB/g.
    G8,
    /// ±16 g, 128 LSB/g.
    G16,
}

impl Range {
    const fn shift(self) -> u32 {
        self as u32
    }
}

/// Normal-mode output data rate (ACC_CONFIG1 acc_odr). Low-power mode is
/// always 25 Hz.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DataRate {
    /// 12.5 Hz.
    Hz12_5,
    /// 25 Hz.
    #[default]
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
}

impl DataRate {
    /// Sample period in µs.
    pub const fn period_us(self) -> u32 {
        80_000 >> (self as u32)
    }

    const fn bits(self) -> u8 {
        self as u8 + 5
    }
}

/// Oversampling: trades current for noise (ACC_CONFIG1 osr,
/// ACC_CONFIG0 osr_lp).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Oversampling {
    /// Lowest current, highest noise (3.5 µA normal, 0.85 µA low-power).
    #[default]
    Osr0,
    /// OSR 1.
    Osr1,
    /// OSR 2.
    Osr2,
    /// Lowest noise (14.5 µA normal, 1.35 µA low-power).
    Osr3,
}

/// Power mode while measuring (ACC_CONFIG0 power_mode_conf).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PowerMode {
    /// Fixed 25 Hz; only the wake-up and data-ready interrupts run.
    #[default]
    LowPower,
    /// 12.5–800 Hz; all interrupts available.
    Normal,
}

impl PowerMode {
    const fn bits(self) -> u8 {
        match self {
            PowerMode::LowPower => 0x01,
            PowerMode::Normal => 0x02,
        }
    }
}

/// Operating state reported by `STATUS.power_mode_stat`; it changes on
/// its own with [`AutoWakeUp`] / [`AutoLowPower`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    /// Sleep mode, no conversions.
    Sleep,
    /// Low-power mode.
    LowPower,
    /// Normal mode.
    Normal,
}

/// Low-power-mode sample period.
pub const LOW_POWER_PERIOD_US: u32 = 40_000;

/// Measurement configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Measurement range.
    pub range: Range,
    /// Power mode.
    pub mode: PowerMode,
    /// Normal-mode output data rate.
    pub rate: DataRate,
    /// Normal-mode oversampling.
    pub osr: Oversampling,
    /// Low-power-mode oversampling.
    pub osr_lp: Oversampling,
    /// Latch interrupts until [`Bma400::read_interrupt_source`]
    /// (INT_CONFIG1.latch_int) instead of following the condition.
    pub latch: bool,
    /// INT1/INT2 active low (default active high, push-pull).
    pub int_active_low: bool,
}

impl Config {
    /// ±2 g, low-power mode (25 Hz), normal-mode 25 Hz, OSR 0, latched,
    /// active-high interrupts.
    pub const DEFAULT: Self = Self {
        range: Range::G2,
        mode: PowerMode::LowPower,
        rate: DataRate::Hz25,
        osr: Oversampling::Osr0,
        osr_lp: Oversampling::Osr0,
        latch: true,
        int_active_low: false,
    };
}

impl Default for Config {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Wake-up time from sleep: 2/ODR (datasheet Table "Specification",
/// t<sub>w_up</sub>).
pub const fn turn_on_us(mode: PowerMode, rate: DataRate) -> WaitUs {
    match mode {
        PowerMode::LowPower => 2 * LOW_POWER_PERIOD_US,
        PowerMode::Normal => 2 * rate.period_us(),
    }
}

/// Interrupt output pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum IntPin {
    /// INT1 (INT1_MAP).
    #[default]
    Int1,
    /// INT2 (INT2_MAP).
    Int2,
}

impl IntPin {
    const fn index(self) -> usize {
        self as usize
    }
}

/// Axes taking part in a detection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Axes {
    /// X axis.
    pub x: bool,
    /// Y axis.
    pub y: bool,
    /// Z axis.
    pub z: bool,
}

impl Axes {
    /// All three axes.
    pub const ALL: Self = Self {
        x: true,
        y: true,
        z: true,
    };

    /// Bits 7:5 (z, y, x) as used by WKUP_INT_CONFIG0 and GENxINT_CONFIG0.
    const fn bits(self) -> u8 {
        (self.z as u8) << 7 | (self.y as u8) << 6 | (self.x as u8) << 5
    }
}

impl Default for Axes {
    fn default() -> Self {
        Self::ALL
    }
}

/// Wake-up interrupt reference (WKUP_INT_CONFIG0 wkup_refu).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum WakeUpReference {
    /// The first sample after entering low-power mode: detects a change of
    /// orientation or position (tilt, door, garage).
    #[default]
    OneTime,
    /// The previous sample: detects sample-to-sample change (vibration,
    /// shock).
    EveryTime,
}

/// Low-power-mode wake-up interrupt (25 Hz evaluation, 0.85 µA).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WakeUpConfig {
    /// Threshold in mg; 8-bit, 15.6/31.3/62.5/125 mg/LSB by range.
    pub threshold_mg: u16,
    /// Consecutive samples above the threshold, 1–8 (40 ms each).
    pub samples: u8,
    /// Reference update mode.
    pub reference: WakeUpReference,
    /// Axes evaluated (any one above the threshold triggers).
    pub axes: Axes,
    /// Output pin.
    pub pin: IntPin,
}

impl Default for WakeUpConfig {
    fn default() -> Self {
        Self {
            threshold_mg: 125,
            samples: 1,
            reference: WakeUpReference::OneTime,
            axes: Axes::ALL,
            pin: IntPin::Int1,
        }
    }
}

/// Generic interrupt engine.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Generic {
    /// Generic interrupt 1 (can trigger auto low-power).
    #[default]
    Gen1,
    /// Generic interrupt 2 (can restart the auto low-power timeout).
    Gen2,
}

/// Generic interrupt criterion (GENxINT_CONFIG1 criterion_sel).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Criterion {
    /// Referenced acceleration above the threshold.
    #[default]
    Activity,
    /// Referenced acceleration below the threshold.
    Inactivity,
}

/// Generic interrupt reference (GENxINT_CONFIG0 act_refu).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GenericReference {
    /// Taken once when the interrupt triggers and on entering normal mode.
    #[default]
    OneTime,
    /// Updated after every evaluation.
    EveryTime,
    /// Updated after every evaluation from the 1 Hz low-pass filter.
    EveryTimeLowPass,
}

/// Generic interrupt hysteresis (GENxINT_CONFIG0 act_hyst).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Hysteresis {
    /// None.
    #[default]
    None,
    /// 24 mg.
    Mg24,
    /// 48 mg.
    Mg48,
    /// 96 mg.
    Mg96,
}

/// Generic activity / inactivity interrupt (normal mode only). Evaluated
/// on the fixed 100 Hz filter (acc_filt2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GenericConfig {
    /// Activity or inactivity.
    pub criterion: Criterion,
    /// Threshold in mg (8 mg/LSB, at most 2040 mg, any range).
    pub threshold_mg: u16,
    /// Samples (10 ms each) the condition must hold, 16-bit.
    pub duration: u16,
    /// Hysteresis.
    pub hysteresis: Hysteresis,
    /// Reference update mode.
    pub reference: GenericReference,
    /// Axes evaluated.
    pub axes: Axes,
    /// Require all enabled axes (AND) instead of any (OR).
    pub all_axes: bool,
    /// Output pin; `None` enables the engine without routing it (e.g.
    /// only for [`AutoLowPower`]).
    pub pin: Option<IntPin>,
}

impl Default for GenericConfig {
    fn default() -> Self {
        Self {
            criterion: Criterion::Activity,
            threshold_mg: 128,
            duration: 0,
            hysteresis: Hysteresis::None,
            reference: GenericReference::OneTime,
            axes: Axes::ALL,
            all_axes: false,
            pin: Some(IntPin::Int1),
        }
    }
}

/// Automatic low-power → normal switch (AUTOWAKEUP_0/1).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct AutoWakeUp {
    /// Switch to normal mode when the wake-up interrupt fires.
    pub on_motion: bool,
    /// Switch to normal mode after this long in low-power mode (2.5 ms
    /// steps, at most 10 237 ms).
    pub timeout_ms: Option<u16>,
}

/// Automatic normal → low-power switch (AUTOLOWPOW_0/1).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct AutoLowPower {
    /// Switch after this long in normal mode (2.5 ms steps, at most
    /// 10 237 ms).
    pub timeout_ms: Option<u16>,
    /// Restart the timeout whenever generic interrupt 2 fires (use Gen2
    /// as an activity detector).
    pub timeout_reset_by_gen2: bool,
    /// Switch when generic interrupt 1 fires (use Gen1 as an inactivity
    /// detector).
    pub on_gen1: bool,
    /// Switch after the first new sample.
    pub on_data_ready: bool,
}

/// INT_STAT0..INT_STAT2.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct InterruptSource {
    /// INT_STAT0 (0Eh), INT_STAT1 (0Fh), INT_STAT2 (10h).
    pub status: [u8; 3],
}

impl InterruptSource {
    /// Low-power wake-up interrupt.
    pub const fn wake_up(self) -> bool {
        self.status[0] & INT_WKUP != 0
    }
    /// Generic interrupt 1.
    pub const fn gen1(self) -> bool {
        self.status[0] & INT_GEN1 != 0
    }
    /// Generic interrupt 2.
    pub const fn gen2(self) -> bool {
        self.status[0] & INT_GEN2 != 0
    }
    /// Orientation changed.
    pub const fn orientation(self) -> bool {
        self.status[0] & INT_ORIENTCH != 0
    }
    /// Data ready.
    pub const fn data_ready(self) -> bool {
        self.status[0] & INT_DRDY != 0
    }
    /// The interrupt engine could not finish in time (ODR too high for the
    /// enabled features).
    pub const fn overrun(self) -> bool {
        self.status[0] & INT_IENG_OVERRUN != 0
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

/// Convert one 12-bit sample to mg (1024 >> range LSB/g, truncated toward
/// zero).
pub const fn raw_to_mg(raw: i16, range: Range) -> i32 {
    (raw as i32 * (1000 << range.shift())) / 1024
}

/// Decode ACC_X_LSB..ACC_Z_MSB to milli-g.
pub fn decode(frame: &[u8; 6], range: Range) -> Vector3 {
    let axis = |i: usize| {
        let v = u16::from_le_bytes([frame[i], frame[i + 1] & 0x0F]);
        raw_to_mg(((v << 4) as i16) >> 4, range)
    };
    Vector3 {
        x: axis(0),
        y: axis(2),
        z: axis(4),
    }
}

fn scale(value: u32, step: u32, max: u32) -> u32 {
    ((value + step / 2) / step).min(max)
}

/// WKUP_INT_CONFIG1 code: LSB = 2^(2 + range) / 256 g (rounded, at most
/// 255).
pub fn wake_up_threshold_reg(threshold_mg: u16, range: Range) -> u8 {
    scale(u32::from(threshold_mg) * 8, 125 << range.shift(), 255) as u8
}

/// GENxINT_CONFIG2 code: 8 mg/LSB (rounded, at most 255).
pub fn generic_threshold_reg(threshold_mg: u16) -> u8 {
    scale(u32::from(threshold_mg), 8, 255) as u8
}

/// 12-bit AUTOLOWPOW / AUTOWAKEUP timeout code: 2.5 ms/LSB.
pub fn timeout_reg(ms: u16) -> u16 {
    scale(u32::from(ms) * 2, 5, 0xFFF) as u16
}

macro_rules! driver {
    ($Iface:path, $Sensor:path, $LowPower:path, [$($async:tt)?], [$($await:tt)*]) => {
        /// BMA400 driver over a register [`Interface`].
        pub struct Bma400<IFACE> {
            iface: IFACE,
            config: Config,
            int_config0: u8,
            int_map: [u8; 2],
            active: bool,
        }

        impl<IFACE> Bma400<IFACE> {
            /// Create a driver over a register interface; no bus traffic.
            pub const fn new(iface: IFACE) -> Self {
                Self {
                    iface,
                    config: Config::DEFAULT,
                    int_config0: 0,
                    int_map: [0; 2],
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

        impl<I2C> Bma400<I2cInterface<I2C>> {
            /// Driver on an I2C bus ([`ADDRESS_SDO_LOW`] or
            /// [`ADDRESS_SDO_HIGH`]).
            pub const fn new_i2c(i2c: I2C, address: u8) -> Self {
                Self::new(I2cInterface::new(i2c, address))
            }
        }

        impl<SPI> Bma400<SpiInterface<SPI>> {
            /// Driver on an SPI device (mode 0 or 3, 4-wire).
            pub const fn new_spi(spi: SPI) -> Self {
                Self::new(SpiInterface::new(spi))
            }
        }

        impl<IFACE: $Iface> Bma400<IFACE> {
            $($async)? fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<IFACE::Error>> {
                self.iface.read_regs(reg, buf)$($await)*.map_err(Error::bus)
            }

            $($async)? fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error<IFACE::Error>> {
                self.iface.write_reg(reg, value)$($await)*.map_err(Error::bus)
            }

            /// Select the bus (one dummy read on SPI) and check CHIPID.
            pub $($async)? fn probe(&mut self) -> Result<(), Error<IFACE::Error>> {
                self.iface.init()$($await)*.map_err(Error::bus)?;
                let mut id = [0u8];
                self.read_regs(REG_CHIPID, &mut id)$($await)*?;
                if id[0] != CHIP_ID {
                    return Err(Error::UnexpectedId(u32::from(id[0])));
                }
                Ok(())
            }

            /// Apply the configuration and start measuring; returns
            /// [`turn_on_us`].
            pub $($async)? fn configure(&mut self, config: Config) -> Result<WaitUs, Error<IFACE::Error>> {
                self.config = config;
                let acc1 = (config.range as u8) << 6 | (config.osr as u8) << 4 | config.rate.bits();
                self.write_reg(REG_ACC_CONFIG1, acc1)$($await)*?;
                let latch = if config.latch { INT_CONFIG1_LATCH } else { 0 };
                self.write_reg(REG_INT_CONFIG1, latch)$($await)*?;
                let io = if config.int_active_low { 0 } else { INT12_IO_ACTIVE_HIGH };
                self.write_reg(REG_INT12_IO_CTRL, io)$($await)*?;
                self.power_up()$($await)*
            }

            $($async)? fn write_power_mode(&mut self, mode: u8) -> Result<(), Error<IFACE::Error>> {
                self.write_reg(REG_ACC_CONFIG0, (self.config.osr_lp as u8) << 5 | mode)$($await)*
            }

            $($async)? fn power_up(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
                self.write_power_mode(self.config.mode.bits())$($await)*?;
                self.active = true;
                Ok(turn_on_us(self.config.mode, self.config.rate))
            }

            $($async)? fn route(&mut self, bit: u8, pin: Option<IntPin>) -> Result<(), Error<IFACE::Error>> {
                self.int_map[0] &= !bit;
                self.int_map[1] &= !bit;
                if let Some(pin) = pin {
                    self.int_map[pin.index()] |= bit;
                }
                self.write_reg(REG_INT1_MAP, self.int_map[0])$($await)*?;
                self.write_reg(REG_INT2_MAP, self.int_map[1])$($await)*
            }

            /// Configure the low-power-mode wake-up interrupt and route it
            /// to `config.pin`. It is evaluated only in
            /// [`PowerMode::LowPower`]. `samples` outside 1–8 is
            /// [`Error::Unsupported`].
            pub $($async)? fn configure_wake_up(&mut self, config: &WakeUpConfig) -> Result<(), Error<IFACE::Error>> {
                if !(1..=8).contains(&config.samples) {
                    return Err(Error::Unsupported);
                }
                let refu = match config.reference {
                    WakeUpReference::OneTime => 0x01,
                    WakeUpReference::EveryTime => 0x02,
                };
                let cfg0 = config.axes.bits() | (config.samples - 1) << 2 | refu;
                let thresh = wake_up_threshold_reg(config.threshold_mg, self.config.range);
                self.write_reg(REG_WKUP_INT_CONFIG0, cfg0)$($await)*?;
                self.write_reg(REG_WKUP_INT_CONFIG1, thresh)$($await)*?;
                self.route(INT_WKUP, Some(config.pin))$($await)*
            }

            /// Stop evaluating the wake-up interrupt (no axes) and unmap it.
            pub $($async)? fn disable_wake_up(&mut self) -> Result<(), Error<IFACE::Error>> {
                self.write_reg(REG_WKUP_INT_CONFIG0, 0)$($await)*?;
                self.route(INT_WKUP, None)$($await)*
            }

            /// Configure and enable a generic activity/inactivity interrupt
            /// (normal mode only). `duration` counts 100 Hz samples.
            pub $($async)? fn configure_generic(&mut self, which: Generic, config: &GenericConfig) -> Result<(), Error<IFACE::Error>> {
                let (base, bit) = match which {
                    Generic::Gen1 => (REG_GEN1INT_CONFIG0, INT_GEN1),
                    Generic::Gen2 => (REG_GEN2INT_CONFIG0, INT_GEN2),
                };
                let refu = match config.reference {
                    GenericReference::OneTime => 0x01,
                    GenericReference::EveryTime => 0x02,
                    GenericReference::EveryTimeLowPass => 0x03,
                };
                let cfg0 = config.axes.bits() | GEN_DATA_SRC_FILT2 | refu << 2 | config.hysteresis as u8;
                let cfg1 = match config.criterion {
                    Criterion::Inactivity => 0x00,
                    Criterion::Activity => 0x02,
                } | config.all_axes as u8;
                let [dur_h, dur_l] = config.duration.to_be_bytes();
                self.write_reg(base, cfg0)$($await)*?;
                self.write_reg(base + 1, cfg1)$($await)*?;
                self.write_reg(base + 2, generic_threshold_reg(config.threshold_mg))$($await)*?;
                self.write_reg(base + 3, dur_h)$($await)*?;
                self.write_reg(base + 4, dur_l)$($await)*?;
                self.route(bit, config.pin)$($await)*?;
                self.int_config0 |= bit;
                self.write_reg(REG_INT_CONFIG0, self.int_config0)$($await)*
            }

            /// Disable and unmap a generic interrupt.
            pub $($async)? fn disable_generic(&mut self, which: Generic) -> Result<(), Error<IFACE::Error>> {
                let bit = match which {
                    Generic::Gen1 => INT_GEN1,
                    Generic::Gen2 => INT_GEN2,
                };
                self.int_config0 &= !bit;
                self.write_reg(REG_INT_CONFIG0, self.int_config0)$($await)*?;
                self.route(bit, None)$($await)*
            }

            /// Configure automatic low-power → normal switching.
            pub $($async)? fn configure_auto_wake_up(&mut self, config: &AutoWakeUp) -> Result<(), Error<IFACE::Error>> {
                let code = config.timeout_ms.map_or(0, timeout_reg);
                let mut cfg1 = ((code & 0x0F) as u8) << 4;
                if config.timeout_ms.is_some() {
                    cfg1 |= AUTOWAKEUP_TIMEOUT;
                }
                if config.on_motion {
                    cfg1 |= AUTOWAKEUP_WKUP_INT;
                }
                self.write_reg(REG_AUTOWAKEUP_0, (code >> 4) as u8)$($await)*?;
                self.write_reg(REG_AUTOWAKEUP_1, cfg1)$($await)*
            }

            /// Configure automatic normal → low-power switching.
            pub $($async)? fn configure_auto_low_power(&mut self, config: &AutoLowPower) -> Result<(), Error<IFACE::Error>> {
                let code = config.timeout_ms.map_or(0, timeout_reg);
                let timeout = match (config.timeout_ms, config.timeout_reset_by_gen2) {
                    (None, _) => 0x00,
                    (Some(_), false) => 0x01,
                    (Some(_), true) => 0x02,
                };
                let cfg1 = ((code & 0x0F) as u8) << 4
                    | timeout << 2
                    | (config.on_gen1 as u8) << 1
                    | config.on_data_ready as u8;
                self.write_reg(REG_AUTOLOWPOW_0, (code >> 4) as u8)$($await)*?;
                self.write_reg(REG_AUTOLOWPOW_1, cfg1)$($await)*
            }

            /// Unmap every function from `pin`. Detection settings are
            /// kept.
            pub $($async)? fn disable_interrupt(&mut self, pin: IntPin) -> Result<(), Error<IFACE::Error>> {
                let (i, reg) = match pin {
                    IntPin::Int1 => (0, REG_INT1_MAP),
                    IntPin::Int2 => (1, REG_INT2_MAP),
                };
                self.int_map[i] = 0;
                self.write_reg(reg, 0)$($await)*
            }

            /// Read INT_STAT0..2; clears latched interrupts.
            pub $($async)? fn read_interrupt_source(&mut self) -> Result<InterruptSource, Error<IFACE::Error>> {
                let mut status = [0u8; 3];
                self.read_regs(REG_INT_STAT0, &mut status)$($await)*?;
                Ok(InterruptSource { status })
            }

            /// Current operating state (STATUS.power_mode_stat), which
            /// changes on its own with auto wake-up / auto low-power.
            pub $($async)? fn state(&mut self) -> Result<State, Error<IFACE::Error>> {
                let mut status = [0u8];
                self.read_regs(REG_STATUS, &mut status)$($await)*?;
                Ok(match (status[0] >> 1) & 0x03 {
                    0x01 => State::LowPower,
                    0x02 => State::Normal,
                    _ => State::Sleep,
                })
            }
        }

        impl<IFACE: $Iface> $Sensor for Bma400<IFACE> {
            type BusError = IFACE::Error;
            type Measurement = Measurement;

            /// Leave sleep if needed (returns 2/ODR), else `0`.
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
                if self.active {
                    return Ok(0);
                }
                self.power_up()$($await)*
            }

            /// Reads STATUS and ACC_X..ACC_Z in one burst.
            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<IFACE::Error>> {
                let mut buf = [0u8; 7];
                self.read_regs(REG_STATUS, &mut buf)$($await)*?;
                if buf[0] & STATUS_DRDY == 0 {
                    return Err(Error::NotReady);
                }
                let mut frame = [0u8; 6];
                frame.copy_from_slice(&buf[1..]);
                Ok(Measurement { acceleration: decode(&frame, self.config.range) })
            }
        }

        impl<IFACE: $Iface> $LowPower for Bma400<IFACE> {
            type BusError = IFACE::Error;

            /// Sleep mode (160 nA): no conversions and no interrupts. Use
            /// [`PowerMode::LowPower`] with [`WakeUpConfig`] to wait for
            /// motion.
            $($async)? fn sleep(&mut self) -> Result<(), Error<IFACE::Error>> {
                self.write_power_mode(0)$($await)*?;
                self.active = false;
                Ok(())
            }

            /// Back to the configured mode; returns 2/ODR.
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
