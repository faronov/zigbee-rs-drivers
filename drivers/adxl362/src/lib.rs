//! Analog Devices ADXL362 micropower three-axis accelerometer (SPI only).
//!
//! The part converts continuously ([`PowerMode`]: 1.8 µA normal at 100 Hz,
//! 270 nA wake-up mode); there is no single-shot mode.
//! [`Sensor::start_measurement`] leaves standby (returns 4/ODR) and
//! [`Sensor::read_measurement`] returns [`Error::NotReady`] until
//! `STATUS.DATA_READY` is set. [`LowPower::sleep`] is standby (10 nA).
//!
//! For a sleepy Zigbee end device use referenced activity detection
//! ([`WakeUpConfig`]) in [`PowerMode::WakeUp`] (≈ 6 samples/s, 270 nA), or
//! in normal mode with inactivity, [`Linking::Loop`] and
//! [`WakeUpConfig::auto_sleep`] so the part drops to wake-up mode by
//! itself when still.
//!
//! Registers 0x00–0x2C may only be changed in standby (datasheet
//! "Register Details"); the driver drops to standby and resumes around
//! every configuration write.
//!
//! Reference: Analog Devices "ADXL362 Data Sheet" Rev. B (Chinese edition,
//! 2012–2013).
//!
//! [`Sensor::start_measurement`]: zigbee_sensor::blocking::Sensor::start_measurement
//! [`Sensor::read_measurement`]: zigbee_sensor::blocking::Sensor::read_measurement
//! [`LowPower::sleep`]: zigbee_sensor::blocking::LowPower::sleep
//!
//! ```ignore
//! let mut acc = Adxl362::new(spi);
//! acc.probe()?;
//! acc.configure(Config { mode: PowerMode::WakeUp, ..Default::default() })?;
//! acc.configure_wake_up(&WakeUpConfig { threshold_mg: 250, ..Default::default() })?;
//! // ... sleep until INT1, then:
//! let src = acc.read_interrupt_source()?; // reading STATUS clears ACT/INACT
//! ```

#![no_std]

use embedded_hal::spi::Operation;
use zigbee_sensor::{Error, Quantity, Readings, Vector3, WaitUs};

/// DEVID_AD, DEVID_MST and PARTID.
pub const DEVICE_ID: [u8; 3] = [0xAD, 0x1D, 0xF2];

const CMD_WRITE: u8 = 0x0A;
const CMD_READ: u8 = 0x0B;

const REG_DEVID_AD: u8 = 0x00;
const REG_STATUS: u8 = 0x0B;
const REG_THRESH_ACT_L: u8 = 0x20;
const REG_THRESH_ACT_H: u8 = 0x21;
const REG_TIME_ACT: u8 = 0x22;
const REG_THRESH_INACT_L: u8 = 0x23;
const REG_THRESH_INACT_H: u8 = 0x24;
const REG_TIME_INACT_L: u8 = 0x25;
const REG_TIME_INACT_H: u8 = 0x26;
const REG_ACT_INACT_CTL: u8 = 0x27;
const REG_INTMAP1: u8 = 0x2A;
const REG_INTMAP2: u8 = 0x2B;
const REG_FILTER_CTL: u8 = 0x2C;
const REG_POWER_CTL: u8 = 0x2D;

const ACT_EN: u8 = 0x01;
const ACT_REF: u8 = 0x02;
const INACT_EN: u8 = 0x04;
const INACT_REF: u8 = 0x08;
const FILTER_HALF_BW: u8 = 0x10;
const POWER_CTL_WAKEUP: u8 = 0x08;
const POWER_CTL_AUTOSLEEP: u8 = 0x04;
const POWER_CTL_MEASURE: u8 = 0x02;
const INTMAP_INT_LOW: u8 = 0x80;
const INTMAP_AWAKE: u8 = 0x40;
const INTMAP_INACT: u8 = 0x20;
const INTMAP_ACT: u8 = 0x10;

const STATUS_ERR_USER_REGS: u8 = 0x80;
const STATUS_AWAKE: u8 = 0x40;
const STATUS_INACT: u8 = 0x20;
const STATUS_ACT: u8 = 0x10;
const STATUS_DATA_READY: u8 = 0x01;

/// Measurement range (FILTER_CTL RANGE). 12-bit data: 1/2/4 mg/LSB.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Range {
    /// ±2 g, 1 mg/LSB.
    #[default]
    G2,
    /// ±4 g, 2 mg/LSB.
    G4,
    /// ±8 g, 4 mg/LSB.
    G8,
}

impl Range {
    /// mg per LSB as a power of two.
    const fn shift(self) -> u32 {
        self as u32
    }
}

/// Output data rate (FILTER_CTL ODR). Ignored in [`PowerMode::WakeUp`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DataRate {
    /// 12.5 Hz.
    #[default]
    Hz12_5,
    /// 25 Hz.
    Hz25,
    /// 50 Hz.
    Hz50,
    /// 100 Hz (1.8 µA in normal mode).
    Hz100,
    /// 200 Hz.
    Hz200,
    /// 400 Hz (3.0 µA).
    Hz400,
}

impl DataRate {
    /// Sample period in µs.
    pub const fn period_us(self) -> u32 {
        80_000 >> (self as u32)
    }
}

/// Power / noise trade-off (POWER_CTL LOW_NOISE and WAKEUP).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PowerMode {
    /// Normal operation, 1.8 µA at 100 Hz.
    #[default]
    Normal,
    /// Low noise, 3.3 µA.
    LowNoise,
    /// Ultra-low noise, 13 µA.
    UltraLowNoise,
    /// Wake-up mode: about 6 samples/s for motion detection only, 270 nA.
    /// The activity timer is ignored.
    WakeUp,
}

/// Approximate sample period in wake-up mode (≈ 6 Hz, datasheet
/// "Wake-Up Mode").
pub const WAKE_UP_PERIOD_US: u32 = 166_667;

/// Measurement configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Measurement range.
    pub range: Range,
    /// Output data rate (measurement mode).
    pub rate: DataRate,
    /// Power / noise mode.
    pub mode: PowerMode,
    /// Anti-aliasing bandwidth ODR/4 (reset default) instead of ODR/2.
    pub half_bandwidth: bool,
    /// INT1/INT2 active low (INTMAPx.INT_LOW, default active high).
    pub int_active_low: bool,
}

impl Config {
    /// ±2 g, 12.5 Hz, normal mode, ODR/4 bandwidth, active-high interrupts.
    pub const DEFAULT: Self = Self {
        range: Range::G2,
        rate: DataRate::Hz12_5,
        mode: PowerMode::Normal,
        half_bandwidth: true,
        int_active_low: false,
    };
}

impl Default for Config {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Measurement command to valid data: 4/ODR (datasheet Table 1, "Turn-On
/// Time"). In wake-up mode the ≈ 6 Hz period is used (assumption).
pub const fn turn_on_us(mode: PowerMode, rate: DataRate) -> WaitUs {
    match mode {
        PowerMode::WakeUp => 4 * WAKE_UP_PERIOD_US,
        _ => 4 * rate.period_us(),
    }
}

/// Interrupt output pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum IntPin {
    /// INT1 (INTMAP1).
    #[default]
    Int1,
    /// INT2 (INTMAP2).
    Int2,
}

/// How activity and inactivity interact (ACT_INACT_CTL LINK/LOOP).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Linking {
    /// Both functions run concurrently; each event is latched until
    /// [`Adxl362::read_interrupt_source`].
    #[default]
    Default,
    /// Alternate activity → inactivity → activity; each event is latched
    /// until [`Adxl362::read_interrupt_source`].
    Linked,
    /// Alternate like `Linked` but acknowledged internally (not latched).
    /// Pair with [`WakeUpConfig::awake_output`] for a motion switch.
    Loop,
}

/// Inactivity detection: every axis within the threshold for `samples`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InactivityConfig {
    /// Threshold in mg (11-bit, 1/2/4 mg/LSB by range).
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
    /// Activity threshold in mg (11-bit, 1/2/4 mg/LSB by range).
    pub threshold_mg: u16,
    /// Consecutive samples above the threshold (TIME_ACT, 0 and 1 both
    /// mean one sample). Ignored in [`PowerMode::WakeUp`].
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

/// STATUS register.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct InterruptSource {
    /// STATUS (0Bh), including ACT/INACT events consumed by earlier
    /// [`Sensor::read_measurement`] calls.
    ///
    /// [`Sensor::read_measurement`]: zigbee_sensor::blocking::Sensor::read_measurement
    pub status: u8,
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

/// Convert one sign-extended 12-bit sample to mg (1/2/4 mg/LSB).
pub const fn raw_to_mg(raw: i16, range: Range) -> i32 {
    (raw as i32) << range.shift()
}

/// Decode XDATA_L..ZDATA_H to milli-g.
pub fn decode(frame: &[u8; 6], range: Range) -> Vector3 {
    let axis = |i: usize| raw_to_mg(i16::from_le_bytes([frame[i], frame[i + 1]]), range);
    Vector3 {
        x: axis(0),
        y: axis(2),
        z: axis(4),
    }
}

/// 11-bit THRESH_ACT / THRESH_INACT code for a threshold in mg (rounded,
/// at most 2047).
pub fn threshold_reg(threshold_mg: u16, range: Range) -> u16 {
    let step = 1u32 << range.shift();
    ((u32::from(threshold_mg) + step / 2) / step).min(0x7FF) as u16
}

const fn pin_bits(map: u8, active_low: bool) -> u8 {
    (map & !INTMAP_INT_LOW) | if active_low { INTMAP_INT_LOW } else { 0 }
}

macro_rules! driver {
    ($Spi:path, $Sensor:path, $LowPower:path, [$($async:tt)?], [$($await:tt)*]) => {
        /// ADXL362 driver over an SPI device (mode 0, 1–5 MHz).
        pub struct Adxl362<SPI> {
            spi: SPI,
            config: Config,
            power_ctl: u8,
            intmap: [u8; 2],
            pending: u8,
            active: bool,
        }

        impl<SPI> Adxl362<SPI> {
            /// Create a driver; no bus traffic.
            pub const fn new(spi: SPI) -> Self {
                Self {
                    spi,
                    config: Config::DEFAULT,
                    power_ctl: 0,
                    intmap: [0; 2],
                    pending: 0,
                    active: false,
                }
            }

            /// Configuration last applied with [`Self::configure`].
            pub const fn config(&self) -> Config {
                self.config
            }

            /// Give the SPI device back.
            pub fn release(self) -> SPI {
                self.spi
            }
        }

        impl<SPI: $Spi> Adxl362<SPI> {
            $($async)? fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<SPI::Error>> {
                self.spi
                    .transaction(&mut [Operation::Write(&[CMD_READ, reg]), Operation::Read(buf)])
                    $($await)*
                    .map_err(Error::bus)
            }

            $($async)? fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error<SPI::Error>> {
                self.spi.write(&[CMD_WRITE, reg, value])$($await)*.map_err(Error::bus)
            }

            /// Check DEVID_AD, DEVID_MST and PARTID. On mismatch the three
            /// bytes are reported as `0x00AD1DF2`-style
            /// [`Error::UnexpectedId`].
            pub $($async)? fn probe(&mut self) -> Result<(), Error<SPI::Error>> {
                let mut id = [0u8; 3];
                self.read_regs(REG_DEVID_AD, &mut id)$($await)*?;
                if id != DEVICE_ID {
                    return Err(Error::UnexpectedId(u32::from_be_bytes([0, id[0], id[1], id[2]])));
                }
                Ok(())
            }

            $($async)? fn pause(&mut self) -> Result<(), Error<SPI::Error>> {
                if self.active {
                    self.write_reg(REG_POWER_CTL, self.power_ctl)$($await)*?;
                }
                Ok(())
            }

            $($async)? fn resume(&mut self) -> Result<(), Error<SPI::Error>> {
                if self.active {
                    self.write_reg(REG_POWER_CTL, self.power_ctl | POWER_CTL_MEASURE)$($await)*?;
                }
                Ok(())
            }

            $($async)? fn write_intmaps(&mut self) -> Result<(), Error<SPI::Error>> {
                let low = self.config.int_active_low;
                self.intmap = [pin_bits(self.intmap[0], low), pin_bits(self.intmap[1], low)];
                self.write_reg(REG_INTMAP1, self.intmap[0])$($await)*?;
                self.write_reg(REG_INTMAP2, self.intmap[1])$($await)*
            }

            /// Apply the configuration and start measuring; returns the
            /// turn-on time (4/ODR).
            pub $($async)? fn configure(&mut self, config: Config) -> Result<WaitUs, Error<SPI::Error>> {
                self.pause()$($await)*?;
                self.config = config;
                let filter = ((config.range as u8) << 6)
                    | if config.half_bandwidth { FILTER_HALF_BW } else { 0 }
                    | config.rate as u8;
                self.write_reg(REG_FILTER_CTL, filter)$($await)*?;
                self.write_intmaps()$($await)*?;
                self.power_ctl = (self.power_ctl & POWER_CTL_AUTOSLEEP)
                    | match config.mode {
                        PowerMode::Normal => 0,
                        PowerMode::LowNoise => 0x10,
                        PowerMode::UltraLowNoise => 0x20,
                        PowerMode::WakeUp => POWER_CTL_WAKEUP,
                    };
                self.power_up()$($await)*
            }

            $($async)? fn power_up(&mut self) -> Result<WaitUs, Error<SPI::Error>> {
                self.write_reg(REG_POWER_CTL, self.power_ctl | POWER_CTL_MEASURE)$($await)*?;
                self.active = true;
                Ok(turn_on_us(self.config.mode, self.config.rate))
            }

            /// Configure activity (and optionally inactivity / autosleep)
            /// detection and route it to `config.pin`. Done in standby; the
            /// part resumes measuring afterwards if it was.
            ///
            /// `Linked`/`Loop` without `inactivity`, or `auto_sleep` with
            /// [`Linking::Default`], is [`Error::Unsupported`].
            pub $($async)? fn configure_wake_up(&mut self, config: &WakeUpConfig) -> Result<(), Error<SPI::Error>> {
                let linked = !matches!(config.linking, Linking::Default);
                if (linked && config.inactivity.is_none()) || (config.auto_sleep && !linked) {
                    return Err(Error::Unsupported);
                }
                self.pause()$($await)*?;
                let range = self.config.range;
                let [act_l, act_h] = threshold_reg(config.threshold_mg, range).to_le_bytes();
                self.write_reg(REG_THRESH_ACT_L, act_l)$($await)*?;
                self.write_reg(REG_THRESH_ACT_H, act_h)$($await)*?;
                self.write_reg(REG_TIME_ACT, config.duration)$($await)*?;
                let mut ctl = ACT_EN | if config.referenced { ACT_REF } else { 0 };
                let mut events = INTMAP_ACT;
                if let Some(inact) = config.inactivity {
                    let [l, h] = threshold_reg(inact.threshold_mg, range).to_le_bytes();
                    self.write_reg(REG_THRESH_INACT_L, l)$($await)*?;
                    self.write_reg(REG_THRESH_INACT_H, h)$($await)*?;
                    let [l, h] = inact.samples.to_le_bytes();
                    self.write_reg(REG_TIME_INACT_L, l)$($await)*?;
                    self.write_reg(REG_TIME_INACT_H, h)$($await)*?;
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
                let (on, off) = match config.pin {
                    IntPin::Int1 => (0, 1),
                    IntPin::Int2 => (1, 0),
                };
                self.intmap[on] = (self.intmap[on] & !all) | events;
                self.intmap[off] &= !all;
                self.write_intmaps()$($await)*?;
                self.power_ctl = (self.power_ctl & !POWER_CTL_AUTOSLEEP)
                    | if config.auto_sleep { POWER_CTL_AUTOSLEEP } else { 0 };
                self.resume()$($await)*
            }

            /// Unmap every function from `pin` (it goes high-impedance).
            /// Detection settings are kept.
            pub $($async)? fn disable_interrupt(&mut self, pin: IntPin) -> Result<(), Error<SPI::Error>> {
                let (i, reg) = match pin {
                    IntPin::Int1 => (0, REG_INTMAP1),
                    IntPin::Int2 => (1, REG_INTMAP2),
                };
                self.intmap[i] &= INTMAP_INT_LOW;
                self.write_reg(reg, self.intmap[i])$($await)*
            }

            /// Read STATUS, which clears latched ACT/INACT events. Events
            /// already seen by [`Sensor::read_measurement`] are merged in.
            ///
            /// [`Sensor::read_measurement`]: zigbee_sensor::blocking::Sensor::read_measurement
            pub $($async)? fn read_interrupt_source(&mut self) -> Result<InterruptSource, Error<SPI::Error>> {
                let mut status = [0u8];
                self.read_regs(REG_STATUS, &mut status)$($await)*?;
                let status = status[0] | self.pending;
                self.pending = 0;
                Ok(InterruptSource { status })
            }
        }

        impl<SPI: $Spi> $Sensor for Adxl362<SPI> {
            type BusError = SPI::Error;
            type Measurement = Measurement;

            /// Leave standby if needed (returns 4/ODR), else `0`.
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<SPI::Error>> {
                if self.active {
                    return Ok(0);
                }
                self.power_up()$($await)*
            }

            /// Reads STATUS, FIFO_ENTRIES and XDATA..ZDATA in one burst.
            /// ACT/INACT events cleared by that STATUS read are kept for
            /// [`Adxl362::read_interrupt_source`].
            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<SPI::Error>> {
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

        impl<SPI: $Spi> $LowPower for Adxl362<SPI> {
            type BusError = SPI::Error;

            /// Standby (10 nA). Pending events are kept but no new ones are
            /// generated; use [`PowerMode::WakeUp`] to wait for motion.
            $($async)? fn sleep(&mut self) -> Result<(), Error<SPI::Error>> {
                self.write_reg(REG_POWER_CTL, self.power_ctl)$($await)*?;
                self.active = false;
                Ok(())
            }

            /// Resume measuring; returns 4/ODR.
            $($async)? fn wake(&mut self) -> Result<WaitUs, Error<SPI::Error>> {
                self.power_up()$($await)*
            }
        }
    };
}

driver!(
    embedded_hal::spi::SpiDevice,
    zigbee_sensor::blocking::Sensor,
    zigbee_sensor::blocking::LowPower,
    [],
    []
);

/// Async driver.
pub mod asynch {
    use super::*;

    driver!(
        embedded_hal_async::spi::SpiDevice,
        zigbee_sensor::asynch::Sensor,
        zigbee_sensor::asynch::LowPower,
        [async],
        [.await]
    );
}

#[cfg(test)]
mod tests;
