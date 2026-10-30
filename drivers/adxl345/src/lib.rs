//! Analog Devices ADXL343 / ADXL345 three-axis accelerometer (I2C or SPI).
//!
//! Both parts share the register map and device ID (`0xE5`). Data is
//! read in full-resolution mode (256 LSB/g, ≈ 3.9 mg/LSB in every range).
//!
//! The part only converts continuously; there is no single-shot mode.
//! [`Sensor::start_measurement`] leaves standby (returns τ + 1.1 ms) and
//! [`Sensor::read_measurement`] returns [`Error::NotReady`] until
//! `INT_SOURCE.DATA_READY` is set. [`LowPower::sleep`] is standby (0.1 µA).
//!
//! For a sleepy Zigbee end device, keep the part measuring at a low rate
//! (e.g. 12.5 Hz in reduced-power mode, 34 µA, or ≤ 6.25 Hz, 23–45 µA) with
//! the activity interrupt ([`WakeUpConfig`]) routed to INT1. With an
//! inactivity threshold, link and [`WakeUpConfig::auto_sleep`] the part
//! drops to an 8–1 Hz sleep rate when still and wakes on activity.
//!
//! Reference: Analog Devices "ADXL345 Data Sheet" Rev. E and
//! "ADXL343 Data Sheet" Rev. 0.
//!
//! [`Sensor::start_measurement`]: zigbee_sensor::blocking::Sensor::start_measurement
//! [`Sensor::read_measurement`]: zigbee_sensor::blocking::Sensor::read_measurement
//! [`LowPower::sleep`]: zigbee_sensor::blocking::LowPower::sleep
//!
//! ```ignore
//! let mut acc = Adxl345::new_i2c(i2c, ADDRESS_ALT_LOW);
//! acc.probe()?;
//! acc.configure(Config::default())?;
//! acc.configure_wake_up(&WakeUpConfig { threshold_mg: 250, ..Default::default() })?;
//! // ... sleep until INT1, then:
//! let src = acc.read_interrupt_source()?; // clears the activity latch
//! ```

#![no_std]

mod interface;

pub use interface::{AsyncInterface, I2cInterface, Interface, SpiInterface};
use zigbee_sensor::{Error, Quantity, Readings, Vector3, WaitUs};

/// I2C address with ALT ADDRESS (pin 12) low.
pub const ADDRESS_ALT_LOW: u8 = 0x53;
/// I2C address with ALT ADDRESS (pin 12) high.
pub const ADDRESS_ALT_HIGH: u8 = 0x1D;

/// DEVID value (ADXL343 and ADXL345).
pub const DEVICE_ID: u8 = 0xE5;

const REG_DEVID: u8 = 0x00;
const REG_THRESH_TAP: u8 = 0x1D;
const REG_DUR: u8 = 0x21;
const REG_LATENT: u8 = 0x22;
const REG_WINDOW: u8 = 0x23;
const REG_THRESH_ACT: u8 = 0x24;
const REG_THRESH_INACT: u8 = 0x25;
const REG_TIME_INACT: u8 = 0x26;
const REG_ACT_INACT_CTL: u8 = 0x27;
const REG_TAP_AXES: u8 = 0x2A;
const REG_ACT_TAP_STATUS: u8 = 0x2B;
const REG_BW_RATE: u8 = 0x2C;
const REG_POWER_CTL: u8 = 0x2D;
const REG_INT_ENABLE: u8 = 0x2E;
const REG_INT_MAP: u8 = 0x2F;
const REG_INT_SOURCE: u8 = 0x30;
const REG_DATA_FORMAT: u8 = 0x31;

const BW_RATE_LOW_POWER: u8 = 0x10;
const POWER_CTL_LINK: u8 = 0x20;
const POWER_CTL_AUTO_SLEEP: u8 = 0x10;
const POWER_CTL_MEASURE: u8 = 0x08;
const DATA_FORMAT_INT_INVERT: u8 = 0x20;
const DATA_FORMAT_FULL_RES: u8 = 0x08;
const ACT_INACT_CTL_ACT_AC: u8 = 0x80;
const ACT_INACT_CTL_ACT_XYZ: u8 = 0x70;
const ACT_INACT_CTL_INACT_AC: u8 = 0x08;
const ACT_INACT_CTL_INACT_XYZ: u8 = 0x07;
const TAP_AXES_XYZ: u8 = 0x07;

const INT_DATA_READY: u8 = 0x80;
const INT_SINGLE_TAP: u8 = 0x40;
const INT_DOUBLE_TAP: u8 = 0x20;
const INT_ACTIVITY: u8 = 0x10;
const INT_INACTIVITY: u8 = 0x08;
const INT_FREE_FALL: u8 = 0x04;
/// INT_SOURCE bits cleared by reading INT_SOURCE (not by reading data).
const INT_EVENTS: u8 =
    INT_SINGLE_TAP | INT_DOUBLE_TAP | INT_ACTIVITY | INT_INACTIVITY | INT_FREE_FALL;

/// g range (DATA_FORMAT Range bits, Table 21). Full resolution keeps
/// 256 LSB/g; the range only sets the clipping point.
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

/// Output data rate (BW_RATE Rate code, Table 7). Typical supply current
/// at 2.5 V in normal / reduced-power mode is given per rate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DataRate {
    /// 0.10 Hz, 23 µA.
    Hz0_10 = 0,
    /// 0.20 Hz, 23 µA.
    Hz0_20 = 1,
    /// 0.39 Hz, 23 µA.
    Hz0_39 = 2,
    /// 0.78 Hz, 23 µA.
    Hz0_78 = 3,
    /// 1.56 Hz, 34 µA.
    Hz1_56 = 4,
    /// 3.13 Hz, 40 µA.
    Hz3_13 = 5,
    /// 6.25 Hz, 45 µA.
    Hz6_25 = 6,
    /// 12.5 Hz, 50 / 34 µA.
    #[default]
    Hz12_5 = 7,
    /// 25 Hz, 60 / 40 µA.
    Hz25 = 8,
    /// 50 Hz, 90 / 45 µA.
    Hz50 = 9,
    /// 100 Hz, 140 / 50 µA.
    Hz100 = 10,
    /// 200 Hz, 140 / 60 µA.
    Hz200 = 11,
    /// 400 Hz, 140 / 90 µA.
    Hz400 = 12,
    /// 800 Hz, 140 µA.
    Hz800 = 13,
    /// 1600 Hz, 90 µA.
    Hz1600 = 14,
    /// 3200 Hz, 140 µA.
    Hz3200 = 15,
}

impl DataRate {
    /// Sample period τ in µs (3200 Hz / 2^(15 − code)), rounded up.
    pub const fn period_us(self) -> u32 {
        (625u32 << (15 - self as u32)).div_ceil(2)
    }
}

/// Measurement configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// g range.
    pub range: Range,
    /// Output data rate.
    pub rate: DataRate,
    /// BW_RATE.LOW_POWER: reduced-power operation with slightly more noise;
    /// only saves current at 12.5–400 Hz (Table 8).
    pub low_power: bool,
    /// INT1/INT2 active low (DATA_FORMAT.INT_INVERT, default active high).
    pub int_active_low: bool,
}

impl Config {
    /// 12.5 Hz, reduced power (34 µA), ±2 g, active-high interrupts.
    pub const DEFAULT: Self = Self {
        range: Range::G2,
        rate: DataRate::Hz12_5,
        low_power: true,
        int_active_low: false,
    };
}

impl Default for Config {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Turn-on and wake-up time: τ + 1.1 ms (datasheet Table 1, note 7).
pub const fn turn_on_us(rate: DataRate) -> WaitUs {
    rate.period_us() + 1_100
}

/// Interrupt output pin (INT_MAP).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum IntPin {
    /// INT1.
    #[default]
    Int1,
    /// INT2.
    Int2,
}

/// Sampling rate while auto-sleep has put the part to sleep
/// (POWER_CTL Wakeup bits, Table 20).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SleepRate {
    /// 8 Hz.
    #[default]
    Hz8,
    /// 4 Hz.
    Hz4,
    /// 2 Hz.
    Hz2,
    /// 1 Hz.
    Hz1,
}

/// Inactivity detection: every axis below the threshold for `time_s`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InactivityConfig {
    /// Threshold in mg (62.5 mg/LSB).
    pub threshold_mg: u16,
    /// Time in seconds (1 s/LSB).
    pub time_s: u8,
    /// AC-coupled (referenced) instead of absolute comparison.
    pub referenced: bool,
}

impl Default for InactivityConfig {
    fn default() -> Self {
        Self {
            threshold_mg: 125,
            time_s: 5,
            referenced: true,
        }
    }
}

/// Activity (motion) interrupt on any axis, optionally paired with
/// inactivity detection and auto-sleep.
///
/// Interrupts are always latched by the part until
/// [`Adxl345::read_interrupt_source`]; activity has no duration filter
/// (one sample above the threshold triggers).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WakeUpConfig {
    /// Activity threshold in mg (62.5 mg/LSB, at least one step).
    pub threshold_mg: u16,
    /// AC-coupled (referenced to the acceleration when detection starts)
    /// instead of absolute magnitude. Use for tilt-insensitive motion.
    pub referenced: bool,
    /// Pin for both activity and inactivity.
    pub pin: IntPin,
    /// Inactivity detection (interrupt on the same pin). `None` disables it.
    pub inactivity: Option<InactivityConfig>,
    /// POWER_CTL.Link: activity is only armed after inactivity and vice
    /// versa (one interrupt per motion start / stop). Needs `inactivity`.
    pub link: bool,
    /// POWER_CTL.AUTO_SLEEP: sleep at this rate while inactive, wake on
    /// activity. Implies `link`; needs `inactivity`.
    pub auto_sleep: Option<SleepRate>,
}

impl Default for WakeUpConfig {
    fn default() -> Self {
        Self {
            threshold_mg: 250,
            referenced: true,
            pin: IntPin::Int1,
            inactivity: None,
            link: false,
            auto_sleep: None,
        }
    }
}

/// Single/double tap detection on all axes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TapConfig {
    /// Threshold in mg (62.5 mg/LSB).
    pub threshold_mg: u16,
    /// Maximum time above the threshold in µs (625 µs/LSB, ≤ 159 375).
    pub duration_us: u32,
    /// Double tap: (latency, window) in ms (1.25 ms/LSB, ≤ 318).
    /// `None` enables single tap only.
    pub double: Option<(u16, u16)>,
    /// Output pin.
    pub pin: IntPin,
}

impl Default for TapConfig {
    fn default() -> Self {
        Self {
            threshold_mg: 3000,
            duration_us: 10_000,
            double: None,
            pin: IntPin::Int1,
        }
    }
}

/// ACT_TAP_STATUS and INT_SOURCE.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct InterruptSource {
    /// INT_SOURCE (30h), including events consumed by earlier
    /// [`Sensor::read_measurement`] calls.
    ///
    /// [`Sensor::read_measurement`]: zigbee_sensor::blocking::Sensor::read_measurement
    pub int_source: u8,
    /// ACT_TAP_STATUS (2Bh): first axes involved and the Asleep bit.
    pub act_tap_status: u8,
}

impl InterruptSource {
    /// Activity event.
    pub const fn activity(self) -> bool {
        self.int_source & INT_ACTIVITY != 0
    }
    /// Inactivity event.
    pub const fn inactivity(self) -> bool {
        self.int_source & INT_INACTIVITY != 0
    }
    /// Single tap.
    pub const fn single_tap(self) -> bool {
        self.int_source & INT_SINGLE_TAP != 0
    }
    /// Double tap.
    pub const fn double_tap(self) -> bool {
        self.int_source & INT_DOUBLE_TAP != 0
    }
    /// Free fall.
    pub const fn free_fall(self) -> bool {
        self.int_source & INT_FREE_FALL != 0
    }
    /// Activity involved X.
    pub const fn activity_x(self) -> bool {
        self.act_tap_status & 0x40 != 0
    }
    /// Activity involved Y.
    pub const fn activity_y(self) -> bool {
        self.act_tap_status & 0x20 != 0
    }
    /// Activity involved Z.
    pub const fn activity_z(self) -> bool {
        self.act_tap_status & 0x10 != 0
    }
    /// Auto-sleep has put the part to sleep.
    pub const fn asleep(self) -> bool {
        self.act_tap_status & 0x08 != 0
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

/// Convert one right-justified full-resolution sample (256 LSB/g) to mg.
pub const fn raw_to_mg(raw: i16) -> i32 {
    (raw as i32) * 1000 / 256
}

/// Decode DATAX0..DATAZ1 to milli-g.
pub fn decode(frame: &[u8; 6]) -> Vector3 {
    let axis = |i: usize| raw_to_mg(i16::from_le_bytes([frame[i], frame[i + 1]]));
    Vector3 {
        x: axis(0),
        y: axis(2),
        z: axis(4),
    }
}

/// THRESH_ACT / THRESH_INACT / THRESH_TAP value for a threshold in mg
/// (62.5 mg/LSB, rounded, clamped to 1..=255 since 0 is not allowed).
pub fn threshold_reg(threshold_mg: u16) -> u8 {
    ((u32::from(threshold_mg) * 2 + 62) / 125).clamp(1, 255) as u8
}

fn scaled(value: u32, step: u32) -> u8 {
    ((value + step / 2) / step).clamp(1, 255) as u8
}

const fn pin_bits(pin: IntPin, bits: u8, map: u8) -> u8 {
    match pin {
        IntPin::Int1 => map & !bits,
        IntPin::Int2 => map | bits,
    }
}

macro_rules! driver {
    ($Iface:ident, $Sensor:path, $LowPower:path, [$($async:tt)?], [$($await:tt)*]) => {
        /// ADXL343 / ADXL345 driver.
        pub struct Adxl345<IFACE> {
            iface: IFACE,
            config: Config,
            power_ctl: u8,
            int_enable: u8,
            int_map: u8,
            pending: u8,
            active: bool,
        }

        impl<IFACE> Adxl345<IFACE> {
            /// Create a driver over a register interface; no bus traffic.
            pub const fn new(iface: IFACE) -> Self {
                Self {
                    iface,
                    config: Config::DEFAULT,
                    power_ctl: 0,
                    int_enable: 0,
                    int_map: 0,
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

        impl<I2C> Adxl345<I2cInterface<I2C>> {
            /// Driver on an I2C bus ([`ADDRESS_ALT_LOW`] or [`ADDRESS_ALT_HIGH`]).
            pub const fn new_i2c(i2c: I2C, address: u8) -> Self {
                Self::new(I2cInterface::new(i2c, address))
            }
        }

        impl<SPI> Adxl345<SpiInterface<SPI>> {
            /// Driver on a 4-wire SPI device (mode 3, up to 5 MHz).
            pub const fn new_spi(spi: SPI) -> Self {
                Self::new(SpiInterface::new(spi))
            }
        }

        impl<IFACE: $Iface> Adxl345<IFACE> {
            $($async)? fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error<IFACE::Error>> {
                self.iface.write_reg(reg, value)$($await)*.map_err(Error::bus)
            }

            $($async)? fn read_reg(&mut self, reg: u8) -> Result<u8, Error<IFACE::Error>> {
                let mut b = [0u8];
                self.iface.read_regs(reg, &mut b)$($await)*.map_err(Error::bus)?;
                Ok(b[0])
            }

            /// Check DEVID.
            pub $($async)? fn probe(&mut self) -> Result<(), Error<IFACE::Error>> {
                let id = self.read_reg(REG_DEVID)$($await)*?;
                if id != DEVICE_ID {
                    return Err(Error::UnexpectedId(u32::from(id)));
                }
                Ok(())
            }

            /// Apply the configuration and start measuring; returns the
            /// turn-on time.
            pub $($async)? fn configure(&mut self, config: Config) -> Result<WaitUs, Error<IFACE::Error>> {
                self.config = config;
                let format = DATA_FORMAT_FULL_RES
                    | config.range as u8
                    | if config.int_active_low { DATA_FORMAT_INT_INVERT } else { 0 };
                let bw = config.rate as u8 | if config.low_power { BW_RATE_LOW_POWER } else { 0 };
                self.write_reg(REG_DATA_FORMAT, format)$($await)*?;
                self.write_reg(REG_BW_RATE, bw)$($await)*?;
                self.power_up()$($await)*
            }

            $($async)? fn power_up(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
                self.write_reg(REG_POWER_CTL, self.power_ctl | POWER_CTL_MEASURE)$($await)*?;
                self.active = true;
                Ok(turn_on_us(self.config.rate))
            }

            $($async)? fn write_int_enable(&mut self, value: u8) -> Result<(), Error<IFACE::Error>> {
                self.int_enable = value;
                self.write_reg(REG_INT_ENABLE, value)$($await)*
            }

            /// Configure activity (and optionally inactivity / auto-sleep)
            /// detection and enable its interrupt. The interrupt is disabled
            /// while the function is reprogrammed, as the datasheet advises.
            ///
            /// `link` or `auto_sleep` without `inactivity` is
            /// [`Error::Unsupported`].
            pub $($async)? fn configure_wake_up(&mut self, config: &WakeUpConfig) -> Result<(), Error<IFACE::Error>> {
                let link = config.link || config.auto_sleep.is_some();
                if link && config.inactivity.is_none() {
                    return Err(Error::Unsupported);
                }
                self.write_int_enable(self.int_enable & !(INT_ACTIVITY | INT_INACTIVITY))$($await)*?;
                self.write_reg(REG_THRESH_ACT, threshold_reg(config.threshold_mg))$($await)*?;
                let mut ctl = ACT_INACT_CTL_ACT_XYZ
                    | if config.referenced { ACT_INACT_CTL_ACT_AC } else { 0 };
                let mut enable = INT_ACTIVITY;
                if let Some(inact) = config.inactivity {
                    self.write_reg(REG_THRESH_INACT, threshold_reg(inact.threshold_mg))$($await)*?;
                    self.write_reg(REG_TIME_INACT, inact.time_s)$($await)*?;
                    ctl |= ACT_INACT_CTL_INACT_XYZ
                        | if inact.referenced { ACT_INACT_CTL_INACT_AC } else { 0 };
                    enable |= INT_INACTIVITY;
                }
                self.write_reg(REG_ACT_INACT_CTL, ctl)$($await)*?;
                self.int_map = pin_bits(config.pin, INT_ACTIVITY | INT_INACTIVITY, self.int_map);
                self.write_reg(REG_INT_MAP, self.int_map)$($await)*?;

                let power_ctl = match (link, config.auto_sleep) {
                    (_, Some(rate)) => POWER_CTL_LINK | POWER_CTL_AUTO_SLEEP | rate as u8,
                    (true, None) => POWER_CTL_LINK,
                    (false, None) => 0,
                };
                if power_ctl != self.power_ctl {
                    let cleared = self.power_ctl & !power_ctl & (POWER_CTL_LINK | POWER_CTL_AUTO_SLEEP);
                    self.power_ctl = power_ctl;
                    if self.active {
                        // Datasheet: go through standby when clearing Link
                        // or AUTO_SLEEP so the part is re-biased.
                        if cleared != 0 {
                            self.write_reg(REG_POWER_CTL, 0)$($await)*?;
                        }
                        self.write_reg(REG_POWER_CTL, power_ctl | POWER_CTL_MEASURE)$($await)*?;
                    }
                }
                self.write_int_enable(self.int_enable | enable)$($await)*
            }

            /// Configure single (and optionally double) tap detection on all
            /// axes and enable its interrupt.
            pub $($async)? fn configure_tap(&mut self, config: &TapConfig) -> Result<(), Error<IFACE::Error>> {
                self.write_int_enable(self.int_enable & !(INT_SINGLE_TAP | INT_DOUBLE_TAP))$($await)*?;
                self.write_reg(REG_THRESH_TAP, threshold_reg(config.threshold_mg))$($await)*?;
                self.write_reg(REG_DUR, scaled(config.duration_us, 625))$($await)*?;
                let (latent, window, enable) = match config.double {
                    Some((latency_ms, window_ms)) => (
                        scaled(u32::from(latency_ms) * 4, 5),
                        scaled(u32::from(window_ms) * 4, 5),
                        INT_SINGLE_TAP | INT_DOUBLE_TAP,
                    ),
                    None => (0, 0, INT_SINGLE_TAP),
                };
                self.write_reg(REG_LATENT, latent)$($await)*?;
                self.write_reg(REG_WINDOW, window)$($await)*?;
                self.write_reg(REG_TAP_AXES, TAP_AXES_XYZ)$($await)*?;
                self.int_map = pin_bits(config.pin, INT_SINGLE_TAP | INT_DOUBLE_TAP, self.int_map);
                self.write_reg(REG_INT_MAP, self.int_map)$($await)*?;
                self.write_int_enable(self.int_enable | enable)$($await)*
            }

            /// Disable every interrupt currently mapped to `pin`. Thresholds,
            /// link and auto-sleep are kept.
            pub $($async)? fn disable_interrupt(&mut self, pin: IntPin) -> Result<(), Error<IFACE::Error>> {
                let on_pin = match pin {
                    IntPin::Int1 => !self.int_map,
                    IntPin::Int2 => self.int_map,
                };
                self.write_int_enable(self.int_enable & !on_pin)$($await)*
            }

            /// Read ACT_TAP_STATUS, then INT_SOURCE (which clears the
            /// latched events and releases the pins). Events already seen by
            /// [`Sensor::read_measurement`] are merged in.
            ///
            /// [`Sensor::read_measurement`]: zigbee_sensor::blocking::Sensor::read_measurement
            pub $($async)? fn read_interrupt_source(&mut self) -> Result<InterruptSource, Error<IFACE::Error>> {
                let act_tap_status = self.read_reg(REG_ACT_TAP_STATUS)$($await)*?;
                let int_source = self.read_reg(REG_INT_SOURCE)$($await)*? | self.pending;
                self.pending = 0;
                Ok(InterruptSource { int_source, act_tap_status })
            }
        }

        impl<IFACE: $Iface> $Sensor for Adxl345<IFACE> {
            type BusError = IFACE::Error;
            type Measurement = Measurement;

            /// Leave standby if needed (returns τ + 1.1 ms), else `0`.
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
                if self.active {
                    return Ok(0);
                }
                self.power_up()$($await)*
            }

            /// Reads INT_SOURCE, DATA_FORMAT and the data registers in one
            /// burst. Motion events cleared by that INT_SOURCE read are kept
            /// for [`Adxl345::read_interrupt_source`]. While auto-sleep has
            /// the part asleep DATA_READY is suppressed, so this returns
            /// [`Error::NotReady`].
            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<IFACE::Error>> {
                let mut buf = [0u8; 8];
                self.iface.read_regs(REG_INT_SOURCE, &mut buf)$($await)*.map_err(Error::bus)?;
                self.pending |= buf[0] & INT_EVENTS;
                if buf[0] & INT_DATA_READY == 0 {
                    return Err(Error::NotReady);
                }
                let mut frame = [0u8; 6];
                frame.copy_from_slice(&buf[2..]);
                Ok(Measurement { acceleration: decode(&frame) })
            }
        }

        impl<IFACE: $Iface> $LowPower for Adxl345<IFACE> {
            type BusError = IFACE::Error;

            /// Standby (0.1 µA); no detection. For wake-on-motion keep the
            /// part measuring with [`WakeUpConfig::auto_sleep`] instead.
            $($async)? fn sleep(&mut self) -> Result<(), Error<IFACE::Error>> {
                self.write_reg(REG_POWER_CTL, 0)$($await)*?;
                self.active = false;
                Ok(())
            }

            /// Resume measuring; returns τ + 1.1 ms.
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
