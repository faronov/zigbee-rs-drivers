//! ST LIS2DU12 ultra-low-power three-axis accelerometer with anti-aliasing
//! filter (I2C or SPI).
//!
//! Two conversion schemes are supported (see [`Conversion`]):
//!
//! * [`Conversion::OnDemand`] (default): one-shot triggered over the bus
//!   (`CTRL5.ODR = 1111`). [`Sensor::start_measurement`] sets `CTRL4.SOC`
//!   and returns [`Config::one_shot_wait_us`]; the part idles at 0.2 µA
//!   between conversions. [`Sensor::read_measurement`] returns
//!   [`Error::NotReady`] until `STATUS.DRDY` is set.
//! * [`Conversion::Continuous`]: free-running at [`Config::rate`].
//!   Required for the wake-up, activity/inactivity and 6D interrupts.
//!
//! For a sleepy Zigbee end device, run continuously at
//! [`DataRate::UltraLowPowerHz1_6`] (0.47 µA) with [`WakeUpConfig`] routed
//! to INT1 or INT2 and sleep the MCU until the pin fires. [`LowPower::sleep`]
//! is a full power-down (20 nA) and stops the detector.
//!
//! `CTRL1.IF_ADD_INC` is cleared at reset, so call [`Lis2du12::probe`] or
//! [`Lis2du12::configure`] (both set it) before any burst read.
//!
//! Reference: ST "LIS2DU12 datasheet", DS12422 Rev 4 (November 2022).
//!
//! [`Sensor::start_measurement`]: zigbee_sensor::blocking::Sensor::start_measurement
//! [`Sensor::read_measurement`]: zigbee_sensor::blocking::Sensor::read_measurement
//! [`LowPower::sleep`]: zigbee_sensor::blocking::LowPower::sleep
//!
//! ```ignore
//! let mut acc = Lis2du12::new_i2c(i2c, ADDRESS_SA0_LOW);
//! acc.probe()?;
//! acc.configure(Config { conversion: Conversion::Continuous, ..Default::default() })?;
//! acc.configure_wake_up(&WakeUpConfig { threshold_mg: 50, ..Default::default() })?;
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
pub const WHO_AM_I: u8 = 0x45;

const REG_CTRL1: u8 = 0x10;
const REG_CTRL4: u8 = 0x13;
const REG_CTRL5: u8 = 0x14;
const REG_INTERRUPT_CFG: u8 = 0x17;
const REG_TAP_THS_X: u8 = 0x18;
const REG_WAKE_UP_THS: u8 = 0x1C;
const REG_WAKE_UP_DUR: u8 = 0x1D;
const REG_MD1_CFG: u8 = 0x1F;
const REG_MD2_CFG: u8 = 0x20;
const REG_WAKE_UP_SRC: u8 = 0x21;
const REG_STATUS: u8 = 0x25;
const REG_WHO_AM_I: u8 = 0x43;

const CTRL1_IF_ADD_INC: u8 = 0x10;
const CTRL1_WU_XYZ: u8 = 0x07;
const CTRL4_BDU: u8 = 0x20;
const CTRL4_SOC: u8 = 0x02;
const CTRL5_ODR_ONE_SHOT: u8 = 0xF0;
const INT_CFG_INT_SHORT_EN: u8 = 0x40;
const INT_CFG_WAKE_THS_W: u8 = 0x20;
const INT_CFG_H_LACTIVE: u8 = 0x04;
const INT_CFG_LIR: u8 = 0x02;
const INT_CFG_INTERRUPTS_ENABLE: u8 = 0x01;
const MD_WU: u8 = 0x20;
const MD_6D: u8 = 0x04;
const WAKE_UP_THS_SLEEP_ON: u8 = 0x40;
const STATUS_DRDY: u8 = 0x01;

/// Full-scale range (CTRL5 FS\[1:0\]).
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

/// Continuous output data rate and power mode (CTRL5 ODR\[3:0\], Table 34).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DataRate {
    /// 1.6 Hz, ultralow-power mode (0.47 µA).
    #[default]
    UltraLowPowerHz1_6 = 1,
    /// 3 Hz, ultralow-power mode.
    UltraLowPowerHz3 = 2,
    /// 6 Hz, ultralow-power mode.
    UltraLowPowerHz6 = 3,
    /// 6 Hz, normal mode (anti-aliasing filter on).
    Hz6 = 4,
    /// 12.5 Hz, normal mode.
    Hz12_5 = 5,
    /// 25 Hz, normal mode.
    Hz25 = 6,
    /// 50 Hz, normal mode.
    Hz50 = 7,
    /// 100 Hz, normal mode.
    Hz100 = 8,
    /// 200 Hz, normal mode.
    Hz200 = 9,
    /// 400 Hz, normal mode.
    Hz400 = 10,
    /// 800 Hz, normal mode (6.1 µA).
    Hz800 = 11,
}

impl DataRate {
    /// Sample period in µs, rounded up.
    pub const fn period_us(self) -> u32 {
        match self {
            Self::UltraLowPowerHz1_6 => 625_000,
            Self::UltraLowPowerHz3 => 333_334,
            Self::UltraLowPowerHz6 | Self::Hz6 => 166_667,
            Self::Hz12_5 => 80_000,
            Self::Hz25 => 40_000,
            Self::Hz50 => 20_000,
            Self::Hz100 => 10_000,
            Self::Hz200 => 5_000,
            Self::Hz400 => 2_500,
            Self::Hz800 => 1_250,
        }
    }
}

/// How samples are produced.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Conversion {
    /// One-shot via the interface (`CTRL5.ODR = 1111`, `CTRL4.SOC`).
    /// No motion detection between conversions.
    #[default]
    OnDemand,
    /// Free-running at [`Config::rate`].
    Continuous,
}

/// Default for [`Config::one_shot_wait_us`].
pub const ONE_SHOT_WAIT_US: WaitUs = 5_000;

/// Measurement configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Full-scale range.
    pub range: Range,
    /// Continuous output data rate.
    pub rate: DataRate,
    /// One-shot or continuous conversion.
    pub conversion: Conversion,
    /// INT1/INT2 active low (default active high).
    pub int_active_low: bool,
    /// Wait returned by `start_measurement` in on-demand mode. DS12422 does
    /// not specify the one-shot conversion time; the default
    /// ([`ONE_SHOT_WAIT_US`]) is a conservative estimate.
    pub one_shot_wait_us: WaitUs,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            range: Range::G2,
            rate: DataRate::UltraLowPowerHz1_6,
            conversion: Conversion::OnDemand,
            int_active_low: false,
            one_shot_wait_us: ONE_SHOT_WAIT_US,
        }
    }
}

/// Time from power-down to the first continuous sample. Not specified in
/// DS12422: this driver waits two sample periods (one discarded sample at
/// BW = ODR/2, Tables 35/36).
pub const fn turn_on_us(rate: DataRate) -> WaitUs {
    2 * rate.period_us()
}

/// Interrupt output pin (MD1_CFG / MD2_CFG routing).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum IntPin {
    /// INT1.
    #[default]
    Int1,
    /// INT2.
    Int2,
}

/// Output data rate while inactive (CTRL4 INACT_ODR\[1:0\]).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum InactiveRate {
    /// Android stationary detection: report inactivity, keep the ODR.
    Stationary,
    /// 6 Hz.
    Hz6,
    /// 3 Hz.
    Hz3,
    /// 1.6 Hz.
    #[default]
    Hz1_6,
}

/// Activity/inactivity (auto-sleep) settings.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct AutoSleep {
    /// Time without motion before sleeping: `512 × n / ODR`
    /// (n = 0..=15, 0 means 16/ODR).
    pub duration: u8,
    /// ODR used while inactive.
    pub rate: InactiveRate,
}

/// Wake-up (motion) interrupt: any axis of the high-pass filtered
/// acceleration above the threshold for `duration` samples.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WakeUpConfig {
    /// Threshold in mg. 1 LSB is FS/256 when the value fits in 63 such
    /// steps, FS/64 otherwise (`WAKE_THS_W`).
    pub threshold_mg: u16,
    /// Minimum event duration in samples (1/ODR each), 0..=3.
    pub duration: u8,
    /// Output pin.
    pub pin: IntPin,
    /// Latch interrupts until [`Lis2du12::read_interrupt_source`]
    /// (`LIR` + `INT_SHORT_EN`, shared by all interrupt functions).
    pub latch: bool,
    /// Activity/inactivity: drop to a lower ODR without motion, return on
    /// wake-up. `None` disables it.
    pub auto_sleep: Option<AutoSleep>,
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

/// 6D threshold angle (TAP_THS_X D6D_THS\[1:0\]).
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

/// 6D/4D orientation interrupt (portrait/landscape/face up/down).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct OrientationConfig {
    /// Angle threshold.
    pub threshold: OrientationThreshold,
    /// Ignore the Z axis (4D).
    pub four_d: bool,
    /// Output pin.
    pub pin: IntPin,
}

/// WAKE_UP_SRC, TAP_SRC, SIXD_SRC and ALL_INT_SRC, read in one burst.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct InterruptSource {
    /// WAKE_UP_SRC (21h).
    pub wake_up_src: u8,
    /// TAP_SRC (22h).
    pub tap_src: u8,
    /// SIXD_SRC (23h): ZH ZL YH YL XH XL in bits 5..0.
    pub sixd_src: u8,
    /// ALL_INT_SRC (24h).
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
    /// The part is in the inactivity state.
    pub const fn sleep_state(self) -> bool {
        self.wake_up_src & 0x10 != 0
    }
    /// Activity ↔ inactivity transition.
    pub const fn sleep_change(self) -> bool {
        self.wake_up_src & 0x40 != 0
    }
    /// Free-fall event.
    pub const fn free_fall(self) -> bool {
        self.wake_up_src & 0x20 != 0
    }
    /// Orientation change (D6D_IA).
    pub const fn orientation_change(self) -> bool {
        self.sixd_src & 0x40 != 0
    }
    /// Any event (INT_GLOBAL).
    pub const fn any(self) -> bool {
        self.all_int_src & 0x40 != 0
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

/// Convert one left-justified 12-bit output word to milli-g
/// (0.976 mg/digit at ±2 g, Table 2).
pub const fn raw_to_mg(raw: i16, range: Range) -> i32 {
    ((raw as i32) >> 4) * (range.full_scale_mg() as i32) / 2048
}

/// Decode OUT_X_L..OUT_Z_H to milli-g.
pub fn decode(frame: &[u8; 6], range: Range) -> Vector3 {
    let axis = |i: usize| raw_to_mg(i16::from_le_bytes([frame[i], frame[i + 1]]), range);
    Vector3 {
        x: axis(0),
        y: axis(2),
        z: axis(4),
    }
}

/// WK_THS value and `WAKE_THS_W` flag for a threshold in mg: FS/256 steps
/// when they fit in six bits, else FS/64 steps (rounded, at most 63).
pub fn threshold_reg(threshold_mg: u16, range: Range) -> (u8, bool) {
    let fs = range.full_scale_mg();
    let fine = (u32::from(threshold_mg) * 256 + fs / 2) / fs;
    if fine <= 63 {
        return (fine as u8, true);
    }
    let coarse = (u32::from(threshold_mg) * 64 + fs / 2) / fs;
    (coarse.min(63) as u8, false)
}

macro_rules! driver {
    ($Iface:ident, $Sensor:path, $LowPower:path, [$($async:tt)?], [$($await:tt)*]) => {
        /// LIS2DU12 driver.
        pub struct Lis2du12<IFACE> {
            iface: IFACE,
            config: Config,
            ctrl4: u8,
            int_cfg: u8,
            md1: u8,
            md2: u8,
            active: bool,
        }

        impl<IFACE> Lis2du12<IFACE> {
            /// Create a driver over a register interface; no bus traffic.
            pub const fn new(iface: IFACE) -> Self {
                Self {
                    iface,
                    config: Config {
                        range: Range::G2,
                        rate: DataRate::UltraLowPowerHz1_6,
                        conversion: Conversion::OnDemand,
                        int_active_low: false,
                        one_shot_wait_us: ONE_SHOT_WAIT_US,
                    },
                    ctrl4: CTRL4_BDU,
                    int_cfg: 0,
                    md1: 0,
                    md2: 0,
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

            const fn ctrl5(&self) -> u8 {
                let odr = match self.config.conversion {
                    Conversion::OnDemand => CTRL5_ODR_ONE_SHOT,
                    Conversion::Continuous => (self.config.rate as u8) << 4,
                };
                odr | self.config.range.bits()
            }
        }

        impl<I2C> Lis2du12<I2cInterface<I2C>> {
            /// Driver on an I2C bus ([`ADDRESS_SA0_LOW`] or [`ADDRESS_SA0_HIGH`]).
            pub const fn new_i2c(i2c: I2C, address: u8) -> Self {
                Self::new(I2cInterface::new(i2c, address))
            }
        }

        impl<SPI> Lis2du12<SpiInterface<SPI>> {
            /// Driver on an SPI device (mode 0 or 3, up to 10 MHz).
            pub const fn new_spi(spi: SPI) -> Self {
                Self::new(SpiInterface::new(spi))
            }
        }

        impl<IFACE: $Iface> Lis2du12<IFACE> {
            $($async)? fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error<IFACE::Error>> {
                self.iface.write_reg(reg, value)$($await)*.map_err(Error::bus)
            }

            /// Check WHO_AM_I, then enable register auto-increment and the
            /// X/Y/Z wake-up axes (CTRL1).
            pub $($async)? fn probe(&mut self) -> Result<(), Error<IFACE::Error>> {
                let mut id = [0u8];
                self.iface.read_regs(REG_WHO_AM_I, &mut id)$($await)*.map_err(Error::bus)?;
                if id[0] != WHO_AM_I {
                    return Err(Error::UnexpectedId(u32::from(id[0])));
                }
                self.write_reg(REG_CTRL1, CTRL1_IF_ADD_INC | CTRL1_WU_XYZ)$($await)*
            }

            /// Apply the configuration. In continuous mode the part starts
            /// converting and the turn-on time is returned; in on-demand mode
            /// it waits for a trigger and `0` is returned.
            pub $($async)? fn configure(&mut self, config: Config) -> Result<WaitUs, Error<IFACE::Error>> {
                self.config = config;
                self.int_cfg = (self.int_cfg & !INT_CFG_H_LACTIVE)
                    | if config.int_active_low { INT_CFG_H_LACTIVE } else { 0 };
                self.write_reg(REG_CTRL1, CTRL1_IF_ADD_INC | CTRL1_WU_XYZ)$($await)*?;
                self.write_reg(REG_CTRL4, self.ctrl4)$($await)*?;
                self.write_reg(REG_INTERRUPT_CFG, self.int_cfg)$($await)*?;
                self.power_up()$($await)*
            }

            $($async)? fn power_up(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
                self.write_reg(REG_CTRL5, self.ctrl5())$($await)*?;
                self.active = true;
                Ok(match self.config.conversion {
                    Conversion::OnDemand => 0,
                    Conversion::Continuous => turn_on_us(self.config.rate),
                })
            }

            $($async)? fn write_routing(&mut self) -> Result<(), Error<IFACE::Error>> {
                self.write_reg(REG_MD1_CFG, self.md1)$($await)*?;
                self.write_reg(REG_MD2_CFG, self.md2)$($await)*?;
                self.int_cfg = (self.int_cfg & !INT_CFG_INTERRUPTS_ENABLE)
                    | if self.md1 | self.md2 != 0 { INT_CFG_INTERRUPTS_ENABLE } else { 0 };
                self.write_reg(REG_INTERRUPT_CFG, self.int_cfg)$($await)*
            }

            $($async)? fn route(&mut self, pin: IntPin, bits: u8, enable: bool) -> Result<(), Error<IFACE::Error>> {
                let md = match pin {
                    IntPin::Int1 => &mut self.md1,
                    IntPin::Int2 => &mut self.md2,
                };
                *md = (*md & !bits) | if enable { bits } else { 0 };
                self.write_routing()$($await)*
            }

            /// Enable the wake-up interrupt on `config.pin` (and optionally
            /// activity/inactivity). Needs [`Conversion::Continuous`].
            pub $($async)? fn configure_wake_up(&mut self, config: &WakeUpConfig) -> Result<(), Error<IFACE::Error>> {
                let (ths, fine) = threshold_reg(config.threshold_mg, self.config.range);
                let (sleep_on, sleep_dur, inact) = match config.auto_sleep {
                    Some(s) => (WAKE_UP_THS_SLEEP_ON, s.duration & 0x0F, s.rate as u8),
                    None => (0, 0, 0),
                };
                self.ctrl4 = CTRL4_BDU | (inact << 6);
                self.int_cfg &= !(INT_CFG_WAKE_THS_W | INT_CFG_LIR | INT_CFG_INT_SHORT_EN);
                self.int_cfg |= if fine { INT_CFG_WAKE_THS_W } else { 0 }
                    | if config.latch { INT_CFG_LIR | INT_CFG_INT_SHORT_EN } else { 0 };
                self.write_reg(REG_CTRL1, CTRL1_IF_ADD_INC | CTRL1_WU_XYZ)$($await)*?;
                self.write_reg(REG_CTRL4, self.ctrl4)$($await)*?;
                self.write_reg(REG_WAKE_UP_DUR, (config.duration.min(3) << 5) | sleep_dur)$($await)*?;
                self.write_reg(REG_WAKE_UP_THS, sleep_on | ths)$($await)*?;
                self.route(config.pin, MD_WU, true)$($await)*
            }

            /// Enable the 6D/4D orientation interrupt on `config.pin`. Needs
            /// [`Conversion::Continuous`]. Latching follows the last
            /// [`WakeUpConfig::latch`].
            pub $($async)? fn configure_orientation(&mut self, config: &OrientationConfig) -> Result<(), Error<IFACE::Error>> {
                let ths = ((config.threshold as u8) << 5) | if config.four_d { 0x80 } else { 0 };
                self.write_reg(REG_TAP_THS_X, ths)$($await)*?;
                self.route(config.pin, MD_6D, true)$($await)*
            }

            /// Remove the wake-up and 6D routing from `pin`; when neither pin
            /// has a source left, `INTERRUPTS_ENABLE` is cleared too.
            pub $($async)? fn disable_interrupt(&mut self, pin: IntPin) -> Result<(), Error<IFACE::Error>> {
                self.route(pin, MD_WU | MD_6D, false)$($await)*
            }

            /// Read the event sources in one burst. Reading ALL_INT_SRC
            /// releases latched pins.
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

        impl<IFACE: $Iface> $Sensor for Lis2du12<IFACE> {
            type BusError = IFACE::Error;
            type Measurement = Measurement;

            /// On demand: set `SOC` and return [`Config::one_shot_wait_us`].
            /// Continuous: power up if asleep (returns the turn-on time),
            /// else `0`.
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
                        self.write_reg(REG_CTRL4, self.ctrl4 | CTRL4_SOC)$($await)*?;
                        Ok(self.config.one_shot_wait_us)
                    }
                }
            }

            /// Reads STATUS through OUT_Z_H in one burst.
            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<IFACE::Error>> {
                let mut buf = [0u8; 9];
                self.iface.read_regs(REG_STATUS, &mut buf)$($await)*.map_err(Error::bus)?;
                if buf[0] & STATUS_DRDY == 0 {
                    return Err(Error::NotReady);
                }
                let mut frame = [0u8; 6];
                frame.copy_from_slice(&buf[3..]);
                Ok(Measurement {
                    acceleration: decode(&frame, self.config.range),
                })
            }
        }

        impl<IFACE: $Iface> $LowPower for Lis2du12<IFACE> {
            type BusError = IFACE::Error;

            /// Power-down (ODR = 0000, 20 nA); configuration is kept but
            /// interrupts and one-shot triggers stop.
            $($async)? fn sleep(&mut self) -> Result<(), Error<IFACE::Error>> {
                self.write_reg(REG_CTRL5, self.config.range.bits())$($await)*?;
                self.active = false;
                Ok(())
            }

            /// Restore CTRL5; returns the continuous turn-on time or `0` in
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
