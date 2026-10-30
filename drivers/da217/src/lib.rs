//! MiraMEMS DA217 (and DA218) three-axis accelerometer (I2C or SPI).
//!
//! The part samples continuously in normal mode (1–500 Hz) and has no
//! single-shot mode. [`Sensor::start_measurement`] leaves suspend mode
//! (returns 1/ODR + 1 ms). With [`Config::data_ready`] set,
//! [`Sensor::read_measurement`] returns [`Error::NotReady`] until
//! NEWDATA_FLAG is set. [`LowPower::sleep`] selects suspend mode (1 µA,
//! no sampling, no interrupts).
//!
//! The active (slope) interrupt ([`MotionConfig`]) wakes a sleepy host on
//! movement. With [`Config::auto_sleep`] the part samples at 12.5 Hz while
//! inactive and switches to the configured ODR while active.
//!
//! Reference: MiraMEMS "DA217 MEMS digital output motion sensor" datasheet
//! Rev 0.1 (27-Mar-2017). DA218 is assumed to share the DA217 register map.
//!
//! [`Sensor::start_measurement`]: zigbee_sensor::blocking::Sensor::start_measurement
//! [`Sensor::read_measurement`]: zigbee_sensor::blocking::Sensor::read_measurement
//! [`LowPower::sleep`]: zigbee_sensor::blocking::LowPower::sleep
//!
//! ```ignore
//! let mut acc = Da217::new_i2c(i2c, ADDRESS_SA0_LOW);
//! acc.probe()?;
//! acc.configure(Config { rate: DataRate::Hz7_81, ..Default::default() })?;
//! acc.configure_motion(&MotionConfig { threshold_mg: 100, ..Default::default() })?;
//! // ... sleep until INT1, then:
//! let src = acc.read_interrupt_source()?; // also resets latched interrupts
//! ```

#![no_std]

mod interface;

pub use interface::{AsyncInterface, I2cInterface, Interface, SpiInterface};
use zigbee_sensor::{Error, Quantity, Readings, Vector3, WaitUs};

/// I2C address with SA0 to GND.
pub const ADDRESS_SA0_LOW: u8 = 0x26;
/// I2C address with SA0 to VDDIO.
pub const ADDRESS_SA0_HIGH: u8 = 0x27;

/// CHIP_ID value.
pub const CHIP_ID: u8 = 0x13;

const REG_CHIP_ID: u8 = 0x01;
const REG_ACC_X_LSB: u8 = 0x02;
const REG_MOTION_FLAG: u8 = 0x09;
const REG_NEWDATA_FLAG: u8 = 0x0A;
const REG_RESOLUTION_RANGE: u8 = 0x0F;
const REG_ODR_AXIS: u8 = 0x10;
const REG_MODE_BW: u8 = 0x11;
const REG_INT_SET1: u8 = 0x16;
const REG_INT_SET2: u8 = 0x17;
const REG_INT_MAP1: u8 = 0x19;
const REG_INT_MAP3: u8 = 0x1B;
const REG_INT_CONFIG: u8 = 0x20;
const REG_INT_LATCH: u8 = 0x21;
const REG_ACTIVE_DUR: u8 = 0x27;
const REG_ACTIVE_THS: u8 = 0x28;

// Watchdog enable (reset value) kept set.
const RESOLUTION_RANGE_WDT_EN: u8 = 0x40;
const MODE_BW_PWR_OFF: u8 = 0x80;
// Undocumented MODE_BW bits 4 and 3 are 1 after reset; kept as is.
const MODE_BW_RESERVED: u8 = 0x18;
const MODE_BW_AUTOSLEEP: u8 = 0x01;
const INT_SET2_NEW_DATA: u8 = 0x10;
const INT_MAP_ACTIVE: u8 = 0x04;
const INT_CONFIG_RESET: u8 = 0x80;
const NEWDATA: u8 = 0x01;

/// Full-scale range (RESOLUTION_RANGE.FS).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Range {
    /// ±2 g.
    #[default]
    G2 = 0,
    /// ±4 g.
    G4 = 1,
    /// ±8 g.
    G8 = 2,
    /// ±16 g.
    G16 = 3,
}

impl Range {
    const fn g(self) -> i32 {
        2 << self as i32
    }
}

/// Output resolution (RESOLUTION_RANGE.Resolution).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Resolution {
    /// 14-bit (4096 LSB/g at ±2 g).
    #[default]
    Bits14 = 0,
    /// 12-bit.
    Bits12 = 1,
    /// 10-bit.
    Bits10 = 2,
    /// 8-bit.
    Bits8 = 3,
}

impl Resolution {
    /// Number of significant bits.
    pub const fn bits(self) -> u8 {
        14 - 2 * self as u8
    }
}

/// Output data rate (ODR_AXIS).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum DataRate {
    /// 1 Hz.
    Hz1 = 0,
    /// 1.95 Hz.
    Hz1_95 = 1,
    /// 3.9 Hz.
    Hz3_9 = 2,
    /// 7.81 Hz.
    Hz7_81 = 3,
    /// 15.63 Hz.
    Hz15_63 = 4,
    /// 31.25 Hz.
    #[default]
    Hz31_25 = 5,
    /// 62.5 Hz.
    Hz62_5 = 6,
    /// 125 Hz.
    Hz125 = 7,
    /// 250 Hz.
    Hz250 = 8,
    /// 500 Hz.
    Hz500 = 9,
}

impl DataRate {
    /// Sample period in µs.
    pub const fn period_us(self) -> u32 {
        match self {
            DataRate::Hz1 => 1_000_000,
            // Codes 1..9 are 1000/512 Hz doubling per step.
            _ => 1_024_000 >> self as u32,
        }
    }
}

/// Filter bandwidth (MODE_BW.BW).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Bandwidth {
    /// 500 Hz.
    #[default]
    Hz500 = 0,
    /// 250 Hz.
    Hz250 = 1,
    /// 100 Hz.
    Hz100 = 2,
}

/// Sampling configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Config {
    /// Full-scale range.
    pub range: Range,
    /// Output resolution.
    pub resolution: Resolution,
    /// Output data rate.
    pub rate: DataRate,
    /// Filter bandwidth.
    pub bandwidth: Bandwidth,
    /// Sample at 12.5 Hz while inactive and at `rate` while the active
    /// interrupt condition holds (MODE_BW.autosleep_en).
    pub auto_sleep: bool,
    /// Enable the new-data interrupt status (not routed to a pin) so
    /// [`Sensor`] reads can return [`Error::NotReady`].
    ///
    /// [`Sensor`]: zigbee_sensor::blocking::Sensor
    pub data_ready: bool,
}

impl Config {
    /// ±2 g, 14-bit, 31.25 Hz, 500 Hz bandwidth, no auto-sleep, no DRDY.
    pub const DEFAULT: Self = Self {
        range: Range::G2,
        resolution: Resolution::Bits14,
        rate: DataRate::Hz31_25,
        bandwidth: Bandwidth::Hz500,
        auto_sleep: false,
        data_ready: false,
    };

    const fn mode_bw(&self) -> u8 {
        let auto = if self.auto_sleep {
            MODE_BW_AUTOSLEEP
        } else {
            0
        };
        MODE_BW_RESERVED | (self.bandwidth as u8) << 1 | auto
    }
}

impl Default for Config {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Wait after leaving suspend: wake-up time 1 ms plus one sample period.
pub const fn turn_on_us(rate: DataRate) -> WaitUs {
    rate.period_us() + 1_000
}

/// Interrupt pin.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum IntPin {
    /// INT1.
    #[default]
    Int1,
    /// INT2.
    Int2,
}

/// Interrupt latching (INT_LATCH codes, common to all interrupts on a pin).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Latch {
    /// Follows the condition.
    NonLatched = 0x0,
    /// Held until reset (see [`Da217::read_interrupt_source`]).
    #[default]
    Latched = 0x7,
    /// Held 1 ms.
    Ms1 = 0x9,
    /// Held 2 ms.
    Ms2 = 0xB,
    /// Held 25 ms.
    Ms25 = 0xC,
    /// Held 50 ms.
    Ms50 = 0xD,
    /// Held 100 ms.
    Ms100 = 0xE,
    /// Held 250 ms.
    Ms250 = 0x1,
    /// Held 500 ms.
    Ms500 = 0x2,
    /// Held 1 s.
    S1 = 0x3,
    /// Held 2 s.
    S2 = 0x4,
    /// Held 4 s.
    S4 = 0x5,
    /// Held 8 s.
    S8 = 0x6,
}

/// Axis enable bits for the active interrupt (INT_SET1 bits 2..0).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Axes(pub u8);

impl Axes {
    /// X, Y and Z.
    pub const ALL: Self = Self(0x07);
    /// X.
    pub const X: Self = Self(0x01);
    /// Y.
    pub const Y: Self = Self(0x02);
    /// Z.
    pub const Z: Self = Self(0x04);

    /// Union of two sets.
    pub const fn or(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }
}

/// Active (motion) interrupt.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MotionConfig {
    /// Slope between successive samples; 3.91 mg steps at ±2 g, scaled by
    /// range (max 1 g at ±2 g). Reconfigure after changing the range.
    pub threshold_mg: u16,
    /// Consecutive samples above the threshold to trigger (1–16).
    pub duration: u8,
    /// Consecutive samples below the threshold to count as inactive, used
    /// by [`Config::auto_sleep`] (1–16).
    pub inactive_duration: u8,
    /// Axes that participate.
    pub axes: Axes,
    /// Output pin.
    pub pin: IntPin,
    /// Latching.
    pub latch: Latch,
    /// Active-low pins (both pins share the setting).
    pub active_low: bool,
    /// Open-drain pins (both pins share the setting).
    pub open_drain: bool,
}

impl Default for MotionConfig {
    fn default() -> Self {
        Self {
            threshold_mg: 100,
            duration: 1,
            inactive_duration: 1,
            axes: Axes::ALL,
            pin: IntPin::Int1,
            latch: Latch::Latched,
            active_low: false,
            open_drain: false,
        }
    }
}

/// MOTION_FLAG, NEWDATA_FLAG and TAP_ACTIVE_STATUS.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct InterruptSource {
    /// MOTION_FLAG.
    pub motion: u8,
    /// NEWDATA_FLAG.
    pub new_data: u8,
    /// TAP_ACTIVE_STATUS.
    pub tap_active: u8,
}

impl InterruptSource {
    /// Active interrupt.
    pub const fn active(self) -> bool {
        self.motion & 0x04 != 0
    }

    /// New-data interrupt.
    pub const fn data_ready(self) -> bool {
        self.new_data & NEWDATA != 0
    }

    /// Axes that triggered the active interrupt first.
    pub const fn active_axes(self) -> Axes {
        let s = self.tap_active;
        Axes((s >> 2) & 0x01 | (s & 0x02) | (s << 2) & 0x04)
    }

    /// Sign of the first active-interrupt sample (true = negative).
    pub const fn active_negative(self) -> bool {
        self.tap_active & 0x08 != 0
    }
}

/// One acceleration sample.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Measurement {
    /// Acceleration in milli-g.
    pub acceleration: Vector3,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Acceleration(self.acceleration));
    }
}

/// Convert one left-justified output word to mg, discarding bits below
/// `bits` resolution (truncated toward zero).
pub const fn raw_to_mg(raw: i16, bits: u8, range: Range) -> i32 {
    let v = (raw as i32) & !((1 << (16 - bits)) - 1);
    v * 1000 * range.g() / 32768
}

/// Decode ACC_X_LSB..ACC_Z_MSB to milli-g.
pub fn decode(frame: &[u8; 6], bits: u8, range: Range) -> Vector3 {
    let axis = |i: usize| raw_to_mg(i16::from_le_bytes([frame[i], frame[i + 1]]), bits, range);
    Vector3 {
        x: axis(0),
        y: axis(2),
        z: axis(4),
    }
}

/// ACTIVE_THS code: 3.91 mg/LSB at ±2 g, doubling per range step
/// (rounded, at most 255).
pub fn active_threshold_reg(threshold_mg: u16, range: Range) -> u8 {
    let fs = range.g() as u32;
    ((u32::from(threshold_mg) * 512 + 500 * fs) / (1000 * fs)).min(255) as u8
}

macro_rules! driver {
    ($Iface:path, $Sensor:path, $LowPower:path, [$($async:tt)?], [$($await:tt)*]) => {
        /// DA217 driver over a register [`Interface`].
        pub struct Da217<IFACE> {
            iface: IFACE,
            config: Config,
            int_config: u8,
            active: bool,
        }

        impl<IFACE> Da217<IFACE> {
            /// Create a driver over a register interface; no bus traffic.
            pub const fn new(iface: IFACE) -> Self {
                Self {
                    iface,
                    config: Config::DEFAULT,
                    int_config: 0,
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

        impl<I2C> Da217<I2cInterface<I2C>> {
            /// Driver on an I2C bus ([`ADDRESS_SA0_LOW`] or
            /// [`ADDRESS_SA0_HIGH`]); CSB must be tied high.
            pub const fn new_i2c(i2c: I2C, address: u8) -> Self {
                Self::new(I2cInterface::new(i2c, address))
            }
        }

        impl<SPI> Da217<SpiInterface<SPI>> {
            /// Driver on an SPI device (4-wire after [`Self::probe`]).
            pub const fn new_spi(spi: SPI) -> Self {
                Self::new(SpiInterface::new(spi))
            }
        }

        impl<IFACE: $Iface> Da217<IFACE> {
            $($async)? fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<IFACE::Error>> {
                self.iface.read_regs(reg, buf)$($await)*.map_err(Error::bus)
            }

            $($async)? fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error<IFACE::Error>> {
                self.iface.write_reg(reg, value)$($await)*.map_err(Error::bus)
            }

            /// Set up the bus (4-wire SPI) and check CHIP_ID. Start-up
            /// from power-off takes 3 ms.
            pub $($async)? fn probe(&mut self) -> Result<(), Error<IFACE::Error>> {
                self.iface.init()$($await)*.map_err(Error::bus)?;
                let mut id = [0u8];
                self.read_regs(REG_CHIP_ID, &mut id)$($await)*?;
                if id[0] != CHIP_ID {
                    return Err(Error::UnexpectedId(u32::from(id[0])));
                }
                Ok(())
            }

            /// Apply the configuration and enter normal mode; returns
            /// [`turn_on_us`].
            pub $($async)? fn configure(&mut self, config: Config) -> Result<WaitUs, Error<IFACE::Error>> {
                self.config = config;
                let rr = RESOLUTION_RANGE_WDT_EN | (config.resolution as u8) << 2 | config.range as u8;
                self.write_reg(REG_RESOLUTION_RANGE, rr)$($await)*?;
                self.write_reg(REG_ODR_AXIS, config.rate as u8)$($await)*?;
                let set2 = if config.data_ready { INT_SET2_NEW_DATA } else { 0 };
                self.write_reg(REG_INT_SET2, set2)$($await)*?;
                self.power_up()$($await)*
            }

            $($async)? fn power_up(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
                self.write_reg(REG_MODE_BW, self.config.mode_bw())$($await)*?;
                self.active = true;
                Ok(turn_on_us(self.config.rate))
            }

            /// Configure the active interrupt and route it to one pin.
            /// Fails with [`Error::Unsupported`] for durations outside
            /// 1–16 or no axes.
            pub $($async)? fn configure_motion(&mut self, config: &MotionConfig) -> Result<(), Error<IFACE::Error>> {
                let ok = |d: u8| (1..=16).contains(&d);
                if !ok(config.duration) || !ok(config.inactive_duration) || config.axes.0 & 0x07 == 0 {
                    return Err(Error::Unsupported);
                }
                let ths = active_threshold_reg(config.threshold_mg, self.config.range);
                self.write_reg(REG_ACTIVE_THS, ths)$($await)*?;
                let dur = (config.inactive_duration - 1) << 4 | (config.duration - 1);
                self.write_reg(REG_ACTIVE_DUR, dur)$($await)*?;
                let latch = config.latch as u8;
                self.write_reg(REG_INT_LATCH, latch << 4 | latch)$($await)*?;
                let lvl = if config.active_low { 0x05 } else { 0 };
                let od = if config.open_drain { 0x0A } else { 0 };
                self.int_config = lvl | od;
                self.write_reg(REG_INT_CONFIG, self.int_config)$($await)*?;
                let (map1, map3) = match config.pin {
                    IntPin::Int1 => (INT_MAP_ACTIVE, 0),
                    IntPin::Int2 => (0, INT_MAP_ACTIVE),
                };
                self.write_reg(REG_INT_MAP1, map1)$($await)*?;
                self.write_reg(REG_INT_MAP3, map3)$($await)*?;
                self.write_reg(REG_INT_SET1, config.axes.0 & 0x07)$($await)*
            }

            /// Disable the active interrupt and unmap it from both pins.
            pub $($async)? fn disable_motion(&mut self) -> Result<(), Error<IFACE::Error>> {
                self.write_reg(REG_INT_SET1, 0)$($await)*?;
                self.write_reg(REG_INT_MAP1, 0)$($await)*?;
                self.write_reg(REG_INT_MAP3, 0)$($await)*
            }

            /// Read MOTION_FLAG..TAP_ACTIVE_STATUS, then reset latched
            /// interrupts (INT_CONFIG.reset_int).
            pub $($async)? fn read_interrupt_source(&mut self) -> Result<InterruptSource, Error<IFACE::Error>> {
                let mut s = [0u8; 3];
                self.read_regs(REG_MOTION_FLAG, &mut s)$($await)*?;
                self.write_reg(REG_INT_CONFIG, self.int_config | INT_CONFIG_RESET)$($await)*?;
                Ok(InterruptSource { motion: s[0], new_data: s[1], tap_active: s[2] })
            }
        }

        impl<IFACE: $Iface> $Sensor for Da217<IFACE> {
            type BusError = IFACE::Error;
            type Measurement = Measurement;

            /// Leave suspend if needed (returns 1/ODR + 1 ms), else `0`.
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
                if self.active {
                    return Ok(0);
                }
                self.power_up()$($await)*
            }

            /// With [`Config::data_ready`] checks NEWDATA_FLAG first.
            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<IFACE::Error>> {
                if self.config.data_ready {
                    let mut flag = [0u8];
                    self.read_regs(REG_NEWDATA_FLAG, &mut flag)$($await)*?;
                    if flag[0] & NEWDATA == 0 {
                        return Err(Error::NotReady);
                    }
                }
                let mut frame = [0u8; 6];
                self.read_regs(REG_ACC_X_LSB, &mut frame)$($await)*?;
                let bits = self.config.resolution.bits();
                Ok(Measurement { acceleration: decode(&frame, bits, self.config.range) })
            }
        }

        impl<IFACE: $Iface> $LowPower for Da217<IFACE> {
            type BusError = IFACE::Error;

            /// Suspend mode (1 µA): no sampling and no interrupts.
            $($async)? fn sleep(&mut self) -> Result<(), Error<IFACE::Error>> {
                self.write_reg(REG_MODE_BW, self.config.mode_bw() | MODE_BW_PWR_OFF)$($await)*?;
                self.active = false;
                Ok(())
            }

            /// Back to normal mode; returns 1/ODR + 1 ms.
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
