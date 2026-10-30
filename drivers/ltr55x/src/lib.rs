//! Lite-On LTR-553ALS-01 / LTR-559ALS-01 ambient light + proximity sensor
//! (I2C).
//!
//! The ALS half is the LTR-303 core (CH0 visible+IR, CH1 IR, same gains,
//! integration times and lux formula); the PS half drives an external
//! (LTR-553) or integrated (LTR-559) IR LED and returns an 11-bit
//! reflectance count. There is no one-shot mode: `start_measurement`
//! activates the enabled channels, `read_measurement` waits for their
//! "new data" flags, reads them and returns both to standby.
//!
//! Lux (LTR-303/329/55x "Appendix A"), `ratio = CH1 / (CH0 + CH1)`:
//! `< 0.45: 1.7743·CH0 + 1.1059·CH1`, `< 0.64: 4.2785·CH0 − 1.9548·CH1`,
//! `< 0.85: 0.5926·CH0 + 0.1185·CH1`, else 0; divided by
//! `GAIN · T_int / 100 ms`.
//!
//! References: Lite-On LTR-553ALS-01 / LTR-559ALS-01 register maps as
//! implemented by the Linux `ltr501` IIO driver, ESPHome `ltr_als_ps` and
//! Pimoroni `ltr559-python` (the Lite-On PDFs are not publicly mirrored).
//!
//! ```ignore
//! let mut ltr = Ltr55x::new(i2c).with_led(LedCurrent::Ma50, 4);
//! ltr.probe()?;
//! let m = ltr.measure(&mut delay)?;
//! ```

#![no_std]

use embedded_hal::i2c::I2c;
use zigbee_sensor::blocking::{LowPower, Sensor};
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Fixed I2C address.
pub const ADDRESS: u8 = 0x23;
/// Expected part number (`PART_ID[7:4]`).
pub const PART_NUMBER: u8 = 0x9;
/// Lite-On manufacturer ID.
pub const MANUFACTURER_ID: u8 = 0x05;
/// PS full scale (11 bit).
pub const PS_FULL_SCALE: u16 = 0x07FF;

const REG_ALS_CONTR: u8 = 0x80;
const REG_PS_CONTR: u8 = 0x81;
const REG_PS_LED: u8 = 0x82;
const REG_PS_N_PULSES: u8 = 0x83;
const REG_PS_MEAS_RATE: u8 = 0x84;
const REG_ALS_MEAS_RATE: u8 = 0x85;
const REG_PART_ID: u8 = 0x86;
const REG_ALS_DATA: u8 = 0x88;
const REG_STATUS: u8 = 0x8C;
const REG_PS_DATA: u8 = 0x8D;
const REG_INTERRUPT: u8 = 0x8F;
const REG_PS_THRES_UP: u8 = 0x90;
const REG_PS_OFFSET: u8 = 0x94;
const REG_ALS_THRES_UP: u8 = 0x97;
const REG_INTERRUPT_PERSIST: u8 = 0x9E;

const CONTR_SW_RESET: u8 = 1 << 1;
const ALS_ACTIVE: u8 = 1 << 0;
const PS_ACTIVE: u8 = 0b11;
const PS_SAT_INDICATOR: u8 = 1 << 5;

const STATUS_ALS_INVALID: u8 = 1 << 7;
const STATUS_ALS_INT: u8 = 1 << 3;
const STATUS_ALS_NEW: u8 = 1 << 2;
const STATUS_PS_INT: u8 = 1 << 1;
const STATUS_PS_NEW: u8 = 1 << 0;

const INT_POLARITY_HIGH: u8 = 1 << 2;
const INT_ALS: u8 = 1 << 1;
const INT_PS: u8 = 1 << 0;

/// PS repeat rate code 0 = 50 ms.
const PS_RATE_MS: u32 = 50;
const PS_SATURATED: u8 = 1 << 7;

/// LED frequency 60 kHz, 100 % duty (reset values).
const LED_FREQ_DUTY: u8 = (0b011 << 5) | (0b11 << 3);

/// Standby → active wake-up (10 ms).
const WAKEUP_US: u32 = 10_000;

/// PS LED peak current.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum LedCurrent {
    Ma5,
    Ma10,
    Ma20,
    Ma50,
    #[default]
    Ma100,
}

/// ALS gain.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum Gain {
    #[default]
    X1,
    X2,
    X4,
    X8,
    X48,
    X96,
}

impl Gain {
    const ALL: [Gain; 6] = [Self::X1, Self::X2, Self::X4, Self::X8, Self::X48, Self::X96];

    const fn code(self) -> u8 {
        match self {
            Self::X1 => 0,
            Self::X2 => 1,
            Self::X4 => 2,
            Self::X8 => 3,
            Self::X48 => 6,
            Self::X96 => 7,
        }
    }

    /// Numeric gain.
    pub const fn factor(self) -> u32 {
        match self {
            Self::X1 => 1,
            Self::X2 => 2,
            Self::X4 => 4,
            Self::X8 => 8,
            Self::X48 => 48,
            Self::X96 => 96,
        }
    }

    const fn index(self) -> usize {
        match self {
            Self::X1 => 0,
            Self::X2 => 1,
            Self::X4 => 2,
            Self::X8 => 3,
            Self::X48 => 4,
            Self::X96 => 5,
        }
    }
}

/// ALS integration time.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum IntegrationTime {
    Ms50,
    #[default]
    Ms100,
    Ms150,
    Ms200,
    Ms250,
    Ms300,
    Ms350,
    Ms400,
}

impl IntegrationTime {
    const fn code(self) -> u8 {
        match self {
            Self::Ms100 => 0,
            Self::Ms50 => 1,
            Self::Ms200 => 2,
            Self::Ms400 => 3,
            Self::Ms150 => 4,
            Self::Ms250 => 5,
            Self::Ms300 => 6,
            Self::Ms350 => 7,
        }
    }

    /// Integration time in milliseconds.
    pub const fn ms(self) -> u32 {
        match self {
            Self::Ms50 => 50,
            Self::Ms100 => 100,
            Self::Ms150 => 150,
            Self::Ms200 => 200,
            Self::Ms250 => 250,
            Self::Ms300 => 300,
            Self::Ms350 => 350,
            Self::Ms400 => 400,
        }
    }

    /// Shortest repeat rate ≥ the integration time: (code, ms).
    const fn rate(self) -> (u8, u32) {
        match self {
            Self::Ms50 => (0, 50),
            Self::Ms100 => (1, 100),
            Self::Ms150 | Self::Ms200 => (2, 200),
            _ => (3, 500),
        }
    }
}

/// Appendix A lux formula, in milli-lux. `pfactor_milli` is the window
/// correction × 1000 (1000 = no window).
pub const fn lux_millilux(
    ch0: u16,
    ch1: u16,
    gain: Gain,
    integration: IntegrationTime,
    pfactor_milli: u32,
) -> u32 {
    let c0 = ch0 as u64;
    let c1 = ch1 as u64;
    let sum = c0 + c1;
    // Coefficients × 10_000.
    let scaled = if sum == 0 {
        0
    } else if c1 * 100 < sum * 45 {
        17_743 * c0 + 11_059 * c1
    } else if c1 * 100 < sum * 64 {
        (42_785 * c0).saturating_sub(19_548 * c1)
    } else if c1 * 100 < sum * 85 {
        5_926 * c0 + 1_185 * c1
    } else {
        0
    };
    // mlx = scaled / 10_000 · 1000 / gain / (t / 100) · pfactor / 1000
    let num = scaled * 10 * pfactor_milli as u64;
    let den = gain.factor() as u64 * integration.ms() as u64 * 1000;
    let v = num / den;
    if v > u32::MAX as u64 {
        u32::MAX
    } else {
        v as u32
    }
}

/// Pick a gain for the next sample from the larger channel of this one:
/// step down above 90 % of full scale, step up while the reading would
/// stay below 50 % of full scale.
pub fn suggest_gain(current: Gain, ch0: u16, ch1: u16) -> Gain {
    let peak = ch0.max(ch1) as u32;
    let idx = current.index();
    if peak > 65_535 * 9 / 10 {
        return if idx == 0 {
            current
        } else {
            Gain::ALL[idx - 1]
        };
    }
    if idx + 1 < Gain::ALL.len() {
        let next = Gain::ALL[idx + 1];
        if peak * next.factor() / current.factor() < 65_535 / 2 {
            return next;
        }
    }
    current
}

/// One sample (fields of disabled channels are `None`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Illuminance in milli-lux.
    pub illuminance: Option<u32>,
    /// CH0 (visible + IR) count.
    pub ch0: u16,
    /// CH1 (IR) count.
    pub ch1: u16,
    /// 11-bit proximity count.
    pub proximity: Option<u16>,
    /// An ALS channel reached full scale.
    pub als_saturated: bool,
    /// PS saturation flag (ambient IR too strong).
    pub ps_saturated: bool,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        if let Some(lux) = self.illuminance {
            f(Quantity::Illuminance(lux));
        }
        if let Some(ps) = self.proximity {
            f(Quantity::Proximity(ps as u32));
        }
    }
}

/// Interrupt window for one channel (raw counts).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Window {
    /// Lower threshold.
    pub low: u16,
    /// Upper threshold.
    pub high: u16,
    /// 0..=15 → 1..=16 consecutive out-of-window samples.
    pub persistence: u8,
}

macro_rules! driver {
    ($($async_:ident)?, $($await_:tt)*) => {
        /// LTR-553 / LTR-559 driver.
        pub struct Ltr55x<I2C> {
            i2c: I2C,
            gain: Gain,
            integration: IntegrationTime,
            pfactor_milli: u32,
            led: u8,
            pulses: u8,
            als: bool,
            ps: bool,
            continuous: bool,
        }

        impl<I2C> Ltr55x<I2C> {
            /// Create a driver (ALS ×1 / 100 ms + PS, 100 mA, 1 pulse); no
            /// bus traffic.
            pub const fn new(i2c: I2C) -> Self {
                Self {
                    i2c,
                    gain: Gain::X1,
                    integration: IntegrationTime::Ms100,
                    pfactor_milli: 1000,
                    led: LED_FREQ_DUTY | LedCurrent::Ma100 as u8,
                    pulses: 1,
                    als: true,
                    ps: true,
                    continuous: false,
                }
            }

            /// Select the ALS gain.
            pub const fn with_gain(mut self, gain: Gain) -> Self {
                self.gain = gain;
                self
            }

            /// Select the ALS integration time.
            pub const fn with_integration_time(mut self, integration: IntegrationTime) -> Self {
                self.integration = integration;
                self
            }

            /// Window correction × 1000 (1000 = no window).
            pub const fn with_window_factor(mut self, pfactor_milli: u32) -> Self {
                self.pfactor_milli = pfactor_milli;
                self
            }

            /// PS LED current and pulse count (1..=15) at 60 kHz, 100 % duty.
            pub const fn with_led(mut self, current: LedCurrent, pulses: u8) -> Self {
                self.led = LED_FREQ_DUTY | current as u8;
                self.pulses = if pulses == 0 {
                    1
                } else if pulses > 15 {
                    15
                } else {
                    pulses
                };
                self
            }

            /// Enable / disable the ALS and PS halves.
            pub const fn with_channels(mut self, als: bool, ps: bool) -> Self {
                self.als = als;
                self.ps = ps;
                self
            }

            /// Keep measuring after a read (needed for interrupts).
            pub const fn with_continuous(mut self, continuous: bool) -> Self {
                self.continuous = continuous;
                self
            }

            /// Change the ALS gain for the next `start_measurement`.
            pub fn set_gain(&mut self, gain: Gain) {
                self.gain = gain;
            }

            /// Current ALS gain.
            pub const fn gain(&self) -> Gain {
                self.gain
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.i2c
            }
        }

        impl<I2C: I2c> Ltr55x<I2C> {
            $($async_)? fn write(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
                self.i2c
                    .write(ADDRESS, &[reg, value])
                    $($await_)*
                    .map_err(Error::bus)
            }

            $($async_)? fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
                self.i2c
                    .write_read(ADDRESS, &[reg], buf)
                    $($await_)*
                    .map_err(Error::bus)
            }

            /// Check `PART_ID[7:4]` = 0x9 and `MANUFAC_ID` = 0x05; returns
            /// the revision (0x2 on LTR-553/559).
            pub $($async_)? fn probe(&mut self) -> Result<u8, Error<I2C::Error>> {
                let mut id = [0u8; 2];
                self.read(REG_PART_ID, &mut id) $($await_)* ?;
                if id[0] >> 4 != PART_NUMBER || id[1] != MANUFACTURER_ID {
                    return Err(Error::UnexpectedId(u16::from_be_bytes(id) as u32));
                }
                Ok(id[0] & 0x0F)
            }

            /// Software reset (returns to standby with default settings).
            pub $($async_)? fn reset(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write(REG_ALS_CONTR, CONTR_SW_RESET) $($await_)*
            }

            /// 10-bit PS offset subtracted from the proximity count
            /// (crosstalk compensation).
            pub $($async_)? fn set_ps_offset(&mut self, offset: u16) -> Result<(), Error<I2C::Error>> {
                let [lo, hi] = offset.min(0x03FF).to_le_bytes();
                self.i2c
                    .write(ADDRESS, &[REG_PS_OFFSET, hi, lo])
                    $($await_)*
                    .map_err(Error::bus)
            }

            /// Program the ALS (CH0) and PS windows and enable the INT pin
            /// for the channels given (`None`, `None` disables it). Call
            /// while in standby.
            pub $($async_)? fn set_interrupt(
                &mut self,
                als: Option<Window>,
                ps: Option<Window>,
                active_high: bool,
            ) -> Result<(), Error<I2C::Error>> {
                let mut mode = if active_high { INT_POLARITY_HIGH } else { 0 };
                let mut persist = 0;
                if let Some(w) = ps {
                    let [h0, h1] = w.high.min(PS_FULL_SCALE).to_le_bytes();
                    let [l0, l1] = w.low.min(PS_FULL_SCALE).to_le_bytes();
                    self.i2c
                        .write(ADDRESS, &[REG_PS_THRES_UP, h0, h1, l0, l1])
                        $($await_)*
                        .map_err(Error::bus)?;
                    persist |= (w.persistence & 0x0F) << 4;
                    mode |= INT_PS;
                }
                if let Some(w) = als {
                    let [h0, h1] = w.high.to_le_bytes();
                    let [l0, l1] = w.low.to_le_bytes();
                    self.i2c
                        .write(ADDRESS, &[REG_ALS_THRES_UP, h0, h1, l0, l1])
                        $($await_)*
                        .map_err(Error::bus)?;
                    persist |= w.persistence & 0x0F;
                    mode |= INT_ALS;
                }
                self.write(REG_INTERRUPT_PERSIST, persist) $($await_)* ?;
                self.write(REG_INTERRUPT, mode) $($await_)*
            }

            /// Read the status; returns `(als_interrupt, ps_interrupt)`.
            pub $($async_)? fn take_interrupt(&mut self) -> Result<(bool, bool), Error<I2C::Error>> {
                let mut status = [0u8; 1];
                self.read(REG_STATUS, &mut status) $($await_)* ?;
                Ok((status[0] & STATUS_ALS_INT != 0, status[0] & STATUS_PS_INT != 0))
            }

            $($async_)? fn standby(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write(REG_ALS_CONTR, 0) $($await_)* ?;
                self.write(REG_PS_CONTR, 0) $($await_)*
            }
        }

        impl<I2C: I2c> Sensor for Ltr55x<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// Write the configuration and activate the enabled channels.
            $($async_)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                if !self.als && !self.ps {
                    return Err(Error::Unsupported);
                }
                let mut period_ms = 0;
                if self.als {
                    let (rate_code, rate_ms) = self.integration.rate();
                    let meas = (self.integration.code() << 3) | rate_code;
                    self.write(REG_ALS_MEAS_RATE, meas) $($await_)* ?;
                    let contr = (self.gain.code() << 2) | ALS_ACTIVE;
                    self.write(REG_ALS_CONTR, contr) $($await_)* ?;
                    period_ms = rate_ms;
                }
                if self.ps {
                    self.write(REG_PS_LED, self.led) $($await_)* ?;
                    self.write(REG_PS_N_PULSES, self.pulses) $($await_)* ?;
                    self.write(REG_PS_MEAS_RATE, 0) $($await_)* ?;
                    self.write(REG_PS_CONTR, PS_SAT_INDICATOR | PS_ACTIVE) $($await_)* ?;
                    period_ms = period_ms.max(PS_RATE_MS);
                }
                let period_us = period_ms * 1000;
                Ok(WAKEUP_US + period_us + period_us / 10)
            }

            /// Wait for the enabled channels' "new data" flags, read them
            /// and go to standby (unless continuous).
            $($async_)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let mut status = [0u8; 1];
                self.read(REG_STATUS, &mut status) $($await_)* ?;
                let s = status[0];
                if (self.als && s & STATUS_ALS_NEW == 0) || (self.ps && s & STATUS_PS_NEW == 0) {
                    return Err(Error::NotReady);
                }
                if self.als && s & STATUS_ALS_INVALID != 0 {
                    return Err(Error::InvalidData);
                }
                let mut m = Measurement {
                    illuminance: None,
                    ch0: 0,
                    ch1: 0,
                    proximity: None,
                    als_saturated: false,
                    ps_saturated: false,
                };
                if self.als {
                    let mut buf = [0u8; 4];
                    self.read(REG_ALS_DATA, &mut buf) $($await_)* ?;
                    m.ch1 = u16::from_le_bytes([buf[0], buf[1]]);
                    m.ch0 = u16::from_le_bytes([buf[2], buf[3]]);
                    m.illuminance = Some(lux_millilux(
                        m.ch0,
                        m.ch1,
                        self.gain,
                        self.integration,
                        self.pfactor_milli,
                    ));
                    m.als_saturated = m.ch0 == u16::MAX || m.ch1 == u16::MAX;
                }
                if self.ps {
                    let mut buf = [0u8; 2];
                    self.read(REG_PS_DATA, &mut buf) $($await_)* ?;
                    m.proximity = Some(u16::from_le_bytes(buf) & PS_FULL_SCALE);
                    m.ps_saturated = buf[1] & PS_SATURATED != 0;
                }
                if !self.continuous {
                    self.standby() $($await_)* ?;
                }
                Ok(m)
            }
        }

        impl<I2C: I2c> LowPower for Ltr55x<I2C> {
            type BusError = I2C::Error;

            $($async_)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.standby() $($await_)*
            }

            /// Samples activate the part themselves; nothing to do.
            $($async_)? fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                Ok(0)
            }
        }
    };
}

driver!(,);

/// Async driver.
pub mod asynch {
    use super::{
        ADDRESS, ALS_ACTIVE, CONTR_SW_RESET, Gain, INT_ALS, INT_POLARITY_HIGH, INT_PS,
        IntegrationTime, LED_FREQ_DUTY, LedCurrent, MANUFACTURER_ID, Measurement, PART_NUMBER,
        PS_ACTIVE, PS_FULL_SCALE, PS_RATE_MS, PS_SAT_INDICATOR, PS_SATURATED, REG_ALS_CONTR,
        REG_ALS_DATA, REG_ALS_MEAS_RATE, REG_ALS_THRES_UP, REG_INTERRUPT, REG_INTERRUPT_PERSIST,
        REG_PART_ID, REG_PS_CONTR, REG_PS_DATA, REG_PS_LED, REG_PS_MEAS_RATE, REG_PS_N_PULSES,
        REG_PS_OFFSET, REG_PS_THRES_UP, REG_STATUS, STATUS_ALS_INT, STATUS_ALS_INVALID,
        STATUS_ALS_NEW, STATUS_PS_INT, STATUS_PS_NEW, WAKEUP_US, Window, lux_millilux,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::asynch::{LowPower, Sensor};
    use zigbee_sensor::{Error, WaitUs};

    driver!(async, .await);
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use std::vec;

    #[test]
    fn helpers() {
        let t = IntegrationTime::Ms100;
        assert_eq!(lux_millilux(1000, 200, Gain::X1, t, 1000), 1_995_480);
        assert_eq!(lux_millilux(1000, 1000, Gain::X4, t, 1000), 580_925);
        assert_eq!(lux_millilux(100, 900, Gain::X1, t, 1000), 0);
        assert_eq!(suggest_gain(Gain::X4, 100, 50), Gain::X8);
        assert_eq!(suggest_gain(Gain::X4, 65_000, 50), Gain::X2);
    }

    #[test]
    fn als_and_ps_cycle() {
        let expectations = [
            Transaction::write_read(ADDRESS, vec![0x86], vec![0x92, 0x05]),
            Transaction::write(ADDRESS, vec![0x85, 0x01]),
            Transaction::write(ADDRESS, vec![0x80, 0x09]),
            Transaction::write(ADDRESS, vec![0x82, 0x7B]),
            Transaction::write(ADDRESS, vec![0x83, 0x04]),
            Transaction::write(ADDRESS, vec![0x84, 0x00]),
            Transaction::write(ADDRESS, vec![0x81, 0x23]),
            // ALS ready, PS not yet.
            Transaction::write_read(ADDRESS, vec![0x8C], vec![0x24]),
            Transaction::write_read(ADDRESS, vec![0x8C], vec![0x25]),
            Transaction::write_read(ADDRESS, vec![0x88], vec![0xE8, 0x03, 0xE8, 0x03]),
            Transaction::write_read(ADDRESS, vec![0x8D], vec![0x34, 0x82]),
            Transaction::write(ADDRESS, vec![0x80, 0x00]),
            Transaction::write(ADDRESS, vec![0x81, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut ltr = Ltr55x::new(&mut i2c)
            .with_gain(Gain::X4)
            .with_led(LedCurrent::Ma50, 4);
        assert_eq!(ltr.probe().unwrap(), 2);
        assert_eq!(ltr.start_measurement().unwrap(), 120_000);
        assert_eq!(ltr.read_measurement(), Err(Error::NotReady));
        let m = ltr.read_measurement().unwrap();
        assert_eq!(m.illuminance, Some(580_925));
        assert_eq!(m.proximity, Some(0x234));
        assert!(m.ps_saturated);
        assert!(!m.als_saturated);
        i2c.done();
    }

    #[test]
    fn ps_only_interrupt() {
        let expectations = [
            Transaction::write_read(ADDRESS, vec![0x86], vec![0xA0, 0x05]),
            Transaction::write(ADDRESS, vec![0x90, 0xFF, 0x07, 0x64, 0x00]),
            Transaction::write(ADDRESS, vec![0x9E, 0x20]),
            Transaction::write(ADDRESS, vec![0x8F, 0x01]),
            Transaction::write(ADDRESS, vec![0x94, 0x01, 0x10]),
            Transaction::write(ADDRESS, vec![0x82, 0x7C]),
            Transaction::write(ADDRESS, vec![0x83, 0x01]),
            Transaction::write(ADDRESS, vec![0x84, 0x00]),
            Transaction::write(ADDRESS, vec![0x81, 0x23]),
            Transaction::write_read(ADDRESS, vec![0x8C], vec![0x03]),
            Transaction::write_read(ADDRESS, vec![0x8D], vec![0xFF, 0x07]),
            Transaction::write_read(ADDRESS, vec![0x8C], vec![0x02]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut ltr = Ltr55x::new(&mut i2c)
            .with_channels(false, true)
            .with_continuous(true);
        assert_eq!(ltr.probe(), Err(Error::UnexpectedId(0xA005)));
        let w = Window {
            low: 100,
            high: 5000,
            persistence: 2,
        };
        ltr.set_interrupt(None, Some(w), false).unwrap();
        ltr.set_ps_offset(0x110).unwrap();
        assert_eq!(ltr.start_measurement().unwrap(), 65_000);
        let m = ltr.read_measurement().unwrap();
        assert_eq!(m.proximity, Some(2047));
        assert_eq!(m.illuminance, None);
        assert_eq!(ltr.take_interrupt().unwrap(), (false, true));
        i2c.done();
    }

    #[test]
    fn async_als_only() {
        use zigbee_sensor::asynch::Sensor as _;

        struct RecordingDelay(u32);
        impl embedded_hal_async::delay::DelayNs for RecordingDelay {
            async fn delay_ns(&mut self, ns: u32) {
                self.0 += ns / 1_000;
            }
        }

        let expectations = [
            Transaction::write(ADDRESS, vec![0x85, 0x12]),
            Transaction::write(ADDRESS, vec![0x80, 0x01]),
            Transaction::write_read(ADDRESS, vec![0x8C], vec![0x04]),
            Transaction::write_read(ADDRESS, vec![0x88], vec![0xC8, 0x00, 0xE8, 0x03]),
            Transaction::write(ADDRESS, vec![0x80, 0x00]),
            Transaction::write(ADDRESS, vec![0x81, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut ltr = asynch::Ltr55x::new(&mut i2c)
            .with_channels(true, false)
            .with_integration_time(IntegrationTime::Ms200);
        let mut delay = RecordingDelay(0);
        let m = block_on(ltr.measure(&mut delay)).unwrap();
        assert_eq!(m.illuminance, Some(997_740));
        assert_eq!(m.proximity, None);
        assert_eq!(delay.0, 230_000);
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
