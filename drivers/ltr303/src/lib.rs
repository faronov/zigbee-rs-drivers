//! Lite-On LTR-303ALS-01 / LTR-329ALS-01 ambient light sensor (I2C).
//!
//! Two 16-bit channels: CH0 (visible + IR) and CH1 (IR). The LTR-329 is
//! the LTR-303 without the interrupt pin. There is no one-shot mode:
//! `start_measurement` programs gain / integration time and switches the
//! part to active mode, `read_measurement` checks the "new data" flag,
//! reads both channels and puts it back to standby.
//!
//! Lux (LTR-303/329 Appendix A, also used by ESPHome / Adafruit), with
//! `ratio = CH1 / (CH0 + CH1)`:
//!
//! | ratio | lux · GAIN · (T_int / 100 ms) |
//! |---|---|
//! | < 0.45 | 1.7743 · CH0 + 1.1059 · CH1 |
//! | < 0.64 | 4.2785 · CH0 − 1.9548 · CH1 |
//! | < 0.85 | 0.5926 · CH0 + 0.1185 · CH1 |
//! | ≥ 0.85 | 0 |
//!
//! The result is multiplied by an optional window (PFactor) correction.
//!
//! References: Lite-On "LTR-303ALS-01" and "LTR-329ALS-01" datasheets
//! (BNS-OD-FC002/A4).
//!
//! ```ignore
//! let mut ltr = Ltr303::new(i2c).with_gain(Gain::X8);
//! ltr.probe()?;
//! let m = ltr.measure(&mut delay)?;
//! ```

#![no_std]

use embedded_hal::i2c::I2c;
use zigbee_sensor::blocking::{LowPower, Sensor};
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Fixed I2C address.
pub const ADDRESS: u8 = 0x29;
/// Expected part number (`PART_ID[7:4]`).
pub const PART_NUMBER: u8 = 0xA;
/// Lite-On manufacturer ID.
pub const MANUFACTURER_ID: u8 = 0x05;

const REG_ALS_CONTR: u8 = 0x80;
const REG_ALS_MEAS_RATE: u8 = 0x85;
const REG_PART_ID: u8 = 0x86;
const REG_ALS_DATA: u8 = 0x88;
const REG_ALS_STATUS: u8 = 0x8C;
const REG_INTERRUPT: u8 = 0x8F;
const REG_ALS_THRES_UP: u8 = 0x97;
const REG_INTERRUPT_PERSIST: u8 = 0x9E;

const CONTR_SW_RESET: u8 = 1 << 1;
const CONTR_ACTIVE: u8 = 1 << 0;

const STATUS_INVALID: u8 = 1 << 7;
const STATUS_INT: u8 = 1 << 3;
const STATUS_NEW: u8 = 1 << 2;

const INT_POLARITY_HIGH: u8 = 1 << 2;
const INT_MODE_ALS: u8 = 1 << 1;

/// Standby → active wake-up (10 ms).
const WAKEUP_US: u32 = 10_000;

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

/// One sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Illuminance in milli-lux.
    pub illuminance: u32,
    /// CH0 (visible + IR) count.
    pub ch0: u16,
    /// CH1 (IR) count.
    pub ch1: u16,
    /// A channel reached full scale.
    pub saturated: bool,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Illuminance(self.illuminance));
    }
}

macro_rules! driver {
    ($($async_:ident)?, $($await_:tt)*) => {
        /// LTR-303 / LTR-329 driver.
        pub struct Ltr303<I2C> {
            i2c: I2C,
            gain: Gain,
            integration: IntegrationTime,
            pfactor_milli: u32,
            continuous: bool,
        }

        impl<I2C> Ltr303<I2C> {
            /// Create a driver (×1, 100 ms); no bus traffic.
            pub const fn new(i2c: I2C) -> Self {
                Self {
                    i2c,
                    gain: Gain::X1,
                    integration: IntegrationTime::Ms100,
                    pfactor_milli: 1000,
                    continuous: false,
                }
            }

            /// Select the gain.
            pub const fn with_gain(mut self, gain: Gain) -> Self {
                self.gain = gain;
                self
            }

            /// Select the integration time.
            pub const fn with_integration_time(mut self, integration: IntegrationTime) -> Self {
                self.integration = integration;
                self
            }

            /// Window correction × 1000 (1000 = no window).
            pub const fn with_window_factor(mut self, pfactor_milli: u32) -> Self {
                self.pfactor_milli = pfactor_milli;
                self
            }

            /// Keep measuring after a read (needed for the interrupt).
            pub const fn with_continuous(mut self, continuous: bool) -> Self {
                self.continuous = continuous;
                self
            }

            /// Change the gain for the next `start_measurement`.
            pub fn set_gain(&mut self, gain: Gain) {
                self.gain = gain;
            }

            /// Current gain.
            pub const fn gain(&self) -> Gain {
                self.gain
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.i2c
            }
        }

        impl<I2C: I2c> Ltr303<I2C> {
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

            /// Check `PART_ID[7:4]` = 0xA and `MANUFAC_ID` = 0x05; returns
            /// the revision.
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

            /// LTR-303 only: program the CH0 window and enable the INT pin
            /// (`None` disables it). `persistence` 0..=15 → 1..=16
            /// consecutive samples. Call while in standby.
            pub $($async_)? fn set_interrupt(
                &mut self,
                window: Option<(u16, u16)>,
                persistence: u8,
                active_high: bool,
            ) -> Result<(), Error<I2C::Error>> {
                let polarity = if active_high { INT_POLARITY_HIGH } else { 0 };
                let Some((low, high)) = window else {
                    return self.write(REG_INTERRUPT, polarity) $($await_)*;
                };
                let [h0, h1] = high.to_le_bytes();
                let [l0, l1] = low.to_le_bytes();
                self.i2c
                    .write(ADDRESS, &[REG_ALS_THRES_UP, h0, h1, l0, l1])
                    $($await_)*
                    .map_err(Error::bus)?;
                self.write(REG_INTERRUPT_PERSIST, persistence & 0x0F) $($await_)* ?;
                self.write(REG_INTERRUPT, polarity | INT_MODE_ALS) $($await_)*
            }

            /// Read `ALS_STATUS`; returns the interrupt flag.
            pub $($async_)? fn take_interrupt(&mut self) -> Result<bool, Error<I2C::Error>> {
                let mut status = [0u8; 1];
                self.read(REG_ALS_STATUS, &mut status) $($await_)* ?;
                Ok(status[0] & STATUS_INT != 0)
            }
        }

        impl<I2C: I2c> Sensor for Ltr303<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// Write the timing and switch to active mode.
            $($async_)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                let (rate_code, rate_ms) = self.integration.rate();
                let meas = (self.integration.code() << 3) | rate_code;
                self.write(REG_ALS_MEAS_RATE, meas) $($await_)* ?;
                let contr = (self.gain.code() << 2) | CONTR_ACTIVE;
                self.write(REG_ALS_CONTR, contr) $($await_)* ?;
                let rate_us = rate_ms * 1000;
                Ok(WAKEUP_US + rate_us + rate_us / 10)
            }

            /// Check the status, read CH1 then CH0 and go to standby
            /// (unless continuous).
            $($async_)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                // Status first: reading the data clears the "new" flag.
                let mut status = [0u8; 1];
                self.read(REG_ALS_STATUS, &mut status) $($await_)* ?;
                if status[0] & STATUS_NEW == 0 {
                    return Err(Error::NotReady);
                }
                if status[0] & STATUS_INVALID != 0 {
                    return Err(Error::InvalidData);
                }
                let mut buf = [0u8; 4];
                self.read(REG_ALS_DATA, &mut buf) $($await_)* ?;
                if !self.continuous {
                    self.write(REG_ALS_CONTR, 0) $($await_)* ?;
                }
                let ch1 = u16::from_le_bytes([buf[0], buf[1]]);
                let ch0 = u16::from_le_bytes([buf[2], buf[3]]);
                Ok(Measurement {
                    illuminance: lux_millilux(
                        ch0,
                        ch1,
                        self.gain,
                        self.integration,
                        self.pfactor_milli,
                    ),
                    ch0,
                    ch1,
                    saturated: ch0 == u16::MAX || ch1 == u16::MAX,
                })
            }
        }

        impl<I2C: I2c> LowPower for Ltr303<I2C> {
            type BusError = I2C::Error;

            $($async_)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write(REG_ALS_CONTR, 0) $($await_)*
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
        ADDRESS, CONTR_ACTIVE, CONTR_SW_RESET, Gain, INT_MODE_ALS, INT_POLARITY_HIGH,
        IntegrationTime, MANUFACTURER_ID, Measurement, PART_NUMBER, REG_ALS_CONTR, REG_ALS_DATA,
        REG_ALS_MEAS_RATE, REG_ALS_STATUS, REG_ALS_THRES_UP, REG_INTERRUPT, REG_INTERRUPT_PERSIST,
        REG_PART_ID, STATUS_INT, STATUS_INVALID, STATUS_NEW, WAKEUP_US, lux_millilux,
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
    fn lux_formula() {
        let t = IntegrationTime::Ms100;
        // ratio 0.17: 1.7743·1000 + 1.1059·200 = 1995.48 lx.
        assert_eq!(lux_millilux(1000, 200, Gain::X1, t, 1000), 1_995_480);
        // ratio 0.5: 4.2785·1000 − 1.9548·1000 = 2323.7 lx.
        assert_eq!(lux_millilux(1000, 1000, Gain::X1, t, 1000), 2_323_700);
        // ratio 0.67: 0.5926·1000 + 0.1185·2000 = 829.6 lx.
        assert_eq!(lux_millilux(1000, 2000, Gain::X1, t, 1000), 829_600);
        // ratio ≥ 0.85 → 0.
        assert_eq!(lux_millilux(100, 900, Gain::X1, t, 1000), 0);
        assert_eq!(lux_millilux(0, 0, Gain::X1, t, 1000), 0);
        // ×8, 200 ms divides by 16; window ×2.
        assert_eq!(
            lux_millilux(1000, 200, Gain::X8, IntegrationTime::Ms200, 2000),
            249_435
        );
    }

    #[test]
    fn gain_helper() {
        assert_eq!(suggest_gain(Gain::X8, 60_000, 0), Gain::X4);
        assert_eq!(suggest_gain(Gain::X1, 60_000, 0), Gain::X1);
        assert_eq!(suggest_gain(Gain::X8, 600, 100), Gain::X48);
        assert_eq!(suggest_gain(Gain::X8, 6000, 100), Gain::X8);
        assert_eq!(suggest_gain(Gain::X96, 10, 10), Gain::X96);
    }

    #[test]
    fn one_shot_cycle() {
        let expectations = [
            Transaction::write_read(ADDRESS, vec![0x86], vec![0xA0, 0x05]),
            Transaction::write(ADDRESS, vec![0x85, 0x01]),
            Transaction::write(ADDRESS, vec![0x80, 0x0D]),
            Transaction::write_read(ADDRESS, vec![0x8C], vec![0x30]),
            Transaction::write_read(ADDRESS, vec![0x8C], vec![0xB4]),
            Transaction::write_read(ADDRESS, vec![0x8C], vec![0x34]),
            Transaction::write_read(ADDRESS, vec![0x88], vec![0xC8, 0x00, 0xE8, 0x03]),
            Transaction::write(ADDRESS, vec![0x80, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut ltr = Ltr303::new(&mut i2c).with_gain(Gain::X8);
        assert_eq!(ltr.probe().unwrap(), 0);
        assert_eq!(ltr.start_measurement().unwrap(), 120_000);
        assert_eq!(ltr.read_measurement(), Err(Error::NotReady));
        assert_eq!(ltr.read_measurement(), Err(Error::InvalidData));
        let m = ltr.read_measurement().unwrap();
        assert_eq!((m.ch0, m.ch1), (1000, 200));
        assert_eq!(m.illuminance, 249_435);
        assert!(!m.saturated);
        i2c.done();
    }

    #[test]
    fn interrupt_continuous() {
        let expectations = [
            Transaction::write_read(ADDRESS, vec![0x86], vec![0x92, 0x05]),
            Transaction::write(ADDRESS, vec![0x97, 0x00, 0x10, 0x20, 0x00]),
            Transaction::write(ADDRESS, vec![0x9E, 0x03]),
            Transaction::write(ADDRESS, vec![0x8F, 0x02]),
            Transaction::write(ADDRESS, vec![0x85, 0x1B]),
            Transaction::write(ADDRESS, vec![0x80, 0x1D]),
            Transaction::write_read(ADDRESS, vec![0x8C], vec![0x7C]),
            Transaction::write_read(ADDRESS, vec![0x88], vec![0x00, 0x00, 0xFF, 0xFF]),
            Transaction::write_read(ADDRESS, vec![0x8C], vec![0x08]),
            Transaction::write(ADDRESS, vec![0x8F, 0x04]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut ltr = Ltr303::new(&mut i2c)
            .with_gain(Gain::X96)
            .with_integration_time(IntegrationTime::Ms400)
            .with_continuous(true);
        assert_eq!(ltr.probe(), Err(Error::UnexpectedId(0x9205)));
        ltr.set_interrupt(Some((0x20, 0x1000)), 3, false).unwrap();
        assert_eq!(ltr.start_measurement().unwrap(), 560_000);
        let m = ltr.read_measurement().unwrap();
        assert!(m.saturated);
        assert!(ltr.take_interrupt().unwrap());
        ltr.set_interrupt(None, 0, true).unwrap();
        i2c.done();
    }

    #[test]
    fn async_measure() {
        use zigbee_sensor::asynch::Sensor as _;

        struct RecordingDelay(u32);
        impl embedded_hal_async::delay::DelayNs for RecordingDelay {
            async fn delay_ns(&mut self, ns: u32) {
                self.0 += ns / 1_000;
            }
        }

        let expectations = [
            Transaction::write(ADDRESS, vec![0x85, 0x08]),
            Transaction::write(ADDRESS, vec![0x80, 0x01]),
            Transaction::write_read(ADDRESS, vec![0x8C], vec![0x04]),
            Transaction::write_read(ADDRESS, vec![0x88], vec![0xE8, 0x03, 0xE8, 0x03]),
            Transaction::write(ADDRESS, vec![0x80, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut ltr = asynch::Ltr303::new(&mut i2c).with_integration_time(IntegrationTime::Ms50);
        let mut delay = RecordingDelay(0);
        let m = block_on(ltr.measure(&mut delay)).unwrap();
        assert_eq!(m.illuminance, 4_647_400);
        assert_eq!(delay.0, 65_000);
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
