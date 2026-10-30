//! Lite-On LTR-F216A ambient light sensor (I2C).
//!
//! A 24-bit-register ALS with a photopic ("green") channel and a clear
//! channel. Register map and `PART_ID` (0xB1) are identical to the
//! Broadcom APDS-9306, whose datasheet documents the bit fields. `start_measurement` enables
//! the sensor, `read_measurement` checks the "new data" flag, reads both
//! channels and returns the part to standby.
//!
//! Lux (Lite-On, as used by the Linux and Zephyr drivers):
//! `lux = 0.45 · ALS · WFAC / (GAIN · T_int / 100 ms)`.
//!
//! References: Linux `drivers/iio/light/ltrf216a.c` (co-authored by
//! Lite-On) and Zephyr `drivers/sensor/liteon/ltrf216a`; the Lite-On PDF is
//! not publicly mirrored.
//!
//! ```ignore
//! let mut ltr = LtrF216a::new(i2c).with_gain(Gain::X9);
//! ltr.probe()?;
//! let m = ltr.measure(&mut delay)?;
//! ```

#![no_std]

use embedded_hal::i2c::I2c;
use zigbee_sensor::blocking::{LowPower, Sensor};
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Fixed I2C address.
pub const ADDRESS: u8 = 0x53;
/// Expected `PART_ID` (part 0xB, revision 0x1).
pub const PART_ID: u8 = 0xB1;

const REG_MAIN_CTRL: u8 = 0x00;
const REG_MEAS_RES: u8 = 0x04;
const REG_GAIN: u8 = 0x05;
const REG_PART_ID: u8 = 0x06;
const REG_MAIN_STATUS: u8 = 0x07;
const REG_INT_CFG: u8 = 0x19;
const REG_INT_PST: u8 = 0x1A;
const REG_THRES_UP: u8 = 0x21;

const CTRL_SW_RESET: u8 = 1 << 4;
const CTRL_ENABLE: u8 = 1 << 1;

const STATUS_INT: u8 = 1 << 4;
const STATUS_DATA: u8 = 1 << 3;

const INT_SEL_ALS: u8 = 0b01 << 4;
const INT_EN: u8 = 1 << 2;

/// Standby → active wake-up allowance.
const WAKEUP_US: u32 = 10_000;

/// ALS gain.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum Gain {
    X1,
    #[default]
    X3,
    X6,
    X9,
    X18,
}

impl Gain {
    const ALL: [Gain; 5] = [Self::X1, Self::X3, Self::X6, Self::X9, Self::X18];

    /// Numeric gain.
    pub const fn factor(self) -> u32 {
        match self {
            Self::X1 => 1,
            Self::X3 => 3,
            Self::X6 => 6,
            Self::X9 => 9,
            Self::X18 => 18,
        }
    }
}

/// Integration time (and matching ADC resolution).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum IntegrationTime {
    /// 400 ms, 20 bit.
    Ms400,
    /// 200 ms, 19 bit.
    Ms200,
    /// 100 ms, 18 bit.
    #[default]
    Ms100,
    /// 50 ms, 17 bit.
    Ms50,
    /// 25 ms, 16 bit.
    Ms25,
}

impl IntegrationTime {
    /// `ALS_MEAS_RES` value (resolution | rate), as programmed by Linux.
    /// A rate faster than the conversion runs at conversion speed.
    const fn reg(self) -> u8 {
        match self {
            Self::Ms400 => 0x03,
            Self::Ms200 => 0x13,
            Self::Ms100 => 0x22,
            Self::Ms50 => 0x31,
            Self::Ms25 => 0x40,
        }
    }

    /// Integration time in milliseconds.
    pub const fn ms(self) -> u32 {
        match self {
            Self::Ms400 => 400,
            Self::Ms200 => 200,
            Self::Ms100 => 100,
            Self::Ms50 => 50,
            Self::Ms25 => 25,
        }
    }

    /// Full-scale count.
    pub const fn full_scale(self) -> u32 {
        (1 << (20 - (self.reg() >> 4))) - 1
    }
}

/// ALS count → milli-lux. `wfac_milli` is the window factor × 1000.
pub const fn als_to_millilux(
    als: u32,
    gain: Gain,
    integration: IntegrationTime,
    wfac_milli: u32,
) -> u32 {
    // mlx = als · 0.45 · 1000 · wfac / (gain · t/100)
    let num = als as u64 * 45_000 * wfac_milli as u64;
    let den = gain.factor() as u64 * integration.ms() as u64 * 1000;
    let v = num / den;
    if v > u32::MAX as u64 {
        u32::MAX
    } else {
        v as u32
    }
}

/// Pick a gain for the next sample: step down above 90 % of full scale,
/// step up while the reading would stay below 50 %.
pub fn suggest_gain(current: Gain, integration: IntegrationTime, raw: u32) -> Gain {
    let fs = integration.full_scale() as u64;
    let idx = Gain::ALL.iter().position(|&g| g == current).unwrap_or(1);
    if raw as u64 * 10 > fs * 9 {
        return Gain::ALL[idx.saturating_sub(1)];
    }
    match Gain::ALL.get(idx + 1) {
        Some(&next) if raw as u64 * next.factor() as u64 / current.factor() as u64 * 2 < fs => next,
        _ => current,
    }
}

/// One sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Illuminance in milli-lux.
    pub illuminance: u32,
    /// Photopic ALS count.
    pub als: u32,
    /// Clear-channel count.
    pub clear: u32,
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
        /// LTR-F216A driver.
        pub struct LtrF216a<I2C> {
            i2c: I2C,
            gain: Gain,
            integration: IntegrationTime,
            wfac_milli: u32,
            continuous: bool,
        }

        impl<I2C> LtrF216a<I2C> {
            /// Create a driver (×3, 100 ms); no bus traffic.
            pub const fn new(i2c: I2C) -> Self {
                Self {
                    i2c,
                    gain: Gain::X3,
                    integration: IntegrationTime::Ms100,
                    wfac_milli: 1000,
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

            /// Window factor × 1000 (1000 = no window).
            pub const fn with_window_factor(mut self, wfac_milli: u32) -> Self {
                self.wfac_milli = wfac_milli;
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

        impl<I2C: I2c> LtrF216a<I2C> {
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

            /// Check `PART_ID` = 0xB1.
            pub $($async_)? fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
                let mut id = [0u8; 1];
                self.read(REG_PART_ID, &mut id) $($await_)* ?;
                if id[0] != PART_ID {
                    return Err(Error::UnexpectedId(id[0] as u32));
                }
                Ok(())
            }

            /// Software reset. The part does not acknowledge it, so bus
            /// errors are ignored.
            pub $($async_)? fn reset(&mut self) {
                let _ = self.write(REG_MAIN_CTRL, CTRL_SW_RESET) $($await_)*;
            }

            /// Program the 20-bit ALS window and enable the interrupt
            /// (`None` disables it). `persistence` 0..=15 → 1..=16 samples.
            pub $($async_)? fn set_interrupt(
                &mut self,
                window: Option<(u32, u32)>,
                persistence: u8,
            ) -> Result<(), Error<I2C::Error>> {
                let Some((low, high)) = window else {
                    return self.write(REG_INT_CFG, INT_SEL_ALS) $($await_)*;
                };
                let [h0, h1, h2, _] = high.min(0xF_FFFF).to_le_bytes();
                let [l0, l1, l2, _] = low.min(0xF_FFFF).to_le_bytes();
                self.i2c
                    .write(ADDRESS, &[REG_THRES_UP, h0, h1, h2, l0, l1, l2])
                    $($await_)*
                    .map_err(Error::bus)?;
                self.write(REG_INT_PST, (persistence & 0x0F) << 4) $($await_)* ?;
                self.write(REG_INT_CFG, INT_SEL_ALS | INT_EN) $($await_)*
            }

            /// Read `MAIN_STATUS` (clears it); returns the interrupt flag.
            pub $($async_)? fn take_interrupt(&mut self) -> Result<bool, Error<I2C::Error>> {
                let mut status = [0u8; 1];
                self.read(REG_MAIN_STATUS, &mut status) $($await_)* ?;
                Ok(status[0] & STATUS_INT != 0)
            }
        }

        impl<I2C: I2c> Sensor for LtrF216a<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// Write gain / integration time and enable the sensor.
            $($async_)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write(REG_MEAS_RES, self.integration.reg()) $($await_)* ?;
                self.write(REG_GAIN, self.gain as u8) $($await_)* ?;
                self.write(REG_MAIN_CTRL, CTRL_ENABLE) $($await_)* ?;
                let period = self.integration.ms() * 1000;
                Ok(WAKEUP_US + period + period / 10)
            }

            /// Check the data flag, read clear + ALS and go to standby
            /// (unless continuous).
            $($async_)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                // MAIN_STATUS .. ALS_DATA_2 in one locked burst.
                let mut buf = [0u8; 9];
                self.read(REG_MAIN_STATUS, &mut buf) $($await_)* ?;
                if buf[0] & STATUS_DATA == 0 {
                    return Err(Error::NotReady);
                }
                if !self.continuous {
                    self.write(REG_MAIN_CTRL, 0) $($await_)* ?;
                }
                let clear = u32::from_le_bytes([buf[3], buf[4], buf[5], 0]);
                let als = u32::from_le_bytes([buf[6], buf[7], buf[8], 0]);
                let fs = self.integration.full_scale();
                Ok(Measurement {
                    illuminance: als_to_millilux(als, self.gain, self.integration, self.wfac_milli),
                    als,
                    clear,
                    saturated: als >= fs || clear >= fs,
                })
            }
        }

        impl<I2C: I2c> LowPower for LtrF216a<I2C> {
            type BusError = I2C::Error;

            $($async_)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write(REG_MAIN_CTRL, 0) $($await_)*
            }

            /// Samples enable the part themselves; nothing to do.
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
        ADDRESS, CTRL_ENABLE, CTRL_SW_RESET, Gain, INT_EN, INT_SEL_ALS, IntegrationTime,
        Measurement, PART_ID, REG_GAIN, REG_INT_CFG, REG_INT_PST, REG_MAIN_CTRL, REG_MAIN_STATUS,
        REG_MEAS_RES, REG_PART_ID, REG_THRES_UP, STATUS_DATA, STATUS_INT, WAKEUP_US,
        als_to_millilux,
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
        // ×3, 100 ms: 0.15 lx per count (Zephyr default scaling).
        assert_eq!(
            als_to_millilux(1000, Gain::X3, IntegrationTime::Ms100, 1000),
            150_000
        );
        assert_eq!(
            als_to_millilux(1000, Gain::X18, IntegrationTime::Ms400, 2000),
            12_500
        );
        assert_eq!(IntegrationTime::Ms400.full_scale(), 0xF_FFFF);
        assert_eq!(IntegrationTime::Ms25.full_scale(), 0xFFFF);
        let t = IntegrationTime::Ms100;
        assert_eq!(suggest_gain(Gain::X3, t, 250_000), Gain::X1);
        assert_eq!(suggest_gain(Gain::X1, t, 250_000), Gain::X1);
        assert_eq!(suggest_gain(Gain::X3, t, 1_000), Gain::X6);
        assert_eq!(suggest_gain(Gain::X3, t, 100_000), Gain::X3);
        assert_eq!(suggest_gain(Gain::X18, t, 10), Gain::X18);
    }

    #[test]
    fn one_shot_cycle() {
        let expectations = [
            Transaction::write_read(ADDRESS, vec![0x06], vec![0xB1]),
            Transaction::write(ADDRESS, vec![0x04, 0x22]),
            Transaction::write(ADDRESS, vec![0x05, 0x01]),
            Transaction::write(ADDRESS, vec![0x00, 0x02]),
            Transaction::write_read(ADDRESS, vec![0x07], vec![0x00; 9]),
            Transaction::write_read(
                ADDRESS,
                vec![0x07],
                vec![0x08, 0, 0, 0xD0, 0x07, 0x00, 0xE8, 0x03, 0x00],
            ),
            Transaction::write(ADDRESS, vec![0x00, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut ltr = LtrF216a::new(&mut i2c);
        ltr.probe().unwrap();
        assert_eq!(ltr.start_measurement().unwrap(), 120_000);
        assert_eq!(ltr.read_measurement(), Err(Error::NotReady));
        let m = ltr.read_measurement().unwrap();
        assert_eq!((m.als, m.clear), (1000, 2000));
        assert_eq!(m.illuminance, 150_000);
        assert!(!m.saturated);
        i2c.done();
    }

    #[test]
    fn interrupt_continuous() {
        let expectations = [
            Transaction::write_read(ADDRESS, vec![0x06], vec![0xB2]),
            Transaction::write(ADDRESS, vec![0x21, 0x00, 0x00, 0x01, 0x10, 0x00, 0x00]),
            Transaction::write(ADDRESS, vec![0x1A, 0x10]),
            Transaction::write(ADDRESS, vec![0x19, 0x14]),
            Transaction::write(ADDRESS, vec![0x04, 0x40]),
            Transaction::write(ADDRESS, vec![0x05, 0x00]),
            Transaction::write(ADDRESS, vec![0x00, 0x02]),
            Transaction::write_read(
                ADDRESS,
                vec![0x07],
                vec![0x18, 0, 0, 0xFF, 0xFF, 0x00, 0x00, 0x10, 0x00],
            ),
            Transaction::write_read(ADDRESS, vec![0x07], vec![0x10]),
            Transaction::write(ADDRESS, vec![0x19, 0x10]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut ltr = LtrF216a::new(&mut i2c)
            .with_gain(Gain::X1)
            .with_integration_time(IntegrationTime::Ms25)
            .with_continuous(true);
        assert_eq!(ltr.probe(), Err(Error::UnexpectedId(0xB2)));
        ltr.set_interrupt(Some((0x10, 0x1_0000)), 1).unwrap();
        assert_eq!(ltr.start_measurement().unwrap(), 37_500);
        let m = ltr.read_measurement().unwrap();
        assert!(m.saturated);
        assert_eq!(m.als, 0x1000);
        assert!(ltr.take_interrupt().unwrap());
        ltr.set_interrupt(None, 0).unwrap();
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
            Transaction::write(ADDRESS, vec![0x04, 0x03]),
            Transaction::write(ADDRESS, vec![0x05, 0x04]),
            Transaction::write(ADDRESS, vec![0x00, 0x02]),
            Transaction::write_read(
                ADDRESS,
                vec![0x07],
                vec![0x08, 0, 0, 0, 0, 0, 0xE8, 0x03, 0x00],
            ),
            Transaction::write(ADDRESS, vec![0x00, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut ltr = asynch::LtrF216a::new(&mut i2c)
            .with_gain(Gain::X18)
            .with_integration_time(IntegrationTime::Ms400)
            .with_window_factor(2000);
        let mut delay = RecordingDelay(0);
        let m = block_on(ltr.measure(&mut delay)).unwrap();
        assert_eq!(m.illuminance, 12_500);
        assert_eq!(delay.0, 450_000);
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
