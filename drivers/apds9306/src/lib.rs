//! Broadcom APDS-9306 / APDS-9306-065 ambient light sensor (I2C).
//!
//! Photopic ALS and clear channels, 13–20 bit. `start_measurement` enables
//! the ALS, `read_measurement` checks the "new data" flag, burst-reads
//! status + both channels (the 0x07–0x18 range is locked during the read)
//! and returns the part to standby (2 µA typ.).
//!
//! Lux: the datasheet gives 2000 counts typ. at 3×, 100 ms for
//! 49.8 µW/cm² (APDS-9306) or 43 µW/cm² (APDS-9306-065). As in the Linux
//! driver this is taken as 340.134 lx resp. 293.69 lx and scaled linearly
//! with gain and integration time:
//! `lux = ALS · K / (GAIN · T_int / 3.125 ms)`, `K` = 16.326432 (‑9306)
//! or 14.09712 (‑9306‑065).
//!
//! Reference: Broadcom "APDS-9306/APDS-9306-065 Digital Ambient Light
//! Sensor" datasheet (AV02-4755EN).
//!
//! ```ignore
//! let mut als = Apds9306::new(i2c);
//! als.probe()?; // also selects the variant
//! let m = als.measure(&mut delay)?;
//! ```

#![no_std]

use embedded_hal::i2c::I2c;
use zigbee_sensor::blocking::{LowPower, Sensor};
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Fixed I2C address.
pub const ADDRESS: u8 = 0x52;
/// `PART_ID` of the APDS-9306.
pub const PART_ID_9306: u8 = 0xB1;
/// `PART_ID` of the APDS-9306-065.
pub const PART_ID_9306_065: u8 = 0xB3;

const REG_MAIN_CTRL: u8 = 0x00;
const REG_MEAS_RATE: u8 = 0x04;
const REG_GAIN: u8 = 0x05;
const REG_PART_ID: u8 = 0x06;
const REG_MAIN_STATUS: u8 = 0x07;
const REG_INT_CFG: u8 = 0x19;
const REG_INT_PERSISTENCE: u8 = 0x1A;
const REG_THRES_UP: u8 = 0x21;
const REG_THRES_VAR: u8 = 0x27;

const CTRL_SW_RESET: u8 = 1 << 4;
const CTRL_ALS_EN: u8 = 1 << 1;

const STATUS_INT: u8 = 1 << 4;
const STATUS_DATA: u8 = 1 << 3;

const INT_SEL_ALS: u8 = 0b01 << 4;
const INT_VAR_MODE: u8 = 1 << 3;
const INT_EN: u8 = 1 << 2;

/// Support-block settling after `ALS_EN` (5 ms typ.), with margin.
const WAKEUP_US: u32 = 10_000;

/// Part variant (selects the lux factor).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Variant {
    /// APDS-9306 (PART_ID 0xB1).
    #[default]
    Apds9306,
    /// APDS-9306-065 (PART_ID 0xB3).
    Apds9306_065,
}

impl Variant {
    /// Lux per count at 1×, 3.125 ms, in micro-lux.
    pub const fn k_microlux(self) -> u64 {
        match self {
            Self::Apds9306 => 16_326_432,
            Self::Apds9306_065 => 14_097_120,
        }
    }
}

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

/// Integration time / resolution.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Resolution {
    /// 20 bit, 400 ms.
    Bits20,
    /// 19 bit, 200 ms.
    Bits19,
    /// 18 bit, 100 ms.
    #[default]
    Bits18,
    /// 17 bit, 50 ms.
    Bits17,
    /// 16 bit, 25 ms.
    Bits16,
    /// 13 bit, 3.125 ms.
    Bits13,
}

impl Resolution {
    /// Conversion time in microseconds.
    pub const fn conversion_us(self) -> u32 {
        match self {
            Self::Bits20 => 400_000,
            Self::Bits19 => 200_000,
            Self::Bits18 => 100_000,
            Self::Bits17 => 50_000,
            Self::Bits16 => 25_000,
            Self::Bits13 => 3_125,
        }
    }

    /// Full-scale count.
    pub const fn full_scale(self) -> u32 {
        let bits = match self {
            Self::Bits20 => 20,
            Self::Bits19 => 19,
            Self::Bits18 => 18,
            Self::Bits17 => 17,
            Self::Bits16 => 16,
            Self::Bits13 => 13,
        };
        (1 << bits) - 1
    }
}

/// ALS count → milli-lux. `wfac_milli` is a window factor × 1000.
pub const fn als_to_millilux(
    als: u32,
    variant: Variant,
    gain: Gain,
    resolution: Resolution,
    wfac_milli: u32,
) -> u32 {
    // mlx = als · K[µlx] / 1000 · 3125 / (gain · t_us) · wfac / 1000
    let num = als as u64 * variant.k_microlux() * 3125 * wfac_milli as u64;
    let den = gain.factor() as u64 * resolution.conversion_us() as u64 * 1_000_000;
    let v = num / den;
    if v > u32::MAX as u64 {
        u32::MAX
    } else {
        v as u32
    }
}

/// Pick a gain for the next sample: step down above 90 % of full scale,
/// step up while the reading would stay below 50 %.
pub fn suggest_gain(current: Gain, resolution: Resolution, raw: u32) -> Gain {
    let fs = resolution.full_scale() as u64;
    let idx = Gain::ALL.iter().position(|&g| g == current).unwrap_or(1);
    if raw as u64 * 10 > fs * 9 {
        return Gain::ALL[idx.saturating_sub(1)];
    }
    match Gain::ALL.get(idx + 1) {
        Some(&next) if raw as u64 * next.factor() as u64 / current.factor() as u64 * 2 < fs => next,
        _ => current,
    }
}

/// Interrupt trigger.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Interrupt {
    /// ALS outside `low..=high` (raw counts).
    Threshold {
        /// Lower threshold.
        low: u32,
        /// Upper threshold.
        high: u32,
    },
    /// |ALS − previous ALS| > 8 · 2^`step` counts (`step` 0..=7).
    Variance {
        /// Variance code.
        step: u8,
    },
}

/// One sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Illuminance in milli-lux.
    pub illuminance: u32,
    /// ALS (photopic) count.
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
        /// APDS-9306 driver.
        pub struct Apds9306<I2C> {
            i2c: I2C,
            variant: Variant,
            gain: Gain,
            resolution: Resolution,
            wfac_milli: u32,
            continuous: bool,
        }

        impl<I2C> Apds9306<I2C> {
            /// Create a driver (APDS-9306, 3×, 100 ms); no bus traffic.
            pub const fn new(i2c: I2C) -> Self {
                Self {
                    i2c,
                    variant: Variant::Apds9306,
                    gain: Gain::X3,
                    resolution: Resolution::Bits18,
                    wfac_milli: 1000,
                    continuous: false,
                }
            }

            /// Select the variant without probing.
            pub const fn with_variant(mut self, variant: Variant) -> Self {
                self.variant = variant;
                self
            }

            /// Select the gain.
            pub const fn with_gain(mut self, gain: Gain) -> Self {
                self.gain = gain;
                self
            }

            /// Select the resolution / integration time.
            pub const fn with_resolution(mut self, resolution: Resolution) -> Self {
                self.resolution = resolution;
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

            /// Detected / configured variant.
            pub const fn variant(&self) -> Variant {
                self.variant
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.i2c
            }
        }

        impl<I2C: I2c> Apds9306<I2C> {
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

            /// Check `PART_ID` (0xB1 / 0xB3) and select the variant.
            pub $($async_)? fn probe(&mut self) -> Result<Variant, Error<I2C::Error>> {
                let mut id = [0u8; 1];
                self.read(REG_PART_ID, &mut id) $($await_)* ?;
                self.variant = match id[0] {
                    PART_ID_9306 => Variant::Apds9306,
                    PART_ID_9306_065 => Variant::Apds9306_065,
                    other => return Err(Error::UnexpectedId(other as u32)),
                };
                Ok(self.variant)
            }

            /// Software reset.
            pub $($async_)? fn reset(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write(REG_MAIN_CTRL, CTRL_SW_RESET) $($await_)*
            }

            /// Configure the ALS-channel interrupt (`None` disables it).
            /// `persistence` 0..=15 → 1..=16 consecutive events.
            pub $($async_)? fn set_interrupt(
                &mut self,
                interrupt: Option<Interrupt>,
                persistence: u8,
            ) -> Result<(), Error<I2C::Error>> {
                let cfg = match interrupt {
                    None => INT_SEL_ALS,
                    Some(Interrupt::Threshold { low, high }) => {
                        let [h0, h1, h2, _] = high.min(0xF_FFFF).to_le_bytes();
                        let [l0, l1, l2, _] = low.min(0xF_FFFF).to_le_bytes();
                        self.i2c
                            .write(ADDRESS, &[REG_THRES_UP, h0, h1, h2, l0, l1, l2])
                            $($await_)*
                            .map_err(Error::bus)?;
                        INT_SEL_ALS | INT_EN
                    }
                    Some(Interrupt::Variance { step }) => {
                        self.write(REG_THRES_VAR, step & 0x07) $($await_)* ?;
                        INT_SEL_ALS | INT_VAR_MODE | INT_EN
                    }
                };
                if interrupt.is_some() {
                    self.write(REG_INT_PERSISTENCE, (persistence & 0x0F) << 4) $($await_)* ?;
                }
                self.write(REG_INT_CFG, cfg) $($await_)*
            }

            /// Read `MAIN_STATUS` (clears it); returns the interrupt flag.
            pub $($async_)? fn take_interrupt(&mut self) -> Result<bool, Error<I2C::Error>> {
                let mut status = [0u8; 1];
                self.read(REG_MAIN_STATUS, &mut status) $($await_)* ?;
                Ok(status[0] & STATUS_INT != 0)
            }
        }

        impl<I2C: I2c> Sensor for Apds9306<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// Write gain / resolution (repeat rate at maximum speed) and
            /// enable the ALS.
            $($async_)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write(REG_MEAS_RATE, (self.resolution as u8) << 4) $($await_)* ?;
                self.write(REG_GAIN, self.gain as u8) $($await_)* ?;
                self.write(REG_MAIN_CTRL, CTRL_ALS_EN) $($await_)* ?;
                let conv = self.resolution.conversion_us();
                Ok(WAKEUP_US + conv + conv / 10)
            }

            /// Check the data flag, read clear + ALS and go to standby
            /// (unless continuous).
            $($async_)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let mut buf = [0u8; 9];
                self.read(REG_MAIN_STATUS, &mut buf) $($await_)* ?;
                if buf[0] & STATUS_DATA == 0 {
                    return Err(Error::NotReady);
                }
                if !self.continuous {
                    self.write(REG_MAIN_CTRL, 0) $($await_)* ?;
                }
                let clear = u32::from_le_bytes([buf[3], buf[4], buf[5] & 0x0F, 0]);
                let als = u32::from_le_bytes([buf[6], buf[7], buf[8] & 0x0F, 0]);
                let fs = self.resolution.full_scale();
                Ok(Measurement {
                    illuminance: als_to_millilux(
                        als,
                        self.variant,
                        self.gain,
                        self.resolution,
                        self.wfac_milli,
                    ),
                    als,
                    clear,
                    saturated: als >= fs || clear >= fs,
                })
            }
        }

        impl<I2C: I2c> LowPower for Apds9306<I2C> {
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
        ADDRESS, CTRL_ALS_EN, CTRL_SW_RESET, Gain, INT_EN, INT_SEL_ALS, INT_VAR_MODE, Interrupt,
        Measurement, PART_ID_9306, PART_ID_9306_065, REG_GAIN, REG_INT_CFG, REG_INT_PERSISTENCE,
        REG_MAIN_CTRL, REG_MAIN_STATUS, REG_MEAS_RATE, REG_PART_ID, REG_THRES_UP, REG_THRES_VAR,
        Resolution, STATUS_DATA, STATUS_INT, Variant, WAKEUP_US, als_to_millilux,
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
        // Datasheet point: 2000 counts at 3×, 100 ms.
        let r = Resolution::Bits18;
        assert_eq!(
            als_to_millilux(2000, Variant::Apds9306, Gain::X3, r, 1000),
            340_134
        );
        assert_eq!(
            als_to_millilux(2000, Variant::Apds9306_065, Gain::X3, r, 1000),
            293_690
        );
        // 1×, 3.125 ms: K lux per count.
        assert_eq!(
            als_to_millilux(1, Variant::Apds9306, Gain::X1, Resolution::Bits13, 1000),
            16_326
        );
        assert_eq!(suggest_gain(Gain::X3, r, 250_000), Gain::X1);
        assert_eq!(suggest_gain(Gain::X3, r, 1_000), Gain::X6);
        assert_eq!(suggest_gain(Gain::X18, r, 10), Gain::X18);
    }

    #[test]
    fn one_shot_cycle() {
        let expectations = [
            Transaction::write_read(ADDRESS, vec![0x06], vec![0xB3]),
            Transaction::write(ADDRESS, vec![0x04, 0x20]),
            Transaction::write(ADDRESS, vec![0x05, 0x01]),
            Transaction::write(ADDRESS, vec![0x00, 0x02]),
            Transaction::write_read(ADDRESS, vec![0x07], vec![0x20, 0, 0, 0, 0, 0, 0, 0, 0]),
            Transaction::write_read(
                ADDRESS,
                vec![0x07],
                vec![0x08, 0, 0, 0xA0, 0x0F, 0x00, 0xD0, 0x07, 0xF0],
            ),
            Transaction::write(ADDRESS, vec![0x00, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut als = Apds9306::new(&mut i2c);
        assert_eq!(als.probe().unwrap(), Variant::Apds9306_065);
        assert_eq!(als.start_measurement().unwrap(), 120_000);
        assert_eq!(als.read_measurement(), Err(Error::NotReady));
        let m = als.read_measurement().unwrap();
        assert_eq!((m.als, m.clear), (2000, 4000));
        assert_eq!(m.illuminance, 293_690);
        assert!(!m.saturated);
        i2c.done();
    }

    #[test]
    fn interrupts() {
        let expectations = [
            Transaction::write_read(ADDRESS, vec![0x06], vec![0xA1]),
            Transaction::write(ADDRESS, vec![0x21, 0xFF, 0xFF, 0x0F, 0x10, 0x00, 0x00]),
            Transaction::write(ADDRESS, vec![0x1A, 0x20]),
            Transaction::write(ADDRESS, vec![0x19, 0x14]),
            Transaction::write(ADDRESS, vec![0x27, 0x03]),
            Transaction::write(ADDRESS, vec![0x1A, 0x00]),
            Transaction::write(ADDRESS, vec![0x19, 0x1C]),
            Transaction::write_read(ADDRESS, vec![0x07], vec![0x10]),
            Transaction::write(ADDRESS, vec![0x19, 0x10]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut als = Apds9306::new(&mut i2c).with_continuous(true);
        assert_eq!(als.probe(), Err(Error::UnexpectedId(0xA1)));
        let th = Interrupt::Threshold {
            low: 0x10,
            high: 0xFF_FFFF,
        };
        als.set_interrupt(Some(th), 2).unwrap();
        als.set_interrupt(Some(Interrupt::Variance { step: 3 }), 0)
            .unwrap();
        assert!(als.take_interrupt().unwrap());
        als.set_interrupt(None, 0).unwrap();
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
            Transaction::write(ADDRESS, vec![0x04, 0x50]),
            Transaction::write(ADDRESS, vec![0x05, 0x04]),
            Transaction::write(ADDRESS, vec![0x00, 0x02]),
            Transaction::write_read(
                ADDRESS,
                vec![0x07],
                vec![0x08, 0, 0, 0xFF, 0x1F, 0, 0x00, 0x01, 0],
            ),
            Transaction::write(ADDRESS, vec![0x00, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut als = asynch::Apds9306::new(&mut i2c)
            .with_gain(Gain::X18)
            .with_resolution(Resolution::Bits13);
        let mut delay = RecordingDelay(0);
        let m = block_on(als.measure(&mut delay)).unwrap();
        assert!(m.saturated);
        assert_eq!(m.illuminance, 232_198);
        assert_eq!(delay.0, 13_437);
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
