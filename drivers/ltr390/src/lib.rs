//! Lite-On LTR-390UV-01 ambient light / UV index sensor (I2C).
//!
//! The part measures either ambient light (ALS mode) or UV-A/B (UVS mode),
//! one at a time; the mode is a driver setting ([`Ltr390::set_mode`]), so a
//! device reporting both alternates samples. There is no one-shot mode:
//! `start_measurement` enables the sensor, `read_measurement` checks the
//! "new data" flag, reads the 20-bit result and returns the part to
//! standby (1 µA typ.).
//!
//! Formulas (datasheet §7):
//! * `lux = 0.6 · ALS / (GAIN · INT) · WFAC`
//! * `UVI = UV / (2300 · GAIN/18 · INT/4) · WFAC` — the sensitivity is
//!   specified as 2300 counts/UVI at ×18, 20-bit (400 ms) and scales with
//!   gain and integration time.
//!
//! Reference: Lite-On "LTR-390UV-01 Optical Sensor" datasheet
//! (BNS-OD-FC002/A4, 33 pages).
//!
//! ```ignore
//! let mut ltr = Ltr390::new(i2c).with_gain(Gain::X18).with_mode(Mode::Uv);
//! ltr.probe()?;
//! let wait_us = ltr.start_measurement()?;
//! // ... sleep wait_us ...
//! let uvi_centi = ltr.read_measurement()?.uv_index;
//! ```

#![no_std]

use embedded_hal::i2c::I2c;
use zigbee_sensor::blocking::{LowPower, Sensor};
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Fixed I2C address.
pub const ADDRESS: u8 = 0x53;
/// Expected part number (`PART_ID[7:4]`).
pub const PART_NUMBER: u8 = 0xB;

const REG_MAIN_CTRL: u8 = 0x00;
const REG_MEAS_RATE: u8 = 0x04;
const REG_GAIN: u8 = 0x05;
const REG_PART_ID: u8 = 0x06;
const REG_MAIN_STATUS: u8 = 0x07;
const REG_INT_CFG: u8 = 0x19;
const REG_INT_PST: u8 = 0x1A;
const REG_THRES_UP: u8 = 0x21;

const CTRL_SW_RESET: u8 = 1 << 4;
const CTRL_UVS_MODE: u8 = 1 << 3;
const CTRL_ENABLE: u8 = 1 << 1;

const STATUS_INT: u8 = 1 << 4;
const STATUS_DATA: u8 = 1 << 3;

const INT_SEL_ALS: u8 = 0b01 << 4;
const INT_SEL_UVS: u8 = 0b11 << 4;
const INT_EN: u8 = 1 << 2;

/// Standby → active wake-up (10 ms max).
const WAKEUP_US: u32 = 10_000;

/// UV sensitivity at ×18, 20-bit: 2300 counts per UV index.
pub const UV_SENSITIVITY: u32 = 2300;

/// Measured channel.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Mode {
    /// Ambient light (lux).
    #[default]
    Als,
    /// UV (UV index).
    Uv,
}

/// ALS/UVS gain range.
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

/// ADC resolution / conversion time.
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
    /// 13 bit, 12.5 ms.
    Bits13,
}

impl Resolution {
    const fn code(self) -> u8 {
        self as u8
    }

    /// Conversion time in microseconds.
    pub const fn conversion_us(self) -> u32 {
        match self {
            Self::Bits20 => 400_000,
            Self::Bits19 => 200_000,
            Self::Bits18 => 100_000,
            Self::Bits17 => 50_000,
            Self::Bits16 => 25_000,
            Self::Bits13 => 12_500,
        }
    }

    /// Datasheet INT factor × 8 (18 bit / 100 ms = 1 → 8).
    pub const fn int_x8(self) -> u32 {
        self.conversion_us() / 12_500
    }

    /// Shortest measurement-rate code not faster than the conversion.
    const fn rate_code(self) -> u8 {
        match self {
            Self::Bits20 => 0b100,
            Self::Bits19 => 0b011,
            Self::Bits18 => 0b010,
            Self::Bits17 => 0b001,
            Self::Bits16 | Self::Bits13 => 0b000,
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

/// ALS count → milli-lux. `wfac_milli` is the window factor × 1000
/// (1000 = no window).
pub const fn als_to_millilux(als: u32, gain: Gain, resolution: Resolution, wfac_milli: u32) -> u32 {
    let num = als as u64 * 600 * 8 * wfac_milli as u64;
    let den = gain.factor() as u64 * resolution.int_x8() as u64 * 1000;
    let v = num / den;
    if v > u32::MAX as u64 {
        u32::MAX
    } else {
        v as u32
    }
}

/// UV count → UV index × 100.
pub const fn uv_to_centi_uvi(uv: u32, gain: Gain, resolution: Resolution, wfac_milli: u32) -> u16 {
    // UVI·100 = uv·100·18·32·wfac / (2300·gain·int_x8·1000)
    let num = uv as u64 * 100 * 18 * 32 * wfac_milli as u64;
    let den = UV_SENSITIVITY as u64 * gain.factor() as u64 * resolution.int_x8() as u64 * 1000;
    let v = num / den;
    if v > u16::MAX as u64 {
        u16::MAX
    } else {
        v as u16
    }
}

/// One sample (only the field for the active mode is set).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Mode the sample was taken in.
    pub mode: Mode,
    /// Raw 20-bit count.
    pub raw: u32,
    /// Illuminance in milli-lux (ALS mode).
    pub illuminance: Option<u32>,
    /// UV index × 100 (UV mode).
    pub uv_index: Option<u16>,
    /// Count reached full scale.
    pub saturated: bool,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        if let Some(lux) = self.illuminance {
            f(Quantity::Illuminance(lux));
        }
        if let Some(uvi) = self.uv_index {
            f(Quantity::UvIndex(uvi));
        }
    }
}

macro_rules! driver {
    ($($async_:ident)?, $($await_:tt)*) => {
        /// LTR-390UV driver.
        pub struct Ltr390<I2C> {
            i2c: I2C,
            mode: Mode,
            gain: Gain,
            resolution: Resolution,
            wfac_milli: u32,
            continuous: bool,
        }

        impl<I2C> Ltr390<I2C> {
            /// Create a driver (ALS, ×3, 18 bit); no bus traffic.
            pub const fn new(i2c: I2C) -> Self {
                Self {
                    i2c,
                    mode: Mode::Als,
                    gain: Gain::X3,
                    resolution: Resolution::Bits18,
                    wfac_milli: 1000,
                    continuous: false,
                }
            }

            /// Select ALS or UV.
            pub const fn with_mode(mut self, mode: Mode) -> Self {
                self.mode = mode;
                self
            }

            /// Select the gain.
            pub const fn with_gain(mut self, gain: Gain) -> Self {
                self.gain = gain;
                self
            }

            /// Select the resolution / conversion time.
            pub const fn with_resolution(mut self, resolution: Resolution) -> Self {
                self.resolution = resolution;
                self
            }

            /// Window factor × 1000 (WFAC; 1000 = no / clear window).
            pub const fn with_window_factor(mut self, wfac_milli: u32) -> Self {
                self.wfac_milli = wfac_milli;
                self
            }

            /// Keep measuring after a read (needed for the interrupt).
            pub const fn with_continuous(mut self, continuous: bool) -> Self {
                self.continuous = continuous;
                self
            }

            /// Switch between ALS and UV for the next `start_measurement`.
            pub fn set_mode(&mut self, mode: Mode) {
                self.mode = mode;
            }

            /// Change the gain for the next `start_measurement`.
            pub fn set_gain(&mut self, gain: Gain) {
                self.gain = gain;
            }

            /// Current mode.
            pub const fn mode(&self) -> Mode {
                self.mode
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.i2c
            }

            const fn main_ctrl(&self) -> u8 {
                match self.mode {
                    Mode::Als => CTRL_ENABLE,
                    Mode::Uv => CTRL_ENABLE | CTRL_UVS_MODE,
                }
            }
        }

        impl<I2C: I2c> Ltr390<I2C> {
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

            /// Check `PART_ID[7:4]` = 0xB; returns the revision.
            pub $($async_)? fn probe(&mut self) -> Result<u8, Error<I2C::Error>> {
                let mut id = [0u8; 1];
                self.read(REG_PART_ID, &mut id) $($await_)* ?;
                if id[0] >> 4 != PART_NUMBER {
                    return Err(Error::UnexpectedId(id[0] as u32));
                }
                Ok(id[0] & 0x0F)
            }

            /// Software reset.
            pub $($async_)? fn reset(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write(REG_MAIN_CTRL, CTRL_SW_RESET) $($await_)*
            }

            /// Program the 20-bit raw thresholds for the current mode's
            /// channel and enable the interrupt (`None` disables it).
            /// `persistence` 0..=15 → 1..=16 consecutive samples.
            pub $($async_)? fn set_interrupt(
                &mut self,
                window: Option<(u32, u32)>,
                persistence: u8,
            ) -> Result<(), Error<I2C::Error>> {
                let sel = match self.mode {
                    Mode::Als => INT_SEL_ALS,
                    Mode::Uv => INT_SEL_UVS,
                };
                let Some((low, high)) = window else {
                    return self.write(REG_INT_CFG, sel) $($await_)*;
                };
                let [h0, h1, h2, _] = high.min(0xF_FFFF).to_le_bytes();
                let [l0, l1, l2, _] = low.min(0xF_FFFF).to_le_bytes();
                self.i2c
                    .write(ADDRESS, &[REG_THRES_UP, h0, h1, h2, l0, l1, l2])
                    $($await_)*
                    .map_err(Error::bus)?;
                self.write(REG_INT_PST, (persistence & 0x0F) << 4) $($await_)* ?;
                self.write(REG_INT_CFG, sel | INT_EN) $($await_)*
            }

            /// Read `MAIN_STATUS` (clears it); returns the interrupt flag.
            pub $($async_)? fn take_interrupt(&mut self) -> Result<bool, Error<I2C::Error>> {
                let mut status = [0u8; 1];
                self.read(REG_MAIN_STATUS, &mut status) $($await_)* ?;
                Ok(status[0] & STATUS_INT != 0)
            }
        }

        impl<I2C: I2c> Sensor for Ltr390<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// Write gain / resolution and enable the sensor in the
            /// selected mode.
            $($async_)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                let rate = (self.resolution.code() << 4) | self.resolution.rate_code();
                self.write(REG_MEAS_RATE, rate) $($await_)* ?;
                self.write(REG_GAIN, self.gain as u8) $($await_)* ?;
                let ctrl = self.main_ctrl();
                self.write(REG_MAIN_CTRL, ctrl) $($await_)* ?;
                let conv = self.resolution.conversion_us();
                Ok(WAKEUP_US + conv + conv / 10)
            }

            /// Check the data flag, read the result and go to standby
            /// (unless continuous).
            $($async_)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                // MAIN_STATUS .. UVS_DATA_2 in one locked burst.
                let mut buf = [0u8; 12];
                self.read(REG_MAIN_STATUS, &mut buf) $($await_)* ?;
                if buf[0] & STATUS_DATA == 0 {
                    return Err(Error::NotReady);
                }
                if !self.continuous {
                    self.write(REG_MAIN_CTRL, 0) $($await_)* ?;
                }
                let als = u32::from_le_bytes([buf[6], buf[7], buf[8] & 0x0F, 0]);
                let uv = u32::from_le_bytes([buf[9], buf[10], buf[11] & 0x0F, 0]);
                let (raw, illuminance, uv_index) = match self.mode {
                    Mode::Als => (
                        als,
                        Some(als_to_millilux(als, self.gain, self.resolution, self.wfac_milli)),
                        None,
                    ),
                    Mode::Uv => (
                        uv,
                        None,
                        Some(uv_to_centi_uvi(uv, self.gain, self.resolution, self.wfac_milli)),
                    ),
                };
                Ok(Measurement {
                    mode: self.mode,
                    raw,
                    illuminance,
                    uv_index,
                    saturated: raw >= self.resolution.full_scale(),
                })
            }
        }

        impl<I2C: I2c> LowPower for Ltr390<I2C> {
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
        ADDRESS, CTRL_ENABLE, CTRL_SW_RESET, CTRL_UVS_MODE, Gain, INT_EN, INT_SEL_ALS, INT_SEL_UVS,
        Measurement, Mode, PART_NUMBER, REG_GAIN, REG_INT_CFG, REG_INT_PST, REG_MAIN_CTRL,
        REG_MAIN_STATUS, REG_MEAS_RATE, REG_PART_ID, REG_THRES_UP, Resolution, STATUS_DATA,
        STATUS_INT, WAKEUP_US, als_to_millilux, uv_to_centi_uvi,
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
    fn formulas() {
        // ×3, 18 bit (INT 1): 0.6 · 1000 / 3 = 200 lx.
        assert_eq!(
            als_to_millilux(1000, Gain::X3, Resolution::Bits18, 1000),
            200_000
        );
        // 13 bit (INT 0.125), ×1.
        assert_eq!(
            als_to_millilux(8191, Gain::X1, Resolution::Bits13, 1000),
            39_316_800
        );
        // Window factor 1.5.
        assert_eq!(
            als_to_millilux(1000, Gain::X3, Resolution::Bits18, 1500),
            300_000
        );
        // ×18, 20 bit: 2300 counts = UVI 1.00.
        assert_eq!(
            uv_to_centi_uvi(2300, Gain::X18, Resolution::Bits20, 1000),
            100
        );
        // ×18, 18 bit: sensitivity 575 counts/UVI → 1150 counts = UVI 2.00.
        assert_eq!(
            uv_to_centi_uvi(1150, Gain::X18, Resolution::Bits18, 1000),
            200
        );
        assert_eq!(Resolution::Bits20.int_x8(), 32);
        assert_eq!(Resolution::Bits13.full_scale(), 8191);
    }

    #[test]
    fn als_cycle() {
        let expectations = [
            Transaction::write_read(ADDRESS, vec![0x06], vec![0xB2]),
            Transaction::write(ADDRESS, vec![0x04, 0x22]),
            Transaction::write(ADDRESS, vec![0x05, 0x01]),
            Transaction::write(ADDRESS, vec![0x00, 0x02]),
            Transaction::write_read(ADDRESS, vec![0x07], vec![0x20; 12]),
            Transaction::write_read(
                ADDRESS,
                vec![0x07],
                vec![0x08, 0, 0, 0, 0, 0, 0xE8, 0x03, 0xF0, 0x01, 0x02, 0x03],
            ),
            Transaction::write(ADDRESS, vec![0x00, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut ltr = Ltr390::new(&mut i2c);
        assert_eq!(ltr.probe().unwrap(), 2);
        assert_eq!(ltr.start_measurement().unwrap(), 120_000);
        assert_eq!(ltr.read_measurement(), Err(Error::NotReady));
        let m = ltr.read_measurement().unwrap();
        assert_eq!(m.raw, 1000);
        assert_eq!(m.illuminance, Some(200_000));
        assert_eq!(m.uv_index, None);
        i2c.done();
    }

    #[test]
    fn uv_continuous_interrupt() {
        let expectations = [
            Transaction::write_read(ADDRESS, vec![0x06], vec![0xA1]),
            Transaction::write(ADDRESS, vec![0x21, 0x00, 0x10, 0x00, 0x10, 0x00, 0x00]),
            Transaction::write(ADDRESS, vec![0x1A, 0x30]),
            Transaction::write(ADDRESS, vec![0x19, 0x34]),
            Transaction::write(ADDRESS, vec![0x04, 0x04]),
            Transaction::write(ADDRESS, vec![0x05, 0x04]),
            Transaction::write(ADDRESS, vec![0x00, 0x0A]),
            Transaction::write_read(
                ADDRESS,
                vec![0x07],
                vec![0x18, 0, 0, 0, 0, 0, 0, 0, 0, 0xFC, 0x08, 0xF0],
            ),
            Transaction::write_read(ADDRESS, vec![0x07], vec![0x10]),
            Transaction::write(ADDRESS, vec![0x19, 0x30]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut ltr = Ltr390::new(&mut i2c)
            .with_mode(Mode::Uv)
            .with_gain(Gain::X18)
            .with_resolution(Resolution::Bits20)
            .with_continuous(true);
        assert_eq!(ltr.probe(), Err(Error::UnexpectedId(0xA1)));
        ltr.set_interrupt(Some((0x10, 0x1000)), 3).unwrap();
        assert_eq!(ltr.start_measurement().unwrap(), 450_000);
        let m = ltr.read_measurement().unwrap();
        assert_eq!(m.raw, 0x08FC);
        assert_eq!(m.uv_index, Some(100));
        assert!(m.illuminance.is_none());
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
            Transaction::write(ADDRESS, vec![0x04, 0x40]),
            Transaction::write(ADDRESS, vec![0x05, 0x00]),
            Transaction::write(ADDRESS, vec![0x00, 0x02]),
            Transaction::write_read(
                ADDRESS,
                vec![0x07],
                vec![0x08, 0, 0, 0, 0, 0, 0xFF, 0xFF, 0x00, 0, 0, 0],
            ),
            Transaction::write(ADDRESS, vec![0x00, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut ltr = asynch::Ltr390::new(&mut i2c)
            .with_gain(Gain::X1)
            .with_resolution(Resolution::Bits16);
        let mut delay = RecordingDelay(0);
        let m = block_on(ltr.measure(&mut delay)).unwrap();
        assert!(m.saturated);
        assert_eq!(m.illuminance, Some(157_284_000));
        assert_eq!(delay.0, 37_500);
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
