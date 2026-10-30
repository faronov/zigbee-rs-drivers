//! ROHM BH1730FVC two-channel (visible + IR) ambient light sensor (I2C).
//!
//! Each sample is a one-time conversion (`CONTROL.ONE_TIME`): the ADC
//! returns to power-down (0.85 µA typ.) by itself once both channels are
//! converted. Illuminance is computed from DATA0 (visible + IR) and DATA1
//! (IR) with the piecewise formula of datasheet section 6, in integer
//! arithmetic.
//!
//! Reference: ROHM "BH1730FVC Digital 16bit Serial Output Type Ambient Light
//! Sensor IC", datasheet Rev.001 (8 Jun 2016).

#![no_std]

use embedded_hal::i2c::I2c;
use zigbee_sensor::blocking::{LowPower, Sensor};
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Fixed I2C address (`0101001`).
pub const ADDRESS: u8 = 0x29;

/// Expected `PART_ID[7:4]`.
pub const PART_NUMBER: u8 = 0x7;

const CMD: u8 = 0x80;
const CMD_SPECIAL: u8 = 0xE0;
const SPECIAL_INT_RESET: u8 = 0x01;
const SPECIAL_SOFT_RESET: u8 = 0x04;

const REG_CONTROL: u8 = 0x00;
const REG_TIMING: u8 = 0x01;
const REG_INTERRUPT: u8 = 0x02;
const REG_TH_LOW: u8 = 0x03;
const REG_GAIN: u8 = 0x07;
const REG_ID: u8 = 0x12;
const REG_DATA0: u8 = 0x14;

const CONTROL_ADC_VALID: u8 = 1 << 4;
const CONTROL_ONE_TIME: u8 = 1 << 3;
const CONTROL_ADC_EN: u8 = 1 << 1;
const CONTROL_POWER: u8 = 1 << 0;
/// `CONTROL` value that starts one conversion of both channels.
pub const CONTROL_START_ONE_SHOT: u8 = CONTROL_ONE_TIME | CONTROL_ADC_EN | CONTROL_POWER;

/// Power-on default `TIMING` value (≈ 102.6 ms integration).
pub const ITIME_DEFAULT: u8 = 0xDA;

/// ADC gain.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Gain {
    /// ×1 (0.83 lx/count at the default integration time).
    #[default]
    X1,
    /// ×2.
    X2,
    /// ×64.
    X64,
    /// ×128 (0.007 lx/count at the default integration time).
    X128,
}

impl Gain {
    const fn bits(self) -> u8 {
        match self {
            Self::X1 => 0b00,
            Self::X2 => 0b01,
            Self::X64 => 0b10,
            Self::X128 => 0b11,
        }
    }

    /// Gain factor.
    pub const fn factor(self) -> u32 {
        match self {
            Self::X1 => 1,
            Self::X2 => 2,
            Self::X64 => 64,
            Self::X128 => 128,
        }
    }
}

/// Maximum measurement time for an `ITIME` value, in microseconds.
///
/// Datasheet TIMING register: `ITIME_ms = Tint × 964 × (256 − ITIME)` and
/// `Tmt = ITIME_ms + Tint × 714`, with `Tint` 4.0 µs max. `ITIME = 0`
/// (manual integration) is treated as 1.
pub const fn wait_us(itime: u8) -> WaitUs {
    let cycles = 256 - if itime == 0 { 1 } else { itime as u32 };
    4 * (964 * cycles + 714)
}

/// Illuminance in milli-lux from DATA0/DATA1 (datasheet section 6, no
/// optical window).
///
/// `102.6 / ITIME_ms` equals `38 / (256 − ITIME)`: the coefficients are
/// normalised to the default integration time of 38 cycles.
pub const fn lux_millilux(data0: u16, data1: u16, gain: Gain, itime: u8) -> u32 {
    let d0 = data0 as u32;
    let d1 = data1 as u32;
    if d0 == 0 {
        return 0;
    }
    // Coefficients ×1000; thresholds compare D1/D0 against r/100.
    let (a, b) = if 100 * d1 < 26 * d0 {
        (1290, 2733)
    } else if 100 * d1 < 55 * d0 {
        (795, 859)
    } else if 100 * d1 < 109 * d0 {
        (510, 345)
    } else if 100 * d1 < 213 * d0 {
        (276, 130)
    } else {
        return 0;
    };
    let pos = a * d0;
    let neg = b * d1;
    if neg >= pos {
        return 0;
    }
    let cycles = 256 - if itime == 0 { 1 } else { itime as u32 };
    // (pos − neg) ≤ 1290 × 65535 ≈ 8.5e7; × 38 fits in u32.
    (pos - neg) * 38 / (gain.factor() * cycles)
}

/// Auto-range helper: suggest the gain for the next sample from DATA0 of
/// the current one (aim for 1/16..3/4 of full scale).
pub const fn suggest_gain(data0: u16, gain: Gain) -> Gain {
    if data0 > 0xC000 {
        match gain {
            Gain::X128 => Gain::X64,
            Gain::X64 => Gain::X2,
            Gain::X2 | Gain::X1 => Gain::X1,
        }
    } else if data0 < 0x0100 {
        match gain {
            Gain::X1 => Gain::X64,
            Gain::X2 => Gain::X64,
            Gain::X64 | Gain::X128 => Gain::X128,
        }
    } else {
        gain
    }
}

/// One sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Illuminance in milli-lux.
    pub illuminance: u32,
    /// DATA0 (visible + IR) raw count.
    pub data0: u16,
    /// DATA1 (IR) raw count.
    pub data1: u16,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Illuminance(self.illuminance));
    }
}

macro_rules! driver {
    ($($async_:ident)?, $($await_:tt)*) => {
        /// BH1730FVC driver.
        pub struct Bh1730<I2C> {
            i2c: I2C,
            gain: Gain,
            itime: u8,
            configured: bool,
        }

        impl<I2C> Bh1730<I2C> {
            /// Create a driver; no bus traffic.
            pub const fn new(i2c: I2C) -> Self {
                Self {
                    i2c,
                    gain: Gain::X1,
                    itime: ITIME_DEFAULT,
                    configured: false,
                }
            }

            /// Select the gain (written before the next conversion).
            pub const fn with_gain(mut self, gain: Gain) -> Self {
                self.gain = gain;
                self.configured = false;
                self
            }

            /// Select the `TIMING` register value (1..=255; integration time
            /// `2.7 ms × (256 − itime)` typ.).
            pub const fn with_itime(mut self, itime: u8) -> Self {
                self.itime = if itime == 0 { 1 } else { itime };
                self.configured = false;
                self
            }

            /// Change the gain at run time (e.g. from [`suggest_gain`]).
            pub fn set_gain(&mut self, gain: Gain) {
                self.gain = gain;
                self.configured = false;
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

        impl<I2C: I2c> Bh1730<I2C> {
            $($async_)? fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
                self.i2c
                    .write(ADDRESS, &[CMD | reg, value])
                    $($await_)*
                    .map_err(Error::bus)
            }

            $($async_)? fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
                self.i2c
                    .write_read(ADDRESS, &[CMD | reg], buf)
                    $($await_)*
                    .map_err(Error::bus)
            }

            $($async_)? fn special(&mut self, command: u8) -> Result<(), Error<I2C::Error>> {
                self.i2c
                    .write(ADDRESS, &[CMD_SPECIAL | command])
                    $($await_)*
                    .map_err(Error::bus)
            }

            /// Check `PART_ID[7:4]` = 0x7. Returns the full ID byte.
            pub $($async_)? fn probe(&mut self) -> Result<u8, Error<I2C::Error>> {
                let mut id = [0u8];
                self.read_regs(REG_ID, &mut id) $($await_)* ?;
                if id[0] >> 4 != PART_NUMBER {
                    return Err(Error::UnexpectedId(u32::from(id[0])));
                }
                Ok(id[0])
            }

            /// Software reset (all registers to default).
            pub $($async_)? fn soft_reset(&mut self) -> Result<(), Error<I2C::Error>> {
                self.configured = false;
                self.special(SPECIAL_SOFT_RESET) $($await_)*
            }

            /// Program the DATA0 interrupt window and enable/disable the INT
            /// pin. `persist`: 0 = every conversion, 1 = on every update,
            /// n ≥ 2 = n consecutive out-of-window results.
            pub $($async_)? fn set_interrupt(
                &mut self,
                enable: bool,
                low: u16,
                high: u16,
                persist: u8,
            ) -> Result<(), Error<I2C::Error>> {
                let [ll, lh] = low.to_le_bytes();
                let [hl, hh] = high.to_le_bytes();
                for (i, byte) in [ll, lh, hl, hh].into_iter().enumerate() {
                    self.write_reg(REG_TH_LOW + i as u8, byte) $($await_)* ?;
                }
                let value = if enable { 1 << 4 } else { 0 } | (persist & 0x0F);
                self.write_reg(REG_INTERRUPT, value) $($await_)*
            }

            /// Release the INT pin ("interrupt output reset").
            pub $($async_)? fn clear_interrupt(&mut self) -> Result<(), Error<I2C::Error>> {
                self.special(SPECIAL_INT_RESET) $($await_)*
            }
        }

        impl<I2C: I2c> Sensor for Bh1730<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            $($async_)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                if !self.configured {
                    self.write_reg(REG_TIMING, self.itime) $($await_)* ?;
                    self.write_reg(REG_GAIN, self.gain.bits()) $($await_)* ?;
                    self.configured = true;
                }
                self.write_reg(REG_CONTROL, CONTROL_START_ONE_SHOT) $($await_)* ?;
                Ok(wait_us(self.itime))
            }

            /// [`Error::NotReady`] until `ADC_VALID` is set.
            $($async_)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let mut control = [0u8];
                self.read_regs(REG_CONTROL, &mut control) $($await_)* ?;
                if control[0] & CONTROL_ADC_VALID == 0 {
                    return Err(Error::NotReady);
                }
                let mut buf = [0u8; 4];
                self.read_regs(REG_DATA0, &mut buf) $($await_)* ?;
                let data0 = u16::from_le_bytes([buf[0], buf[1]]);
                let data1 = u16::from_le_bytes([buf[2], buf[3]]);
                Ok(Measurement {
                    illuminance: lux_millilux(data0, data1, self.gain, self.itime),
                    data0,
                    data1,
                })
            }
        }

        impl<I2C: I2c> LowPower for Bh1730<I2C> {
            type BusError = I2C::Error;

            /// Stop the ADC and power it down.
            $($async_)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write_reg(REG_CONTROL, 0) $($await_)*
            }

            /// Nothing to do: the next one-time conversion powers the ADC.
            $($async_)? fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                Ok(0)
            }
        }
    };
}

driver!(,);

/// Async driver.
pub mod asynch {
    #[allow(unused_imports)]
    use super::suggest_gain;
    use super::{
        ADDRESS, CMD, CMD_SPECIAL, CONTROL_ADC_VALID, CONTROL_START_ONE_SHOT, Gain, ITIME_DEFAULT,
        Measurement, PART_NUMBER, REG_CONTROL, REG_DATA0, REG_GAIN, REG_ID, REG_INTERRUPT,
        REG_TH_LOW, REG_TIMING, SPECIAL_INT_RESET, SPECIAL_SOFT_RESET, lux_millilux, wait_us,
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
    fn lux_formula_regions() {
        // Datasheet electrical characteristics: 1000 lx ≈ 1200 / 180 counts
        // (ratio 0.15 → first region) at gain ×1, ITIME DAh.
        assert_eq!(lux_millilux(1200, 180, Gain::X1, 0xDA), 1_056_060);
        // Ratio 0.4: 0.795 × 1000 − 0.859 × 400 = 451.4 lx.
        assert_eq!(lux_millilux(1000, 400, Gain::X1, 0xDA), 451_400);
        // Ratio 0.8: (510 − 276) = 234 lx, halved by gain ×2.
        assert_eq!(lux_millilux(1000, 800, Gain::X2, 0xDA), 117_000);
        // Ratio 1.5: 276 − 195 = 81 lx; doubled integration time halves it.
        assert_eq!(lux_millilux(1000, 1500, Gain::X1, 180), 40_500);
        // Ratio ≥ 2.13 and dark.
        assert_eq!(lux_millilux(100, 300, Gain::X1, 0xDA), 0);
        assert_eq!(lux_millilux(0, 0, Gain::X128, 0xDA), 0);
    }

    #[test]
    fn measurement_time() {
        // Datasheet: Tmt 150 ms max at TIMING = DAh.
        assert_eq!(wait_us(0xDA), 149_384);
        assert_eq!(wait_us(0xFF), 6_712);
    }

    #[test]
    fn auto_range() {
        assert_eq!(suggest_gain(0xFFFF, Gain::X128), Gain::X64);
        assert_eq!(suggest_gain(10, Gain::X1), Gain::X64);
        assert_eq!(suggest_gain(5_000, Gain::X2), Gain::X2);
    }

    #[test]
    fn one_shot_and_not_ready() {
        let expectations = [
            Transaction::write(ADDRESS, vec![0x81, 0xDA]),
            Transaction::write(ADDRESS, vec![0x87, 0x03]),
            Transaction::write(ADDRESS, vec![0x80, 0x0B]),
            Transaction::write_read(ADDRESS, vec![0x80], vec![0x0B]),
            Transaction::write_read(ADDRESS, vec![0x80], vec![0x13]),
            Transaction::write_read(ADDRESS, vec![0x94], vec![0xE8, 0x03, 0x90, 0x01]),
            Transaction::write(ADDRESS, vec![0x80, 0x0B]),
            Transaction::write(ADDRESS, vec![0x80, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut bh = Bh1730::new(&mut i2c).with_gain(Gain::X128);
        assert_eq!(bh.start_measurement().unwrap(), 149_384);
        assert_eq!(bh.read_measurement(), Err(Error::NotReady));
        let m = bh.read_measurement().unwrap();
        assert_eq!((m.data0, m.data1), (1000, 400));
        assert_eq!(m.illuminance, 451_400 / 128);
        // Configuration is written only once.
        bh.start_measurement().unwrap();
        bh.sleep().unwrap();
        i2c.done();
    }

    #[test]
    fn probe_and_interrupt() {
        let expectations = [
            Transaction::write_read(ADDRESS, vec![0x92], vec![0x71]),
            Transaction::write_read(ADDRESS, vec![0x92], vec![0x50]),
            Transaction::write(ADDRESS, vec![0x83, 0x34]),
            Transaction::write(ADDRESS, vec![0x84, 0x12]),
            Transaction::write(ADDRESS, vec![0x85, 0xCD]),
            Transaction::write(ADDRESS, vec![0x86, 0xAB]),
            Transaction::write(ADDRESS, vec![0x82, 0x12]),
            Transaction::write(ADDRESS, vec![0xE1]),
            Transaction::write(ADDRESS, vec![0xE4]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut bh = Bh1730::new(&mut i2c);
        assert_eq!(bh.probe().unwrap(), 0x71);
        assert_eq!(bh.probe(), Err(Error::UnexpectedId(0x50)));
        bh.set_interrupt(true, 0x1234, 0xABCD, 2).unwrap();
        bh.clear_interrupt().unwrap();
        bh.soft_reset().unwrap();
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
            Transaction::write(ADDRESS, vec![0x81, 0xFF]),
            Transaction::write(ADDRESS, vec![0x87, 0x00]),
            Transaction::write(ADDRESS, vec![0x80, 0x0B]),
            Transaction::write_read(ADDRESS, vec![0x80], vec![0x13]),
            Transaction::write_read(ADDRESS, vec![0x94], vec![0x0A, 0x00, 0x00, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut bh = asynch::Bh1730::new(&mut i2c).with_itime(0xFF);
        let mut delay = RecordingDelay(0);
        let m = block_on(bh.measure(&mut delay)).unwrap();
        // 10 counts × 1.29 × 38 = 490.2 lx.
        assert_eq!(m.illuminance, 490_200);
        assert_eq!(delay.0, 6_712);
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
