//! Vishay VCNL4040 ambient light + proximity sensor (I2C).
//!
//! `start_measurement` powers up the ALS and/or the proximity sensor (PS).
//! In one-shot mode (the default) the PS runs in active force mode and one
//! PS cycle is triggered. `read_measurement` reads the data words and then
//! shuts both blocks down (0.2 µA typ.). The part has no data-ready flag, so
//! the returned wait time must be respected.
//!
//! Illuminance: 0.1 lx/count at 80 ms, halved for each doubling of the
//! integration time (datasheet Table 14).
//!
//! Reference: Vishay 84274 "VCNL4040 Fully Integrated Proximity and Ambient
//! Light Sensor" datasheet.
//!
//! ```ignore
//! let mut s = Vcnl4040::new(i2c);
//! s.probe()?;
//! let m = s.measure(&mut delay)?;
//! ```

#![no_std]

use embedded_hal::i2c::I2c;
use zigbee_sensor::blocking::{LowPower, Sensor};
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Fixed I2C address.
pub const ADDRESS: u8 = 0x60;
/// Low byte of the `ID` word.
pub const DEVICE_ID: u8 = 0x86;

const REG_ALS_CONF: u8 = 0x00;
const REG_ALS_THDH: u8 = 0x01;
const REG_ALS_THDL: u8 = 0x02;
const REG_PS_CONF12: u8 = 0x03;
const REG_PS_CONF3_MS: u8 = 0x04;
const REG_PS_CANC: u8 = 0x05;
const REG_PS_THDL: u8 = 0x06;
const REG_PS_THDH: u8 = 0x07;
const REG_PS_DATA: u8 = 0x08;
const REG_ALS_DATA: u8 = 0x09;
const REG_WHITE_DATA: u8 = 0x0A;
const REG_INT_FLAG: u8 = 0x0B;
const REG_ID: u8 = 0x0C;

const SD: u8 = 1 << 0;
const ALS_INT_EN: u8 = 1 << 1;
const PS_HD: u8 = 1 << 3;
const PS_INT_BOTH: u8 = 0b11;
const PS_AF: u8 = 1 << 3;
const PS_TRIG: u8 = 1 << 2;

/// `take_interrupt` flag: PS dropped below the low threshold.
pub const INT_PS_AWAY: u8 = 1 << 0;
/// `take_interrupt` flag: PS rose above the high threshold.
pub const INT_PS_CLOSE: u8 = 1 << 1;
/// `take_interrupt` flag: ALS crossed the high threshold.
pub const INT_ALS_HIGH: u8 = 1 << 4;
/// `take_interrupt` flag: ALS crossed the low threshold.
pub const INT_ALS_LOW: u8 = 1 << 5;
/// `take_interrupt` flag: PS entered sunlight protection mode.
pub const INT_PS_PROTECTION: u8 = 1 << 6;

/// Settling margin added to proximity waits. The datasheet gives no PS
/// conversion time, so this is a conservative estimate.
pub const PS_SETTLE_US: u32 = 5_000;

/// ALS integration time (`ALS_IT`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum AlsIt {
    #[default]
    Ms80,
    Ms160,
    Ms320,
    Ms640,
}

impl AlsIt {
    const ALL: [AlsIt; 4] = [Self::Ms80, Self::Ms160, Self::Ms320, Self::Ms640];

    /// Integration time in microseconds.
    pub const fn us(self) -> u32 {
        80_000 << self as u32
    }
}

/// PS integration time (`PS_IT`), in units of T ≈ 100 µs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum PsIt {
    #[default]
    T1,
    T1_5,
    T2,
    T2_5,
    T3,
    T3_5,
    T4,
    T8,
}

impl PsIt {
    /// Approximate integration time in microseconds (T = 100 µs, as in the
    /// Linux driver).
    pub const fn us(self) -> u32 {
        match self {
            Self::T8 => 800,
            t => 100 + 50 * t as u32,
        }
    }
}

/// IRED on/off duty ratio (`PS_Duty`), used in continuous mode only.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum Duty {
    #[default]
    D40,
    D80,
    D160,
    D320,
}

impl Duty {
    /// Divider (40, 80, 160, 320).
    pub const fn ratio(self) -> u32 {
        40 << self as u32
    }
}

/// Multi-pulse count (`PS_MPS`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum Pulses {
    #[default]
    P1,
    P2,
    P4,
    P8,
}

/// IRED current (`LED_I`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum LedCurrent {
    #[default]
    Ma50,
    Ma75,
    Ma100,
    Ma120,
    Ma140,
    Ma160,
    Ma180,
    Ma200,
}

/// Proximity settings.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Proximity {
    /// Integration time.
    pub it: PsIt,
    /// Pulses per measurement.
    pub pulses: Pulses,
    /// IRED current.
    pub led: LedCurrent,
    /// Duty ratio (continuous mode).
    pub duty: Duty,
    /// 16-bit output (`PS_HD`); 12-bit when `false`.
    pub high_resolution: bool,
}

impl Default for Proximity {
    /// 1T, one pulse, 50 mA, 1/40, 16-bit.
    fn default() -> Self {
        Self {
            it: PsIt::T1,
            pulses: Pulses::P1,
            led: LedCurrent::Ma50,
            duty: Duty::D40,
            high_resolution: true,
        }
    }
}

impl Proximity {
    /// Wait time for one PS result: integration time × pulses (× duty
    /// ratio in continuous mode) plus [`PS_SETTLE_US`].
    pub const fn wait_us(&self, continuous: bool) -> u32 {
        let t = self.it.us() << self.pulses as u32;
        if continuous {
            t + self.it.us() * self.duty.ratio() + PS_SETTLE_US
        } else {
            t + PS_SETTLE_US
        }
    }
}

/// Interrupt window in raw counts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Window {
    /// Lower threshold.
    pub low: u16,
    /// Upper threshold.
    pub high: u16,
    /// Raw persistence code 0..=3 (ALS: 1/2/4/8 samples, PS: 1/2/3/4).
    pub persistence: u8,
}

/// ALS counts to milli-lux: `counts · 100 >> ALS_IT`.
pub const fn to_millilux(counts: u16, it: AlsIt) -> u32 {
    (counts as u32 * 100) >> it as u32
}

/// Pick the ALS integration time for the next sample: step down above
/// 90 % of full scale, step up while the reading would stay below 50 %.
pub fn suggest_integration(current: AlsIt, counts: u16) -> AlsIt {
    let idx = current as usize;
    let counts = counts as u32;
    if counts * 10 > 65_535 * 9 {
        return AlsIt::ALL[idx.saturating_sub(1)];
    }
    match AlsIt::ALL.get(idx + 1) {
        Some(&next) if counts * 4 < 65_535 => next,
        _ => current,
    }
}

/// One sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Illuminance in milli-lux, if the ALS is enabled.
    pub illuminance: Option<u32>,
    /// Raw ALS count.
    pub als: u16,
    /// Raw white-channel count.
    pub white: u16,
    /// Raw proximity count, if the PS is enabled.
    pub proximity: Option<u16>,
    /// ALS count at full scale.
    pub saturated: bool,
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

macro_rules! driver {
    ($($async_:ident)?, $($await_:tt)*) => {
        /// VCNL4040 driver.
        pub struct Vcnl4040<I2C> {
            i2c: I2C,
            als: Option<AlsIt>,
            proximity: Option<Proximity>,
            als_pers: u8,
            ps_pers: u8,
            als_int: bool,
            ps_int: bool,
            continuous: bool,
        }

        impl<I2C> Vcnl4040<I2C> {
            /// Create a driver (ALS 80 ms, PS [`Proximity::default`]); no
            /// bus traffic.
            pub const fn new(i2c: I2C) -> Self {
                Self {
                    i2c,
                    als: Some(AlsIt::Ms80),
                    proximity: Some(Proximity {
                        it: PsIt::T1,
                        pulses: Pulses::P1,
                        led: LedCurrent::Ma50,
                        duty: Duty::D40,
                        high_resolution: true,
                    }),
                    als_pers: 0,
                    ps_pers: 0,
                    als_int: false,
                    ps_int: false,
                    continuous: false,
                }
            }

            /// ALS integration time (`None` disables the ALS).
            pub const fn with_als(mut self, it: Option<AlsIt>) -> Self {
                self.als = it;
                self
            }

            /// Proximity settings (`None` disables the PS).
            pub const fn with_proximity(mut self, proximity: Option<Proximity>) -> Self {
                self.proximity = proximity;
                self
            }

            /// Keep both blocks running after a read and use the PS duty
            /// cycle instead of active force mode (needed for interrupts).
            pub const fn with_continuous(mut self, continuous: bool) -> Self {
                self.continuous = continuous;
                self
            }

            /// Change the ALS integration time for the next
            /// `start_measurement`.
            pub fn set_integration(&mut self, it: AlsIt) {
                self.als = Some(it);
            }

            /// Current ALS integration time.
            pub const fn integration(&self) -> Option<AlsIt> {
                self.als
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.i2c
            }

            const fn als_conf(&self, shutdown: bool) -> u8 {
                let it = match self.als {
                    Some(it) => it as u8,
                    None => 0,
                };
                let mut v = it << 6 | (self.als_pers & 3) << 2;
                if self.als_int {
                    v |= ALS_INT_EN;
                }
                if shutdown || self.als.is_none() {
                    v |= SD;
                }
                v
            }

            const fn ps_conf(&self, shutdown: bool) -> [u8; 2] {
                let Some(p) = self.proximity else {
                    return [SD, 0];
                };
                let mut conf1 = (p.duty as u8) << 6 | (self.ps_pers & 3) << 4 | (p.it as u8) << 1;
                if shutdown {
                    conf1 |= SD;
                }
                let mut conf2 = 0;
                if p.high_resolution {
                    conf2 |= PS_HD;
                }
                if self.ps_int {
                    conf2 |= PS_INT_BOTH;
                }
                [conf1, conf2]
            }
        }

        impl<I2C: I2c> Vcnl4040<I2C> {
            $($async_)? fn write(&mut self, reg: u8, value: [u8; 2]) -> Result<(), Error<I2C::Error>> {
                self.i2c
                    .write(ADDRESS, &[reg, value[0], value[1]])
                    $($await_)*
                    .map_err(Error::bus)
            }

            $($async_)? fn read(&mut self, reg: u8) -> Result<u16, Error<I2C::Error>> {
                let mut buf = [0u8; 2];
                self.i2c
                    .write_read(ADDRESS, &[reg], &mut buf)
                    $($await_)*
                    .map_err(Error::bus)?;
                Ok(u16::from_le_bytes(buf))
            }

            /// Check the low byte of the `ID` word against [`DEVICE_ID`];
            /// returns the whole word (high byte = version).
            pub $($async_)? fn probe(&mut self) -> Result<u16, Error<I2C::Error>> {
                let id = self.read(REG_ID) $($await_)* ?;
                if id as u8 == DEVICE_ID {
                    Ok(id)
                } else {
                    Err(Error::UnexpectedId(id as u32))
                }
            }

            /// Proximity crosstalk cancellation, subtracted from each result.
            pub $($async_)? fn set_proximity_cancellation(
                &mut self,
                level: u16,
            ) -> Result<(), Error<I2C::Error>> {
                self.write(REG_PS_CANC, level.to_le_bytes()) $($await_)*
            }

            /// Configure the ALS and PS interrupt windows (PS triggers on
            /// both close and away). Takes effect at the next
            /// `start_measurement`; only useful in continuous mode.
            pub $($async_)? fn set_interrupt(
                &mut self,
                als: Option<Window>,
                ps: Option<Window>,
            ) -> Result<(), Error<I2C::Error>> {
                self.als_int = false;
                self.ps_int = false;
                if let Some(w) = als {
                    self.write(REG_ALS_THDH, w.high.to_le_bytes()) $($await_)* ?;
                    self.write(REG_ALS_THDL, w.low.to_le_bytes()) $($await_)* ?;
                    self.als_pers = w.persistence & 3;
                    self.als_int = true;
                }
                if let Some(w) = ps {
                    self.write(REG_PS_THDL, w.low.to_le_bytes()) $($await_)* ?;
                    self.write(REG_PS_THDH, w.high.to_le_bytes()) $($await_)* ?;
                    self.ps_pers = w.persistence & 3;
                    self.ps_int = true;
                }
                Ok(())
            }

            /// Read and clear `INT_Flag`; returns the `INT_*` bits.
            pub $($async_)? fn take_interrupt(&mut self) -> Result<u8, Error<I2C::Error>> {
                let v = self.read(REG_INT_FLAG) $($await_)* ?;
                Ok((v >> 8) as u8)
            }
        }

        impl<I2C: I2c> Sensor for Vcnl4040<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// Power up the enabled blocks; in one-shot mode trigger one
            /// PS cycle in active force mode.
            $($async_)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                if self.als.is_none() && self.proximity.is_none() {
                    return Err(Error::Unsupported);
                }
                let mut t = 0;
                if let Some(it) = self.als {
                    self.write(REG_ALS_CONF, [self.als_conf(false), 0]) $($await_)* ?;
                    t = it.us() + it.us() / 10;
                }
                if let Some(p) = self.proximity {
                    let mut conf3 = (p.pulses as u8) << 5;
                    if !self.continuous {
                        conf3 |= PS_AF;
                    }
                    let ms = p.led as u8;
                    self.write(REG_PS_CONF3_MS, [conf3, ms]) $($await_)* ?;
                    let conf = self.ps_conf(false);
                    self.write(REG_PS_CONF12, conf) $($await_)* ?;
                    if !self.continuous {
                        self.write(REG_PS_CONF3_MS, [conf3 | PS_TRIG, ms]) $($await_)* ?;
                    }
                    t = t.max(p.wait_us(self.continuous));
                }
                Ok(t)
            }

            /// Read PS, ALS and white data and shut down (unless
            /// continuous).
            $($async_)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let mut m = Measurement {
                    illuminance: None,
                    als: 0,
                    white: 0,
                    proximity: None,
                    saturated: false,
                };
                if self.proximity.is_some() {
                    m.proximity = Some(self.read(REG_PS_DATA) $($await_)* ?);
                }
                if let Some(it) = self.als {
                    m.als = self.read(REG_ALS_DATA) $($await_)* ?;
                    m.white = self.read(REG_WHITE_DATA) $($await_)* ?;
                    m.illuminance = Some(to_millilux(m.als, it));
                    m.saturated = m.als == u16::MAX;
                }
                if !self.continuous {
                    self.sleep() $($await_)* ?;
                }
                Ok(m)
            }
        }

        impl<I2C: I2c> LowPower for Vcnl4040<I2C> {
            type BusError = I2C::Error;

            /// Shut down the ALS and the PS (configuration is kept).
            $($async_)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write(REG_ALS_CONF, [self.als_conf(true), 0]) $($await_)* ?;
                let conf = self.ps_conf(true);
                self.write(REG_PS_CONF12, conf) $($await_)*
            }

            /// Samples power the blocks themselves; nothing to do.
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
        ADDRESS, ALS_INT_EN, AlsIt, DEVICE_ID, Duty, LedCurrent, Measurement, PS_AF, PS_HD,
        PS_INT_BOTH, PS_TRIG, Proximity, PsIt, Pulses, REG_ALS_CONF, REG_ALS_DATA, REG_ALS_THDH,
        REG_ALS_THDL, REG_ID, REG_INT_FLAG, REG_PS_CANC, REG_PS_CONF3_MS, REG_PS_CONF12,
        REG_PS_DATA, REG_PS_THDH, REG_PS_THDL, REG_WHITE_DATA, SD, Window, to_millilux,
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
        assert_eq!(to_millilux(1000, AlsIt::Ms80), 100_000);
        assert_eq!(to_millilux(1, AlsIt::Ms640), 12);
        assert_eq!(to_millilux(u16::MAX, AlsIt::Ms80), 6_553_500);
        assert_eq!(PsIt::T2_5.us(), 250);
        assert_eq!(PsIt::T8.us(), 800);
        let p = Proximity {
            it: PsIt::T8,
            pulses: Pulses::P8,
            ..Proximity::default()
        };
        assert_eq!(p.wait_us(false), 11_400);
        assert_eq!(Proximity::default().wait_us(true), 9_100);
        assert_eq!(suggest_integration(AlsIt::Ms160, 60_000), AlsIt::Ms80);
        assert_eq!(suggest_integration(AlsIt::Ms160, 10_000), AlsIt::Ms320);
        assert_eq!(suggest_integration(AlsIt::Ms160, 20_000), AlsIt::Ms160);
        assert_eq!(suggest_integration(AlsIt::Ms640, 10), AlsIt::Ms640);
    }

    #[test]
    fn one_shot_als_and_proximity() {
        let expectations = [
            Transaction::write_read(ADDRESS, vec![0x0C], vec![0x86, 0x01]),
            Transaction::write(ADDRESS, vec![0x00, 0x00, 0x00]),
            Transaction::write(ADDRESS, vec![0x04, 0x08, 0x00]),
            Transaction::write(ADDRESS, vec![0x03, 0x00, 0x08]),
            Transaction::write(ADDRESS, vec![0x04, 0x0C, 0x00]),
            Transaction::write_read(ADDRESS, vec![0x08], vec![0x34, 0x12]),
            Transaction::write_read(ADDRESS, vec![0x09], vec![0xE8, 0x03]),
            Transaction::write_read(ADDRESS, vec![0x0A], vec![0xD0, 0x07]),
            Transaction::write(ADDRESS, vec![0x00, 0x01, 0x00]),
            Transaction::write(ADDRESS, vec![0x03, 0x01, 0x08]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut s = Vcnl4040::new(&mut i2c);
        assert_eq!(s.probe().unwrap(), 0x0186);
        assert_eq!(s.start_measurement().unwrap(), 88_000);
        let m = s.read_measurement().unwrap();
        assert_eq!(m.proximity, Some(0x1234));
        assert_eq!(m.illuminance, Some(100_000));
        assert_eq!((m.als, m.white), (1000, 2000));
        assert!(!m.saturated);
        i2c.done();
    }

    #[test]
    fn interrupts_continuous() {
        let expectations = [
            Transaction::write_read(ADDRESS, vec![0x0C], vec![0x87, 0x01]),
            Transaction::write(ADDRESS, vec![0x01, 0x88, 0x13]),
            Transaction::write(ADDRESS, vec![0x02, 100, 0]),
            Transaction::write(ADDRESS, vec![0x06, 10, 0]),
            Transaction::write(ADDRESS, vec![0x07, 0x2C, 0x01]),
            Transaction::write(ADDRESS, vec![0x05, 0x20, 0x00]),
            Transaction::write(ADDRESS, vec![0x00, 0xCA, 0x00]),
            Transaction::write(ADDRESS, vec![0x04, 0x60, 0x07]),
            Transaction::write(ADDRESS, vec![0x03, 0xFE, 0x03]),
            Transaction::write_read(ADDRESS, vec![0x0B], vec![0x00, 0x12]),
            Transaction::write_read(ADDRESS, vec![0x08], vec![0xFF, 0x0F]),
            Transaction::write_read(ADDRESS, vec![0x09], vec![0x10, 0x00]),
            Transaction::write_read(ADDRESS, vec![0x0A], vec![0x20, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let prox = Proximity {
            it: PsIt::T8,
            pulses: Pulses::P8,
            led: LedCurrent::Ma200,
            duty: Duty::D320,
            high_resolution: false,
        };
        let mut s = Vcnl4040::new(&mut i2c)
            .with_als(Some(AlsIt::Ms640))
            .with_proximity(Some(prox))
            .with_continuous(true);
        assert_eq!(s.probe(), Err(Error::UnexpectedId(0x0187)));
        let als = Window {
            low: 100,
            high: 5000,
            persistence: 2,
        };
        let ps = Window {
            low: 10,
            high: 300,
            persistence: 3,
        };
        s.set_interrupt(Some(als), Some(ps)).unwrap();
        s.set_proximity_cancellation(32).unwrap();
        assert_eq!(s.start_measurement().unwrap(), 704_000);
        assert_eq!(s.take_interrupt().unwrap(), INT_PS_CLOSE | INT_ALS_HIGH);
        let m = s.read_measurement().unwrap();
        assert_eq!(m.proximity, Some(4095));
        assert_eq!(m.illuminance, Some(200));
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
            Transaction::write(ADDRESS, vec![0x00, 0x40, 0x00]),
            Transaction::write_read(ADDRESS, vec![0x09], vec![0xFF, 0xFF]),
            Transaction::write_read(ADDRESS, vec![0x0A], vec![0x00, 0x80]),
            Transaction::write(ADDRESS, vec![0x00, 0x41, 0x00]),
            Transaction::write(ADDRESS, vec![0x03, 0x01, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut s = asynch::Vcnl4040::new(&mut i2c)
            .with_proximity(None)
            .with_als(Some(AlsIt::Ms160));
        let mut delay = RecordingDelay(0);
        let m = block_on(s.measure(&mut delay)).unwrap();
        assert!(m.saturated);
        assert_eq!(m.illuminance, Some(3_276_750));
        assert_eq!(m.white, 0x8000);
        assert_eq!(m.proximity, None);
        assert_eq!(delay.0, 176_000);
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
