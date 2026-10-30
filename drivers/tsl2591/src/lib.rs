//! ams TSL2591 high-dynamic-range light-to-digital converter (I2C).
//!
//! Samples are taken as enable → integrate → read → disable: `PON | AEN` is
//! set in `start_measurement`, `AVALID` is checked and all four data bytes
//! are read in one auto-increment transfer in `read_measurement`, then
//! the oscillator is turned off again (sleep current 2.3 µA typ.).
//!
//! The datasheet has no lux equation. [`calculate_millilux`] uses the ams
//! reference formula used by the Linux `tsl2591` IIO driver and the
//! Adafruit library: `lux = (CH0 − CH1)·(1 − CH1/CH0) / CPL` with
//! `CPL = ATIME_ms · AGAIN / 408`, evaluated in integer milli-lux.
//!
//! Reference: ams "TSL2591 Light-to-Digital Converter" datasheet
//! (ams163.5, April 2013).
//!
//! ```ignore
//! let mut tsl = Tsl2591::new(i2c).with_gain(Gain::Medium);
//! tsl.probe()?;
//! let wait_us = tsl.start_measurement()?;
//! // ... sleep wait_us ...
//! let lux_milli = tsl.read_measurement()?.illuminance;
//! ```

#![no_std]

use embedded_hal::i2c::I2c;
use zigbee_sensor::blocking::{LowPower, Sensor};
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Fixed I2C address.
pub const ADDRESS: u8 = 0x29;
/// Expected `ID` register value.
pub const DEVICE_ID: u8 = 0x50;

/// COMMAND: CMD + TRANSACTION = normal operation.
const CMD_NORMAL: u8 = 0xA0;
/// COMMAND: CMD + TRANSACTION = special function.
const CMD_SPECIAL: u8 = 0xE0;
const SF_FORCE_INTERRUPT: u8 = 0b00100;
const SF_CLEAR_ALL: u8 = 0b00111;

const REG_ENABLE: u8 = 0x00;
const REG_CONTROL: u8 = 0x01;
const REG_AILTL: u8 = 0x04;
const REG_NPAILTL: u8 = 0x08;
const REG_PERSIST: u8 = 0x0C;
const REG_ID: u8 = 0x12;
const REG_STATUS: u8 = 0x13;
const REG_C0DATAL: u8 = 0x14;

const ENABLE_NPIEN: u8 = 1 << 7;
const ENABLE_SAI: u8 = 1 << 6;
const ENABLE_AIEN: u8 = 1 << 4;
const ENABLE_AEN: u8 = 1 << 1;
const ENABLE_PON: u8 = 1 << 0;

const CONTROL_SRESET: u8 = 1 << 7;

const STATUS_NPINTR: u8 = 1 << 5;
const STATUS_AINT: u8 = 1 << 4;
const STATUS_AVALID: u8 = 1 << 0;

/// Max integration step (datasheet: 95 / 101 / 108 ms per 100 ms step).
const STEP_MAX_US: u32 = 108_000;
/// Oscillator start-up margin.
const STARTUP_US: u32 = 2_000;

/// Lux device factor from the ams reference code.
pub const LUX_DF: u64 = 408;

/// ALS gain (CONTROL.AGAIN).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Gain {
    /// ×1.
    #[default]
    Low,
    /// ×25.
    Medium,
    /// ×428.
    High,
    /// ×9876.
    Max,
}

impl Gain {
    /// Typical gain relative to `Low` (datasheet "Gain scaling").
    pub const fn factor(self) -> u32 {
        match self {
            Self::Low => 1,
            Self::Medium => 25,
            Self::High => 428,
            Self::Max => 9876,
        }
    }

    const fn bits(self) -> u8 {
        (self as u8) << 4
    }

    const fn up(self) -> Option<Self> {
        match self {
            Self::Low => Some(Self::Medium),
            Self::Medium => Some(Self::High),
            Self::High => Some(Self::Max),
            Self::Max => None,
        }
    }

    const fn down(self) -> Option<Self> {
        match self {
            Self::Low => None,
            Self::Medium => Some(Self::Low),
            Self::High => Some(Self::Medium),
            Self::Max => Some(Self::High),
        }
    }
}

/// ALS integration time (CONTROL.ATIME).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum IntegrationTime {
    #[default]
    Ms100,
    Ms200,
    Ms300,
    Ms400,
    Ms500,
    Ms600,
}

impl IntegrationTime {
    /// Nominal integration time in milliseconds.
    pub const fn millis(self) -> u32 {
        (self as u32 + 1) * 100
    }

    /// Maximum ADC count (37888 at 100 ms, otherwise 65535).
    pub const fn full_scale(self) -> u16 {
        match self {
            Self::Ms100 => 37888,
            _ => 65535,
        }
    }
}

/// Interrupt persistence filter (PERSIST.APERS), in consecutive
/// out-of-range cycles.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(missing_docs)]
pub enum Persistence {
    /// Every ALS cycle generates an interrupt.
    EveryCycle,
    /// Any single value outside the threshold range.
    Any,
    Cycles2,
    Cycles3,
    Cycles5,
    Cycles10,
    Cycles15,
    Cycles20,
    Cycles25,
    Cycles30,
    Cycles35,
    Cycles40,
    Cycles45,
    Cycles50,
    Cycles55,
    Cycles60,
}

/// Interrupt configuration (only effective with
/// [`Tsl2591::with_continuous`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InterruptConfig {
    /// Persisted window on CH0 (raw counts): `(low, high)`.
    pub window: Option<(u16, u16)>,
    /// Persistence filter for `window`.
    pub persistence: Persistence,
    /// No-persist window on CH0 (raw counts), fires after one cycle.
    pub no_persist_window: Option<(u16, u16)>,
    /// Sleep after interrupt (SAI).
    pub sleep_after_interrupt: bool,
}

/// Lux from raw counts, in milli-lux (ams reference formula, DF = 408).
pub const fn calculate_millilux(ch0: u16, ch1: u16, gain: Gain, time: IntegrationTime) -> u32 {
    if ch0 == 0 || ch1 >= ch0 {
        return 0;
    }
    let (c0, c1) = (ch0 as u64, ch1 as u64);
    let num = (c0 - c1) * (1000 - c1 * 1000 / c0) * LUX_DF;
    let den = time.millis() as u64 * gain.factor() as u64;
    let mlx = num / den;
    if mlx > u32::MAX as u64 {
        u32::MAX
    } else {
        mlx as u32
    }
}

/// Pure auto-gain step: drop the gain when CH0 is above 90 % of full scale,
/// raise it when the next gain would still keep CH0 below 50 % of full scale.
pub const fn suggest_gain(ch0: u16, gain: Gain, time: IntegrationTime) -> Gain {
    let fs = time.full_scale() as u64;
    let c0 = ch0 as u64;
    if c0 * 10 > fs * 9 {
        if let Some(g) = gain.down() {
            return g;
        }
    } else if let Some(g) = gain.up() {
        if c0 * g.factor() as u64 * 2 < fs * gain.factor() as u64 {
            return g;
        }
    }
    gain
}

/// One illuminance sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Illuminance in milli-lux.
    pub illuminance: u32,
    /// CH0 (full spectrum) raw count.
    pub ch0: u16,
    /// CH1 (infrared) raw count.
    pub ch1: u16,
    /// A channel reached full scale; `illuminance` is unreliable.
    pub saturated: bool,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Illuminance(self.illuminance));
    }
}

/// Interrupt status flags.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InterruptStatus {
    /// Persisted ALS interrupt (AINT).
    pub als: bool,
    /// No-persist interrupt (NPINTR).
    pub no_persist: bool,
}

macro_rules! driver {
    ($($async_:ident)?, $($await_:tt)*) => {
        /// TSL2591 driver.
        pub struct Tsl2591<I2C> {
            i2c: I2C,
            gain: Gain,
            time: IntegrationTime,
            continuous: bool,
            int_enable: u8,
        }

        impl<I2C> Tsl2591<I2C> {
            /// Create a driver (low gain, 100 ms); no bus traffic.
            pub const fn new(i2c: I2C) -> Self {
                Self {
                    i2c,
                    gain: Gain::Low,
                    time: IntegrationTime::Ms100,
                    continuous: false,
                    int_enable: 0,
                }
            }

            /// Select the gain.
            pub const fn with_gain(mut self, gain: Gain) -> Self {
                self.gain = gain;
                self
            }

            /// Select the integration time.
            pub const fn with_integration_time(mut self, time: IntegrationTime) -> Self {
                self.time = time;
                self
            }

            /// Leave the ALS running after a read (needed for interrupts).
            pub const fn with_continuous(mut self, continuous: bool) -> Self {
                self.continuous = continuous;
                self
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

        impl<I2C: I2c> Tsl2591<I2C> {
            $($async_)? fn write(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
                self.i2c
                    .write(ADDRESS, &[CMD_NORMAL | reg, value])
                    $($await_)*
                    .map_err(Error::bus)
            }

            $($async_)? fn write_window(&mut self, reg: u8, low: u16, high: u16) -> Result<(), Error<I2C::Error>> {
                let [ll, lh] = low.to_le_bytes();
                let [hl, hh] = high.to_le_bytes();
                self.i2c
                    .write(ADDRESS, &[CMD_NORMAL | reg, ll, lh, hl, hh])
                    $($await_)*
                    .map_err(Error::bus)
            }

            $($async_)? fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
                self.i2c
                    .write_read(ADDRESS, &[CMD_NORMAL | reg], buf)
                    $($await_)*
                    .map_err(Error::bus)
            }

            $($async_)? fn special(&mut self, function: u8) -> Result<(), Error<I2C::Error>> {
                self.i2c
                    .write(ADDRESS, &[CMD_SPECIAL | function])
                    $($await_)*
                    .map_err(Error::bus)
            }

            /// Check the device ID (0x50).
            pub $($async_)? fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
                let mut id = [0u8; 1];
                self.read(REG_ID, &mut id) $($await_)* ?;
                if id[0] != DEVICE_ID {
                    return Err(Error::UnexpectedId(id[0] as u32));
                }
                Ok(())
            }

            /// Software reset (equivalent to power-on reset).
            pub $($async_)? fn reset(&mut self) -> Result<(), Error<I2C::Error>> {
                self.int_enable = 0;
                self.write(REG_CONTROL, CONTROL_SRESET) $($await_)*
            }

            /// Change the gain; it is written on the next `start_measurement`.
            pub fn set_gain(&mut self, gain: Gain) {
                self.gain = gain;
            }

            /// Program thresholds, persistence and interrupt enables.
            pub $($async_)? fn set_interrupt(&mut self, config: InterruptConfig) -> Result<(), Error<I2C::Error>> {
                let mut enable = 0;
                if let Some((low, high)) = config.window {
                    self.write_window(REG_AILTL, low, high) $($await_)* ?;
                    self.write(REG_PERSIST, config.persistence as u8) $($await_)* ?;
                    enable |= ENABLE_AIEN;
                }
                if let Some((low, high)) = config.no_persist_window {
                    self.write_window(REG_NPAILTL, low, high) $($await_)* ?;
                    enable |= ENABLE_NPIEN;
                }
                if config.sleep_after_interrupt {
                    enable |= ENABLE_SAI;
                }
                self.int_enable = enable;
                Ok(())
            }

            /// Read and clear the interrupt flags.
            pub $($async_)? fn take_interrupt(&mut self) -> Result<InterruptStatus, Error<I2C::Error>> {
                let mut status = [0u8; 1];
                self.read(REG_STATUS, &mut status) $($await_)* ?;
                let flags = InterruptStatus {
                    als: status[0] & STATUS_AINT != 0,
                    no_persist: status[0] & STATUS_NPINTR != 0,
                };
                if flags.als || flags.no_persist {
                    self.special(SF_CLEAR_ALL) $($await_)* ?;
                }
                Ok(flags)
            }

            /// Force an interrupt (wiring test); clear it with
            /// [`Self::take_interrupt`].
            pub $($async_)? fn force_interrupt(&mut self) -> Result<(), Error<I2C::Error>> {
                self.special(SF_FORCE_INTERRUPT) $($await_)*
            }
        }

        impl<I2C: I2c> Sensor for Tsl2591<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// Write gain / ATIME and enable the ALS; the first result is
            /// valid after one full integration.
            $($async_)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                let control = self.gain.bits() | self.time as u8;
                self.write(REG_CONTROL, control) $($await_)* ?;
                let enable = self.int_enable | ENABLE_AEN | ENABLE_PON;
                self.write(REG_ENABLE, enable) $($await_)* ?;
                Ok(STARTUP_US + (self.time as u32 + 1) * STEP_MAX_US)
            }

            /// Check `AVALID`, read CH0/CH1 and disable (unless continuous).
            $($async_)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let mut buf = [0u8; 5];
                self.read(REG_STATUS, &mut buf) $($await_)* ?;
                if buf[0] & STATUS_AVALID == 0 {
                    return Err(Error::NotReady);
                }
                if !self.continuous {
                    self.write(REG_ENABLE, 0) $($await_)* ?;
                }
                let ch0 = u16::from_le_bytes([buf[1], buf[2]]);
                let ch1 = u16::from_le_bytes([buf[3], buf[4]]);
                let fs = self.time.full_scale();
                Ok(Measurement {
                    illuminance: calculate_millilux(ch0, ch1, self.gain, self.time),
                    ch0,
                    ch1,
                    saturated: ch0 >= fs || ch1 >= fs,
                })
            }
        }

        impl<I2C: I2c> LowPower for Tsl2591<I2C> {
            type BusError = I2C::Error;

            $($async_)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write(REG_ENABLE, 0) $($await_)*
            }

            /// Samples power the part up themselves; nothing to do.
            $($async_)? fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                Ok(0)
            }
        }
    };
}

driver!(,);

const _: () = assert!(REG_C0DATAL == REG_STATUS + 1);

/// Async driver.
pub mod asynch {
    use super::{
        ADDRESS, CMD_NORMAL, CMD_SPECIAL, CONTROL_SRESET, DEVICE_ID, ENABLE_AEN, ENABLE_AIEN,
        ENABLE_NPIEN, ENABLE_PON, ENABLE_SAI, Gain, IntegrationTime, InterruptConfig,
        InterruptStatus, Measurement, REG_AILTL, REG_CONTROL, REG_ENABLE, REG_ID, REG_NPAILTL,
        REG_PERSIST, REG_STATUS, SF_CLEAR_ALL, SF_FORCE_INTERRUPT, STARTUP_US, STATUS_AINT,
        STATUS_AVALID, STATUS_NPINTR, STEP_MAX_US, calculate_millilux,
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
        // CPL = 100·1/408; (900·0.9)/CPL = 3304.8 lx.
        assert_eq!(
            calculate_millilux(1000, 100, Gain::Low, IntegrationTime::Ms100),
            3_304_800
        );
        // Max gain, 600 ms: 1 count of pure visible light ≈ 0.069 mlx → 0.
        assert_eq!(
            calculate_millilux(1, 0, Gain::Max, IntegrationTime::Ms600),
            0
        );
        assert_eq!(
            calculate_millilux(60_000, 0, Gain::Max, IntegrationTime::Ms600),
            4_131
        );
        assert_eq!(
            calculate_millilux(100, 100, Gain::Low, IntegrationTime::Ms100),
            0
        );
        assert_eq!(
            calculate_millilux(0, 0, Gain::Low, IntegrationTime::Ms100),
            0
        );
    }

    #[test]
    fn auto_gain() {
        let t = IntegrationTime::Ms100;
        assert_eq!(suggest_gain(36_000, Gain::Medium, t), Gain::Low);
        assert_eq!(suggest_gain(36_000, Gain::Low, t), Gain::Low);
        assert_eq!(suggest_gain(500, Gain::Low, t), Gain::Medium);
        assert_eq!(suggest_gain(1_000, Gain::Low, t), Gain::Low);
        assert_eq!(suggest_gain(10, Gain::Max, t), Gain::Max);
    }

    #[test]
    fn one_shot_cycle() {
        let expectations = [
            Transaction::write_read(ADDRESS, vec![0xB2], vec![0x50]),
            Transaction::write(ADDRESS, vec![0xA1, 0x12]),
            Transaction::write(ADDRESS, vec![0xA0, 0x03]),
            Transaction::write_read(ADDRESS, vec![0xB3], vec![0x00, 0, 0, 0, 0]),
            Transaction::write_read(ADDRESS, vec![0xB3], vec![0x01, 0xE8, 0x03, 0x64, 0x00]),
            Transaction::write(ADDRESS, vec![0xA0, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut tsl = Tsl2591::new(&mut i2c)
            .with_gain(Gain::Medium)
            .with_integration_time(IntegrationTime::Ms300);
        tsl.probe().unwrap();
        assert_eq!(tsl.start_measurement().unwrap(), 326_000);
        assert_eq!(tsl.read_measurement(), Err(Error::NotReady));
        let m = tsl.read_measurement().unwrap();
        assert_eq!((m.ch0, m.ch1, m.saturated), (1000, 100, false));
        // 3304.8 lx / (3 · 25).
        assert_eq!(m.illuminance, 44_064);
        i2c.done();
    }

    #[test]
    fn interrupt_continuous() {
        let expectations = [
            Transaction::write_read(ADDRESS, vec![0xB2], vec![0x51]),
            Transaction::write(ADDRESS, vec![0xA4, 0x10, 0x00, 0x00, 0x10]),
            Transaction::write(ADDRESS, vec![0xAC, 0x04]),
            Transaction::write(ADDRESS, vec![0xA8, 0x00, 0x00, 0xFF, 0xFF]),
            Transaction::write(ADDRESS, vec![0xA1, 0x00]),
            Transaction::write(ADDRESS, vec![0xA0, 0x93]),
            Transaction::write_read(ADDRESS, vec![0xB3], vec![0x01, 0x00, 0x94, 0, 0]),
            Transaction::write_read(ADDRESS, vec![0xB3], vec![0x31]),
            Transaction::write(ADDRESS, vec![0xE7]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut tsl = Tsl2591::new(&mut i2c).with_continuous(true);
        assert_eq!(tsl.probe(), Err(Error::UnexpectedId(0x51)));
        tsl.set_interrupt(InterruptConfig {
            window: Some((0x10, 0x1000)),
            persistence: Persistence::Cycles5,
            no_persist_window: Some((0, 0xFFFF)),
            sleep_after_interrupt: false,
        })
        .unwrap();
        tsl.start_measurement().unwrap();
        assert!(tsl.read_measurement().unwrap().saturated);
        let s = tsl.take_interrupt().unwrap();
        assert!(s.als && s.no_persist);
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
            Transaction::write(ADDRESS, vec![0xA1, 0x00]),
            Transaction::write(ADDRESS, vec![0xA0, 0x03]),
            Transaction::write_read(ADDRESS, vec![0xB3], vec![0x01, 0xE8, 0x03, 0x64, 0x00]),
            Transaction::write(ADDRESS, vec![0xA0, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut tsl = asynch::Tsl2591::new(&mut i2c);
        let mut delay = RecordingDelay(0);
        let m = block_on(tsl.measure(&mut delay)).unwrap();
        assert_eq!(m.illuminance, 3_304_800);
        assert_eq!(delay.0, 110_000);
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
