//! Renesas (Intersil) ISL29035 ambient light sensor (I2C).
//!
//! 4/8/12/16-bit integrating ADC with four full-scale ranges
//! (1k/4k/16k/64k lux): `lux = Range / 2^n · DATA` (datasheet Eq. 3).
//! `start_measurement` writes the range/resolution and starts continuous
//! ALS sensing, `read_measurement` reads `DATA` and powers the part down
//! (0.24 µA typ.). The "ALS once" mode is not used because the datasheet
//! advises against it (Table 7, note 11). There is no data-ready flag.
//!
//! Reference: Renesas FN8371 "ISL29035 Integrated Digital Light Sensor
//! with Interrupt" datasheet.
//!
//! ```ignore
//! let mut als = Isl29035::new(i2c);
//! als.probe()?; // also clears the brown-out flag
//! let m = als.measure(&mut delay)?;
//! ```

#![no_std]

use embedded_hal::i2c::I2c;
use zigbee_sensor::blocking::{LowPower, Sensor};
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Fixed I2C address.
pub const ADDRESS: u8 = 0x44;
/// Device ID in `ID` bits 5:3.
pub const DEVICE_ID: u8 = 0b101;

const REG_COMMAND_I: u8 = 0x00;
const REG_COMMAND_II: u8 = 0x01;
const REG_DATA: u8 = 0x02;
const REG_INT_LT: u8 = 0x04;
const REG_ID: u8 = 0x0F;

const OP_ALS_CONTINUOUS: u8 = 0b101 << 5;
const CMD_INT: u8 = 1 << 2;
const ID_BOUT: u8 = 1 << 7;
const ID_MASK: u8 = 0b111 << 3;

/// Full-scale range.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Range {
    /// 1000 lux.
    #[default]
    Lux1k,
    /// 4000 lux.
    Lux4k,
    /// 16000 lux.
    Lux16k,
    /// 64000 lux.
    Lux64k,
}

impl Range {
    const ALL: [Range; 4] = [Self::Lux1k, Self::Lux4k, Self::Lux16k, Self::Lux64k];

    /// Full scale in lux.
    pub const fn lux(self) -> u32 {
        1000 << (2 * self as u32)
    }
}

/// ADC resolution (sets the integration time).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Resolution {
    /// 16 bit, 105 ms.
    #[default]
    Bits16,
    /// 12 bit, 6.5 ms.
    Bits12,
    /// 8 bit, 0.41 ms.
    Bits8,
    /// 4 bit, 0.0256 ms.
    Bits4,
}

impl Resolution {
    /// Number of bits.
    pub const fn bits(self) -> u32 {
        16 - 4 * self as u32
    }

    /// Integration time in microseconds (rounded up).
    pub const fn integration_us(self) -> u32 {
        match self {
            Self::Bits16 => 105_000,
            Self::Bits12 => 6_500,
            Self::Bits8 => 410,
            Self::Bits4 => 26,
        }
    }
}

/// Interrupt persistence.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub enum Persistence {
    #[default]
    Cycles1,
    Cycles4,
    Cycles8,
    Cycles16,
}

/// `DATA` → milli-lux.
pub const fn to_millilux(data: u16, range: Range, resolution: Resolution) -> u32 {
    ((data as u64 * range.lux() as u64 * 1000) >> resolution.bits()) as u32
}

/// Pick a range for the next sample: next wider range above 90 % of full
/// scale, next narrower range while the reading would stay below 50 %.
pub fn suggest_range(current: Range, resolution: Resolution, data: u16) -> Range {
    let fs = 1u32 << resolution.bits();
    let idx = current as usize;
    if data as u32 * 10 > fs * 9 {
        return Range::ALL[(idx + 1).min(3)];
    }
    if idx > 0 && data as u32 * 4 * 2 < fs {
        return Range::ALL[idx - 1];
    }
    current
}

/// One sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Illuminance in milli-lux.
    pub illuminance: u32,
    /// Raw ADC count.
    pub raw: u16,
    /// ADC at full scale.
    pub saturated: bool,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Illuminance(self.illuminance));
    }
}

macro_rules! driver {
    ($($async_:ident)?, $($await_:tt)*) => {
        /// ISL29035 driver.
        pub struct Isl29035<I2C> {
            i2c: I2C,
            range: Range,
            resolution: Resolution,
            persistence: Persistence,
            continuous: bool,
        }

        impl<I2C> Isl29035<I2C> {
            /// Create a driver (1k lux, 16 bit); no bus traffic.
            pub const fn new(i2c: I2C) -> Self {
                Self {
                    i2c,
                    range: Range::Lux1k,
                    resolution: Resolution::Bits16,
                    persistence: Persistence::Cycles1,
                    continuous: false,
                }
            }

            /// Select the full-scale range.
            pub const fn with_range(mut self, range: Range) -> Self {
                self.range = range;
                self
            }

            /// Select the ADC resolution.
            pub const fn with_resolution(mut self, resolution: Resolution) -> Self {
                self.resolution = resolution;
                self
            }

            /// Keep measuring after a read (needed for the interrupt).
            pub const fn with_continuous(mut self, continuous: bool) -> Self {
                self.continuous = continuous;
                self
            }

            /// Change the range for the next `start_measurement`.
            pub fn set_range(&mut self, range: Range) {
                self.range = range;
            }

            /// Current range.
            pub const fn range(&self) -> Range {
                self.range
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.i2c
            }
        }

        impl<I2C: I2c> Isl29035<I2C> {
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

            /// Check the device ID (`ID` bits 5:3 = 0b101) and clear the
            /// brown-out flag if set.
            pub $($async_)? fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
                let mut id = [0u8; 1];
                self.read(REG_ID, &mut id) $($await_)* ?;
                if (id[0] & ID_MASK) >> 3 != DEVICE_ID {
                    return Err(Error::UnexpectedId(id[0] as u32));
                }
                if id[0] & ID_BOUT != 0 {
                    self.write(REG_ID, id[0] & !ID_BOUT) $($await_)* ?;
                }
                Ok(())
            }

            /// Set the interrupt window (raw counts) and persistence. The
            /// INT pin cannot be disabled; `None` restores the reset window
            /// 0x0000..0xFFFF.
            pub $($async_)? fn set_interrupt(
                &mut self,
                window: Option<(u16, u16)>,
                persistence: Persistence,
            ) -> Result<(), Error<I2C::Error>> {
                let (low, high) = window.unwrap_or((0, u16::MAX));
                self.persistence = persistence;
                let [l0, l1] = low.to_le_bytes();
                let [h0, h1] = high.to_le_bytes();
                self.i2c
                    .write(ADDRESS, &[REG_INT_LT, l0, l1, h0, h1])
                    $($await_)*
                    .map_err(Error::bus)
            }

            /// Read `COMMAND-I` (clears the interrupt); returns the flag.
            pub $($async_)? fn take_interrupt(&mut self) -> Result<bool, Error<I2C::Error>> {
                let mut cmd = [0u8; 1];
                self.read(REG_COMMAND_I, &mut cmd) $($await_)* ?;
                Ok(cmd[0] & CMD_INT != 0)
            }
        }

        impl<I2C: I2c> Sensor for Isl29035<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// Write range/resolution and start continuous ALS sensing.
            $($async_)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                let cmd2 = (self.resolution as u8) << 2 | self.range as u8;
                self.write(REG_COMMAND_II, cmd2) $($await_)* ?;
                self.write(REG_COMMAND_I, OP_ALS_CONTINUOUS | self.persistence as u8) $($await_)* ?;
                let t = self.resolution.integration_us();
                Ok(t + t / 10)
            }

            /// Read `DATA` and power down (unless continuous).
            $($async_)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let mut buf = [0u8; 2];
                self.read(REG_DATA, &mut buf) $($await_)* ?;
                if !self.continuous {
                    self.write(REG_COMMAND_I, self.persistence as u8) $($await_)* ?;
                }
                let raw = u16::from_le_bytes(buf);
                let fs = (1u32 << self.resolution.bits()) - 1;
                Ok(Measurement {
                    illuminance: to_millilux(raw, self.range, self.resolution),
                    raw,
                    saturated: raw as u32 >= fs,
                })
            }
        }

        impl<I2C: I2c> LowPower for Isl29035<I2C> {
            type BusError = I2C::Error;

            $($async_)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write(REG_COMMAND_I, self.persistence as u8) $($await_)*
            }

            /// Samples start the part themselves; nothing to do.
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
        ADDRESS, CMD_INT, DEVICE_ID, ID_BOUT, ID_MASK, Measurement, OP_ALS_CONTINUOUS, Persistence,
        REG_COMMAND_I, REG_COMMAND_II, REG_DATA, REG_ID, REG_INT_LT, Range, Resolution,
        to_millilux,
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
        assert_eq!(Range::Lux64k.lux(), 64_000);
        assert_eq!(
            to_millilux(20_000, Range::Lux1k, Resolution::Bits16),
            305_175
        );
        assert_eq!(to_millilux(1, Range::Lux1k, Resolution::Bits16), 15);
        let r = Resolution::Bits16;
        assert_eq!(suggest_range(Range::Lux1k, r, 62_000), Range::Lux4k);
        assert_eq!(suggest_range(Range::Lux64k, r, 65_535), Range::Lux64k);
        assert_eq!(suggest_range(Range::Lux4k, r, 1_000), Range::Lux1k);
        assert_eq!(suggest_range(Range::Lux4k, r, 10_000), Range::Lux4k);
    }

    #[test]
    fn one_shot_cycle() {
        let expectations = [
            Transaction::write_read(ADDRESS, vec![0x0F], vec![0xA8]),
            Transaction::write(ADDRESS, vec![0x0F, 0x28]),
            Transaction::write(ADDRESS, vec![0x01, 0x00]),
            Transaction::write(ADDRESS, vec![0x00, 0xA0]),
            Transaction::write_read(ADDRESS, vec![0x02], vec![0x20, 0x4E]),
            Transaction::write(ADDRESS, vec![0x00, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut als = Isl29035::new(&mut i2c);
        als.probe().unwrap();
        assert_eq!(als.start_measurement().unwrap(), 115_500);
        let m = als.read_measurement().unwrap();
        assert_eq!(m.raw, 20_000);
        assert_eq!(m.illuminance, 305_175);
        assert!(!m.saturated);
        i2c.done();
    }

    #[test]
    fn interrupts() {
        let expectations = [
            Transaction::write_read(ADDRESS, vec![0x0F], vec![0x00]),
            Transaction::write(ADDRESS, vec![0x04, 100, 0, 0x88, 0x13]),
            Transaction::write(ADDRESS, vec![0x01, 0x07]),
            Transaction::write(ADDRESS, vec![0x00, 0xA2]),
            Transaction::write_read(ADDRESS, vec![0x00], vec![0xA6]),
            Transaction::write(ADDRESS, vec![0x04, 0, 0, 0xFF, 0xFF]),
            Transaction::write(ADDRESS, vec![0x00, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut als = Isl29035::new(&mut i2c)
            .with_range(Range::Lux64k)
            .with_resolution(Resolution::Bits12)
            .with_continuous(true);
        assert_eq!(als.probe(), Err(Error::UnexpectedId(0)));
        als.set_interrupt(Some((100, 5000)), Persistence::Cycles8)
            .unwrap();
        assert_eq!(als.start_measurement().unwrap(), 7_150);
        assert!(als.take_interrupt().unwrap());
        als.set_interrupt(None, Persistence::Cycles1).unwrap();
        als.sleep().unwrap();
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
            Transaction::write(ADDRESS, vec![0x01, 0x0B]),
            Transaction::write(ADDRESS, vec![0x00, 0xA0]),
            Transaction::write_read(ADDRESS, vec![0x02], vec![0xFF, 0x00]),
            Transaction::write(ADDRESS, vec![0x00, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut als = asynch::Isl29035::new(&mut i2c)
            .with_range(Range::Lux64k)
            .with_resolution(Resolution::Bits8);
        let mut delay = RecordingDelay(0);
        let m = block_on(als.measure(&mut delay)).unwrap();
        assert!(m.saturated);
        assert_eq!(m.illuminance, 63_750_000);
        assert_eq!(delay.0, 451);
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
