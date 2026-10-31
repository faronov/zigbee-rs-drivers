//! ROHM BH1750FVI ambient light sensor (I2C).
//!
//! The driver only uses the *one-time* measurement modes: the part converts
//! once and drops back to power-down (about 0.01 µA typ.) by itself, so a
//! sleepy end device pays for exactly one integration per sample.
//!
//! The sensitivity can be changed with the measurement-time register
//! (MTreg, 31..=254, default 69): a larger MTreg integrates longer, which
//! extends the range down to 0.11 lx (H-resolution mode 2, MTreg 254); a
//! smaller one reaches 100 000 lx (MTreg 31).
//!
//! Reference: ROHM "BH1750FVI Digital 16bit Serial Output Type Ambient Light
//! Sensor IC", datasheet Rev. B (2011.11).
//!
//! ```ignore
//! let mut bh = Bh1750::new(i2c, ADDRESS_LOW).with_resolution(Resolution::High2);
//! let wait_us = bh.start_measurement()?;
//! // ... let the Zigbee stack run / sleep for wait_us ...
//! let lux_milli = bh.read_measurement()?.illuminance;
//! ```

#![no_std]

use embedded_hal::i2c::I2c;
use zigbee_sensor::blocking::{LowPower, Sensor};
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address with ADDR low (≤ 0.3 VCC).
pub const ADDRESS_LOW: u8 = 0x23;
/// I2C address with ADDR high (≥ 0.7 VCC).
pub const ADDRESS_HIGH: u8 = 0x5C;

const OP_POWER_DOWN: u8 = 0x00;
const OP_POWER_ON: u8 = 0x01;
const OP_RESET: u8 = 0x07;

/// Default measurement-time register value.
pub const MTREG_DEFAULT: u8 = 69;
/// Smallest MTreg value accepted by the part.
pub const MTREG_MIN: u8 = 31;
/// Largest MTreg value accepted by the part.
pub const MTREG_MAX: u8 = 254;

/// One-time measurement resolution.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Resolution {
    /// H-resolution mode: 1 lx step, 120 ms typ. / 180 ms max.
    #[default]
    High,
    /// H-resolution mode 2: 0.5 lx step, 120 ms typ. / 180 ms max.
    High2,
    /// L-resolution mode: 4 lx step, 16 ms typ. / 24 ms max.
    Low,
}

impl Resolution {
    /// One-time measurement opcode (datasheet "Instruction Set Architecture").
    pub const fn opcode(self) -> u8 {
        match self {
            Self::High => 0x20,
            Self::High2 => 0x21,
            Self::Low => 0x23,
        }
    }

    /// Maximum conversion time for `mtreg`, in microseconds. The datasheet
    /// gives 180 ms (H) / 24 ms (L) at MTreg 69; the time scales linearly.
    pub const fn wait_us(self, mtreg: u8) -> WaitUs {
        let base: u32 = match self {
            Self::High | Self::High2 => 180_000,
            Self::Low => 24_000,
        };
        (base * mtreg as u32).div_ceil(MTREG_DEFAULT as u32)
    }
}

/// Convert a raw count to milli-lux.
///
/// Datasheet "Measurement sequence example" and "Adjust measurement
/// result for influence of optical window": `lx = count / 1.2 × (69 / MTreg)`, halved again
/// in H-resolution mode 2. `count × 1000 × 69 / (1.2 × MTreg)` reduces to
/// `count × 57 500 / MTreg`, which fits in `u32` for every 16-bit count.
pub const fn raw_to_millilux(count: u16, resolution: Resolution, mtreg: u8) -> u32 {
    let mt = if mtreg < MTREG_MIN { MTREG_MIN } else { mtreg } as u32;
    let factor: u32 = match resolution {
        Resolution::High2 => 28_750,
        Resolution::High | Resolution::Low => 57_500,
    };
    count as u32 * factor / mt
}

/// MTreg write sequence: `01000_MT[7:5]` then `011_MT[4:0]`.
pub const fn mtreg_opcodes(mtreg: u8) -> [u8; 2] {
    [0x40 | (mtreg >> 5), 0x60 | (mtreg & 0x1F)]
}

/// One illuminance sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Illuminance in milli-lux.
    pub illuminance: u32,
    /// Raw 16-bit count.
    pub raw: u16,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Illuminance(self.illuminance));
    }
}

macro_rules! driver {
    ($($async_:ident)?, $($await_:tt)*) => {
        /// BH1750 driver.
        pub struct Bh1750<I2C> {
            i2c: I2C,
            address: u8,
            resolution: Resolution,
            mtreg: u8,
        }

        impl<I2C> Bh1750<I2C> {
            /// Create a driver; no bus traffic.
            pub const fn new(i2c: I2C, address: u8) -> Self {
                Self {
                    i2c,
                    address,
                    resolution: Resolution::High,
                    mtreg: MTREG_DEFAULT,
                }
            }

            /// Select the resolution used by `start_measurement`.
            pub const fn with_resolution(mut self, resolution: Resolution) -> Self {
                self.resolution = resolution;
                self
            }

            /// Current MTreg value assumed by the conversion.
            pub const fn mtreg(&self) -> u8 {
                self.mtreg
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.i2c
            }
        }

        impl<I2C: I2c> Bh1750<I2C> {
            $($async_)? fn op(&mut self, opcode: u8) -> Result<(), Error<I2C::Error>> {
                self.i2c
                    .write(self.address, &[opcode])
                    $($await_)*
                    .map_err(Error::bus)
            }

            /// Change the measurement time register (clamped to 31..=254).
            /// Takes effect from the next measurement.
            pub $($async_)? fn set_mtreg(&mut self, mtreg: u8) -> Result<(), Error<I2C::Error>> {
                let mtreg = mtreg.clamp(MTREG_MIN, MTREG_MAX);
                let [high, low] = mtreg_opcodes(mtreg);
                self.op(high) $($await_)* ?;
                self.op(low) $($await_)* ?;
                self.mtreg = mtreg;
                Ok(())
            }

            /// Clear the data register (only valid while powered on).
            pub $($async_)? fn reset(&mut self) -> Result<(), Error<I2C::Error>> {
                self.op(OP_POWER_ON) $($await_)* ?;
                self.op(OP_RESET) $($await_)*
            }
        }

        impl<I2C: I2c> Sensor for Bh1750<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// Power on and trigger a one-time conversion.
            $($async_)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.op(OP_POWER_ON) $($await_)* ?;
                self.op(self.resolution.opcode()) $($await_)* ?;
                Ok(self.resolution.wait_us(self.mtreg))
            }

            /// Read the result; the part is already back in power-down.
            $($async_)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let mut buf = [0u8; 2];
                self.i2c
                    .read(self.address, &mut buf)
                    $($await_)*
                    .map_err(Error::bus)?;
                let raw = u16::from_be_bytes(buf);
                Ok(Measurement {
                    illuminance: raw_to_millilux(raw, self.resolution, self.mtreg),
                    raw,
                })
            }
        }

        impl<I2C: I2c> LowPower for Bh1750<I2C> {
            type BusError = I2C::Error;

            $($async_)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.op(OP_POWER_DOWN) $($await_)*
            }

            $($async_)? fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.op(OP_POWER_ON) $($await_)* ?;
                Ok(0)
            }
        }
    };
}

driver!(,);

/// Async driver.
pub mod asynch {
    use super::{
        MTREG_DEFAULT, MTREG_MAX, MTREG_MIN, Measurement, OP_POWER_DOWN, OP_POWER_ON, OP_RESET,
        Resolution, mtreg_opcodes, raw_to_millilux,
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
    fn datasheet_conversion() {
        // Datasheet "Measurement sequence example": 0x8390 = 33 680 → 28 067 lx (H mode, MTreg 69).
        assert_eq!(raw_to_millilux(0x8390, Resolution::High, 69), 28_066_666);
        // "Adjust measurement result": H-res mode 2, MTreg 254 resolves 0.11 lx per count.
        assert_eq!(raw_to_millilux(1, Resolution::High2, 254), 113);
        // MTreg 31 extends the range to ~100 000 lx and beyond.
        assert_eq!(raw_to_millilux(0xFFFF, Resolution::High, 31), 121_556_854);
        assert_eq!(raw_to_millilux(12, Resolution::Low, 69), 10_000);
    }

    #[test]
    fn wait_scales_with_mtreg() {
        assert_eq!(Resolution::High.wait_us(69), 180_000);
        assert_eq!(Resolution::Low.wait_us(69), 24_000);
        assert_eq!(Resolution::High2.wait_us(138), 360_000);
    }

    #[test]
    fn mtreg_opcodes_match_datasheet() {
        // Default MTreg 0b0100_0101 (69) → 0x42, 0x65.
        assert_eq!(mtreg_opcodes(69), [0x42, 0x65]);
        assert_eq!(mtreg_opcodes(254), [0x47, 0x7E]);
    }

    #[test]
    fn one_time_measurement() {
        let expectations = [
            Transaction::write(ADDRESS_LOW, vec![0x47]),
            Transaction::write(ADDRESS_LOW, vec![0x7E]),
            Transaction::write(ADDRESS_LOW, vec![OP_POWER_ON]),
            Transaction::write(ADDRESS_LOW, vec![0x21]),
            Transaction::read(ADDRESS_LOW, vec![0x00, 0x0A]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut bh = Bh1750::new(&mut i2c, ADDRESS_LOW).with_resolution(Resolution::High2);
        bh.set_mtreg(255).unwrap();
        assert_eq!(bh.mtreg(), 254);
        assert_eq!(bh.start_measurement().unwrap(), 662_609);
        let m = bh.read_measurement().unwrap();
        assert_eq!(m.raw, 10);
        assert_eq!(m.illuminance, 1_131);
        i2c.done();
    }

    #[test]
    fn sleep_and_reset() {
        let expectations = [
            Transaction::write(ADDRESS_HIGH, vec![OP_POWER_DOWN]),
            Transaction::write(ADDRESS_HIGH, vec![OP_POWER_ON]),
            Transaction::write(ADDRESS_HIGH, vec![OP_POWER_ON]),
            Transaction::write(ADDRESS_HIGH, vec![OP_RESET]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut bh = Bh1750::new(&mut i2c, ADDRESS_HIGH);
        bh.sleep().unwrap();
        assert_eq!(bh.wake().unwrap(), 0);
        bh.reset().unwrap();
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
            Transaction::write(ADDRESS_LOW, vec![OP_POWER_ON]),
            Transaction::write(ADDRESS_LOW, vec![0x23]),
            Transaction::read(ADDRESS_LOW, vec![0x00, 0x0C]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut bh = asynch::Bh1750::new(&mut i2c, ADDRESS_LOW).with_resolution(Resolution::Low);
        let mut delay = RecordingDelay(0);
        let m = block_on(bh.measure(&mut delay)).unwrap();
        assert_eq!(m.illuminance, 10_000);
        assert_eq!(delay.0, 24_000);
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
