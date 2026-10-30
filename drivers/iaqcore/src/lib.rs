//! ams iAQ-Core C / iAQ-Core P indoor air quality module over I2C.
//!
//! The module measures on its own (C: continuous, sample every 1 s;
//! P: pulsed, sample every 11 s) and exposes a read-only 9-byte result:
//! CO2-equivalent prediction (ppm), status, sensor resistance (Ω) and
//! TVOC-equivalent prediction (ppb). It has no registers and must only be
//! addressed for reading.
//!
//! [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//! does no bus traffic and returns `0`;
//! [`read_measurement`](zigbee_sensor::blocking::Sensor::read_measurement)
//! returns [`Error::NotReady`] while the status is BUSY or RUNIN
//! (warm-up, ≈ 5 min after power-on) and [`Error::InvalidData`] on ERROR.
//!
//! Reference: ams "iAQ-Core Indoor Air Quality Module" datasheet v1-00
//! (2015-Apr-30).

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Fixed I2C address.
pub const ADDRESS: u8 = 0x5A;

/// Warm-up (RUNIN) time after power-on before the first valid reading.
pub const RUN_IN_US: WaitUs = 300_000_000;

/// Module variant (only the sample interval differs on the bus).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Variant {
    /// iAQ-Core C: continuous, 1 s sample interval, 66 mW.
    #[default]
    C,
    /// iAQ-Core P: pulsed, 11 s sample interval, 9 mW.
    P,
}

impl Variant {
    /// Sample interval; poll no faster than this.
    pub const fn interval_us(self) -> WaitUs {
        match self {
            Self::C => 1_000_000,
            Self::P => 11_000_000,
        }
    }
}

/// Status byte.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    /// 0x00: data valid.
    Ok,
    /// 0x10: warm-up phase.
    RunIn,
    /// 0x01: data being updated, re-read.
    Busy,
    /// 0x80: sensor error (replace the module if persistent).
    Error,
    /// Any other value.
    Unknown(u8),
}

impl Status {
    /// Decode the status byte.
    pub const fn from_byte(b: u8) -> Self {
        match b {
            0x00 => Self::Ok,
            0x10 => Self::RunIn,
            0x01 => Self::Busy,
            0x80 => Self::Error,
            other => Self::Unknown(other),
        }
    }
}

/// Raw 9-byte result, decoded without status checking.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame {
    /// CO2-equivalent prediction in ppm (450..=2000).
    pub co2_eq_ppm: u16,
    /// Status.
    pub status: Status,
    /// Sensor resistance in Ω.
    pub resistance_ohm: u32,
    /// TVOC-equivalent prediction in ppb (125..=600).
    pub tvoc_eq_ppb: u16,
}

/// Decode the 9-byte result.
pub fn decode_frame(b: &[u8; 9]) -> Frame {
    Frame {
        co2_eq_ppm: u16::from_be_bytes([b[0], b[1]]),
        status: Status::from_byte(b[2]),
        resistance_ohm: u32::from_be_bytes([b[3], b[4], b[5], b[6]]),
        tvoc_eq_ppb: u16::from_be_bytes([b[7], b[8]]),
    }
}

/// A valid sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// CO2-equivalent prediction in ppm.
    pub co2_eq_ppm: u16,
    /// TVOC-equivalent prediction in ppb.
    pub tvoc_eq_ppb: u16,
    /// Sensor resistance in Ω.
    pub resistance_ohm: u32,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Eco2(u32::from(self.co2_eq_ppm)));
        f(Quantity::Tvoc(u32::from(self.tvoc_eq_ppb)));
        f(Quantity::GasResistance(self.resistance_ohm));
    }
}

/// Check the status of a 9-byte result and convert it to a [`Measurement`].
pub fn decode_measurement<E>(b: &[u8; 9]) -> Result<Measurement, Error<E>> {
    let frame = decode_frame(b);
    match frame.status {
        Status::Ok => Ok(Measurement {
            co2_eq_ppm: frame.co2_eq_ppm,
            tvoc_eq_ppb: frame.tvoc_eq_ppb,
            resistance_ohm: frame.resistance_ohm,
        }),
        Status::Busy | Status::RunIn => Err(Error::NotReady),
        Status::Error | Status::Unknown(_) => Err(Error::InvalidData),
    }
}

macro_rules! constructors {
    () => {
        /// Create a driver; no bus traffic.
        pub const fn new(i2c: I2C, variant: Variant) -> Self {
            Self { i2c, variant }
        }

        /// Module variant.
        pub const fn variant(&self) -> Variant {
            self.variant
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }
    };
}

/// Blocking driver.
pub struct IaqCore<I2C> {
    i2c: I2C,
    variant: Variant,
}

impl<I2C> IaqCore<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> IaqCore<I2C> {
    /// Read the 9-byte result without interpreting the status.
    pub fn read_frame(&mut self) -> Result<Frame, Error<I2C::Error>> {
        let mut b = [0u8; 9];
        self.i2c.read(ADDRESS, &mut b).map_err(Error::bus)?;
        Ok(decode_frame(&b))
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for IaqCore<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    /// Free-running module: nothing to trigger, returns `0`.
    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        Ok(0)
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        let mut b = [0u8; 9];
        self.i2c.read(ADDRESS, &mut b).map_err(Error::bus)?;
        decode_measurement(&b)
    }
}

/// Async driver.
pub mod asynch {
    use super::{ADDRESS, Frame, Measurement, Variant, decode_frame, decode_measurement};
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::IaqCore`].
    pub struct IaqCore<I2C> {
        i2c: I2C,
        variant: Variant,
    }

    impl<I2C> IaqCore<I2C> {
        constructors!();
    }

    impl<I2C: I2c> IaqCore<I2C> {
        /// See [`super::IaqCore::read_frame`].
        pub async fn read_frame(&mut self) -> Result<Frame, Error<I2C::Error>> {
            let mut b = [0u8; 9];
            self.i2c.read(ADDRESS, &mut b).await.map_err(Error::bus)?;
            Ok(decode_frame(&b))
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for IaqCore<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            Ok(0)
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let mut b = [0u8; 9];
            self.i2c.read(ADDRESS, &mut b).await.map_err(Error::bus)?;
            decode_measurement(&b)
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use std::vec;
    use zigbee_sensor::blocking::Sensor;

    // 600 ppm, status OK, 254 kΩ (0x0003_E030), 180 ppb.
    const OK: [u8; 9] = [0x02, 0x58, 0x00, 0x00, 0x03, 0xE0, 0x30, 0x00, 0xB4];

    #[test]
    fn decodes_frame_and_readings() {
        let m = decode_measurement::<()>(&OK).unwrap();
        assert_eq!(
            m,
            Measurement {
                co2_eq_ppm: 600,
                tvoc_eq_ppb: 180,
                resistance_ohm: 254_000
            }
        );
        let mut seen = vec![];
        m.for_each(&mut |q| seen.push(q));
        assert_eq!(
            seen,
            vec![
                Quantity::Eco2(600),
                Quantity::Tvoc(180),
                Quantity::GasResistance(254_000)
            ]
        );
    }

    #[test]
    fn status_codes() {
        let with = |s: u8| {
            let mut b = OK;
            b[2] = s;
            b
        };
        assert_eq!(decode_measurement::<()>(&with(0x01)), Err(Error::NotReady));
        assert_eq!(decode_measurement::<()>(&with(0x10)), Err(Error::NotReady));
        assert_eq!(
            decode_measurement::<()>(&with(0x80)),
            Err(Error::InvalidData)
        );
        assert_eq!(decode_frame(&with(0x10)).status, Status::RunIn);
        assert_eq!(decode_frame(&with(0x42)).status, Status::Unknown(0x42));
    }

    #[test]
    fn reads_without_writing() {
        let mut busy = OK;
        busy[2] = 0x01;
        let expectations = [
            Transaction::read(ADDRESS, busy.to_vec()),
            Transaction::read(ADDRESS, OK.to_vec()),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut iaq = IaqCore::new(&mut i2c, Variant::P);
        assert_eq!(iaq.variant().interval_us(), 11_000_000);
        assert_eq!(iaq.start_measurement().unwrap(), 0);
        assert_eq!(iaq.read_measurement(), Err(Error::NotReady));
        assert_eq!(iaq.read_measurement().unwrap().co2_eq_ppm, 600);
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
            Transaction::read(ADDRESS, OK.to_vec()),
            Transaction::read(ADDRESS, OK.to_vec()),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut iaq = asynch::IaqCore::new(&mut i2c, Variant::C);
        let mut delay = RecordingDelay(0);
        let m = block_on(iaq.measure(&mut delay)).unwrap();
        assert_eq!(m.tvoc_eq_ppb, 180);
        assert_eq!(delay.0, 0);
        assert_eq!(block_on(iaq.read_frame()).unwrap().status, Status::Ok);
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
