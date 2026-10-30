//! MAX6675 cold-junction compensated K-type thermocouple-to-digital
//! converter (0 °C to +1023.75 °C, 0.25 °C resolution).
//!
//! Read-only SPI device returning a 16-bit frame: D15 dummy sign bit (0),
//! D14:3 the 12-bit temperature, D2 open-thermocouple flag, D1 device ID
//! (0), D0 three-state. Pulling CS low stops the conversion in progress; a
//! new one starts when CS returns high and takes up to 220 ms, so reads more
//! often than every 220 ms return the previous value.
//!
//! Reference: Maxim MAX6675 datasheet 19-2235 Rev 1 ("Serial Interface",
//! Electrical Characteristics: conversion time 0.17 s typ / 0.22 s max).
//!
//! Wiring: SPI mode 0, ≤ 4.3 MHz, MISO only. T− must be grounded for the
//! open-thermocouple detector to work. Use an `embedded_hal::spi::SpiDevice`
//! that drives CS.

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Maximum conversion time after CS goes high.
pub const CONVERSION_US: WaitUs = 220_000;

/// Raw 16-bit frame (D15 first).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame(pub u16);

impl Frame {
    /// Temperature in 0.01 °C (D14:3, 0.25 °C per LSB).
    pub const fn temperature(&self) -> i32 {
        ((self.0 >> 3) & 0x0FFF) as i32 * 25
    }

    /// Thermocouple input open (D2).
    pub const fn open_thermocouple(&self) -> bool {
        self.0 & 0b100 != 0
    }

    /// The dummy sign bit (D15) and device ID bit (D1) read 0 on a MAX6675;
    /// otherwise MISO is floating or another device answered.
    pub const fn id_ok(&self) -> bool {
        self.0 & 0x8002 == 0
    }

    /// Validate and decode: wrong ID bits or an open thermocouple is
    /// [`Error::InvalidData`].
    pub fn decode<E>(&self) -> Result<Measurement, Error<E>> {
        if !self.id_ok() || self.open_thermocouple() {
            return Err(Error::InvalidData);
        }
        Ok(Measurement {
            temperature: self.temperature(),
        })
    }
}

/// One reading.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Thermocouple temperature in 0.01 °C.
    pub temperature: i32,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Temperature(self.temperature));
    }
}

macro_rules! constructors {
    () => {
        /// Create a driver; no bus traffic.
        pub const fn new(spi: SPI) -> Self {
            Self { spi }
        }

        /// Give the bus back.
        pub fn release(self) -> SPI {
            self.spi
        }
    };
}

/// Blocking driver.
pub struct Max6675<SPI> {
    spi: SPI,
}

impl<SPI> Max6675<SPI> {
    constructors!();
}

impl<SPI: embedded_hal::spi::SpiDevice> Max6675<SPI> {
    /// Read the raw frame (no validation).
    pub fn read_raw(&mut self) -> Result<Frame, Error<SPI::Error>> {
        let mut buf = [0u8; 2];
        self.spi.read(&mut buf).map_err(Error::bus)?;
        Ok(Frame(u16::from_be_bytes(buf)))
    }
}

impl<SPI: embedded_hal::spi::SpiDevice> zigbee_sensor::blocking::Sensor for Max6675<SPI> {
    type BusError = SPI::Error;
    type Measurement = Measurement;

    /// No bus traffic: the part converts continuously. Returns
    /// [`CONVERSION_US`], the worst case since the previous read.
    fn start_measurement(&mut self) -> Result<WaitUs, Error<SPI::Error>> {
        Ok(CONVERSION_US)
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<SPI::Error>> {
        self.read_raw()?.decode()
    }
}

/// Async driver.
pub mod asynch {
    use super::{CONVERSION_US, Frame, Measurement};
    use embedded_hal_async::spi::SpiDevice;
    use zigbee_sensor::{Error, WaitUs};

    /// Async driver; same API as [`super::Max6675`].
    pub struct Max6675<SPI> {
        spi: SPI,
    }

    impl<SPI> Max6675<SPI> {
        constructors!();
    }

    impl<SPI: SpiDevice> Max6675<SPI> {
        /// See [`super::Max6675::read_raw`].
        pub async fn read_raw(&mut self) -> Result<Frame, Error<SPI::Error>> {
            let mut buf = [0u8; 2];
            self.spi.read(&mut buf).await.map_err(Error::bus)?;
            Ok(Frame(u16::from_be_bytes(buf)))
        }
    }

    impl<SPI: SpiDevice> zigbee_sensor::asynch::Sensor for Max6675<SPI> {
        type BusError = SPI::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<SPI::Error>> {
            Ok(CONVERSION_US)
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<SPI::Error>> {
            self.read_raw().await?.decode()
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::spi::{Mock, Transaction};
    use std::vec;
    use zigbee_sensor::blocking::Sensor;

    fn read(word: u16) -> [Transaction<u8>; 3] {
        [
            Transaction::transaction_start(),
            Transaction::read_vec(word.to_be_bytes().to_vec()),
            Transaction::transaction_end(),
        ]
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

    #[test]
    fn decodes_range_and_flags() {
        // Datasheet: full scale 1111 1111 1111 = +1023.75 °C, 0 = 0 °C.
        assert_eq!(Frame(0x7FF8).decode::<()>().unwrap().temperature, 102_375);
        assert_eq!(Frame(0x0000).decode::<()>().unwrap().temperature, 0);
        // 25.00 °C = 100 LSB; D0 is three-state and ignored.
        assert_eq!(
            Frame(100 << 3 | 1).decode::<()>().unwrap().temperature,
            2_500
        );
        assert_eq!(
            Frame(100 << 3 | 0b100).decode::<()>(),
            Err(Error::InvalidData)
        );
        assert!(Frame(0b100).open_thermocouple());
        assert_eq!(Frame(0xFFFF).decode::<()>(), Err(Error::InvalidData));
        assert_eq!(
            Frame(100 << 3 | 0b10).decode::<()>(),
            Err(Error::InvalidData)
        );
    }

    #[test]
    fn blocking_and_async_reads() {
        use zigbee_sensor::asynch::Sensor as _;
        // 412.25 °C = 1649 LSB.
        let mut script = vec![];
        script.extend(read(1649 << 3));
        script.extend(read(0x0004));
        let mut spi = Mock::new(&script);
        let mut dev = Max6675::new(spi.clone());
        assert_eq!(dev.start_measurement(), Ok(CONVERSION_US));
        assert_eq!(dev.read_measurement().unwrap().temperature, 41_225);
        let mut adev = asynch::Max6675::new(dev.release());
        assert_eq!(block_on(adev.read_measurement()), Err(Error::InvalidData));
        spi.done();
    }
}
