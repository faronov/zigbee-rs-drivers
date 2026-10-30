//! MAX31855 cold-junction compensated thermocouple-to-digital converter.
//!
//! Read-only SPI device: every read returns one 32-bit frame with the
//! thermocouple temperature (14-bit, 0.25 °C), the internal cold-junction
//! temperature (12-bit, 0.0625 °C) and three fault bits. The part converts
//! continuously while CS is high; pulling CS low stops the conversion in
//! progress and the next one starts when CS returns high.
//!
//! All variants (MAX31855K/J/N/T/E/R/S, one per thermocouple type) share
//! the frame format; the type only changes the internal gain. The output is
//! a linear approximation (e.g. 41.276 µV/°C for type K); apply a NIST
//! correction in the application if better than the datasheet accuracy
//! (±2 °C for K, −200..+700 °C) is needed.
//!
//! Reference: Maxim MAX31855 datasheet 19-5793 Rev 2 (Tables 2–5, tCONV).
//!
//! Wiring: SPI mode 0, ≤ 5 MHz, MISO only (no MOSI). Use an
//! `embedded_hal::spi::SpiDevice` that drives CS.

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Maximum conversion time (tCONV); also the time after a read before the
/// next fresh result.
pub const CONVERSION_US: WaitUs = 100_000;
/// Maximum power-up time before the first valid result (tCONV_PU).
pub const POWER_UP_US: WaitUs = 200_000;

/// Fault bits of a frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Faults {
    /// Thermocouple open (D0).
    pub open_circuit: bool,
    /// Thermocouple shorted to GND (D1).
    pub short_to_gnd: bool,
    /// Thermocouple shorted to VCC (D2).
    pub short_to_vcc: bool,
}

impl Faults {
    /// Any fault present.
    pub const fn any(&self) -> bool {
        self.open_circuit || self.short_to_gnd || self.short_to_vcc
    }
}

/// Raw 32-bit frame (D31 first).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame(pub u32);

impl Frame {
    /// Thermocouple temperature in 0.01 °C (D31:18, 0.25 °C per LSB).
    pub const fn thermocouple(&self) -> i32 {
        ((self.0 as i32) >> 18) * 25
    }

    /// Cold-junction temperature in 0.01 °C (D15:4, 0.0625 °C per LSB),
    /// rounded half away from zero.
    pub const fn cold_junction(&self) -> i32 {
        let raw = ((self.0 as i32) << 16) >> 20;
        let x = raw * 25;
        if x >= 0 { (x + 2) / 4 } else { (x - 2) / 4 }
    }

    /// Fault summary bit (D16).
    pub const fn fault(&self) -> bool {
        self.0 & (1 << 16) != 0
    }

    /// Individual fault bits.
    pub const fn faults(&self) -> Faults {
        Faults {
            open_circuit: self.0 & 1 != 0,
            short_to_gnd: self.0 & 2 != 0,
            short_to_vcc: self.0 & 4 != 0,
        }
    }

    /// Reserved bits D17 and D3 read as 0 on a working part; a set bit
    /// means MISO is floating or the device is missing.
    pub const fn reserved_ok(&self) -> bool {
        self.0 & ((1 << 17) | (1 << 3)) == 0
    }

    /// Validate and decode. A missing device (reserved bits set) or any
    /// thermocouple fault is [`Error::InvalidData`]; use [`Self::faults`]
    /// to tell them apart.
    pub fn decode<E>(&self) -> Result<Measurement, Error<E>> {
        if !self.reserved_ok() || self.fault() || self.faults().any() {
            return Err(Error::InvalidData);
        }
        Ok(Measurement {
            temperature: self.thermocouple(),
            cold_junction: self.cold_junction(),
        })
    }
}

/// One reading.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Thermocouple (hot junction) temperature in 0.01 °C.
    pub temperature: i32,
    /// Cold-junction (die) temperature in 0.01 °C.
    pub cold_junction: i32,
}

impl Readings for Measurement {
    /// Emits the thermocouple temperature only; the cold junction is
    /// available as a field.
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
pub struct Max31855<SPI> {
    spi: SPI,
}

impl<SPI> Max31855<SPI> {
    constructors!();
}

impl<SPI: embedded_hal::spi::SpiDevice> Max31855<SPI> {
    /// Read the raw frame (no validation).
    pub fn read_raw(&mut self) -> Result<Frame, Error<SPI::Error>> {
        let mut buf = [0u8; 4];
        self.spi.read(&mut buf).map_err(Error::bus)?;
        Ok(Frame(u32::from_be_bytes(buf)))
    }
}

impl<SPI: embedded_hal::spi::SpiDevice> zigbee_sensor::blocking::Sensor for Max31855<SPI> {
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

    /// Async driver; same API as [`super::Max31855`].
    pub struct Max31855<SPI> {
        spi: SPI,
    }

    impl<SPI> Max31855<SPI> {
        constructors!();
    }

    impl<SPI: SpiDevice> Max31855<SPI> {
        /// See [`super::Max31855::read_raw`].
        pub async fn read_raw(&mut self) -> Result<Frame, Error<SPI::Error>> {
            let mut buf = [0u8; 4];
            self.spi.read(&mut buf).await.map_err(Error::bus)?;
            Ok(Frame(u32::from_be_bytes(buf)))
        }
    }

    impl<SPI: SpiDevice> zigbee_sensor::asynch::Sensor for Max31855<SPI> {
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

    fn frame(tc14: u16, cj12: u16, low: u32) -> Frame {
        Frame(u32::from(tc14) << 18 | u32::from(cj12) << 4 | low)
    }

    fn read(bytes: [u8; 4]) -> [Transaction<u8>; 3] {
        [
            Transaction::transaction_start(),
            Transaction::read_vec(bytes.to_vec()),
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
    fn datasheet_thermocouple_table() {
        // Table 4, D[31:18].
        for (bits, centi) in [
            (0b01_1001_0000_0000, 160_000),
            (0b00_1111_1010_0000, 100_000),
            (0b00_0001_1001_0011, 10_075),
            (0b00_0000_0110_0100, 2_500),
            (0, 0),
            (0b11_1111_1111_1111, -25),
            (0b11_1111_1111_1100, -100),
            (0b11_1100_0001_1000, -25_000),
        ] {
            assert_eq!(frame(bits, 0, 0).thermocouple(), centi, "{bits:#016b}");
        }
    }

    #[test]
    fn datasheet_cold_junction_table() {
        // Table 5, D[15:4].
        for (bits, centi) in [
            (0b0111_1111_0000, 12_700),
            (0b0110_0100_1001, 10_056),
            (0b0001_1001_0000, 2_500),
            (0, 0),
            (0b1111_1111_1111, -6),
            (0b1111_1111_0000, -100),
            (0b1110_1100_0000, -2_000),
            (0b1100_1001_0000, -5_500),
        ] {
            assert_eq!(frame(0, bits, 0).cold_junction(), centi, "{bits:#014b}");
        }
    }

    #[test]
    fn faults_and_missing_device() {
        let open = frame(0, 0x190, 1 << 16 | 1);
        assert_eq!(open.decode::<()>(), Err(Error::InvalidData));
        assert_eq!(
            open.faults(),
            Faults {
                open_circuit: true,
                ..Faults::default()
            }
        );
        assert!(frame(0, 0, 1 << 16 | 4).faults().short_to_vcc);
        assert_eq!(Frame(0xFFFF_FFFF).decode::<()>(), Err(Error::InvalidData));
        assert_eq!(frame(0, 0, 1 << 3).decode::<()>(), Err(Error::InvalidData));
    }

    #[test]
    fn blocking_read() {
        // +100.75 °C hot, +25 °C cold junction.
        let f = frame(0b00_0001_1001_0011, 0b0001_1001_0000, 0);
        let mut spi = Mock::new(&read(f.0.to_be_bytes()));
        let mut dev = Max31855::new(spi.clone());
        assert_eq!(dev.start_measurement(), Ok(CONVERSION_US));
        assert_eq!(
            dev.read_measurement(),
            Ok(Measurement {
                temperature: 10_075,
                cold_junction: 2_500
            })
        );
        spi.done();
    }

    #[test]
    fn async_read_reports_short_to_gnd() {
        use zigbee_sensor::asynch::Sensor as _;
        let ok = frame(0b11_1100_0001_1000, 0b1110_1100_0000, 0);
        let bad = frame(0, 0x190, 1 << 16 | 2);
        let mut script = vec![];
        script.extend(read(ok.0.to_be_bytes()));
        script.extend(read(bad.0.to_be_bytes()));
        let mut spi = Mock::new(&script);
        let mut dev = asynch::Max31855::new(spi.clone());
        let m = block_on(dev.read_measurement()).unwrap();
        assert_eq!((m.temperature, m.cold_junction), (-25_000, -2_000));
        let raw = block_on(dev.read_raw()).unwrap();
        assert!(raw.faults().short_to_gnd);
        spi.done();
    }
}
