//! Aosong (ASAIR) AGS10 MEMS metal-oxide TVOC sensor over I2C.
//!
//! The sensor measures on its own; a "data acquisition" command (register
//! 0x00) returns the latest TVOC value in ppb with a status byte and CRC.
//! [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//! selects register 0x00 and returns its 1.5 s processing time;
//! [`read_measurement`](zigbee_sensor::blocking::Sensor::read_measurement)
//! reads the 5-byte result and returns [`Error::NotReady`] while the RDY
//! bit is set (pre-heating or too-frequent polling).
//!
//! Bus constraints: SCL **≤ 15 kHz**, ≥ 30 ms between any two commands and
//! ≥ 1.5 s between data acquisitions (≥ 2 s sampling period). Polling
//! faster makes the sensor keep RDY set.
//!
//! Reference: Aosong "Datasheet AGS10" (June 2022).
//!
//! ```ignore
//! let mut ags = Ags10::new(i2c); // bus clocked ≤ 15 kHz
//! let wait_us = ags.start_measurement()?; // 1.5 s
//! // ... wait_us later:
//! let m = ags.read_measurement()?;
//! ```

#![no_std]

use zigbee_sensor::crc::crc8_sensirion;
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Factory I2C address (changeable with [`Ags10::set_address`]).
pub const DEFAULT_ADDRESS: u8 = 0x1A;

/// Register addresses.
pub mod reg {
    /// Data acquisition: status, TVOC (24 bit), CRC.
    pub const DATA: u8 = 0x00;
    /// Zero-point calibration.
    pub const CALIBRATION: u8 = 0x01;
    /// Firmware version (Data4).
    pub const VERSION: u8 = 0x11;
    /// Sensor resistance in 0.1 kΩ.
    pub const RESISTANCE: u8 = 0x20;
    /// Slave address change.
    pub const ADDRESS: u8 = 0x21;
}

/// Processing time of data acquisition and resistance reads (datasheet
/// table 2); also the minimum interval between data acquisitions.
pub const ACQUISITION_US: WaitUs = 1_500_000;
/// Processing time of the other commands and the minimum gap between any
/// two commands.
pub const COMMAND_US: WaitUs = 30_000;
/// Pre-heating time after power-on.
pub const PREHEAT_US: WaitUs = 120_000_000;

/// One TVOC sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// TVOC in ppb (0..=99999), ethanol-calibrated.
    pub tvoc_ppb: u32,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Tvoc(self.tvoc_ppb));
    }
}

/// Zero-point calibration target (lost at power-off). Calibrate after
/// ≥ 15 minutes in fresh air.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ZeroPoint {
    /// Restore the factory zero point (Data3/Data4 = 0xFFFF).
    Factory,
    /// Use the current sensor resistance (0x0000).
    CurrentResistance,
    /// Use the given resistance in 0.1 kΩ.
    Resistance(u16),
}

fn check_frame<E>(frame: &[u8; 5]) -> Result<[u8; 4], Error<E>> {
    let data = [frame[0], frame[1], frame[2], frame[3]];
    if crc8_sensirion(&data) == frame[4] {
        Ok(data)
    } else {
        Err(Error::Crc)
    }
}

/// Decode the 5-byte data acquisition frame.
///
/// [`Error::Crc`] on a bad checksum, [`Error::NotReady`] when RDY (status
/// bit 0) is set, [`Error::InvalidData`] when the unit bits (3:1) are not
/// ppb.
pub fn decode_measurement<E>(frame: &[u8; 5]) -> Result<Measurement, Error<E>> {
    let [status, a, b, c] = check_frame(frame)?;
    if status & 0x01 != 0 {
        return Err(Error::NotReady);
    }
    if status & 0x0E != 0 {
        return Err(Error::InvalidData);
    }
    Ok(Measurement {
        tvoc_ppb: u32::from_be_bytes([0, a, b, c]),
    })
}

/// Decode the resistance frame to ohms (the sensor reports 0.1 kΩ).
pub fn decode_resistance<E>(frame: &[u8; 5]) -> Result<u32, Error<E>> {
    let data = check_frame(frame)?;
    Ok(u32::from_be_bytes(data).saturating_mul(100))
}

/// Decode the version frame (version in Data4).
pub fn decode_version<E>(frame: &[u8; 5]) -> Result<u8, Error<E>> {
    Ok(check_frame(frame)?[3])
}

/// Calibration command: `[0x01, 0x00, 0x0C, D3, D4, CRC]`.
pub fn calibration_command(zero: ZeroPoint) -> [u8; 6] {
    let [d3, d4] = match zero {
        ZeroPoint::Factory => [0xFF, 0xFF],
        ZeroPoint::CurrentResistance => [0x00, 0x00],
        ZeroPoint::Resistance(r) => r.to_be_bytes(),
    };
    let data = [0x00, 0x0C, d3, d4];
    [
        reg::CALIBRATION,
        data[0],
        data[1],
        data[2],
        data[3],
        crc8_sensirion(&data),
    ]
}

/// Address change command: `[0x21, new, !new, new, !new, CRC]`.
pub fn address_command(new_address: u8) -> [u8; 6] {
    let data = [new_address, !new_address, new_address, !new_address];
    [
        reg::ADDRESS,
        data[0],
        data[1],
        data[2],
        data[3],
        crc8_sensirion(&data),
    ]
}

macro_rules! constructors {
    () => {
        /// Driver at [`DEFAULT_ADDRESS`]; no bus traffic.
        pub const fn new(i2c: I2C) -> Self {
            Self::with_address(i2c, DEFAULT_ADDRESS)
        }

        /// Driver at a programmed address.
        pub const fn with_address(i2c: I2C, address: u8) -> Self {
            Self { i2c, address }
        }

        /// Current I2C address.
        pub const fn address(&self) -> u8 {
            self.address
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }
    };
}

/// Blocking driver.
pub struct Ags10<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C> Ags10<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Ags10<I2C> {
    fn write(&mut self, bytes: &[u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(self.address, bytes).map_err(Error::bus)
    }

    fn read_frame(&mut self) -> Result<[u8; 5], Error<I2C::Error>> {
        let mut frame = [0u8; 5];
        self.i2c
            .read(self.address, &mut frame)
            .map_err(Error::bus)?;
        Ok(frame)
    }

    /// Select the version register; read with [`Self::read_version`] after
    /// the returned [`COMMAND_US`].
    pub fn request_version(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.write(&[reg::VERSION])?;
        Ok(COMMAND_US)
    }

    /// Read the firmware version selected by [`Self::request_version`].
    pub fn read_version(&mut self) -> Result<u8, Error<I2C::Error>> {
        decode_version(&self.read_frame()?)
    }

    /// Select the resistance register; read with [`Self::read_resistance`]
    /// after the returned [`ACQUISITION_US`].
    pub fn request_resistance(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.write(&[reg::RESISTANCE])?;
        Ok(ACQUISITION_US)
    }

    /// Read the sensor resistance in Ω selected by
    /// [`Self::request_resistance`].
    pub fn read_resistance(&mut self) -> Result<u32, Error<I2C::Error>> {
        decode_resistance(&self.read_frame()?)
    }

    /// Zero-point calibration; returns [`COMMAND_US`].
    pub fn calibrate_zero(&mut self, zero: ZeroPoint) -> Result<WaitUs, Error<I2C::Error>> {
        self.write(&calibration_command(zero))?;
        Ok(COMMAND_US)
    }

    /// Change the I2C address (stored in the sensor, effective at once).
    pub fn set_address(&mut self, new_address: u8) -> Result<WaitUs, Error<I2C::Error>> {
        self.write(&address_command(new_address))?;
        self.address = new_address;
        Ok(COMMAND_US)
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Ags10<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.write(&[reg::DATA])?;
        Ok(ACQUISITION_US)
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        decode_measurement(&self.read_frame()?)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        ACQUISITION_US, COMMAND_US, DEFAULT_ADDRESS, Measurement, ZeroPoint, address_command,
        calibration_command, decode_measurement, decode_resistance, decode_version, reg,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Ags10`].
    pub struct Ags10<I2C> {
        i2c: I2C,
        address: u8,
    }

    impl<I2C> Ags10<I2C> {
        constructors!();
    }

    impl<I2C: I2c> Ags10<I2C> {
        async fn write(&mut self, bytes: &[u8]) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write(self.address, bytes)
                .await
                .map_err(Error::bus)
        }

        async fn read_frame(&mut self) -> Result<[u8; 5], Error<I2C::Error>> {
            let mut frame = [0u8; 5];
            self.i2c
                .read(self.address, &mut frame)
                .await
                .map_err(Error::bus)?;
            Ok(frame)
        }

        /// See [`super::Ags10::request_version`].
        pub async fn request_version(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.write(&[reg::VERSION]).await?;
            Ok(COMMAND_US)
        }

        /// See [`super::Ags10::read_version`].
        pub async fn read_version(&mut self) -> Result<u8, Error<I2C::Error>> {
            decode_version(&self.read_frame().await?)
        }

        /// See [`super::Ags10::request_resistance`].
        pub async fn request_resistance(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.write(&[reg::RESISTANCE]).await?;
            Ok(ACQUISITION_US)
        }

        /// See [`super::Ags10::read_resistance`].
        pub async fn read_resistance(&mut self) -> Result<u32, Error<I2C::Error>> {
            decode_resistance(&self.read_frame().await?)
        }

        /// See [`super::Ags10::calibrate_zero`].
        pub async fn calibrate_zero(
            &mut self,
            zero: ZeroPoint,
        ) -> Result<WaitUs, Error<I2C::Error>> {
            self.write(&calibration_command(zero)).await?;
            Ok(COMMAND_US)
        }

        /// See [`super::Ags10::set_address`].
        pub async fn set_address(&mut self, new_address: u8) -> Result<WaitUs, Error<I2C::Error>> {
            self.write(&address_command(new_address)).await?;
            self.address = new_address;
            Ok(COMMAND_US)
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Ags10<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.write(&[reg::DATA]).await?;
            Ok(ACQUISITION_US)
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            decode_measurement(&self.read_frame().await?)
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

    const A: u8 = DEFAULT_ADDRESS;

    fn frame(data: [u8; 4]) -> vec::Vec<u8> {
        let mut v = data.to_vec();
        v.push(crc8_sensirion(&data));
        v
    }

    #[test]
    fn calibration_commands_match_datasheet_table() {
        assert_eq!(
            calibration_command(ZeroPoint::Factory),
            [0x01, 0x00, 0x0C, 0xFF, 0xFF, 0x81]
        );
        assert_eq!(
            calibration_command(ZeroPoint::CurrentResistance),
            [0x01, 0x00, 0x0C, 0x00, 0x00, 0xAC]
        );
        assert_eq!(
            calibration_command(ZeroPoint::Resistance(0x1CBC)),
            [0x01, 0x00, 0x0C, 0x1C, 0xBC, 0xB4]
        );
    }

    #[test]
    fn measurement_ready_not_ready_and_crc() {
        let good = frame([0x00, 0x00, 0x01, 0x2C]);
        let mut bad = good.clone();
        bad[4] ^= 0xFF;
        let expectations = [
            Transaction::write(A, vec![0x00]),
            Transaction::read(A, good),
            Transaction::write(A, vec![0x00]),
            Transaction::read(A, frame([0x01, 0, 0, 0])),
            Transaction::write(A, vec![0x00]),
            Transaction::read(A, bad),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut ags = Ags10::new(&mut i2c);
        assert_eq!(ags.start_measurement().unwrap(), ACQUISITION_US);
        let m = ags.read_measurement().unwrap();
        assert_eq!(m.tvoc_ppb, 300);
        let mut seen = vec![];
        m.for_each(&mut |q| seen.push(q));
        assert_eq!(seen, vec![Quantity::Tvoc(300)]);
        ags.start_measurement().unwrap();
        assert_eq!(ags.read_measurement(), Err(Error::NotReady));
        ags.start_measurement().unwrap();
        assert_eq!(ags.read_measurement(), Err(Error::Crc));
        i2c.done();
    }

    #[test]
    fn unexpected_unit_is_invalid() {
        let f: [u8; 5] = frame([0x02, 0, 0, 1]).try_into().unwrap();
        assert_eq!(decode_measurement::<()>(&f), Err(Error::InvalidData));
    }

    #[test]
    fn version_resistance_and_address_change() {
        let new = 0x1B;
        let expectations = [
            Transaction::write(A, vec![0x11]),
            Transaction::read(A, frame([0, 0, 0, 0x0B])),
            Transaction::write(A, vec![0x20]),
            Transaction::read(A, frame([0, 0, 0x03, 0xE8])),
            Transaction::write(A, address_command(new).to_vec()),
            Transaction::write(new, vec![0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut ags = Ags10::new(&mut i2c);
        assert_eq!(ags.request_version().unwrap(), COMMAND_US);
        assert_eq!(ags.read_version().unwrap(), 0x0B);
        ags.request_resistance().unwrap();
        assert_eq!(ags.read_resistance().unwrap(), 100_000);
        ags.set_address(new).unwrap();
        assert_eq!(ags.address(), new);
        ags.start_measurement().unwrap();
        assert_eq!(address_command(new)[1..5], [0x1B, 0xE4, 0x1B, 0xE4]);
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
            Transaction::write(A, vec![0x00]),
            Transaction::read(A, frame([0x00, 0x01, 0x86, 0x9F])),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut ags = asynch::Ags10::new(&mut i2c);
        let mut delay = RecordingDelay(0);
        let m = block_on(ags.measure(&mut delay)).unwrap();
        assert_eq!(m.tvoc_ppb, 99_999);
        assert_eq!(delay.0, ACQUISITION_US);
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
