//! Aosong AM2320 temperature and humidity sensor (I2C, Modbus-like
//! framing with CRC-16).
//!
//! The AM2320 sleeps between transactions and does not answer its address
//! until woken. A transaction has three steps, each separated by a wait:
//!
//! 1. [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement):
//!    address-only write to wake the part (the NACK is expected and
//!    ignored); returns [`WAKE_WAIT_US`].
//! 2. First [`read_measurement`](zigbee_sensor::blocking::Sensor::read_measurement):
//!    sends "read registers 0x00..0x03" (function 0x03) and returns
//!    [`Error::NotReady`]; wait [`Am2320::pending_wait_us`] (≥ 1.5 ms).
//! 3. Second `read_measurement`: reads the 8-byte answer and checks the
//!    CRC-16 (Modbus, low byte first).
//!
//! The whole transaction must finish within 3 s, after which the part goes
//! back to sleep. Each transaction also triggers the *next* conversion: the
//! values returned are those measured at the end of the previous
//! transaction, so read twice (≥ 2 s apart) after a long pause. Reading
//! more often than every 2 s is not allowed.
//!
//! The bus must run at ≤ 100 kHz, and the part needs ≥ 30 µs after its
//! read address before the first data clock; slow the bus if reads fail.
//!
//! Reference: Aosong "AM2320 Digital Temperature and Humidity Sensor"
//! product manual (§8.2).
//!
//! ```ignore
//! let mut am = Am2320::new(i2c);
//! let wake_us = am.start_measurement()?;
//! // ... sleep wake_us ...
//! assert_eq!(am.read_measurement(), Err(Error::NotReady));
//! // ... sleep am.pending_wait_us() ...
//! let m = am.read_measurement()?;
//! ```

#![no_std]

use zigbee_sensor::crc::crc16_modbus;
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Fixed 7-bit I2C address (0xB8 write byte).
pub const ADDRESS: u8 = 0x5C;

/// Wait after the wake-up write (the datasheet's 0.8–3 ms wake hold).
pub const WAKE_WAIT_US: WaitUs = 1_000;
/// Wait between the read command and reading the answer (≥ 1.5 ms).
pub const READ_WAIT_US: WaitUs = 2_000;
/// Minimum interval between transactions.
pub const MIN_INTERVAL_US: WaitUs = 2_000_000;

const FUNCTION_READ: u8 = 0x03;
const CMD_READ_MEASUREMENT: [u8; 3] = [FUNCTION_READ, 0x00, 0x04];
const CMD_READ_INFO: [u8; 3] = [FUNCTION_READ, 0x08, 0x07];

/// One temperature + humidity sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Temperature in 0.01 °C.
    pub temperature: i32,
    /// Relative humidity in 0.01 %RH.
    pub humidity: u16,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Temperature(self.temperature));
        f(Quantity::Humidity(self.humidity));
    }
}

/// Registers 0x08..0x0E.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeviceInfo {
    /// Model number.
    pub model: u16,
    /// Version number.
    pub version: u8,
    /// Device ID.
    pub id: u32,
}

/// Check a `[function, length, data.., crc_lo, crc_hi]` answer and return
/// the data bytes. Error answers (function | 0x80) and wrong lengths return
/// [`Error::InvalidData`].
pub fn check_response<E>(frame: &[u8]) -> Result<&[u8], Error<E>> {
    let n = frame.len();
    if n < 4 {
        return Err(Error::InvalidData);
    }
    let crc = u16::from_le_bytes([frame[n - 2], frame[n - 1]]);
    if crc16_modbus(&frame[..n - 2]) != crc {
        return Err(Error::Crc);
    }
    if frame[0] != FUNCTION_READ || usize::from(frame[1]) != n - 4 {
        return Err(Error::InvalidData);
    }
    Ok(&frame[2..n - 2])
}

/// Decode the 8-byte answer to "read registers 0x00..0x03":
/// RH = u16 / 10 %, T = sign-magnitude (bit 15 = negative) / 10 °C.
pub fn decode_measurement<E>(frame: &[u8; 8]) -> Result<Measurement, Error<E>> {
    let data = check_response(frame)?;
    let humidity = u32::from(u16::from_be_bytes([data[0], data[1]])) * 10;
    let magnitude = i32::from(u16::from_be_bytes([data[2] & 0x7F, data[3]])) * 10;
    if humidity > 10_000 {
        return Err(Error::InvalidData);
    }
    Ok(Measurement {
        temperature: if data[2] & 0x80 != 0 {
            -magnitude
        } else {
            magnitude
        },
        humidity: humidity as u16,
    })
}

/// Decode the 11-byte answer to "read registers 0x08..0x0E".
pub fn decode_device_info<E>(frame: &[u8; 11]) -> Result<DeviceInfo, Error<E>> {
    let data = check_response(frame)?;
    Ok(DeviceInfo {
        model: u16::from_be_bytes([data[0], data[1]]),
        version: data[2],
        id: u32::from_be_bytes([data[3], data[4], data[5], data[6]]),
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    Idle,
    Awake,
    Commanded,
}

macro_rules! constructors {
    () => {
        /// Create a driver at [`ADDRESS`](crate::ADDRESS); no bus traffic.
        pub const fn new(i2c: I2C) -> Self {
            Self {
                i2c,
                phase: Phase::Idle,
            }
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }

        /// Wait before the next `read_measurement` after it returned
        /// [`Error::NotReady`](zigbee_sensor::Error::NotReady).
        pub const fn pending_wait_us(&self) -> WaitUs {
            READ_WAIT_US
        }
    };
}

/// Blocking driver.
pub struct Am2320<I2C> {
    i2c: I2C,
    phase: Phase,
}

impl<I2C> Am2320<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Am2320<I2C> {
    fn wake(&mut self) {
        // The sleeping part NACKs its address; that is the wake-up.
        let _ = self.i2c.write(ADDRESS, &[]);
    }

    fn command(&mut self, command: &[u8; 3]) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(ADDRESS, command).map_err(Error::bus)
    }

    /// Wake, command, wait and read in one call (convenience for bring-up;
    /// blocks ~3 ms).
    pub fn measure<D: embedded_hal::delay::DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<Measurement, Error<I2C::Error>> {
        use zigbee_sensor::blocking::Sensor as _;
        delay.delay_us(self.start_measurement()?);
        match self.read_measurement() {
            Err(Error::NotReady) => {
                delay.delay_us(READ_WAIT_US);
                self.read_measurement()
            }
            other => other,
        }
    }

    /// Read model, version and device ID (blocks ~3 ms on `delay`).
    pub fn device_info<D: embedded_hal::delay::DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<DeviceInfo, Error<I2C::Error>> {
        self.phase = Phase::Idle;
        self.wake();
        delay.delay_us(WAKE_WAIT_US);
        self.command(&CMD_READ_INFO)?;
        delay.delay_us(READ_WAIT_US);
        let mut frame = [0u8; 11];
        self.i2c.read(ADDRESS, &mut frame).map_err(Error::bus)?;
        decode_device_info(&frame)
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Am2320<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    /// Wake the sensor; wait [`WAKE_WAIT_US`].
    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.wake();
        self.phase = Phase::Awake;
        Ok(WAKE_WAIT_US)
    }

    /// First call after waking sends the read command and returns
    /// [`Error::NotReady`]; the next call reads the answer.
    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        if self.phase == Phase::Awake {
            self.phase = Phase::Idle;
            self.command(&CMD_READ_MEASUREMENT)?;
            self.phase = Phase::Commanded;
            return Err(Error::NotReady);
        }
        self.phase = Phase::Idle;
        let mut frame = [0u8; 8];
        self.i2c.read(ADDRESS, &mut frame).map_err(Error::bus)?;
        decode_measurement(&frame)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        ADDRESS, CMD_READ_INFO, CMD_READ_MEASUREMENT, DeviceInfo, Measurement, Phase, READ_WAIT_US,
        WAKE_WAIT_US, decode_device_info, decode_measurement,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Am2320`].
    pub struct Am2320<I2C> {
        i2c: I2C,
        phase: Phase,
    }

    impl<I2C> Am2320<I2C> {
        constructors!();
    }

    impl<I2C: I2c> Am2320<I2C> {
        async fn wake(&mut self) {
            let _ = self.i2c.write(ADDRESS, &[]).await;
        }

        async fn command(&mut self, command: &[u8; 3]) -> Result<(), Error<I2C::Error>> {
            self.i2c.write(ADDRESS, command).await.map_err(Error::bus)
        }

        /// See [`super::Am2320::measure`].
        pub async fn measure_all<D: embedded_hal_async::delay::DelayNs>(
            &mut self,
            delay: &mut D,
        ) -> Result<Measurement, Error<I2C::Error>> {
            use zigbee_sensor::asynch::Sensor as _;
            let wait = self.start_measurement().await?;
            delay.delay_us(wait).await;
            match self.read_measurement().await {
                Err(Error::NotReady) => {
                    delay.delay_us(READ_WAIT_US).await;
                    self.read_measurement().await
                }
                other => other,
            }
        }

        /// See [`super::Am2320::device_info`].
        pub async fn device_info<D: embedded_hal_async::delay::DelayNs>(
            &mut self,
            delay: &mut D,
        ) -> Result<DeviceInfo, Error<I2C::Error>> {
            self.phase = Phase::Idle;
            self.wake().await;
            delay.delay_us(WAKE_WAIT_US).await;
            self.command(&CMD_READ_INFO).await?;
            delay.delay_us(READ_WAIT_US).await;
            let mut frame = [0u8; 11];
            self.i2c
                .read(ADDRESS, &mut frame)
                .await
                .map_err(Error::bus)?;
            decode_device_info(&frame)
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Am2320<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.wake().await;
            self.phase = Phase::Awake;
            Ok(WAKE_WAIT_US)
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            if self.phase == Phase::Awake {
                self.phase = Phase::Idle;
                self.command(&CMD_READ_MEASUREMENT).await?;
                self.phase = Phase::Commanded;
                return Err(Error::NotReady);
            }
            self.phase = Phase::Idle;
            let mut frame = [0u8; 8];
            self.i2c
                .read(ADDRESS, &mut frame)
                .await
                .map_err(Error::bus)?;
            decode_measurement(&frame)
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal::i2c::ErrorKind;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use std::vec;
    use zigbee_sensor::blocking::Sensor;

    // §8.2.4 example: 82.5 %RH, 27.7 °C, CRC 0xFEE1 sent low byte first.
    const FRAME: [u8; 8] = [0x03, 0x04, 0x03, 0x39, 0x01, 0x15, 0xE1, 0xFE];
    const EXPECTED: Measurement = Measurement {
        temperature: 2_770,
        humidity: 8_250,
    };

    #[test]
    fn decode_datasheet_example() {
        assert_eq!(decode_measurement::<()>(&FRAME), Ok(EXPECTED));
        // Bit 15 set = negative: 0x8065 = −10.1 °C, 0x01F4 = 50.0 %RH.
        assert_eq!(
            decode_measurement::<()>(&[0x03, 0x04, 0x01, 0xF4, 0x80, 0x65, 0x10, 0x0D]),
            Ok(Measurement {
                temperature: -1_010,
                humidity: 5_000
            })
        );
    }

    #[test]
    fn crc_and_error_answers() {
        let mut bad = FRAME;
        bad[7] ^= 1;
        assert_eq!(decode_measurement::<()>(&bad), Err(Error::Crc));
        // Function byte other than 0x03 (e.g. an error answer) with valid CRC.
        let mut err = [0x83, 0x04, 0, 0, 0, 0, 0, 0];
        let crc = crc16_modbus(&err[..6]).to_le_bytes();
        err[6] = crc[0];
        err[7] = crc[1];
        assert_eq!(decode_measurement::<()>(&err), Err(Error::InvalidData));
    }

    #[test]
    fn three_phase_read_ignores_wake_nack() {
        let expectations = [
            Transaction::write(ADDRESS, vec![]).with_error(ErrorKind::Other),
            Transaction::write(ADDRESS, vec![0x03, 0x00, 0x04]),
            Transaction::read(ADDRESS, FRAME.to_vec()),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut am = Am2320::new(&mut i2c);
        assert_eq!(am.start_measurement(), Ok(WAKE_WAIT_US));
        assert_eq!(am.read_measurement(), Err(Error::NotReady));
        assert_eq!(am.pending_wait_us(), 2_000);
        assert_eq!(am.read_measurement(), Ok(EXPECTED));
        i2c.done();
    }

    #[test]
    fn device_info() {
        struct CountingDelay(u32);
        impl embedded_hal::delay::DelayNs for CountingDelay {
            fn delay_ns(&mut self, ns: u32) {
                self.0 += ns / 1_000;
            }
        }
        let expectations = [
            Transaction::write(ADDRESS, vec![]),
            Transaction::write(ADDRESS, vec![0x03, 0x08, 0x07]),
            Transaction::read(
                ADDRESS,
                vec![
                    0x03, 0x07, 0x23, 0x20, 0x10, 0x12, 0x34, 0x56, 0x78, 0xF0, 0xB3,
                ],
            ),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut am = Am2320::new(&mut i2c);
        let mut delay = CountingDelay(0);
        assert_eq!(
            am.device_info(&mut delay),
            Ok(DeviceInfo {
                model: 0x2320,
                version: 0x10,
                id: 0x1234_5678
            })
        );
        assert_eq!(delay.0, 3_000);
        i2c.done();
    }

    #[test]
    fn async_measure() {
        struct RecordingDelay(u32);
        impl embedded_hal_async::delay::DelayNs for RecordingDelay {
            async fn delay_ns(&mut self, ns: u32) {
                self.0 += ns / 1_000;
            }
        }

        let expectations = [
            Transaction::write(ADDRESS, vec![]).with_error(ErrorKind::Other),
            Transaction::write(ADDRESS, vec![0x03, 0x00, 0x04]),
            Transaction::read(ADDRESS, FRAME.to_vec()),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut am = asynch::Am2320::new(&mut i2c);
        let mut delay = RecordingDelay(0);
        assert_eq!(block_on(am.measure_all(&mut delay)), Ok(EXPECTED));
        assert_eq!(delay.0, WAKE_WAIT_US + READ_WAIT_US);
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
