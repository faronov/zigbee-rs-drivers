//! Aosong (ASAIR) AHT10 / AHT20 / AHT21 / AHT25 / AHT30 and AM2315C
//! temperature and humidity sensors (I2C, address 0x38).
//!
//! All parts use the same measurement protocol: write `0xAC 0x33 0x00`,
//! wait ~80 ms, read a status byte (bit 7 = busy) followed by 20-bit
//! humidity and 20-bit temperature. AHT20 and later append a CRC-8
//! (poly 0x31, init 0xFF); AHT10 does not. The sensor sleeps between
//! measurements on its own.
//!
//! Power-on calibration check (once after power-up, see
//! [`Variant::power_up_us`]): read [`Aht::status`] and, if
//! [`Variant::is_calibrated`] is false, run [`Aht::initialize`] (AHT10,
//! AHT20, AHT30) or the register restore sequence (AHT21 / AHT25 /
//! AM2315C, see [`Aht::restore_register_begin`]).
//!
//! References: AHT10 datasheet V1.1 (2018), AHT20 datasheet V1.1 (English),
//! AHT21 / AHT25 / AM2315C datasheets V1.0 (May 2021), AHT30 datasheet
//! (Sept 2023).
//!
//! ```ignore
//! let mut aht = Aht::new(i2c, Variant::Aht20);
//! // ... 40 ms after power-on ...
//! if !Variant::Aht20.is_calibrated(aht.status()?) {
//!     let wait_us = aht.initialize()?;
//!     // ... sleep wait_us ...
//! }
//! let wait_us = aht.start_measurement()?;
//! // ... sleep wait_us ...
//! let m = aht.read_measurement()?;
//! ```

#![no_std]

use zigbee_sensor::crc::crc8_sensirion;
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address of all parts.
pub const DEFAULT_ADDRESS: u8 = 0x38;
/// AHT10 address with the ADR pin high.
pub const AHT10_ALTERNATE_ADDRESS: u8 = 0x39;

const CMD_TRIGGER: [u8; 3] = [0xAC, 0x33, 0x00];
const CMD_SOFT_RESET: [u8; 1] = [0xBA];
const CMD_INIT_AHT10: [u8; 3] = [0xE1, 0x08, 0x00];
const CMD_INIT_AHT20: [u8; 3] = [0xBE, 0x08, 0x00];

/// Status bit: measurement in progress.
pub const STATUS_BUSY: u8 = 1 << 7;
/// Status bit: calibration enabled.
pub const STATUS_CALIBRATED: u8 = 1 << 3;

/// Soft reset duration (≤ 20 ms).
pub const RESET_WAIT_US: WaitUs = 20_000;
/// Wait after the initialization command.
pub const INIT_WAIT_US: WaitUs = 10_000;

/// Supported parts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// AHT10: no CRC, init command 0xE1.
    Aht10,
    /// AHT20: CRC, init command 0xBE.
    Aht20,
    /// AHT21 / AHT25: CRC, register restore when status & 0x18 != 0x18.
    Aht21,
    /// AHT30: CRC, init command 0xBE.
    Aht30,
    /// AM2315C (encapsulated AHT21 core): CRC, register restore.
    Am2315c,
}

impl Variant {
    /// Time from power-on until the part accepts commands.
    pub const fn power_up_us(self) -> WaitUs {
        match self {
            Self::Aht10 => 20_000,
            Self::Aht20 => 40_000,
            Self::Aht30 => 5_000,
            Self::Aht21 | Self::Am2315c => 100_000,
        }
    }

    /// Conversion time after the trigger command.
    pub const fn measurement_us(self) -> WaitUs {
        match self {
            Self::Aht10 => 75_000,
            _ => 80_000,
        }
    }

    /// Whether the frame ends with a CRC byte.
    pub const fn has_crc(self) -> bool {
        !matches!(self, Self::Aht10)
    }

    /// Whether a status byte shows a usable calibration: bit 3 for
    /// AHT10 / AHT20 / AHT30, bits 4 and 3 for AHT21 / AHT25 / AM2315C.
    pub const fn is_calibrated(self, status: u8) -> bool {
        match self {
            Self::Aht21 | Self::Am2315c => status & 0x18 == 0x18,
            _ => status & STATUS_CALIBRATED != 0,
        }
    }

    const fn init_command(self) -> Option<[u8; 3]> {
        match self {
            Self::Aht10 => Some(CMD_INIT_AHT10),
            Self::Aht20 | Self::Aht30 => Some(CMD_INIT_AHT20),
            Self::Aht21 | Self::Am2315c => None,
        }
    }

    const fn has_soft_reset(self) -> bool {
        matches!(self, Self::Aht10 | Self::Aht20 | Self::Aht21)
    }
}

/// Registers rewritten by the AHT21 / AHT25 / AM2315C restore sequence.
pub const RESTORE_REGISTERS: [u8; 3] = [0x1B, 0x1C, 0x1E];
/// Wait between [`Aht::restore_register_begin`] and
/// [`Aht::restore_register_read`].
pub const RESTORE_READ_WAIT_US: WaitUs = 5_000;
/// Wait between [`Aht::restore_register_read`] and
/// [`Aht::restore_register_write`].
pub const RESTORE_WRITE_WAIT_US: WaitUs = 10_000;

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

/// 20-bit raw values (upper bits ignored) to engineering units:
/// RH = S_RH / 2^20 × 100, T = S_T / 2^20 × 200 − 50.
pub fn convert(raw_humidity: u32, raw_temperature: u32) -> Measurement {
    // 10000 / 2^20 = 625 / 2^16 and 20000 / 2^20 = 1250 / 2^16.
    let rh = ((raw_humidity & 0xF_FFFF) * 625 + 32_768) >> 16;
    let t = (((raw_temperature & 0xF_FFFF) * 1_250 + 32_768) >> 16) as i32 - 5_000;
    Measurement {
        temperature: t,
        humidity: rh as u16,
    }
}

/// Decode the status byte and 5 data bytes, plus the CRC byte when
/// `variant` has one (`frame[6]` is ignored otherwise). A set busy bit
/// returns [`Error::NotReady`].
pub fn decode_measurement<E>(variant: Variant, frame: &[u8; 7]) -> Result<Measurement, Error<E>> {
    if variant.has_crc() && crc8_sensirion(&frame[..6]) != frame[6] {
        return Err(Error::Crc);
    }
    if frame[0] & STATUS_BUSY != 0 {
        return Err(Error::NotReady);
    }
    let rh = (u32::from(frame[1]) << 12) | (u32::from(frame[2]) << 4) | u32::from(frame[3] >> 4);
    let t = (u32::from(frame[3] & 0x0F) << 16) | (u32::from(frame[4]) << 8) | u32::from(frame[5]);
    Ok(convert(rh, t))
}

const fn frame_len(variant: Variant) -> usize {
    if variant.has_crc() { 7 } else { 6 }
}

macro_rules! constructors {
    () => {
        /// Create a driver at [`DEFAULT_ADDRESS`](crate::DEFAULT_ADDRESS);
        /// no bus traffic.
        pub const fn new(i2c: I2C, variant: Variant) -> Self {
            Self {
                i2c,
                address: DEFAULT_ADDRESS,
                variant,
            }
        }

        /// Use another I2C address (AHT10 ADR pin high: 0x39).
        pub const fn with_address(mut self, address: u8) -> Self {
            self.address = address;
            self
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }
    };
}

/// Blocking driver.
pub struct Aht<I2C> {
    i2c: I2C,
    address: u8,
    variant: Variant,
}

impl<I2C> Aht<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Aht<I2C> {
    fn write(&mut self, bytes: &[u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(self.address, bytes).map_err(Error::bus)
    }

    /// Read the status byte (busy bit 7, calibration bit 3).
    pub fn status(&mut self) -> Result<u8, Error<I2C::Error>> {
        let mut status = [0u8; 1];
        self.i2c
            .read(self.address, &mut status)
            .map_err(Error::bus)?;
        Ok(status[0])
    }

    /// Send the calibration / initialization command (`0xE1 0x08 0x00` on
    /// AHT10, `0xBE 0x08 0x00` on AHT20 / AHT30). Wait [`INIT_WAIT_US`].
    /// AHT21 / AHT25 / AM2315C return [`Error::Unsupported`]; use the
    /// register restore sequence instead.
    pub fn initialize(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        let command = self.variant.init_command().ok_or(Error::Unsupported)?;
        self.write(&command)?;
        Ok(INIT_WAIT_US)
    }

    /// Soft reset (`0xBA`, AHT10 / AHT20 / AHT21 / AHT25). Wait
    /// [`RESET_WAIT_US`].
    pub fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        if !self.variant.has_soft_reset() {
            return Err(Error::Unsupported);
        }
        self.write(&CMD_SOFT_RESET)?;
        Ok(RESET_WAIT_US)
    }

    /// Restore step 1 for one of [`RESTORE_REGISTERS`] (AHT21 / AHT25 /
    /// AM2315C when status & 0x18 != 0x18). The datasheets only name the
    /// registers; the sequence follows Aosong's sample code. Wait
    /// [`RESTORE_READ_WAIT_US`].
    pub fn restore_register_begin(&mut self, register: u8) -> Result<WaitUs, Error<I2C::Error>> {
        self.write(&[register, 0x00, 0x00])?;
        Ok(RESTORE_READ_WAIT_US)
    }

    /// Restore step 2: read the register contents. Then wait
    /// [`RESTORE_WRITE_WAIT_US`].
    pub fn restore_register_read(&mut self) -> Result<[u8; 3], Error<I2C::Error>> {
        let mut value = [0u8; 3];
        self.i2c
            .read(self.address, &mut value)
            .map_err(Error::bus)?;
        Ok(value)
    }

    /// Restore step 3: write the contents back with `0xB0 | register`.
    pub fn restore_register_write(
        &mut self,
        register: u8,
        value: [u8; 3],
    ) -> Result<(), Error<I2C::Error>> {
        self.write(&[0xB0 | register, value[1], value[2]])
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Aht<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.write(&CMD_TRIGGER)?;
        Ok(self.variant.measurement_us())
    }

    /// Returns [`Error::NotReady`] while the busy bit is set.
    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        let mut frame = [0u8; 7];
        let len = frame_len(self.variant);
        self.i2c
            .read(self.address, &mut frame[..len])
            .map_err(Error::bus)?;
        decode_measurement(self.variant, &frame)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        CMD_SOFT_RESET, CMD_TRIGGER, DEFAULT_ADDRESS, INIT_WAIT_US, Measurement, RESET_WAIT_US,
        RESTORE_READ_WAIT_US, Variant, decode_measurement, frame_len,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Aht`].
    pub struct Aht<I2C> {
        i2c: I2C,
        address: u8,
        variant: Variant,
    }

    impl<I2C> Aht<I2C> {
        constructors!();
    }

    impl<I2C: I2c> Aht<I2C> {
        async fn write(&mut self, bytes: &[u8]) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write(self.address, bytes)
                .await
                .map_err(Error::bus)
        }

        /// See [`super::Aht::status`].
        pub async fn status(&mut self) -> Result<u8, Error<I2C::Error>> {
            let mut status = [0u8; 1];
            self.i2c
                .read(self.address, &mut status)
                .await
                .map_err(Error::bus)?;
            Ok(status[0])
        }

        /// See [`super::Aht::initialize`].
        pub async fn initialize(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            let command = self.variant.init_command().ok_or(Error::Unsupported)?;
            self.write(&command).await?;
            Ok(INIT_WAIT_US)
        }

        /// See [`super::Aht::soft_reset`].
        pub async fn soft_reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            if !self.variant.has_soft_reset() {
                return Err(Error::Unsupported);
            }
            self.write(&CMD_SOFT_RESET).await?;
            Ok(RESET_WAIT_US)
        }

        /// See [`super::Aht::restore_register_begin`].
        pub async fn restore_register_begin(
            &mut self,
            register: u8,
        ) -> Result<WaitUs, Error<I2C::Error>> {
            self.write(&[register, 0x00, 0x00]).await?;
            Ok(RESTORE_READ_WAIT_US)
        }

        /// See [`super::Aht::restore_register_read`].
        pub async fn restore_register_read(&mut self) -> Result<[u8; 3], Error<I2C::Error>> {
            let mut value = [0u8; 3];
            self.i2c
                .read(self.address, &mut value)
                .await
                .map_err(Error::bus)?;
            Ok(value)
        }

        /// See [`super::Aht::restore_register_write`].
        pub async fn restore_register_write(
            &mut self,
            register: u8,
            value: [u8; 3],
        ) -> Result<(), Error<I2C::Error>> {
            self.write(&[0xB0 | register, value[1], value[2]]).await
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Aht<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.write(&CMD_TRIGGER).await?;
            Ok(self.variant.measurement_us())
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let mut frame = [0u8; 7];
            let len = frame_len(self.variant);
            self.i2c
                .read(self.address, &mut frame[..len])
                .await
                .map_err(Error::bus)?;
            decode_measurement(self.variant, &frame)
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

    // Status 0x1C, S_RH = 0x80000 (50 %), S_T = 0x2FFAB, CRC 0xF0.
    const FRAME: [u8; 7] = [0x1C, 0x80, 0x00, 0x02, 0xFF, 0xAB, 0xF0];
    const EXPECTED: Measurement = Measurement {
        temperature: -1_252,
        humidity: 5_000,
    };

    #[test]
    fn conversion_matches_datasheet_example() {
        // AHT20 §6: S_T = 0x2FFAB → 196523 / 2^20 × 200 − 50 = −12.52 °C.
        assert_eq!(convert(0, 0x2FFAB).temperature, -1_252);
        assert_eq!(
            convert(0xFFFFF, 0xFFFFF),
            Measurement {
                temperature: 15_000,
                humidity: 10_000
            }
        );
        assert_eq!(convert(0, 0).temperature, -5_000);
        assert_eq!(
            decode_measurement::<()>(Variant::Aht20, &FRAME),
            Ok(EXPECTED)
        );
    }

    #[test]
    fn crc_busy_and_calibration() {
        let mut bad = FRAME;
        bad[6] ^= 1;
        assert_eq!(
            decode_measurement::<()>(Variant::Aht21, &bad),
            Err(Error::Crc)
        );
        // AHT10 has no CRC byte.
        assert_eq!(decode_measurement::<()>(Variant::Aht10, &bad), Ok(EXPECTED));
        let mut busy = FRAME;
        busy[0] = 0x9C;
        busy[6] = 0x1C;
        assert_eq!(
            decode_measurement::<()>(Variant::Aht20, &busy),
            Err(Error::NotReady)
        );
        assert!(Variant::Aht20.is_calibrated(0x08));
        assert!(!Variant::Aht21.is_calibrated(0x08));
        assert!(Variant::Am2315c.is_calibrated(0x18));
    }

    #[test]
    fn init_and_measure_aht20() {
        let expectations = [
            Transaction::read(DEFAULT_ADDRESS, vec![0x10]),
            Transaction::write(DEFAULT_ADDRESS, vec![0xBE, 0x08, 0x00]),
            Transaction::write(DEFAULT_ADDRESS, vec![0xAC, 0x33, 0x00]),
            Transaction::read(DEFAULT_ADDRESS, busy_frame()),
            Transaction::read(DEFAULT_ADDRESS, FRAME.to_vec()),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut aht = Aht::new(&mut i2c, Variant::Aht20);
        assert!(!Variant::Aht20.is_calibrated(aht.status().unwrap()));
        assert_eq!(aht.initialize(), Ok(INIT_WAIT_US));
        assert_eq!(aht.start_measurement(), Ok(80_000));
        assert_eq!(aht.read_measurement(), Err(Error::NotReady));
        assert_eq!(aht.read_measurement(), Ok(EXPECTED));
        i2c.done();
    }

    fn busy_frame() -> std::vec::Vec<u8> {
        vec![0x9C, 0x80, 0x00, 0x02, 0xFF, 0xAB, 0x1C]
    }

    #[test]
    fn aht10_and_am2315c_specifics() {
        let expectations = [
            Transaction::write(AHT10_ALTERNATE_ADDRESS, vec![0xE1, 0x08, 0x00]),
            Transaction::write(AHT10_ALTERNATE_ADDRESS, vec![0xBA]),
            Transaction::write(AHT10_ALTERNATE_ADDRESS, vec![0xAC, 0x33, 0x00]),
            Transaction::read(AHT10_ALTERNATE_ADDRESS, FRAME[..6].to_vec()),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut aht = Aht::new(&mut i2c, Variant::Aht10).with_address(AHT10_ALTERNATE_ADDRESS);
        aht.initialize().unwrap();
        assert_eq!(aht.soft_reset(), Ok(RESET_WAIT_US));
        assert_eq!(aht.start_measurement(), Ok(75_000));
        assert_eq!(aht.read_measurement(), Ok(EXPECTED));
        i2c.done();

        let expectations = [
            Transaction::write(DEFAULT_ADDRESS, vec![0x1B, 0x00, 0x00]),
            Transaction::read(DEFAULT_ADDRESS, vec![0x1C, 0x12, 0x34]),
            Transaction::write(DEFAULT_ADDRESS, vec![0xBB, 0x12, 0x34]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut aht = Aht::new(&mut i2c, Variant::Am2315c);
        assert_eq!(aht.initialize(), Err(Error::Unsupported));
        assert_eq!(aht.soft_reset(), Err(Error::Unsupported));
        assert_eq!(aht.restore_register_begin(RESTORE_REGISTERS[0]), Ok(5_000));
        let value = aht.restore_register_read().unwrap();
        aht.restore_register_write(RESTORE_REGISTERS[0], value)
            .unwrap();
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
            Transaction::write(DEFAULT_ADDRESS, vec![0xAC, 0x33, 0x00]),
            Transaction::read(DEFAULT_ADDRESS, FRAME.to_vec()),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut aht = asynch::Aht::new(&mut i2c, Variant::Aht30);
        let mut delay = RecordingDelay(0);
        assert_eq!(block_on(aht.measure(&mut delay)), Ok(EXPECTED));
        assert_eq!(delay.0, 80_000);
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
