//! Sensirion SPS30 particulate matter sensor over I2C (0x69), using the
//! unsigned 16-bit integer output format (firmware ≥ 2.0): PM1.0 / PM2.5 /
//! PM4.0 / PM10 mass concentration, number concentrations, typical particle
//! size; data-ready flag, sleep / wake-up (`LowPower`, firmware ≥ 2.0), fan
//! cleaning and its interval, device status register, product type
//! (`probe`), serial number, firmware version and reset.
//!
//! # Measurement through the [`Sensor`](zigbee_sensor::blocking::Sensor) trait
//!
//! The first `start_measurement` enters Measurement-Mode and returns 1 s
//! (the sampling interval); later calls return 0 without bus traffic since
//! the sensor is free-running. `read_measurement` reads the data-ready
//! flag and returns [`Error::NotReady`] until a new sample is available.
//! Readings stabilise 8–30 s after the start depending on the particle
//! concentration (datasheet table 1).
//!
//! The integer format has a resolution of 1 µg/m³; the driver reports mass
//! in 0.1 µg/m³ (the [`Quantity::Pm25`] unit), so the last digit is 0.
//!
//! Reference: Sensirion "Datasheet SPS30", version 2.0 (June 2023, D1).

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address.
pub const ADDRESS: u8 = 0x69;

mod cmd {
    pub const START_MEASUREMENT: u16 = 0x0010;
    pub const STOP_MEASUREMENT: u16 = 0x0104;
    pub const READ_DATA_READY: u16 = 0x0202;
    pub const READ_MEASURED_VALUES: u16 = 0x0300;
    pub const SLEEP: u16 = 0x1001;
    pub const WAKE_UP: u16 = 0x1103;
    pub const START_FAN_CLEANING: u16 = 0x5607;
    pub const AUTO_CLEANING_INTERVAL: u16 = 0x8004;
    pub const PRODUCT_TYPE: u16 = 0xD002;
    pub const SERIAL_NUMBER: u16 = 0xD033;
    pub const FIRMWARE_VERSION: u16 = 0xD100;
    pub const READ_DEVICE_STATUS: u16 = 0xD206;
    pub const CLEAR_DEVICE_STATUS: u16 = 0xD210;
    pub const RESET: u16 = 0xD304;
}

/// `Start Measurement` argument: big-endian unsigned 16-bit integers
/// (format byte 0x05, dummy byte 0x00).
const FORMAT_UINT16: u16 = 0x0500;

/// Start / stop measurement and interval write execution time.
pub const COMMAND_WAIT_US: WaitUs = 20_000;
/// Sleep, wake-up, fan cleaning and clear-status execution time.
pub const SHORT_COMMAND_WAIT_US: WaitUs = 5_000;
/// Device reset execution time.
pub const RESET_WAIT_US: WaitUs = 100_000;
/// Sampling interval in Measurement-Mode.
pub const SAMPLE_INTERVAL_US: WaitUs = 1_000_000;
/// Product type string returned by `Read Product Type`.
pub const PRODUCT_TYPE: [u8; 8] = *b"00080000";

/// Device status register: fan speed out of range.
pub const STATUS_FAN_SPEED: u32 = 1 << 21;
/// Device status register: laser current out of range.
pub const STATUS_LASER: u32 = 1 << 5;
/// Device status register: fan blocked or broken (not self-clearing).
pub const STATUS_FAN: u32 = 1 << 4;

/// One `Read Measured Values` result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// PM1.0 mass concentration, 0.1 µg/m³.
    pub pm1_0: u32,
    /// PM2.5 mass concentration, 0.1 µg/m³.
    pub pm2_5: u32,
    /// PM4.0 mass concentration, 0.1 µg/m³.
    pub pm4_0: u32,
    /// PM10 mass concentration, 0.1 µg/m³.
    pub pm10: u32,
    /// Number concentration of PM0.5, PM1.0, PM2.5, PM4.0, PM10 in #/cm³.
    pub number: [u16; 5],
    /// Typical particle size in nm.
    pub typical_size_nm: u16,
}

impl Readings for Measurement {
    /// Mass concentrations only; number concentrations and particle size
    /// have no [`Quantity`] yet.
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Pm1(self.pm1_0));
        f(Quantity::Pm25(self.pm2_5));
        f(Quantity::Pm4(self.pm4_0));
        f(Quantity::Pm10(self.pm10));
    }
}

fn convert(w: &[u16; 10]) -> Measurement {
    let mass = |i: usize| u32::from(w[i]) * 10;
    Measurement {
        pm1_0: mass(0),
        pm2_5: mass(1),
        pm4_0: mass(2),
        pm10: mass(3),
        number: [w[4], w[5], w[6], w[7], w[8]],
        typical_size_nm: w[9],
    }
}

fn u32_from_words(w: [u16; 2]) -> u32 {
    (u32::from(w[0]) << 16) | u32::from(w[1])
}

macro_rules! constructors {
    () => {
        /// Create a driver; no bus traffic.
        pub const fn new(i2c: I2C) -> Self {
            Self {
                bus: Interface::new(i2c, ADDRESS),
                running: false,
            }
        }

        /// `true` while the driver has the sensor in Measurement-Mode.
        pub const fn is_running(&self) -> bool {
            self.running
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.bus.release()
        }
    };
}

// Bus methods shared by the blocking and async drivers: `$async` is empty or
// `async`, `$aw` is empty or `.await`. The SPS30 needs no delay between
// setting a pointer and reading it (datasheet table 8).
macro_rules! methods {
    ([$($async:tt)*] [$($aw:tt)*]) => {
        $($async)* fn send(&mut self, cmd: u16, args: &[u16], wait: WaitUs) -> Result<WaitUs, Error<I2C::Error>> {
            self.bus.write(cmd, args) $($aw)* ?;
            Ok(wait)
        }

        $($async)* fn get<const N: usize>(&mut self, cmd: u16) -> Result<[u16; N], Error<I2C::Error>> {
            self.bus.write(cmd, &[]) $($aw)* ?;
            let mut w = [0u16; N];
            self.bus.read(&mut w) $($aw)* .map_err(nack_as_not_ready)?;
            Ok(w)
        }

        /// Product type; must be `"00080000"`, else
        /// [`Error::UnexpectedId`] with the first four characters.
        pub $($async)* fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
            let product = self.product_type() $($aw)* ?;
            if product != PRODUCT_TYPE {
                let [a, b, c, d, ..] = product;
                return Err(Error::UnexpectedId(u32::from_be_bytes([a, b, c, d])));
            }
            Ok(())
        }

        /// Product type string (8 ASCII characters).
        pub $($async)* fn product_type(&mut self) -> Result<[u8; 8], Error<I2C::Error>> {
            let w: [u16; 4] = self.get(cmd::PRODUCT_TYPE) $($aw)* ?;
            let mut out = [0u8; 8];
            words_to_bytes(&w, &mut out);
            Ok(out)
        }

        /// Serial number: NUL-terminated ASCII, up to 32 bytes.
        pub $($async)* fn serial_number(&mut self) -> Result<[u8; 32], Error<I2C::Error>> {
            let w: [u16; 16] = self.get(cmd::SERIAL_NUMBER) $($aw)* ?;
            let mut out = [0u8; 32];
            words_to_bytes(&w, &mut out);
            Ok(out)
        }

        /// Firmware `(major, minor)`; sleep and the integer format need ≥ 2.0.
        pub $($async)* fn firmware_version(&mut self) -> Result<(u8, u8), Error<I2C::Error>> {
            let [w]: [u16; 1] = self.get(cmd::FIRMWARE_VERSION) $($aw)* ?;
            let [major, minor] = w.to_be_bytes();
            Ok((major, minor))
        }

        /// Enter Measurement-Mode with the integer output format (from
        /// Idle only). Returns the 1 s sampling interval.
        pub $($async)* fn start_continuous_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::START_MEASUREMENT, &[FORMAT_UINT16], 0) $($aw)* ?;
            self.running = true;
            Ok(SAMPLE_INTERVAL_US)
        }

        /// Return to Idle-Mode (20 ms).
        pub $($async)* fn stop_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::STOP_MEASUREMENT, &[], COMMAND_WAIT_US) $($aw)* ?;
            self.running = false;
            Ok(COMMAND_WAIT_US)
        }

        /// `true` if a new sample is available.
        pub $($async)* fn read_data_ready(&mut self) -> Result<bool, Error<I2C::Error>> {
            let [w]: [u16; 1] = self.get(cmd::READ_DATA_READY) $($aw)* ?;
            Ok(w & 0x00FF == 1)
        }

        /// Latest values regardless of the data-ready flag (which this
        /// clears).
        pub $($async)* fn read_measured_values(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let w = self.get(cmd::READ_MEASURED_VALUES) $($aw)* ?;
            Ok(convert(&w))
        }

        /// Start a manual 10 s fan cleaning (Measurement-Mode only; values
        /// are not updated meanwhile).
        pub $($async)* fn start_fan_cleaning(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::START_FAN_CLEANING, &[], SHORT_COMMAND_WAIT_US) $($aw)*
        }

        /// Automatic fan-cleaning interval in seconds (default 604 800).
        pub $($async)* fn auto_cleaning_interval(&mut self) -> Result<u32, Error<I2C::Error>> {
            let w = self.get(cmd::AUTO_CLEANING_INTERVAL) $($aw)* ?;
            Ok(u32_from_words(w))
        }

        /// Set the automatic fan-cleaning interval in seconds (0 disables;
        /// non-volatile). Firmware < 2.2 reads back the old value until the
        /// next reset.
        pub $($async)* fn set_auto_cleaning_interval(&mut self, seconds: u32) -> Result<WaitUs, Error<I2C::Error>> {
            let args = [(seconds >> 16) as u16, seconds as u16];
            self.send(cmd::AUTO_CLEANING_INTERVAL, &args, COMMAND_WAIT_US) $($aw)*
        }

        /// Device status register (see the `STATUS_*` masks; firmware ≥ 2.2).
        pub $($async)* fn device_status(&mut self) -> Result<u32, Error<I2C::Error>> {
            let w = self.get(cmd::READ_DEVICE_STATUS) $($aw)* ?;
            Ok(u32_from_words(w))
        }

        /// Clear the device status register.
        pub $($async)* fn clear_device_status(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::CLEAR_DEVICE_STATUS, &[], SHORT_COMMAND_WAIT_US) $($aw)*
        }

        /// Software reset (100 ms; state as after power-up). From Sleep-Mode,
        /// wake the sensor first.
        pub $($async)* fn reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.send(cmd::RESET, &[], RESET_WAIT_US) $($aw)* ?;
            self.running = false;
            Ok(RESET_WAIT_US)
        }

        /// Enter Sleep-Mode (< 50 µA, I2C disabled; firmware ≥ 2.0). Only
        /// from Idle: while measuring this returns [`Error::NotReady`]
        /// without bus traffic — call `stop_measurement` and wait 20 ms.
        pub $($async)* fn enter_sleep(&mut self) -> Result<(), Error<I2C::Error>> {
            if self.running {
                return Err(Error::NotReady);
            }
            self.bus.write(cmd::SLEEP, &[]) $($aw)*
        }

        /// Wake-up: the command is sent twice — the first only activates
        /// the disabled interface and is not acknowledged (section 6.3.6).
        pub $($async)* fn wake_up(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            ignore_nack(self.bus.write(cmd::WAKE_UP, &[]) $($aw)*)?;
            self.bus.write(cmd::WAKE_UP, &[]) $($aw)* ?;
            Ok(SHORT_COMMAND_WAIT_US)
        }

        $($async)* fn start(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            if self.running {
                return Ok(0);
            }
            self.start_continuous_measurement() $($aw)*
        }

        $($async)* fn read(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            if !self.running || !self.read_data_ready() $($aw)* ? {
                return Err(Error::NotReady);
            }
            self.read_measured_values() $($aw)*
        }
    };
}

macro_rules! trait_impls {
    ($Sensor:path, $LowPower:path, $I2c:path, [$($async:tt)*] [$($aw:tt)*]) => {
        impl<I2C: $I2c> $Sensor for Sps30<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// Enters Measurement-Mode on the first call (returns 1 s),
            /// then returns 0 without bus traffic.
            $($async)* fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.start() $($aw)*
            }

            /// [`Error::NotReady`] if not measuring or no new sample.
            $($async)* fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                self.read() $($aw)*
            }
        }

        impl<I2C: $I2c> $LowPower for Sps30<I2C> {
            type BusError = I2C::Error;

            /// Sleep-Mode (< 50 µA); see `enter_sleep` for the Idle-only rule.
            $($async)* fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.enter_sleep() $($aw)*
            }

            /// Double wake-up command; 5 ms.
            $($async)* fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.wake_up() $($aw)*
            }
        }
    };
}

use zigbee_sensirion_common::blocking::Interface;
use zigbee_sensirion_common::{ignore_nack, nack_as_not_ready, words_to_bytes};

/// Blocking driver.
#[derive(Debug)]
pub struct Sps30<I2C> {
    bus: Interface<I2C>,
    running: bool,
}

impl<I2C> Sps30<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Sps30<I2C> {
    methods!([] []);
}

trait_impls!(
    zigbee_sensor::blocking::Sensor,
    zigbee_sensor::blocking::LowPower,
    embedded_hal::i2c::I2c,
    [] []
);

/// Async driver.
pub mod asynch {
    use super::{
        ADDRESS, COMMAND_WAIT_US, FORMAT_UINT16, Measurement, PRODUCT_TYPE, RESET_WAIT_US,
        SAMPLE_INTERVAL_US, SHORT_COMMAND_WAIT_US, cmd, convert, u32_from_words,
    };
    use zigbee_sensirion_common::asynch::Interface;
    use zigbee_sensirion_common::{ignore_nack, nack_as_not_ready, words_to_bytes};
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Sps30`].
    #[derive(Debug)]
    pub struct Sps30<I2C> {
        bus: Interface<I2C>,
        running: bool,
    }

    impl<I2C> Sps30<I2C> {
        constructors!();
    }

    impl<I2C: embedded_hal_async::i2c::I2c> Sps30<I2C> {
        methods!([async] [.await]);
    }

    trait_impls!(
        zigbee_sensor::asynch::Sensor,
        zigbee_sensor::asynch::LowPower,
        embedded_hal_async::i2c::I2c,
        [async] [.await]
    );
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};
    use embedded_hal_mock::eh1::delay::{CheckedDelay, Transaction as D};
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use std::vec;
    use std::vec::Vec;
    use zigbee_sensirion_common::encode_word;
    use zigbee_sensor::blocking::{LowPower, Sensor};

    const SAMPLE: [u16; 10] = [3, 5, 6, 7, 20, 24, 25, 25, 25, 480];

    fn words(w: &[u16]) -> Vec<u8> {
        w.iter().flat_map(|&x| encode_word(x)).collect()
    }

    fn nack(t: Transaction) -> Transaction {
        t.with_error(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address))
    }

    #[test]
    fn measurement_cycle() {
        let expectations = [
            // 0x0500 → CRC 0xF6 (format 0x05, dummy 0x00).
            Transaction::write(ADDRESS, vec![0x00, 0x10, 0x05, 0x00, 0xF6]),
            Transaction::write(ADDRESS, vec![0x02, 0x02]),
            Transaction::read(ADDRESS, words(&[0x0000])),
            Transaction::write(ADDRESS, vec![0x02, 0x02]),
            Transaction::read(ADDRESS, words(&[0x0001])),
            Transaction::write(ADDRESS, vec![0x03, 0x00]),
            Transaction::read(ADDRESS, words(&SAMPLE)),
            Transaction::write(ADDRESS, vec![0x01, 0x04]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut sps = Sps30::new(&mut i2c);
        assert_eq!(sps.read_measurement(), Err(Error::NotReady));
        assert_eq!(sps.start_measurement().unwrap(), 1_000_000);
        assert_eq!(sps.start_measurement().unwrap(), 0);
        assert_eq!(sps.read_measurement(), Err(Error::NotReady));
        let m = sps.read_measurement().unwrap();
        assert_eq!(
            m,
            Measurement {
                pm1_0: 30,
                pm2_5: 50,
                pm4_0: 60,
                pm10: 70,
                number: [20, 24, 25, 25, 25],
                typical_size_nm: 480,
            }
        );
        let mut seen = Vec::new();
        m.for_each(&mut |q| seen.push(q));
        assert_eq!(
            seen,
            [
                Quantity::Pm1(30),
                Quantity::Pm25(50),
                Quantity::Pm4(60),
                Quantity::Pm10(70)
            ]
        );
        assert_eq!(sps.stop_measurement().unwrap(), 20_000);
        i2c.done();
    }

    #[test]
    fn device_info_and_status() {
        let mut serial = words(&[0x3030; 16]);
        let n = serial.len();
        serial[n - 3..].copy_from_slice(&encode_word(0x3000));
        let expectations = [
            Transaction::write(ADDRESS, vec![0xD0, 0x02]),
            Transaction::read(ADDRESS, words(&[0x3030, 0x3038, 0x3030, 0x3030])),
            Transaction::write(ADDRESS, vec![0xD0, 0x02]),
            Transaction::read(ADDRESS, words(&[0x3030, 0x3030, 0x3030, 0x3030])),
            Transaction::write(ADDRESS, vec![0xD0, 0x33]),
            Transaction::read(ADDRESS, serial),
            Transaction::write(ADDRESS, vec![0xD1, 0x00]),
            Transaction::read(ADDRESS, words(&[0x0201])),
            Transaction::write(ADDRESS, vec![0xD2, 0x06]),
            Transaction::read(ADDRESS, words(&[0x0020, 0x0010])),
            Transaction::write(ADDRESS, vec![0xD2, 0x10]),
            Transaction::write(ADDRESS, vec![0x80, 0x04]),
            Transaction::read(ADDRESS, words(&[0x0009, 0x3A80])),
            // 345 600 s = 0x0005_4600.
            Transaction::write(
                ADDRESS,
                vec![0x80, 0x04, 0x00, 0x05, 0x74, 0x46, 0x00, 0x52],
            ),
            Transaction::write(ADDRESS, vec![0xD3, 0x04]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut sps = Sps30::new(&mut i2c);
        sps.probe().unwrap();
        assert_eq!(sps.probe(), Err(Error::UnexpectedId(0x3030_3030)));
        let s = sps.serial_number().unwrap();
        assert_eq!(&s[..31], &[b'0'; 31]);
        assert_eq!(s[31], 0);
        assert_eq!(sps.firmware_version().unwrap(), (2, 1));
        let status = sps.device_status().unwrap();
        assert_eq!(status, STATUS_FAN_SPEED | STATUS_FAN);
        assert_eq!(status & STATUS_LASER, 0);
        sps.clear_device_status().unwrap();
        assert_eq!(sps.auto_cleaning_interval().unwrap(), 604_800);
        assert_eq!(sps.set_auto_cleaning_interval(345_600).unwrap(), 20_000);
        assert_eq!(sps.reset().unwrap(), 100_000);
        i2c.done();
    }

    #[test]
    fn sleep_wake_and_crc() {
        let mut bad = words(&[1]);
        bad[1] ^= 1;
        let expectations = [
            Transaction::write(ADDRESS, vec![0x00, 0x10, 0x05, 0x00, 0xF6]),
            Transaction::write(ADDRESS, vec![0x56, 0x07]),
            Transaction::write(ADDRESS, vec![0x02, 0x02]),
            Transaction::read(ADDRESS, bad),
            Transaction::write(ADDRESS, vec![0x02, 0x02]),
            nack(Transaction::read(ADDRESS, vec![0; 3])),
            Transaction::write(ADDRESS, vec![0x01, 0x04]),
            Transaction::write(ADDRESS, vec![0x10, 0x01]),
            nack(Transaction::write(ADDRESS, vec![0x11, 0x03])),
            Transaction::write(ADDRESS, vec![0x11, 0x03]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut sps = Sps30::new(&mut i2c);
        sps.start_measurement().unwrap();
        sps.start_fan_cleaning().unwrap();
        assert_eq!(sps.read_measurement(), Err(Error::Crc));
        assert_eq!(sps.read_measurement(), Err(Error::NotReady));
        // Sleep is refused while measuring.
        assert_eq!(sps.sleep(), Err(Error::NotReady));
        sps.stop_measurement().unwrap();
        sps.sleep().unwrap();
        assert_eq!(sps.wake().unwrap(), 5_000);
        i2c.done();
    }

    #[test]
    fn async_measure() {
        use zigbee_sensor::asynch::{LowPower as _, Sensor as _};

        let expectations = [
            Transaction::write(ADDRESS, vec![0x00, 0x10, 0x05, 0x00, 0xF6]),
            Transaction::write(ADDRESS, vec![0x02, 0x02]),
            Transaction::read(ADDRESS, words(&[0x0001])),
            Transaction::write(ADDRESS, vec![0x03, 0x00]),
            Transaction::read(ADDRESS, words(&SAMPLE)),
            Transaction::write(ADDRESS, vec![0x11, 0x03]),
            Transaction::write(ADDRESS, vec![0x11, 0x03]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut delay = CheckedDelay::new(&[D::async_delay_us(1_000_000)]);
        let mut sps = asynch::Sps30::new(&mut i2c);
        block_on(async {
            let m = sps.measure(&mut delay).await.unwrap();
            assert_eq!(m.pm2_5, 50);
            assert_eq!(m.typical_size_nm, 480);
            assert_eq!(sps.wake().await.unwrap(), 5_000);
        });
        delay.done();
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
