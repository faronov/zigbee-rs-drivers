//! ScioSense ENS210 temperature and relative humidity sensor (I2C).
//!
//! The part idles in standby (40 nA) with automatic low power: writing
//! SENS_START boots the measurement engine, runs a single-shot
//! temperature + humidity conversion (≤ 130 ms including booting) and
//! falls back to standby. Each result register carries a valid flag and a
//! CRC-7 over the flag and the 16-bit data.
//!
//! PART_ID, DIE_REV and UID are only readable while the part is active,
//! so [`Ens210::device_info`] briefly disables low power and needs a
//! delay provider.
//!
//! Reference: ScioSense SC-000897-DS-7 "ENS210 Datasheet", Revision 5
//! (2022-05-11).
//!
//! ```ignore
//! let mut ens = Ens210::new(i2c);
//! ens.verify_id(&mut delay)?;
//! let wait_us = ens.start_measurement()?;
//! // ... sleep wait_us ...
//! let m = ens.read_measurement()?;
//! ```

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Fixed I2C address.
pub const ADDRESS: u8 = 0x43;
/// PART_ID value.
pub const PART_ID: u16 = 0x0210;

const REG_PART_ID: u8 = 0x00;
const REG_SYS_CTRL: u8 = 0x10;
const REG_SYS_STAT: u8 = 0x11;
const REG_SENS_RUN: u8 = 0x21;
const REG_T_VAL: u8 = 0x30;

const SYS_CTRL_RESET: u8 = 1 << 7;
const SYS_CTRL_LOW_POWER: u8 = 1 << 0;
const SYS_STAT_ACTIVE: u8 = 1 << 0;
const SENS_START_T_AND_H: u8 = 0b11;

/// Maximum booting time (standby → active), t_booting.
pub const BOOT_WAIT_US: WaitUs = 1_200;
/// Maximum single-shot T + RH conversion time, booting included.
pub const CONVERSION_WAIT_US: WaitUs = 130_000;

/// CRC-7 (x^7 + x^3 + 1, initial vector 0x7F) over a 17-bit payload
/// (valid flag followed by the 16 data bits), as in the datasheet's
/// `crc7()` listing.
pub const fn crc7(payload: u32) -> u8 {
    const WIDTH: u32 = 7;
    const DATA_WIDTH: u32 = 17;
    let mut val = ((payload & 0x1_FFFF) << WIDTH) | 0x7F;
    let mut bit = 1u32 << (DATA_WIDTH - 1 + WIDTH);
    let mut pol = 0x89u32 << (DATA_WIDTH - 1);
    while bit & (0x1_FFFF << WIDTH) != 0 {
        if bit & val != 0 {
            val ^= pol;
        }
        bit >>= 1;
        pol >>= 1;
    }
    val as u8
}

/// Check a 3-byte T_VAL / H_VAL register (little endian) and return the
/// 16-bit data: [`Error::Crc`] on a bad checksum, [`Error::NotReady`] if
/// the valid flag is clear.
pub fn decode_value<E>(bytes: &[u8; 3]) -> Result<u16, Error<E>> {
    let val = u32::from(bytes[0]) | u32::from(bytes[1]) << 8 | u32::from(bytes[2]) << 16;
    if crc7(val) != (val >> 17) as u8 {
        return Err(Error::Crc);
    }
    if val & (1 << 16) == 0 {
        return Err(Error::NotReady);
    }
    Ok(val as u16)
}

/// T_DATA (1/64 K) → 0.01 °C.
pub const fn temperature_centi_celsius(raw: u16) -> i32 {
    ((raw as i32) * 25 + 8) / 16 - 27_315
}

/// H_DATA (1/512 %RH) → 0.01 %RH, clipped to 100 %RH.
pub const fn humidity_centi_percent(raw: u16) -> u16 {
    let rh = ((raw as u32) * 25 + 64) / 128;
    if rh > 10_000 { 10_000 } else { rh as u16 }
}

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

/// Identification registers (active state only).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeviceInfo {
    /// PART_ID, 0x0210 for the ENS210.
    pub part_id: u16,
    /// DIE_REV.
    pub die_rev: u16,
    /// Unique device ID.
    pub uid: u64,
}

fn decode_measurement<E>(buf: &[u8; 6]) -> Result<Measurement, Error<E>> {
    let t = decode_value(&[buf[0], buf[1], buf[2]])?;
    let h = decode_value(&[buf[3], buf[4], buf[5]])?;
    Ok(Measurement {
        temperature: temperature_centi_celsius(t),
        humidity: humidity_centi_percent(h),
    })
}

fn decode_device_info(buf: &[u8; 12]) -> DeviceInfo {
    let mut uid = [0u8; 8];
    uid.copy_from_slice(&buf[4..12]);
    DeviceInfo {
        part_id: u16::from_le_bytes([buf[0], buf[1]]),
        die_rev: u16::from_le_bytes([buf[2], buf[3]]),
        uid: u64::from_le_bytes(uid),
    }
}

fn check_part_id<E>(info: DeviceInfo) -> Result<DeviceInfo, Error<E>> {
    if info.part_id == PART_ID {
        Ok(info)
    } else {
        Err(Error::UnexpectedId(u32::from(info.part_id)))
    }
}

const fn sys_ctrl(low_power: bool) -> u8 {
    if low_power { SYS_CTRL_LOW_POWER } else { 0 }
}

// SENS_RUN = 0 (single shot) and SENS_START = T + H in one auto-increment
// write.
const START_FRAME: [u8; 3] = [REG_SENS_RUN, 0x00, SENS_START_T_AND_H];

macro_rules! constructors {
    () => {
        /// Create a driver at [`ADDRESS`](crate::ADDRESS); no bus traffic.
        pub const fn new(i2c: I2C) -> Self {
            Self {
                i2c,
                low_power: true,
            }
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }
    };
}

/// Blocking driver.
pub struct Ens210<I2C> {
    i2c: I2C,
    low_power: bool,
}

impl<I2C> Ens210<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Ens210<I2C> {
    fn write(&mut self, bytes: &[u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(ADDRESS, bytes).map_err(Error::bus)
    }

    fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write_read(ADDRESS, &[reg], buf)
            .map_err(Error::bus)
    }

    /// Read PART_ID, DIE_REV and UID. Disables low power for
    /// [`BOOT_WAIT_US`] on `delay`, then restores it.
    pub fn device_info<D: embedded_hal::delay::DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<DeviceInfo, Error<I2C::Error>> {
        if self.low_power {
            self.write(&[REG_SYS_CTRL, sys_ctrl(false)])?;
            delay.delay_us(BOOT_WAIT_US);
        }
        let result = self.read_ids();
        if self.low_power {
            self.write(&[REG_SYS_CTRL, sys_ctrl(true)])?;
        }
        result
    }

    fn read_ids(&mut self) -> Result<DeviceInfo, Error<I2C::Error>> {
        let mut stat = [0u8; 1];
        self.read(REG_SYS_STAT, &mut stat)?;
        if stat[0] & SYS_STAT_ACTIVE == 0 {
            return Err(Error::NotReady);
        }
        let mut buf = [0u8; 12];
        self.read(REG_PART_ID, &mut buf)?;
        Ok(decode_device_info(&buf))
    }

    /// Check PART_ID (0x0210); see [`Ens210::device_info`].
    pub fn verify_id<D: embedded_hal::delay::DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<DeviceInfo, Error<I2C::Error>> {
        let info = self.device_info(delay)?;
        check_part_id(info)
    }

    /// Software reset (low power enabled afterwards). The datasheet gives
    /// no reset time; the returned wait is the booting time.
    pub fn reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.write(&[REG_SYS_CTRL, SYS_CTRL_RESET])?;
        self.low_power = true;
        Ok(BOOT_WAIT_US)
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Ens210<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    /// Start a single-shot T + RH measurement.
    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.write(&START_FRAME)?;
        Ok(CONVERSION_WAIT_US)
    }

    /// [`Error::NotReady`] while a valid flag is still clear.
    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        let mut buf = [0u8; 6];
        self.read(REG_T_VAL, &mut buf)?;
        decode_measurement(&buf)
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::LowPower for Ens210<I2C> {
    type BusError = I2C::Error;

    /// Re-enable automatic low power: standby (40 nA) whenever no
    /// measurement runs. This is the power-on default.
    fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write(&[REG_SYS_CTRL, sys_ctrl(true)])?;
        self.low_power = true;
        Ok(())
    }

    /// Disable low power so the part stays active; returns the booting
    /// time. Measurements work in either state.
    fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.write(&[REG_SYS_CTRL, sys_ctrl(false)])?;
        self.low_power = false;
        Ok(BOOT_WAIT_US)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        ADDRESS, BOOT_WAIT_US, CONVERSION_WAIT_US, DeviceInfo, Measurement, REG_PART_ID,
        REG_SYS_CTRL, REG_SYS_STAT, REG_T_VAL, START_FRAME, SYS_CTRL_RESET, SYS_STAT_ACTIVE,
        check_part_id, decode_device_info, decode_measurement, sys_ctrl,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Ens210`].
    pub struct Ens210<I2C> {
        i2c: I2C,
        low_power: bool,
    }

    impl<I2C> Ens210<I2C> {
        constructors!();
    }

    impl<I2C: I2c> Ens210<I2C> {
        async fn write(&mut self, bytes: &[u8]) -> Result<(), Error<I2C::Error>> {
            self.i2c.write(ADDRESS, bytes).await.map_err(Error::bus)
        }

        async fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write_read(ADDRESS, &[reg], buf)
                .await
                .map_err(Error::bus)
        }

        /// See [`super::Ens210::device_info`].
        pub async fn device_info<D: embedded_hal_async::delay::DelayNs>(
            &mut self,
            delay: &mut D,
        ) -> Result<DeviceInfo, Error<I2C::Error>> {
            if self.low_power {
                self.write(&[REG_SYS_CTRL, sys_ctrl(false)]).await?;
                delay.delay_us(BOOT_WAIT_US).await;
            }
            let result = self.read_ids().await;
            if self.low_power {
                self.write(&[REG_SYS_CTRL, sys_ctrl(true)]).await?;
            }
            result
        }

        async fn read_ids(&mut self) -> Result<DeviceInfo, Error<I2C::Error>> {
            let mut stat = [0u8; 1];
            self.read(REG_SYS_STAT, &mut stat).await?;
            if stat[0] & SYS_STAT_ACTIVE == 0 {
                return Err(Error::NotReady);
            }
            let mut buf = [0u8; 12];
            self.read(REG_PART_ID, &mut buf).await?;
            Ok(decode_device_info(&buf))
        }

        /// See [`super::Ens210::verify_id`].
        pub async fn verify_id<D: embedded_hal_async::delay::DelayNs>(
            &mut self,
            delay: &mut D,
        ) -> Result<DeviceInfo, Error<I2C::Error>> {
            let info = self.device_info(delay).await?;
            check_part_id(info)
        }

        /// See [`super::Ens210::reset`].
        pub async fn reset(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.write(&[REG_SYS_CTRL, SYS_CTRL_RESET]).await?;
            self.low_power = true;
            Ok(BOOT_WAIT_US)
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Ens210<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.write(&START_FRAME).await?;
            Ok(CONVERSION_WAIT_US)
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let mut buf = [0u8; 6];
            self.read(REG_T_VAL, &mut buf).await?;
            decode_measurement(&buf)
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::LowPower for Ens210<I2C> {
        type BusError = I2C::Error;

        async fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
            self.write(&[REG_SYS_CTRL, sys_ctrl(true)]).await?;
            self.low_power = true;
            Ok(())
        }

        async fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.write(&[REG_SYS_CTRL, sys_ctrl(false)]).await?;
            self.low_power = false;
            Ok(BOOT_WAIT_US)
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use std::vec;
    use zigbee_sensor::blocking::{LowPower, Sensor};

    // Datasheet "Processing T_VAL and H_VAL": registers 0x30..0x35.
    const VALUES: [u8; 6] = [0xFD, 0x49, 0x0B, 0x6C, 0x2E, 0xF5];
    // 0x49FD / 64 K = 295.95 K = 22.80 °C; 0x2E6C / 512 = 23.21 %RH.
    const EXPECTED: Measurement = Measurement {
        temperature: 2_280,
        humidity: 2_321,
    };

    struct CountingDelay(u32);
    impl embedded_hal::delay::DelayNs for CountingDelay {
        fn delay_ns(&mut self, ns: u32) {
            self.0 += ns / 1_000;
        }
    }

    #[test]
    fn datasheet_crc_and_conversion() {
        // "CRC of 05 over a payload of 149FD"; H: crc 7a over 12e6c.
        assert_eq!(crc7(0x1_49FD), 0x05);
        assert_eq!(crc7(0x1_2E6C), 0x7A);
        assert_eq!(decode_measurement::<()>(&VALUES), Ok(EXPECTED));
        // 0 °C = 273.15 K = 17481.6 / 64.
        assert_eq!(temperature_centi_celsius(17_482), 1);
        assert_eq!(humidity_centi_percent(51_200), 10_000);
        assert_eq!(humidity_centi_percent(u16::MAX), 10_000);
        let mut bad = VALUES;
        bad[2] ^= 0x02;
        assert_eq!(decode_measurement::<()>(&bad), Err(Error::Crc));
        // Valid flag clear (with its matching CRC) → not ready.
        let crc = crc7(0x49FD);
        assert_eq!(
            decode_value::<()>(&[0xFD, 0x49, crc << 1]),
            Err(Error::NotReady)
        );
    }

    #[test]
    fn single_shot_and_low_power() {
        let not_valid = crc7(0x49FD) << 1;
        let expectations = [
            Transaction::write(ADDRESS, vec![0x21, 0x00, 0x03]),
            Transaction::write_read(
                ADDRESS,
                vec![0x30],
                vec![0xFD, 0x49, not_valid, 0x6C, 0x2E, 0xF5],
            ),
            Transaction::write_read(ADDRESS, vec![0x30], VALUES.to_vec()),
            Transaction::write(ADDRESS, vec![0x10, 0x00]),
            Transaction::write(ADDRESS, vec![0x10, 0x01]),
            Transaction::write(ADDRESS, vec![0x10, 0x80]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut ens = Ens210::new(&mut i2c);
        assert_eq!(ens.start_measurement(), Ok(CONVERSION_WAIT_US));
        assert_eq!(ens.read_measurement(), Err(Error::NotReady));
        assert_eq!(ens.read_measurement(), Ok(EXPECTED));
        assert_eq!(ens.wake(), Ok(BOOT_WAIT_US));
        ens.sleep().unwrap();
        assert_eq!(ens.reset(), Ok(BOOT_WAIT_US));
        i2c.done();
    }

    #[test]
    fn device_info_toggles_low_power() {
        let ids = vec![
            0x10, 0x02, 0x01, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        ];
        let mut wrong = ids.clone();
        wrong[1] = 0xA2;
        let expectations = [
            Transaction::write(ADDRESS, vec![0x10, 0x00]),
            Transaction::write_read(ADDRESS, vec![0x11], vec![0x01]),
            Transaction::write_read(ADDRESS, vec![0x00], ids),
            Transaction::write(ADDRESS, vec![0x10, 0x01]),
            Transaction::write(ADDRESS, vec![0x10, 0x00]),
            Transaction::write_read(ADDRESS, vec![0x11], vec![0x00]),
            Transaction::write(ADDRESS, vec![0x10, 0x01]),
            Transaction::write(ADDRESS, vec![0x10, 0x00]),
            Transaction::write_read(ADDRESS, vec![0x11], vec![0x01]),
            Transaction::write_read(ADDRESS, vec![0x00], wrong),
            Transaction::write(ADDRESS, vec![0x10, 0x01]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut ens = Ens210::new(&mut i2c);
        let mut delay = CountingDelay(0);
        assert_eq!(
            ens.verify_id(&mut delay),
            Ok(DeviceInfo {
                part_id: PART_ID,
                die_rev: 1,
                uid: 0x0807_0605_0403_0201,
            })
        );
        assert_eq!(ens.device_info(&mut delay), Err(Error::NotReady));
        assert_eq!(ens.verify_id(&mut delay), Err(Error::UnexpectedId(0xA210)));
        assert_eq!(delay.0, 3 * BOOT_WAIT_US);
        i2c.done();
    }

    #[test]
    fn async_measure_and_bus_error() {
        use zigbee_sensor::asynch::Sensor as _;

        struct RecordingDelay(u32);
        impl embedded_hal_async::delay::DelayNs for RecordingDelay {
            async fn delay_ns(&mut self, ns: u32) {
                self.0 += ns / 1_000;
            }
        }

        let expectations = [
            Transaction::write(ADDRESS, vec![0x21, 0x00, 0x03]),
            Transaction::write_read(ADDRESS, vec![0x30], VALUES.to_vec()),
            Transaction::write(ADDRESS, vec![0x21, 0x00, 0x03])
                .with_error(embedded_hal::i2c::ErrorKind::Other),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut ens = asynch::Ens210::new(&mut i2c);
        let mut delay = RecordingDelay(0);
        assert_eq!(block_on(ens.measure(&mut delay)), Ok(EXPECTED));
        assert_eq!(delay.0, CONVERSION_WAIT_US);
        assert!(matches!(
            block_on(ens.start_measurement()),
            Err(Error::Bus(_))
        ));
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
