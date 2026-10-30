//! Sensylink CHT8310 / CHT8315 temperature and relative humidity sensor
//! (I2C / SMBus).
//!
//! After power-on the part converts continuously (1 Hz). [`Cht831x::init`]
//! switches it to standby (SD = 1, 55 nA typ. on the CHT8310), where every
//! write to the one-shot register runs one temperature + humidity
//! conversion and the part goes back to standby. Every register read
//! carries an SMBus PEC byte (CRC-8, polynomial 0x07) that the driver
//! checks.
//!
//! References: Sensylink CHT8310 datasheet Rev 1.1 (2023-09) and CHT8315
//! datasheet Rev 1.6. Device ID values are not in the datasheets; see
//! [`Variant`].
//!
//! ```ignore
//! let mut cht = Cht831x::new(i2c);
//! cht.verify_id()?;
//! cht.init()?;
//! let wait_us = cht.start_measurement()?;
//! // ... sleep wait_us ...
//! let m = cht.read_measurement()?;
//! ```

#![no_std]

use zigbee_sensor::crc::crc8_msb;
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address with AD0 to GND.
pub const ADDRESS_AD0_GND: u8 = 0x40;
/// I2C address with AD0 to VCC.
pub const ADDRESS_AD0_VCC: u8 = 0x44;
/// I2C address with AD0 to SDA.
pub const ADDRESS_AD0_SDA: u8 = 0x48;
/// I2C address with AD0 to SCL.
pub const ADDRESS_AD0_SCL: u8 = 0x4C;
/// Manufacturer ID register value (shared with the CHT8305).
pub const MANUFACTURER_ID: u16 = 0x5959;

const REG_TEMPERATURE: u8 = 0x00;
const REG_HUMIDITY: u8 = 0x01;
const REG_STATUS: u8 = 0x02;
const REG_CONFIG: u8 = 0x03;
const REG_CONVERT_RATE: u8 = 0x04;
const REG_ONE_SHOT: u8 = 0x0F;
const REG_MANUFACTURER_ID: u8 = 0xFE;
const REG_DEVICE_ID: u8 = 0xFF;

const STATUS_BUSY: u16 = 1 << 15;
const CONFIG_SD: u16 = 1 << 14;
const CONFIG_EM: u16 = 1 << 12;
const CONFIG_EHT: u16 = 1 << 8;
const CONVERT_RATE_SHIFT: u16 = 8;
const CONVERT_RATE_MASK: u16 = 0b111 << CONVERT_RATE_SHIFT;
const HUMIDITY_OVERFLOW: u16 = 1 << 15;

/// PEC polynomial x⁸ + x² + x + 1.
const PEC_POLYNOMIAL: u8 = 0x07;

/// Wait after a one-shot trigger: 11 ms typical (no maximum is
/// published) plus a 9 ms margin, as in the Zephyr driver. The status
/// busy bit catches an early read.
pub const ONE_SHOT_WAIT_US: WaitUs = 20_000;

/// Part identified by [`Cht831x::verify_id`] from the device ID register
/// (0xFF), which the datasheets only describe as "version or part no.".
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// CHT8310, device ID 0x8215 (value used by pvvx firmware; the part
    /// is also sold as CHT8215).
    Cht8310,
    /// CHT8315, device ID 0x8315 (value used by the Zephyr driver).
    Cht8315,
}

impl Variant {
    /// Device ID register value.
    pub const fn device_id(self) -> u16 {
        match self {
            Self::Cht8310 => 0x8215,
            Self::Cht8315 => 0x8315,
        }
    }
}

/// Continuous conversion period (convert-rate register bits 10:8).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ConvertRate {
    /// One conversion every 120 s.
    Every120s = 0b000,
    /// Every 60 s.
    Every60s = 0b001,
    /// Every 10 s.
    Every10s = 0b010,
    /// Every 5 s.
    Every5s = 0b011,
    /// Every 1 s (power-on default).
    #[default]
    Every1s = 0b100,
    /// Every 500 ms.
    Every500ms = 0b101,
    /// Every 250 ms.
    Every250ms = 0b110,
    /// Every 125 ms.
    Every125ms = 0b111,
}

/// Temperature register → 0.01 °C. The value is a left-justified two's
/// complement number with an LSB of 0.03125 °C: 13 bits when
/// `extended` (EM) is false, 14 bits (range to 150 °C) when true.
pub const fn temperature_centi_celsius(raw: u16, extended: bool) -> i32 {
    let shift = if extended { 2 } else { 3 };
    let counts = (raw as i16 >> shift) as i32;
    // counts · 3.125 = counts · 25 / 8, rounded half up.
    (counts * 25 + 4) >> 3
}

/// Humidity register → 0.01 %RH: `100 · raw[14:0] / 32768`. Bit 15 flags
/// an overflow (≥ 100 %RH) and clips to 100 %.
pub const fn humidity_centi_percent(raw: u16) -> u16 {
    if raw & HUMIDITY_OVERFLOW != 0 {
        return 10_000;
    }
    ((raw as u32 * 10_000 + 16_384) >> 15) as u16
}

/// SMBus PEC of a register read: CRC-8 over the address + W, register,
/// address + R and the two data bytes.
pub fn pec(address: u8, register: u8, data: [u8; 2]) -> u8 {
    crc8_msb(
        &[address << 1, register, (address << 1) | 1, data[0], data[1]],
        PEC_POLYNOMIAL,
        0x00,
    )
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

fn decode_register<E>(address: u8, register: u8, frame: &[u8; 3]) -> Result<u16, Error<E>> {
    let data = [frame[0], frame[1]];
    if pec(address, register, data) == frame[2] {
        Ok(u16::from_be_bytes(data))
    } else {
        Err(Error::Crc)
    }
}

fn check_manufacturer<E>(id: u16) -> Result<(), Error<E>> {
    if id == MANUFACTURER_ID {
        Ok(())
    } else {
        Err(Error::UnexpectedId(u32::from(id)))
    }
}

fn variant_from_id<E>(id: u16) -> Result<Variant, Error<E>> {
    match id {
        0x8215 => Ok(Variant::Cht8310),
        0x8315 => Ok(Variant::Cht8315),
        _ => Err(Error::UnexpectedId(u32::from(id))),
    }
}

fn check_ready<E>(status: u16) -> Result<(), Error<E>> {
    if status & STATUS_BUSY != 0 {
        Err(Error::NotReady)
    } else {
        Ok(())
    }
}

const fn with_bit(value: u16, bit: u16, on: bool) -> u16 {
    if on { value | bit } else { value & !bit }
}

const fn with_rate(value: u16, rate: ConvertRate) -> u16 {
    (value & !CONVERT_RATE_MASK) | ((rate as u16) << CONVERT_RATE_SHIFT)
}

const fn write_frame(register: u8, value: u16) -> [u8; 3] {
    let [msb, lsb] = value.to_be_bytes();
    [register, msb, lsb]
}

/// Any value written to the one-shot register starts a conversion.
const ONE_SHOT_FRAME: [u8; 3] = [REG_ONE_SHOT, 0x00, 0x00];

macro_rules! constructors {
    () => {
        /// Create a driver at [`ADDRESS_AD0_GND`](crate::ADDRESS_AD0_GND)
        /// assuming the power-on state (continuous, 13-bit temperature);
        /// no bus traffic.
        pub const fn new(i2c: I2C) -> Self {
            Self {
                i2c,
                address: ADDRESS_AD0_GND,
                extended: false,
                continuous: true,
            }
        }

        /// Use another AD0 address.
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
pub struct Cht831x<I2C> {
    i2c: I2C,
    address: u8,
    extended: bool,
    continuous: bool,
}

impl<I2C> Cht831x<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Cht831x<I2C> {
    fn read_register(&mut self, register: u8) -> Result<u16, Error<I2C::Error>> {
        let mut frame = [0u8; 3];
        self.i2c
            .write_read(self.address, &[register], &mut frame)
            .map_err(Error::bus)?;
        decode_register(self.address, register, &frame)
    }

    /// Writes are sent without PEC, as the Zephyr and pvvx drivers do; the
    /// datasheets do not say whether the part checks a write PEC.
    fn write_register(&mut self, register: u8, value: u16) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, &write_frame(register, value))
            .map_err(Error::bus)
    }

    /// Check the manufacturer ID (0x5959) and identify the part from the
    /// device ID. An unknown device ID gives [`Error::UnexpectedId`]
    /// (a CHT8305 reads 0x8305).
    pub fn verify_id(&mut self) -> Result<Variant, Error<I2C::Error>> {
        check_manufacturer(self.read_register(REG_MANUFACTURER_ID)?)?;
        variant_from_id(self.read_register(REG_DEVICE_ID)?)
    }

    /// Read the config register.
    pub fn config(&mut self) -> Result<u16, Error<I2C::Error>> {
        self.read_register(REG_CONFIG)
    }

    fn update_config(&mut self, bit: u16, on: bool) -> Result<u16, Error<I2C::Error>> {
        let config = with_bit(self.config()?, bit, on);
        self.write_register(REG_CONFIG, config)?;
        Ok(config)
    }

    /// Enter one-shot standby (SD = 1) and pick up the EM setting.
    /// Other config bits keep their values.
    pub fn init(&mut self) -> Result<(), Error<I2C::Error>> {
        let config = self.update_config(CONFIG_SD, true)?;
        self.extended = config & CONFIG_EM != 0;
        self.continuous = false;
        Ok(())
    }

    /// Select 14-bit temperature data (EM = 1, range to 150 °C) or the
    /// default 13-bit format.
    pub fn set_extended_range(&mut self, on: bool) -> Result<(), Error<I2C::Error>> {
        self.update_config(CONFIG_EM, on)?;
        self.extended = on;
        Ok(())
    }

    /// Switch the heater (EHT).
    pub fn set_heater(&mut self, on: bool) -> Result<(), Error<I2C::Error>> {
        self.update_config(CONFIG_EHT, on).map(|_| ())
    }

    /// `Some(rate)`: convert continuously at `rate` (SD = 0); the sensor
    /// trait then just reads the latest result. `None`: one-shot standby.
    pub fn set_continuous(&mut self, rate: Option<ConvertRate>) -> Result<(), Error<I2C::Error>> {
        if let Some(rate) = rate {
            let value = with_rate(self.read_register(REG_CONVERT_RATE)?, rate);
            self.write_register(REG_CONVERT_RATE, value)?;
        }
        self.update_config(CONFIG_SD, rate.is_none())?;
        self.continuous = rate.is_some();
        Ok(())
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Cht831x<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    /// Write the one-shot register; in continuous mode nothing is sent
    /// and the wait is 0.
    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        if self.continuous {
            return Ok(0);
        }
        self.i2c
            .write(self.address, &ONE_SHOT_FRAME)
            .map_err(Error::bus)?;
        Ok(ONE_SHOT_WAIT_US)
    }

    /// Check the busy bit ([`Error::NotReady`]), then read T and RH.
    /// Reading the status also clears a pending ALERT.
    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        check_ready(self.read_register(REG_STATUS)?)?;
        let temperature = self.read_register(REG_TEMPERATURE)?;
        let humidity = self.read_register(REG_HUMIDITY)?;
        Ok(Measurement {
            temperature: temperature_centi_celsius(temperature, self.extended),
            humidity: humidity_centi_percent(humidity),
        })
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        ADDRESS_AD0_GND, CONFIG_EHT, CONFIG_EM, CONFIG_SD, ConvertRate, Measurement,
        ONE_SHOT_FRAME, ONE_SHOT_WAIT_US, REG_CONFIG, REG_CONVERT_RATE, REG_DEVICE_ID,
        REG_HUMIDITY, REG_MANUFACTURER_ID, REG_STATUS, REG_TEMPERATURE, Variant,
        check_manufacturer, check_ready, decode_register, humidity_centi_percent,
        temperature_centi_celsius, variant_from_id, with_bit, with_rate, write_frame,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Cht831x`].
    pub struct Cht831x<I2C> {
        i2c: I2C,
        address: u8,
        extended: bool,
        continuous: bool,
    }

    impl<I2C> Cht831x<I2C> {
        constructors!();
    }

    impl<I2C: I2c> Cht831x<I2C> {
        async fn read_register(&mut self, register: u8) -> Result<u16, Error<I2C::Error>> {
            let mut frame = [0u8; 3];
            self.i2c
                .write_read(self.address, &[register], &mut frame)
                .await
                .map_err(Error::bus)?;
            decode_register(self.address, register, &frame)
        }

        async fn write_register(
            &mut self,
            register: u8,
            value: u16,
        ) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write(self.address, &write_frame(register, value))
                .await
                .map_err(Error::bus)
        }

        /// See [`super::Cht831x::verify_id`].
        pub async fn verify_id(&mut self) -> Result<Variant, Error<I2C::Error>> {
            check_manufacturer(self.read_register(REG_MANUFACTURER_ID).await?)?;
            variant_from_id(self.read_register(REG_DEVICE_ID).await?)
        }

        /// See [`super::Cht831x::config`].
        pub async fn config(&mut self) -> Result<u16, Error<I2C::Error>> {
            self.read_register(REG_CONFIG).await
        }

        async fn update_config(&mut self, bit: u16, on: bool) -> Result<u16, Error<I2C::Error>> {
            let config = with_bit(self.config().await?, bit, on);
            self.write_register(REG_CONFIG, config).await?;
            Ok(config)
        }

        /// See [`super::Cht831x::init`].
        pub async fn init(&mut self) -> Result<(), Error<I2C::Error>> {
            let config = self.update_config(CONFIG_SD, true).await?;
            self.extended = config & CONFIG_EM != 0;
            self.continuous = false;
            Ok(())
        }

        /// See [`super::Cht831x::set_extended_range`].
        pub async fn set_extended_range(&mut self, on: bool) -> Result<(), Error<I2C::Error>> {
            self.update_config(CONFIG_EM, on).await?;
            self.extended = on;
            Ok(())
        }

        /// See [`super::Cht831x::set_heater`].
        pub async fn set_heater(&mut self, on: bool) -> Result<(), Error<I2C::Error>> {
            self.update_config(CONFIG_EHT, on).await.map(|_| ())
        }

        /// See [`super::Cht831x::set_continuous`].
        pub async fn set_continuous(
            &mut self,
            rate: Option<ConvertRate>,
        ) -> Result<(), Error<I2C::Error>> {
            if let Some(rate) = rate {
                let value = with_rate(self.read_register(REG_CONVERT_RATE).await?, rate);
                self.write_register(REG_CONVERT_RATE, value).await?;
            }
            self.update_config(CONFIG_SD, rate.is_none()).await?;
            self.continuous = rate.is_some();
            Ok(())
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Cht831x<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            if self.continuous {
                return Ok(0);
            }
            self.i2c
                .write(self.address, &ONE_SHOT_FRAME)
                .await
                .map_err(Error::bus)?;
            Ok(ONE_SHOT_WAIT_US)
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            check_ready(self.read_register(REG_STATUS).await?)?;
            let temperature = self.read_register(REG_TEMPERATURE).await?;
            let humidity = self.read_register(REG_HUMIDITY).await?;
            Ok(Measurement {
                temperature: temperature_centi_celsius(temperature, self.extended),
                humidity: humidity_centi_percent(humidity),
            })
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use std::vec;
    use std::vec::Vec;
    use zigbee_sensor::blocking::Sensor;

    /// Register read with a correct PEC.
    fn read(address: u8, register: u8, value: u16) -> Transaction {
        let data = value.to_be_bytes();
        let mut frame: Vec<u8> = data.to_vec();
        frame.push(pec(address, register, data));
        Transaction::write_read(address, vec![register], frame)
    }

    #[test]
    fn datasheet_conversions_and_pec() {
        // Temperature tables, 13-bit (EM = 0) and 14-bit (EM = 1).
        assert_eq!(temperature_centi_celsius(0x1900, false), 2_500);
        assert_eq!(temperature_centi_celsius(0xE700, false), -2_500);
        assert_eq!(temperature_centi_celsius(0xFFC0, false), -25);
        assert_eq!(temperature_centi_celsius(0x0C80, true), 2_500);
        assert_eq!(temperature_centi_celsius(0xF380, true), -2_500);
        assert_eq!(temperature_centi_celsius(0x4B00, true), 15_000);
        // Humidity table and overflow flag.
        assert_eq!(humidity_centi_percent(0x428F), 5_200);
        assert_eq!(humidity_centi_percent(0x1AE1), 2_100);
        assert_eq!(humidity_centi_percent(0x8123), 10_000);
        // PEC example: CRC(0x80, 0x00, 0x81, 0x19, 0xE0) = 0xB3.
        assert_eq!(pec(0x40, 0x00, [0x19, 0xE0]), 0xB3);
    }

    #[test]
    fn one_shot_flow_and_errors() {
        let a = ADDRESS_AD0_SCL;
        let expectations = [
            read(a, 0xFE, 0x5959),
            read(a, 0xFF, 0x8215),
            // init: power-on config 0x0880 plus EM set earlier.
            read(a, 0x03, 0x1880),
            Transaction::write(a, vec![0x03, 0x58, 0x80]),
            Transaction::write(a, vec![0x0F, 0x00, 0x00]),
            read(a, 0x02, 0x8000),
            read(a, 0x02, 0x0000),
            read(a, 0x00, 0x0C80),
            read(a, 0x01, 0x428F),
            Transaction::write_read(a, vec![0x02], vec![0x00, 0x00, 0x00]),
            read(a, 0x03, 0x5880),
            Transaction::write(a, vec![0x03, 0x59, 0x80]),
            read(a, 0xFE, 0x5959),
            read(a, 0xFF, 0x8305),
            Transaction::write_read(a, vec![0x02], vec![0, 0, 0])
                .with_error(embedded_hal::i2c::ErrorKind::Other),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut cht = Cht831x::new(&mut i2c).with_address(a);
        assert_eq!(cht.verify_id(), Ok(Variant::Cht8310));
        cht.init().unwrap();
        assert_eq!(cht.start_measurement(), Ok(ONE_SHOT_WAIT_US));
        assert_eq!(cht.read_measurement(), Err(Error::NotReady));
        assert_eq!(
            cht.read_measurement(),
            Ok(Measurement {
                temperature: 2_500,
                humidity: 5_200,
            })
        );
        assert_eq!(cht.read_measurement(), Err(Error::Crc));
        cht.set_heater(true).unwrap();
        assert_eq!(cht.verify_id(), Err(Error::UnexpectedId(0x8305)));
        assert!(matches!(cht.read_measurement(), Err(Error::Bus(_))));
        i2c.done();
    }

    #[test]
    fn async_continuous_mode() {
        use zigbee_sensor::asynch::Sensor as _;

        struct RecordingDelay(u32);
        impl embedded_hal_async::delay::DelayNs for RecordingDelay {
            async fn delay_ns(&mut self, ns: u32) {
                self.0 += ns / 1_000;
            }
        }

        let a = ADDRESS_AD0_GND;
        let expectations = [
            read(a, 0x03, 0x4880),
            Transaction::write(a, vec![0x03, 0x48, 0x80]),
            Transaction::write(a, vec![0x0F, 0x00, 0x00]),
            read(a, 0x02, 0x0000),
            read(a, 0x00, 0x1900),
            read(a, 0x01, 0x1AE1),
            read(a, 0x03, 0x4880),
            Transaction::write(a, vec![0x03, 0x58, 0x80]),
            read(a, 0x04, 0x0400),
            Transaction::write(a, vec![0x04, 0x03, 0x00]),
            read(a, 0x03, 0x5880),
            Transaction::write(a, vec![0x03, 0x18, 0x80]),
            read(a, 0x02, 0x0000),
            read(a, 0x00, 0x4B00),
            read(a, 0x01, 0x1AE1),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut cht = asynch::Cht831x::new(&mut i2c);
        let mut delay = RecordingDelay(0);
        block_on(cht.init()).unwrap();
        assert_eq!(
            block_on(cht.measure(&mut delay)),
            Ok(Measurement {
                temperature: 2_500,
                humidity: 2_100,
            })
        );
        block_on(cht.set_extended_range(true)).unwrap();
        block_on(cht.set_continuous(Some(ConvertRate::Every5s))).unwrap();
        assert_eq!(
            block_on(cht.measure(&mut delay)),
            Ok(Measurement {
                temperature: 15_000,
                humidity: 2_100,
            })
        );
        assert_eq!(delay.0, ONE_SHOT_WAIT_US);
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
