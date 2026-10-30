//! Melexis MLX90614 and MLX90615 infrared thermometers over SMBus with
//! PEC (CRC-8, polynomial 0x07).
//!
//! Both parts convert continuously and report the object and the
//! ambient (package) temperature in 0.02 K steps. Every read is checked
//! against its PEC, and every write carries one.
//!
//! Sleep is entered with an SMBus command, but **waking requires driving
//! the bus pins as GPIOs**, which this driver cannot do:
//!
//! * MLX90614: hold **SDA low for > 33 ms** (SCL high). Not available on
//!   the 5 V MLX90614Axx.
//! * MLX90615: hold **SCL low for ≥ 50 ms**. Keep SCL high during sleep.
//!
//! [`LowPower::wake`](zigbee_sensor::blocking::LowPower::wake) assumes the
//! application has already generated that pulse. It issues no bus traffic
//! and returns the time until valid data.
//!
//! References: MLX90614 data sheet rev 021 (DOC#3901090614), MLX90615 data
//! sheet rev 001 (3901090615).
//!
//! ```ignore
//! let mut ir = Mlx90614::new(i2c, Variant::Mlx90614.default_address());
//! ir.probe()?; // checks the SMBus address stored in EEPROM
//! let m = ir.read_measurement()?; // object + ambient, 0.01 °C
//! ir.sleep()?;
//! // ... later: drive SDA low for > 33 ms, then
//! let wait_us = ir.wake()?;
//! ```
//!
//! The blocking driver and its [`asynch`] twin are generated from one macro
//! body, so they issue identical bus transactions.

#![no_std]

use zigbee_sensor::crc::crc8_msb;
use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Supported parts.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Variant {
    /// MLX90614 (single or dual zone, TO39).
    #[default]
    Mlx90614,
    /// MLX90615 (TO46).
    Mlx90615,
}

impl Variant {
    /// Factory SMBus address (0x5A / 0x5B).
    pub const fn default_address(self) -> u8 {
        match self {
            Self::Mlx90614 => 0x5A,
            Self::Mlx90615 => 0x5B,
        }
    }

    const fn ram(self, address: u8) -> u8 {
        match self {
            Self::Mlx90614 => address,
            Self::Mlx90615 => 0x20 | address,
        }
    }

    const fn eeprom(self, address: u8) -> u8 {
        match self {
            Self::Mlx90614 => 0x20 | address,
            Self::Mlx90615 => 0x10 | address,
        }
    }

    const fn sleep_command(self) -> u8 {
        match self {
            Self::Mlx90614 => 0xFF,
            Self::Mlx90615 => 0xC6,
        }
    }

    const fn emissivity_cell(self) -> u8 {
        match self {
            Self::Mlx90614 => 0x04,
            Self::Mlx90615 => 0x03,
        }
    }

    const fn address_cell(self) -> u8 {
        match self {
            Self::Mlx90614 => 0x0E,
            Self::Mlx90615 => 0x00,
        }
    }

    /// Minimum length of the wake pulse the application must generate
    /// (MLX90614: SDA low 33 ms; MLX90615: SCL low 50 ms).
    pub const fn wake_pulse_us(self) -> WaitUs {
        match self {
            Self::Mlx90614 => 33_000,
            Self::Mlx90615 => 50_000,
        }
    }

    /// Time from wake (or power-up) to valid data: 1.5 × the typical
    /// 250 ms (MLX90614) / 300 ms (MLX90615).
    pub const fn data_valid_us(self) -> WaitUs {
        match self {
            Self::Mlx90614 => 375_000,
            Self::Mlx90615 => 450_000,
        }
    }

    /// Convert an emissivity register to ε × 10 000 (MLX90614:
    /// 65535 = 1.0; MLX90615: 0x4000 = 1.0).
    pub const fn emissivity_from_raw(self, raw: u16) -> u16 {
        let raw = raw as u32;
        let value = match self {
            Self::Mlx90614 => (raw * 10_000 + 32_767) / 65_535,
            Self::Mlx90615 => (raw * 10_000 + 8_192) / 16_384,
        };
        value as u16
    }
}

const RAM_TA: u8 = 0x06;
const RAM_TOBJ1: u8 = 0x07;
const RAM_TOBJ2: u8 = 0x08;

/// SMBus PEC: CRC-8, polynomial 0x07, initial value 0, over every byte of
/// the transaction including the address bytes.
pub fn pec(bytes: &[u8]) -> u8 {
    crc8_msb(bytes, 0x07, 0x00)
}

/// Decode a linearized temperature RAM word (0.02 K/LSB) into 0.01 °C.
/// Returns `None` if the error flag (bit 15) is set.
pub const fn decode_temperature(raw: u16) -> Option<i32> {
    if raw & 0x8000 != 0 {
        return None;
    }
    Some(raw as i32 * 2 - 27_315)
}

/// Object and ambient temperature.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Object temperature (zone 1) in 0.01 °C.
    pub object: i32,
    /// Ambient (package) temperature in 0.01 °C.
    pub ambient: i32,
}

impl Readings for Measurement {
    /// [`Quantity::ObjectTemperature`] for the object, then
    /// [`Quantity::Temperature`] for the ambient.
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::ObjectTemperature(self.object));
        f(Quantity::Temperature(self.ambient));
    }
}

macro_rules! constructors {
    () => {
        /// Create an MLX90614 driver at `address`; no bus traffic.
        pub const fn new(i2c: I2C, address: u8) -> Self {
            Self {
                i2c,
                address,
                variant: Variant::Mlx90614,
            }
        }

        /// Select the part (default MLX90614).
        pub const fn with_variant(mut self, variant: Variant) -> Self {
            self.variant = variant;
            self
        }

        /// The selected part.
        pub const fn variant(&self) -> Variant {
            self.variant
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }
    };
}

macro_rules! driver_impl {
    ($i2c:path, $sensor:path, $low_power:path, $($async:ident)?; $($await:tt)*) => {
        impl<I2C: $i2c> Mlx90614<I2C> {
            /// SMBus Read Word with PEC check.
            $($async)? fn read_word(&mut self, command: u8) -> Result<u16, Error<I2C::Error>> {
                let mut buf = [0u8; 3];
                self.i2c
                    .write_read(self.address, &[command], &mut buf)
                    $($await)*
                    .map_err(Error::bus)?;
                let sa = self.address << 1;
                if pec(&[sa, command, sa | 1, buf[0], buf[1]]) != buf[2] {
                    return Err(Error::Crc);
                }
                Ok(u16::from_le_bytes([buf[0], buf[1]]))
            }

            $($async)? fn read_temperature(&mut self, ram: u8) -> Result<i32, Error<I2C::Error>> {
                let raw = self.read_word(self.variant.ram(ram)) $($await)*?;
                decode_temperature(raw).ok_or(Error::InvalidData)
            }

            /// Read an EEPROM cell (`address` is the cell index, 0x00 …
            /// 0x1F on MLX90614, 0x00 … 0x0F on MLX90615).
            pub $($async)? fn read_eeprom(&mut self, address: u8) -> Result<u16, Error<I2C::Error>> {
                let command = self.variant.eeprom(address);
                self.read_word(command) $($await)*
            }

            /// Check that the SMBus address stored in EEPROM matches the
            /// address in use (skipped for the 0x00 catch-all address).
            pub $($async)? fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
                let cell = self.read_eeprom(self.variant.address_cell()) $($await)*?;
                if self.address != 0 && (cell & 0x7F) as u8 != self.address {
                    return Err(Error::UnexpectedId(u32::from(cell)));
                }
                Ok(())
            }

            /// Factory ID number from EEPROM: four words (0x1C … 0x1F) on
            /// MLX90614, two (0x0E, 0x0F) on MLX90615, lowest cell in the
            /// most significant position.
            pub $($async)? fn read_id(&mut self) -> Result<u64, Error<I2C::Error>> {
                let (first, count) = match self.variant {
                    Variant::Mlx90614 => (0x1C, 4),
                    Variant::Mlx90615 => (0x0E, 2),
                };
                let mut id = 0u64;
                for i in 0..count {
                    let word = self.read_eeprom(first + i) $($await)*?;
                    id = (id << 16) | u64::from(word);
                }
                Ok(id)
            }

            /// Programmed object emissivity as ε × 10 000 (factory 10 000).
            pub $($async)? fn read_emissivity(&mut self) -> Result<u16, Error<I2C::Error>> {
                let raw = self.read_eeprom(self.variant.emissivity_cell()) $($await)*?;
                Ok(self.variant.emissivity_from_raw(raw))
            }

            /// Ambient (package) temperature in 0.01 °C.
            pub $($async)? fn read_ambient(&mut self) -> Result<i32, Error<I2C::Error>> {
                self.read_temperature(RAM_TA) $($await)*
            }

            /// Object temperature in 0.01 °C.
            pub $($async)? fn read_object(&mut self) -> Result<i32, Error<I2C::Error>> {
                self.read_temperature(RAM_TOBJ1) $($await)*
            }

            /// Second-zone object temperature of dual-zone MLX90614 parts
            /// ([`Error::Unsupported`] on MLX90615).
            pub $($async)? fn read_object2(&mut self) -> Result<i32, Error<I2C::Error>> {
                if self.variant != Variant::Mlx90614 {
                    return Err(Error::Unsupported);
                }
                self.read_temperature(RAM_TOBJ2) $($await)*
            }
        }

        impl<I2C: $i2c> $sensor for Mlx90614<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// The sensor converts continuously, so there is nothing to
            /// trigger: no bus traffic, returns 0.
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                Ok(0)
            }

            /// Reads the object, then the ambient temperature.
            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let object = self.read_object() $($await)*?;
                let ambient = self.read_ambient() $($await)*?;
                Ok(Measurement { object, ambient })
            }
        }

        impl<I2C: $i2c> $low_power for Mlx90614<I2C> {
            type BusError = I2C::Error;

            /// Send the Enter SLEEP command (with PEC). The part then
            /// ignores the bus until the pin wake pulse.
            $($async)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                let command = self.variant.sleep_command();
                let crc = pec(&[self.address << 1, command]);
                self.i2c
                    .write(self.address, &[command, crc])
                    $($await)*
                    .map_err(Error::bus)
            }

            /// Call **after** generating the wake pulse
            /// ([`Variant::wake_pulse_us`]) on the bus pins. No bus
            /// traffic; returns [`Variant::data_valid_us`].
            $($async)? fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                Ok(self.variant.data_valid_us())
            }
        }
    };
}

/// Blocking driver for MLX90614 and MLX90615.
pub struct Mlx90614<I2C> {
    i2c: I2C,
    address: u8,
    variant: Variant,
}

impl<I2C> Mlx90614<I2C> {
    constructors!();
}

driver_impl!(
    embedded_hal::i2c::I2c,
    zigbee_sensor::blocking::Sensor,
    zigbee_sensor::blocking::LowPower,
    ;
);

/// Async driver.
pub mod asynch {
    use super::*;

    /// Async twin of [`super::Mlx90614`].
    pub struct Mlx90614<I2C> {
        i2c: I2C,
        address: u8,
        variant: Variant,
    }

    impl<I2C> Mlx90614<I2C> {
        constructors!();
    }

    driver_impl!(
        embedded_hal_async::i2c::I2c,
        zigbee_sensor::asynch::Sensor,
        zigbee_sensor::asynch::LowPower,
        async; .await
    );
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use std::vec;
    use zigbee_sensor::blocking::{LowPower, Sensor};

    const A: u8 = 0x5A;
    const B: u8 = 0x5B;

    /// Read-word response with a correct PEC.
    fn word(address: u8, command: u8, value: u16) -> Transaction {
        let [lo, hi] = value.to_le_bytes();
        let crc = pec(&[address << 1, command, (address << 1) | 1, lo, hi]);
        Transaction::write_read(address, vec![command], vec![lo, hi, crc])
    }

    #[test]
    fn pec_matches_datasheet_examples() {
        // MLX90614 figures 6–8.
        assert_eq!(pec(&[0xB4, 0x07, 0xB5, 0xD2, 0x3A]), 0x30);
        assert_eq!(pec(&[0xB4, 0x22, 0x07, 0xC8]), 0x48);
        assert_eq!(pec(&[0xB4, 0xFF]), 0xE8);
        // MLX90615 sleep command.
        assert_eq!(pec(&[0xB6, 0xC6]), 0x6D);
    }

    #[test]
    fn decodes_temperatures_and_emissivity() {
        assert_eq!(decode_temperature(0x3AD2), Some(2_801)); // 28.01 °C
        assert_eq!(decode_temperature(0x2D8A), Some(-3_999)); // MLX90615 −40 °C
        assert_eq!(decode_temperature(0x4BD0), Some(11_501)); // MLX90615 +115 °C
        assert_eq!(decode_temperature(0x8000), None);
        assert_eq!(Variant::Mlx90614.emissivity_from_raw(0xFFFF), 10_000);
        assert_eq!(Variant::Mlx90614.emissivity_from_raw(0xF332), 9_500);
        assert_eq!(Variant::Mlx90615.emissivity_from_raw(0x4000), 10_000);
        assert_eq!(Variant::Mlx90615.emissivity_from_raw(0x3CCD), 9_500);
    }

    #[test]
    fn mlx90614_reads_and_sleeps() {
        let expectations = [
            word(A, 0x2E, 0xBE5A),
            word(A, 0x24, 0xFFFF),
            word(A, 0x3C, 0x1234),
            word(A, 0x3D, 0x5678),
            word(A, 0x3E, 0x9ABC),
            word(A, 0x3F, 0xDEF0),
            word(A, 0x07, 0x3AD2),
            word(A, 0x06, 0x3A98),
            word(A, 0x08, 0x3B00),
            Transaction::write(A, vec![0xFF, 0xE8]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut ir = Mlx90614::new(&mut i2c, A);
        ir.probe().unwrap();
        assert_eq!(ir.read_emissivity().unwrap(), 10_000);
        assert_eq!(ir.read_id().unwrap(), 0x1234_5678_9ABC_DEF0);
        assert_eq!(ir.start_measurement().unwrap(), 0);
        let m = ir.read_measurement().unwrap();
        assert_eq!(
            m,
            Measurement {
                object: 2_801,
                ambient: 2_685
            }
        );
        assert_eq!(ir.read_object2().unwrap(), 2_893);
        ir.sleep().unwrap();
        assert_eq!(ir.wake().unwrap(), 375_000);
        i2c.done();
    }

    #[test]
    fn mlx90615_commands() {
        let expectations = [
            word(B, 0x10, 0x355B),
            word(B, 0x13, 0x4000),
            word(B, 0x1E, 0xAAAA),
            word(B, 0x1F, 0x5555),
            word(B, 0x27, 0x3AD2),
            word(B, 0x26, 0x3A98),
            Transaction::write(B, vec![0xC6, 0x6D]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut ir = Mlx90614::new(&mut i2c, B).with_variant(Variant::Mlx90615);
        ir.probe().unwrap();
        assert_eq!(ir.read_emissivity().unwrap(), 10_000);
        assert_eq!(ir.read_id().unwrap(), 0xAAAA_5555);
        assert_eq!(ir.read_measurement().unwrap().object, 2_801);
        assert_eq!(ir.read_object2(), Err(Error::Unsupported));
        ir.sleep().unwrap();
        assert_eq!(ir.wake().unwrap(), 450_000);
        i2c.done();
    }

    #[test]
    fn rejects_bad_pec_error_flag_and_address() {
        let expectations = [
            Transaction::write_read(A, vec![0x07], vec![0xD2, 0x3A, 0x31]),
            word(A, 0x07, 0x8000),
            word(A, 0x2E, 0xBE5B),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut ir = Mlx90614::new(&mut i2c, A);
        assert_eq!(ir.read_object(), Err(Error::Crc));
        assert_eq!(ir.read_object(), Err(Error::InvalidData));
        assert_eq!(ir.probe(), Err(Error::UnexpectedId(0xBE5B)));
        i2c.done();
    }

    #[test]
    fn async_reads_measurement() {
        use zigbee_sensor::asynch::Sensor as _;
        let expectations = [word(A, 0x07, 0x3AD2), word(A, 0x06, 0x3A98)];
        let mut i2c = Mock::new(&expectations);
        let mut ir = asynch::Mlx90614::new(&mut i2c, A);
        assert_eq!(block_on(ir.start_measurement()).unwrap(), 0);
        let m = block_on(ir.read_measurement()).unwrap();
        assert_eq!((m.object, m.ambient), (2_801, 2_685));
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
