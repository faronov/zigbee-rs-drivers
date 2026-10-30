//! Microchip MCP9808 and generic JEDEC JC-42.4 (TSE2002/TSE2004-style)
//! digital temperature sensors (I2C / SMBus).
//!
//! These parts have no one-shot mode. [`configure`](Mcp9808::configure)
//! leaves the sensor in shutdown (MCP9808: 0.1 µA typ.);
//! [`start_measurement`](zigbee_sensor::blocking::Sensor::start_measurement)
//! clears SHDN (continuous conversion starts) and returns the conversion
//! time, and
//! [`read_measurement`](zigbee_sensor::blocking::Sensor::read_measurement)
//! reads `T_A` and sets SHDN again.
//!
//! Reference: Microchip "MCP9808 ±0.5°C Maximum Accuracy Digital
//! Temperature Sensor", DS25095A (2011). The generic variant uses only the
//! JC-42.4 registers shared with the MCP9808 (no resolution register).
//!
//! ```ignore
//! let mut mcp = Mcp9808::new(i2c, ADDRESS_DEFAULT);
//! mcp.probe()?;
//! mcp.configure()?;
//! let wait_us = mcp.start_measurement()?;
//! // ... let the Zigbee stack run / sleep for wait_us ...
//! let m = mcp.read_measurement()?;
//! ```
//!
//! The blocking driver and its [`asynch`] twin are generated from one macro
//! body, so they issue identical bus transactions.

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Address with A2 = A1 = A0 = GND (0x18..=0x1F).
pub const ADDRESS_DEFAULT: u8 = 0x18;

/// MCP9808 Manufacturer ID (Microchip, JEDEC).
pub const MCP9808_MANUFACTURER_ID: u16 = 0x0054;
/// MCP9808 Device ID (upper byte of the Device ID/Revision register).
pub const MCP9808_DEVICE_ID: u8 = 0x04;

const REG_CONFIG: u8 = 0x01;
const REG_T_UPPER: u8 = 0x02;
const REG_T_LOWER: u8 = 0x03;
const REG_T_CRIT: u8 = 0x04;
const REG_T_A: u8 = 0x05;
const REG_MANUFACTURER_ID: u8 = 0x06;
const REG_DEVICE_ID: u8 = 0x07;
const REG_RESOLUTION: u8 = 0x08;

const CFG_SHDN: u16 = 1 << 8;
const CFG_INT_CLEAR: u16 = 1 << 5;
const CFG_ALERT_ENABLE: u16 = 1 << 3;
const CFG_ALERT_CRIT_ONLY: u16 = 1 << 2;
const CFG_ALERT_POL: u16 = 1 << 1;
const CFG_ALERT_INTERRUPT: u16 = 1 << 0;

/// Part family.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Variant {
    /// MCP9808: ID check and resolution register.
    #[default]
    Mcp9808,
    /// Any JC-42.4 compliant sensor: no ID check, no resolution register,
    /// fixed conversion wait (125 ms, the JC-42.4 maximum update period).
    Jc42,
}

/// MCP9808 resolution (resolution register, 0x08).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Resolution {
    /// 0.5 °C, 30 ms typ.
    Half,
    /// 0.25 °C, 65 ms typ.
    Quarter,
    /// 0.125 °C, 130 ms typ.
    Eighth,
    /// 0.0625 °C, 250 ms typ.
    #[default]
    Sixteenth,
}

impl Resolution {
    /// Conversion wait in µs: typical × 1.5 (the data sheet gives no
    /// maximum).
    pub const fn conversion_us(self) -> WaitUs {
        match self {
            Self::Half => 45_000,
            Self::Quarter => 97_500,
            Self::Eighth => 195_000,
            Self::Sixteenth => 375_000,
        }
    }
}

/// Generic JC-42.4 conversion wait (maximum update period, 125 ms).
pub const JC42_CONVERSION_US: WaitUs = 125_000;

/// T_UPPER/T_LOWER hysteresis (THYST).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Hysteresis {
    /// 0 °C (default).
    #[default]
    C0,
    /// 1.5 °C.
    C1_5,
    /// 3 °C.
    C3,
    /// 6 °C.
    C6,
}

/// Alert output behaviour (Alert Mod.).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AlertMode {
    /// Comparator: asserted while outside the window (default).
    #[default]
    Comparator,
    /// Interrupt: latched until [`Mcp9808::clear_interrupt`]. `T_CRIT`
    /// always behaves as comparator.
    Interrupt,
}

/// Configuration written by [`Mcp9808::configure`] and every mode change.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// MCP9808 resolution (ignored for [`Variant::Jc42`]).
    pub resolution: Resolution,
    /// Window hysteresis.
    pub hysteresis: Hysteresis,
    /// Drive the Alert output (Alert Cnt.).
    pub alert_enabled: bool,
    /// Alert on `T_A > T_CRIT` only (Alert Sel.).
    pub alert_critical_only: bool,
    /// Alert active high (needs no pull-up but loses wired-OR).
    pub alert_active_high: bool,
    /// Comparator or interrupt output.
    pub alert_mode: AlertMode,
}

impl Config {
    /// Configuration register word.
    pub const fn register(self, shutdown: bool) -> u16 {
        let mut word = (self.hysteresis as u16) << 9;
        if shutdown {
            word |= CFG_SHDN;
        }
        if self.alert_enabled {
            word |= CFG_ALERT_ENABLE;
        }
        if self.alert_critical_only {
            word |= CFG_ALERT_CRIT_ONLY;
        }
        if self.alert_active_high {
            word |= CFG_ALERT_POL;
        }
        if matches!(self.alert_mode, AlertMode::Interrupt) {
            word |= CFG_ALERT_INTERRUPT;
        }
        word
    }
}

/// Window flags in bits 15:13 of `T_A` (independent of the Alert output).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Flags {
    /// `T_A >= T_CRIT`.
    pub critical: bool,
    /// `T_A > T_UPPER`.
    pub upper: bool,
    /// `T_A < T_LOWER`.
    pub lower: bool,
}

impl Flags {
    /// Flags from a raw `T_A` word.
    pub const fn from_raw(raw: u16) -> Self {
        Self {
            critical: raw & (1 << 15) != 0,
            upper: raw & (1 << 14) != 0,
            lower: raw & (1 << 13) != 0,
        }
    }
}

/// Manufacturer and device ID registers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Ids {
    /// Manufacturer ID (register 0x06).
    pub manufacturer: u16,
    /// Device ID (high byte) and revision (low byte), register 0x07.
    pub device: u16,
}

/// One temperature sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Temperature in 0.01 °C.
    pub temperature: i32,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Temperature(self.temperature));
    }
}

/// Decode `T_A` (13-bit two's complement in bits 12:0, 0.0625 °C; flag
/// bits 15:13 are ignored) to 0.01 °C, rounded.
pub const fn decode_temperature(raw: u16) -> i32 {
    let counts = (((raw << 3) as i16) >> 3) as i32;
    (counts * 100 + 8) >> 4
}

/// Encode a limit in 0.01 °C (`T_UPPER`/`T_LOWER`/`T_CRIT`: 11-bit two's
/// complement in bits 12:2, 0.25 °C), saturating.
pub const fn encode_limit(centi_celsius: i32) -> u16 {
    let counts = (centi_celsius * 4 + 50).div_euclid(100);
    let c = if counts < -1024 {
        -1024
    } else if counts > 1023 {
        1023
    } else {
        counts
    };
    ((c << 2) as u16) & 0x1FFC
}

macro_rules! constructors {
    () => {
        /// Create an MCP9808 driver with the default [`Config`]; no bus
        /// traffic.
        pub const fn new(i2c: I2C, address: u8) -> Self {
            Self {
                i2c,
                address,
                variant: Variant::Mcp9808,
                config: Config {
                    resolution: Resolution::Sixteenth,
                    hysteresis: Hysteresis::C0,
                    alert_enabled: false,
                    alert_critical_only: false,
                    alert_active_high: false,
                    alert_mode: AlertMode::Comparator,
                },
                auto_shutdown: false,
            }
        }

        /// Select the generic JC-42.4 variant.
        pub const fn with_variant(mut self, variant: Variant) -> Self {
            self.variant = variant;
            self
        }

        /// Use `config` for [`Self::configure`] and later mode changes.
        pub const fn with_config(mut self, config: Config) -> Self {
            self.config = config;
            self
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }

        const fn conversion_wait(&self) -> WaitUs {
            match self.variant {
                Variant::Mcp9808 => self.config.resolution.conversion_us(),
                Variant::Jc42 => JC42_CONVERSION_US,
            }
        }
    };
}

macro_rules! driver_impl {
    ($i2c:path, $sensor:path, $low_power:path, $($async:ident)?; $($await:tt)*) => {
        impl<I2C: $i2c> Mcp9808<I2C> {
            $($async)? fn write_register(&mut self, register: u8, value: u16) -> Result<(), Error<I2C::Error>> {
                let [hi, lo] = value.to_be_bytes();
                self.i2c
                    .write(self.address, &[register, hi, lo])
                    $($await)*
                    .map_err(Error::bus)
            }

            $($async)? fn read_register(&mut self, register: u8) -> Result<u16, Error<I2C::Error>> {
                let mut buf = [0u8; 2];
                self.i2c
                    .write_read(self.address, &[register], &mut buf)
                    $($await)*
                    .map_err(Error::bus)?;
                Ok(u16::from_be_bytes(buf))
            }

            /// Read the manufacturer and device IDs. For
            /// [`Variant::Mcp9808`] returns [`Error::UnexpectedId`]
            /// (`manufacturer << 16 | device`) unless they are 0x0054 /
            /// 0x04xx; the JC-42.4 variant accepts any ID.
            pub $($async)? fn probe(&mut self) -> Result<Ids, Error<I2C::Error>> {
                let manufacturer = self.read_register(REG_MANUFACTURER_ID) $($await)*?;
                let device = self.read_register(REG_DEVICE_ID) $($await)*?;
                if matches!(self.variant, Variant::Mcp9808)
                    && (manufacturer != MCP9808_MANUFACTURER_ID
                        || device >> 8 != MCP9808_DEVICE_ID as u16)
                {
                    return Err(Error::UnexpectedId(
                        (u32::from(manufacturer) << 16) | u32::from(device),
                    ));
                }
                Ok(Ids { manufacturer, device })
            }

            /// Write the configuration (and, on MCP9808, the resolution) and
            /// enter shutdown. Call once after power-up: the part starts in
            /// continuous mode. Fails silently on the part if the lock bits
            /// were set since power-up.
            pub $($async)? fn configure(&mut self) -> Result<(), Error<I2C::Error>> {
                self.auto_shutdown = false;
                if matches!(self.variant, Variant::Mcp9808) {
                    self.i2c
                        .write(self.address, &[REG_RESOLUTION, self.config.resolution as u8])
                        $($await)*
                        .map_err(Error::bus)?;
                }
                self.write_register(REG_CONFIG, self.config.register(true)) $($await)*
            }

            /// Change the configuration and write it (enters shutdown).
            pub $($async)? fn set_config(&mut self, config: Config) -> Result<(), Error<I2C::Error>> {
                self.config = config;
                self.configure() $($await)*
            }

            /// Program the alert window in 0.01 °C (0.25 °C steps).
            pub $($async)? fn set_limits(&mut self, lower: i32, upper: i32) -> Result<(), Error<I2C::Error>> {
                self.write_register(REG_T_LOWER, encode_limit(lower)) $($await)*?;
                self.write_register(REG_T_UPPER, encode_limit(upper)) $($await)*
            }

            /// Program the critical limit in 0.01 °C.
            pub $($async)? fn set_critical(&mut self, critical: i32) -> Result<(), Error<I2C::Error>> {
                self.write_register(REG_T_CRIT, encode_limit(critical)) $($await)*
            }

            /// Release a latched interrupt-mode Alert (Int. Clear). Writes
            /// the configuration with SHDN = 0, as the bit cannot be set in
            /// shutdown; call [`Self::configure`] afterwards to shut down.
            pub $($async)? fn clear_interrupt(&mut self) -> Result<(), Error<I2C::Error>> {
                self.write_register(REG_CONFIG, self.config.register(false) | CFG_INT_CLEAR) $($await)*
            }

            /// Read the window flags from `T_A`.
            pub $($async)? fn read_flags(&mut self) -> Result<Flags, Error<I2C::Error>> {
                Ok(Flags::from_raw(self.read_register(REG_T_A) $($await)*?))
            }
        }

        impl<I2C: $i2c> $sensor for Mcp9808<I2C> {
            type BusError = I2C::Error;
            type Measurement = Measurement;

            /// Leave shutdown (no one-shot mode); `read_measurement` shuts
            /// the part down again.
            $($async)? fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.write_register(REG_CONFIG, self.config.register(false)) $($await)*?;
                self.auto_shutdown = true;
                Ok(self.conversion_wait())
            }

            $($async)? fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
                let raw = self.read_register(REG_T_A) $($await)*?;
                if self.auto_shutdown {
                    self.auto_shutdown = false;
                    self.write_register(REG_CONFIG, self.config.register(true)) $($await)*?;
                }
                if raw == 0xFFFF {
                    return Err(Error::InvalidData);
                }
                Ok(Measurement { temperature: decode_temperature(raw) })
            }
        }

        impl<I2C: $i2c> $low_power for Mcp9808<I2C> {
            type BusError = I2C::Error;

            /// Shutdown (SHDN = 1).
            $($async)? fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
                self.auto_shutdown = false;
                self.write_register(REG_CONFIG, self.config.register(true)) $($await)*
            }

            /// Continuous conversion. Returns the wait until the first
            /// result.
            $($async)? fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
                self.auto_shutdown = false;
                self.write_register(REG_CONFIG, self.config.register(false)) $($await)*?;
                Ok(self.conversion_wait())
            }
        }
    };
}

/// Blocking driver.
pub struct Mcp9808<I2C> {
    i2c: I2C,
    address: u8,
    variant: Variant,
    config: Config,
    auto_shutdown: bool,
}

impl<I2C> Mcp9808<I2C> {
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

    /// Async twin of [`super::Mcp9808`].
    pub struct Mcp9808<I2C> {
        i2c: I2C,
        address: u8,
        variant: Variant,
        config: Config,
        auto_shutdown: bool,
    }

    impl<I2C> Mcp9808<I2C> {
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

    const A: u8 = ADDRESS_DEFAULT;

    #[test]
    fn decode_ignores_flags() {
        assert_eq!(decode_temperature(0x0190), 2_500);
        assert_eq!(decode_temperature(0xE190), 2_500);
        assert_eq!(decode_temperature(0x1E70), -2_500);
        assert_eq!(decode_temperature(0x0001), 6);
        assert_eq!(decode_temperature(0x1FFF), -6);
        assert_eq!(decode_temperature(0x07D0), 12_500);
        assert_eq!(decode_temperature(0x1C90), -5_500);
        let f = Flags::from_raw(0xA190);
        assert!(f.critical && !f.upper && f.lower);
    }

    #[test]
    fn limit_encoding() {
        assert_eq!(encode_limit(2_500), 0x0190);
        assert_eq!(encode_limit(-2_500), 0x1E70);
        assert_eq!(encode_limit(2_510), 0x0190);
        assert_eq!(encode_limit(2_513), 0x0194);
        assert_eq!(encode_limit(100_000), 0x0FFC);
        assert_eq!(encode_limit(-100_000), 0x1000);
    }

    #[test]
    fn config_word() {
        let c = Config {
            resolution: Resolution::Half,
            hysteresis: Hysteresis::C6,
            alert_enabled: true,
            alert_critical_only: true,
            alert_active_high: true,
            alert_mode: AlertMode::Interrupt,
        };
        assert_eq!(c.register(true), 0x070F);
        assert_eq!(Config::default().register(false), 0x0000);
    }

    #[test]
    fn probe_checks_ids() {
        let expectations = [
            Transaction::write_read(A, vec![REG_MANUFACTURER_ID], vec![0x00, 0x54]),
            Transaction::write_read(A, vec![REG_DEVICE_ID], vec![0x04, 0x01]),
            Transaction::write_read(A, vec![REG_MANUFACTURER_ID], vec![0x11, 0x31]),
            Transaction::write_read(A, vec![REG_DEVICE_ID], vec![0x22, 0x01]),
            Transaction::write_read(A, vec![REG_MANUFACTURER_ID], vec![0x11, 0x31]),
            Transaction::write_read(A, vec![REG_DEVICE_ID], vec![0x22, 0x01]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut mcp = Mcp9808::new(&mut i2c, A);
        assert_eq!(
            mcp.probe(),
            Ok(Ids {
                manufacturer: 0x0054,
                device: 0x0401
            })
        );
        assert_eq!(mcp.probe(), Err(Error::UnexpectedId(0x1131_2201)));
        let mut jc = mcp.with_variant(Variant::Jc42);
        assert_eq!(jc.probe().unwrap().manufacturer, 0x1131);
        i2c.done();
    }

    #[test]
    fn emulated_one_shot() {
        let expectations = [
            Transaction::write(A, vec![REG_RESOLUTION, 0x01]),
            Transaction::write(A, vec![REG_CONFIG, 0x01, 0x08]),
            Transaction::write(A, vec![REG_CONFIG, 0x00, 0x08]),
            Transaction::write_read(A, vec![REG_T_A], vec![0xC1, 0x94]),
            Transaction::write(A, vec![REG_CONFIG, 0x01, 0x08]),
        ];
        let config = Config {
            resolution: Resolution::Quarter,
            alert_enabled: true,
            ..Config::default()
        };
        let mut i2c = Mock::new(&expectations);
        let mut mcp = Mcp9808::new(&mut i2c, A).with_config(config);
        mcp.configure().unwrap();
        assert_eq!(mcp.start_measurement().unwrap(), 97_500);
        assert_eq!(mcp.read_measurement().unwrap().temperature, 2_525);
        i2c.done();
    }

    #[test]
    fn limits_interrupt_and_power() {
        let expectations = [
            Transaction::write(A, vec![REG_T_LOWER, 0x00, 0x40]),
            Transaction::write(A, vec![REG_T_UPPER, 0x01, 0xE0]),
            Transaction::write(A, vec![REG_T_CRIT, 0x02, 0x80]),
            Transaction::write(A, vec![REG_CONFIG, 0x00, 0x20]),
            Transaction::write_read(A, vec![REG_T_A], vec![0x41, 0xF0]),
            Transaction::write(A, vec![REG_CONFIG, 0x00, 0x00]),
            Transaction::write_read(A, vec![REG_T_A], vec![0xFF, 0xFF]),
            Transaction::write(A, vec![REG_CONFIG, 0x01, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut mcp = Mcp9808::new(&mut i2c, A).with_variant(Variant::Jc42);
        mcp.set_limits(400, 3_000).unwrap();
        mcp.set_critical(4_000).unwrap();
        mcp.clear_interrupt().unwrap();
        assert!(mcp.read_flags().unwrap().upper);
        assert_eq!(mcp.wake().unwrap(), JC42_CONVERSION_US);
        assert_eq!(mcp.read_measurement(), Err(Error::InvalidData));
        mcp.sleep().unwrap();
        i2c.done();
    }

    #[test]
    fn async_emulated_one_shot() {
        use zigbee_sensor::asynch::Sensor as _;
        let expectations = [
            Transaction::write(A, vec![REG_CONFIG, 0x00, 0x00]),
            Transaction::write_read(A, vec![REG_T_A], vec![0x1E, 0x70]),
            Transaction::write(A, vec![REG_CONFIG, 0x01, 0x00]),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut mcp = asynch::Mcp9808::new(&mut i2c, A);
        assert_eq!(block_on(mcp.start_measurement()).unwrap(), 375_000);
        assert_eq!(
            block_on(mcp.read_measurement()).unwrap().temperature,
            -2_500
        );
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
