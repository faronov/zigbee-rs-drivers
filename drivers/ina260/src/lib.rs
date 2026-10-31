//! Texas Instruments INA260 precision current, bus voltage and power
//! monitor with integrated 2 mΩ shunt (I2C, 16-bit).
//!
//! The internal shunt is factory-trimmed, so no calibration register exists:
//! current is 1.25 mA/LSB (signed), bus voltage 1.25 mV/LSB and power
//! 10 mW/LSB. The power register holds the magnitude `|I| × V`; the driver
//! reports it unsigned, as the device does.
//!
//! In [`Mode::Triggered`] (default) each `start_measurement` starts one
//! averaged current + bus conversion and returns `AVG × (VBUSCT + ISHCT)` plus
//! the 10 % maximum tolerance; the device is in power-down in between.
//!
//! Reference: TI "INA260 Precision Digital Current and Power Monitor With
//! Low-Drift, Precision Integrated Shunt", SBOS656C (December 2016).

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Address with A1 = A0 = GND. A0/A1 (GND/VS/SDA/SCL) select 0x40..=0x4F.
pub const DEFAULT_ADDRESS: u8 = 0x40;

/// Register addresses.
pub mod reg {
    /// Configuration.
    pub const CONFIG: u8 = 0x00;
    /// Current (1.25 mA LSB, signed).
    pub const CURRENT: u8 = 0x01;
    /// Bus voltage (1.25 mV LSB).
    pub const BUS: u8 = 0x02;
    /// Power (10 mW LSB).
    pub const POWER: u8 = 0x03;
    /// Mask/Enable (alert function select, flags).
    pub const MASK_ENABLE: u8 = 0x06;
    /// Alert limit.
    pub const ALERT_LIMIT: u8 = 0x07;
    /// Manufacturer ID.
    pub const MANUFACTURER_ID: u8 = 0xFE;
    /// Die ID.
    pub const DIE_ID: u8 = 0xFF;
}

/// Manufacturer ID register value ("TI").
pub const MANUFACTURER_ID: u16 = 0x5449;
/// Die ID (bits 15..4 of register FFh; bits 3..0 are the revision).
pub const DIE_ID: u16 = 0x227;

const CONFIG_RESET: u16 = 1 << 15;
/// Bits 14..12 of the reset value 6127h (read-only, preserved on writes).
const CONFIG_FIXED: u16 = 0b110 << 12;
const MODE_POWER_DOWN: u16 = 0b000;
const MODE_CURRENT_BUS_TRIGGERED: u16 = 0b011;
const MODE_CURRENT_BUS_CONTINUOUS: u16 = 0b111;

/// Mask/Enable: conversion ready flag.
pub const MASK_CVRF: u16 = 1 << 3;
/// Mask/Enable: math overflow flag.
pub const MASK_OVF: u16 = 1 << 2;

/// Recovery time from power-down (section 8.3.3: 40 µs).
pub const POWER_DOWN_RECOVERY_US: WaitUs = 40;

/// Number of averaged samples (AVG bits 11..9).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Averaging {
    /// 1 sample (power-on default).
    #[default]
    X1,
    /// 4 samples.
    X4,
    /// 16 samples.
    X16,
    /// 64 samples.
    X64,
    /// 128 samples.
    X128,
    /// 256 samples.
    X256,
    /// 512 samples.
    X512,
    /// 1024 samples.
    X1024,
}

impl Averaging {
    /// Number of samples.
    pub const fn samples(self) -> u32 {
        1 << match self {
            Self::X1 => 0,
            Self::X4 => 2,
            Self::X16 => 4,
            Self::X64 => 6,
            Self::X128 => 7,
            Self::X256 => 8,
            Self::X512 => 9,
            Self::X1024 => 10,
        }
    }
}

/// ADC conversion time per sample (VBUSCT / ISHCT).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ConversionTime {
    /// 140 µs.
    Us140,
    /// 204 µs.
    Us204,
    /// 332 µs.
    Us332,
    /// 588 µs.
    Us588,
    /// 1.1 ms (power-on default).
    #[default]
    Us1100,
    /// 2.116 ms.
    Us2116,
    /// 4.156 ms.
    Us4156,
    /// 8.244 ms.
    Us8244,
}

impl ConversionTime {
    /// Typical conversion time in µs.
    pub const fn micros(self) -> u32 {
        match self {
            Self::Us140 => 140,
            Self::Us204 => 204,
            Self::Us332 => 332,
            Self::Us588 => 588,
            Self::Us1100 => 1_100,
            Self::Us2116 => 2_116,
            Self::Us4156 => 4_156,
            Self::Us8244 => 8_244,
        }
    }
}

/// Conversion mode used by the [`zigbee_sensor::blocking::Sensor`] impl.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Mode {
    /// One averaged current + bus conversion per `start_measurement`.
    #[default]
    Triggered,
    /// Free-running current + bus conversions.
    Continuous,
}

/// Averaging and conversion time (Configuration register 00h).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// Samples averaged per result.
    pub averaging: Averaging,
    /// Bus voltage conversion time.
    pub bus_conversion: ConversionTime,
    /// Shunt current conversion time.
    pub current_conversion: ConversionTime,
}

impl Config {
    /// Configuration register value with the given MODE bits.
    pub const fn register(&self, mode_bits: u16) -> u16 {
        CONFIG_FIXED
            | ((self.averaging as u16) << 9)
            | ((self.bus_conversion as u16) << 6)
            | ((self.current_conversion as u16) << 3)
            | (mode_bits & 0b111)
    }

    /// Worst-case time of one averaged result in µs (typical + 10 %).
    pub const fn conversion_us(&self) -> WaitUs {
        let typ = self.averaging.samples()
            * (self.bus_conversion.micros() + self.current_conversion.micros());
        typ + typ / 10
    }
}

/// One measurement.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Bus voltage in mV.
    pub bus_voltage: i32,
    /// Current in µA (positive from IN+ to IN−).
    pub current: i32,
    /// Power magnitude in mW.
    pub power: i32,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Voltage(self.bus_voltage));
        f(Quantity::Current(self.current));
        f(Quantity::Power(self.power));
    }
}

/// Current in µA from the raw Current register (1.25 mA LSB).
pub const fn current_microamps(raw: i16) -> i32 {
    raw as i32 * 1_250
}

/// Bus voltage in mV from the raw Bus Voltage register (1.25 mV LSB).
pub const fn bus_millivolts(raw: u16) -> i32 {
    (raw as i32 * 5 + 2) / 4
}

/// Power in mW from the raw Power register (10 mW LSB).
pub const fn power_milliwatts(raw: u16) -> i32 {
    raw as i32 * 10
}

/// Decode the result registers; OVF in `mask_enable` gives
/// [`Error::InvalidData`].
pub fn decode<E>(
    mask_enable: u16,
    current: u16,
    bus: u16,
    power: u16,
) -> Result<Measurement, Error<E>> {
    if mask_enable & MASK_OVF != 0 {
        return Err(Error::InvalidData);
    }
    Ok(Measurement {
        bus_voltage: bus_millivolts(bus),
        current: current_microamps(current as i16),
        power: power_milliwatts(power),
    })
}

/// Check the ID registers (FEh, FFh).
pub fn check_id<E>(manufacturer: u16, die: u16) -> Result<(), Error<E>> {
    if manufacturer != MANUFACTURER_ID || die >> 4 != DIE_ID {
        return Err(Error::UnexpectedId(
            (u32::from(manufacturer) << 16) | u32::from(die),
        ));
    }
    Ok(())
}

macro_rules! constructors {
    () => {
        /// Create a driver; no bus traffic. Call `init` before measuring.
        pub const fn new(i2c: I2C, address: u8) -> Self {
            Self {
                i2c,
                address,
                config: Config {
                    averaging: Averaging::X1,
                    bus_conversion: ConversionTime::Us1100,
                    current_conversion: ConversionTime::Us1100,
                },
                mode: Mode::Triggered,
            }
        }

        /// Averaging / conversion time written by `init`.
        pub const fn with_config(mut self, config: Config) -> Self {
            self.config = config;
            self
        }

        /// Triggered (default) or continuous conversions.
        pub const fn with_mode(mut self, mode: Mode) -> Self {
            self.mode = mode;
            self
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }

        const fn idle_config(&self) -> u16 {
            match self.mode {
                Mode::Triggered => self.config.register(MODE_POWER_DOWN),
                Mode::Continuous => self.config.register(MODE_CURRENT_BUS_CONTINUOUS),
            }
        }
    };
}

/// Blocking driver.
pub struct Ina260<I2C> {
    i2c: I2C,
    address: u8,
    config: Config,
    mode: Mode,
}

impl<I2C> Ina260<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Ina260<I2C> {
    /// Write a 16-bit register.
    pub fn write_register(&mut self, register: u8, value: u16) -> Result<(), Error<I2C::Error>> {
        let [msb, lsb] = value.to_be_bytes();
        self.i2c
            .write(self.address, &[register, msb, lsb])
            .map_err(Error::bus)
    }

    /// Read a 16-bit register.
    pub fn read_register(&mut self, register: u8) -> Result<u16, Error<I2C::Error>> {
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(self.address, &[register], &mut buf)
            .map_err(Error::bus)?;
        Ok(u16::from_be_bytes(buf))
    }

    /// Verify the manufacturer and die ID.
    pub fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
        let manufacturer = self.read_register(reg::MANUFACTURER_ID)?;
        let die = self.read_register(reg::DIE_ID)?;
        check_id(manufacturer, die)
    }

    /// Write the configuration (power-down in triggered mode).
    pub fn init(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_register(reg::CONFIG, self.idle_config())
    }

    /// Reset all registers to defaults. Call `init` afterwards.
    pub fn reset(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_register(reg::CONFIG, CONFIG_RESET)
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Ina260<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        match self.mode {
            Mode::Triggered => {
                self.write_register(
                    reg::CONFIG,
                    self.config.register(MODE_CURRENT_BUS_TRIGGERED),
                )?;
                Ok(self.config.conversion_us())
            }
            Mode::Continuous => Ok(0),
        }
    }

    /// Reading Mask/Enable clears CVRF; in triggered mode returns
    /// [`Error::NotReady`] until it is set.
    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        let mask = self.read_register(reg::MASK_ENABLE)?;
        if self.mode == Mode::Triggered && mask & MASK_CVRF == 0 {
            return Err(Error::NotReady);
        }
        let current = self.read_register(reg::CURRENT)?;
        let bus = self.read_register(reg::BUS)?;
        let power = self.read_register(reg::POWER)?;
        decode(mask, current, bus, power)
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::LowPower for Ina260<I2C> {
    type BusError = I2C::Error;

    fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_register(reg::CONFIG, self.config.register(MODE_POWER_DOWN))
    }

    fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.write_register(reg::CONFIG, self.idle_config())?;
        Ok(POWER_DOWN_RECOVERY_US)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        Averaging, CONFIG_RESET, Config, ConversionTime, MASK_CVRF, MODE_CURRENT_BUS_CONTINUOUS,
        MODE_CURRENT_BUS_TRIGGERED, MODE_POWER_DOWN, Measurement, Mode, POWER_DOWN_RECOVERY_US,
        check_id, decode, reg,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Ina260`].
    pub struct Ina260<I2C> {
        i2c: I2C,
        address: u8,
        config: Config,
        mode: Mode,
    }

    impl<I2C> Ina260<I2C> {
        constructors!();
    }

    impl<I2C: I2c> Ina260<I2C> {
        /// See [`super::Ina260::write_register`].
        pub async fn write_register(
            &mut self,
            register: u8,
            value: u16,
        ) -> Result<(), Error<I2C::Error>> {
            let [msb, lsb] = value.to_be_bytes();
            self.i2c
                .write(self.address, &[register, msb, lsb])
                .await
                .map_err(Error::bus)
        }

        /// See [`super::Ina260::read_register`].
        pub async fn read_register(&mut self, register: u8) -> Result<u16, Error<I2C::Error>> {
            let mut buf = [0u8; 2];
            self.i2c
                .write_read(self.address, &[register], &mut buf)
                .await
                .map_err(Error::bus)?;
            Ok(u16::from_be_bytes(buf))
        }

        /// See [`super::Ina260::probe`].
        pub async fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
            let manufacturer = self.read_register(reg::MANUFACTURER_ID).await?;
            let die = self.read_register(reg::DIE_ID).await?;
            check_id(manufacturer, die)
        }

        /// See [`super::Ina260::init`].
        pub async fn init(&mut self) -> Result<(), Error<I2C::Error>> {
            self.write_register(reg::CONFIG, self.idle_config()).await
        }

        /// See [`super::Ina260::reset`].
        pub async fn reset(&mut self) -> Result<(), Error<I2C::Error>> {
            self.write_register(reg::CONFIG, CONFIG_RESET).await
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Ina260<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            match self.mode {
                Mode::Triggered => {
                    self.write_register(
                        reg::CONFIG,
                        self.config.register(MODE_CURRENT_BUS_TRIGGERED),
                    )
                    .await?;
                    Ok(self.config.conversion_us())
                }
                Mode::Continuous => Ok(0),
            }
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let mask = self.read_register(reg::MASK_ENABLE).await?;
            if self.mode == Mode::Triggered && mask & MASK_CVRF == 0 {
                return Err(Error::NotReady);
            }
            let current = self.read_register(reg::CURRENT).await?;
            let bus = self.read_register(reg::BUS).await?;
            let power = self.read_register(reg::POWER).await?;
            decode(mask, current, bus, power)
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::LowPower for Ina260<I2C> {
        type BusError = I2C::Error;

        async fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
            self.write_register(reg::CONFIG, self.config.register(MODE_POWER_DOWN))
                .await
        }

        async fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.write_register(reg::CONFIG, self.idle_config()).await?;
            Ok(POWER_DOWN_RECOVERY_US)
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

    const ADDR: u8 = DEFAULT_ADDRESS;

    fn read(reg: u8, value: u16) -> Transaction {
        Transaction::write_read(ADDR, vec![reg], value.to_be_bytes().to_vec())
    }

    fn write(reg: u8, value: u16) -> Transaction {
        let [m, l] = value.to_be_bytes();
        Transaction::write(ADDR, vec![reg, m, l])
    }

    /// 10 A at 11.98 V: 8000 × 1.25 mA, 9584 × 1.25 mV, 11974 × 10 mW.
    fn example_reads(mask: u16) -> [Transaction; 4] {
        [
            read(reg::MASK_ENABLE, mask),
            read(reg::CURRENT, 8_000),
            read(reg::BUS, 0x2570),
            read(reg::POWER, 11_974),
        ]
    }

    const EXAMPLE: Measurement = Measurement {
        bus_voltage: 11_980,
        current: 10_000_000,
        power: 119_740,
    };

    #[test]
    fn conversions_and_config() {
        // Section 8.6.2: two's complement, 1.25 mA LSB.
        assert_eq!(current_microamps(-1), -1_250);
        assert_eq!(current_microamps(i16::MAX), 40_958_750);
        assert_eq!(bus_millivolts(0x7FFF), 40_959);
        assert_eq!(power_milliwatts(0xFFFF), 655_350);
        // Reset value 6127h with continuous mode.
        assert_eq!(Config::default().register(0b111), 0x6127);
        let config = Config {
            averaging: Averaging::X16,
            bus_conversion: ConversionTime::Us140,
            current_conversion: ConversionTime::Us8244,
        };
        assert_eq!(config.register(0), 0x6438);
        assert_eq!(config.conversion_us(), 147_558);
    }

    #[test]
    fn probe_checks_ids() {
        let expectations = [
            read(reg::MANUFACTURER_ID, 0x5449),
            read(reg::DIE_ID, 0x2270),
            read(reg::MANUFACTURER_ID, 0x5449),
            read(reg::DIE_ID, 0x2260),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut ina = Ina260::new(&mut i2c, ADDR);
        ina.probe().unwrap();
        assert_eq!(ina.probe(), Err(Error::UnexpectedId(0x5449_2260)));
        i2c.done();
    }

    #[test]
    fn triggered_measurement() {
        let mut expectations = vec![
            write(reg::CONFIG, 0x6120),
            write(reg::CONFIG, 0x6123),
            read(reg::MASK_ENABLE, 0),
        ];
        expectations.extend(example_reads(MASK_CVRF));
        let mut i2c = Mock::new(&expectations);
        let mut ina = Ina260::new(&mut i2c, ADDR);
        ina.init().unwrap();
        assert_eq!(ina.start_measurement().unwrap(), 2_420);
        assert_eq!(ina.read_measurement(), Err(Error::NotReady));
        assert_eq!(ina.read_measurement().unwrap(), EXAMPLE);
        i2c.done();
    }

    #[test]
    fn continuous_overflow_and_power_down() {
        let mut expectations = vec![write(reg::CONFIG, 0x6127)];
        expectations.extend(example_reads(MASK_OVF));
        expectations.extend([write(reg::CONFIG, 0x6120), write(reg::CONFIG, 0x6127)]);
        let mut i2c = Mock::new(&expectations);
        let mut ina = Ina260::new(&mut i2c, ADDR).with_mode(Mode::Continuous);
        ina.init().unwrap();
        assert_eq!(ina.start_measurement().unwrap(), 0);
        assert_eq!(ina.read_measurement(), Err(Error::InvalidData));
        ina.sleep().unwrap();
        assert_eq!(ina.wake().unwrap(), 40);
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

        let mut expectations = vec![write(reg::CONFIG, 0x6123)];
        expectations.extend(example_reads(MASK_CVRF));
        let mut i2c = Mock::new(&expectations);
        let mut ina = asynch::Ina260::new(&mut i2c, ADDR);
        let mut delay = RecordingDelay(0);
        assert_eq!(block_on(ina.measure(&mut delay)).unwrap(), EXAMPLE);
        assert_eq!(delay.0, 2_420);
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
