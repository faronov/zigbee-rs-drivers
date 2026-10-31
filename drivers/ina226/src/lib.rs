//! Texas Instruments INA226 / INA230 / INA231 current, bus voltage and power
//! monitors (I2C, 16-bit).
//!
//! All three parts share the register map used here: shunt voltage
//! (2.5 µV LSB), bus voltage (1.25 mV LSB), current and power computed from
//! a calibration register (`CAL = 0.00512 / (Current_LSB × R_shunt)`, power
//! LSB = 25 × Current_LSB), Mask/Enable with the conversion-ready flag and
//! an alert limit. Differences:
//!
//! - INA226: bus input 0–36 V, separate VBUS pin, manufacturer/die ID
//!   registers (`probe`).
//! - INA230: 0–28 V common mode, die ID register without documented value.
//! - INA231: WCSP, 0–28 V, no ID registers.
//!
//! In [`Mode::Triggered`] (default) each `start_measurement` starts one
//! averaged shunt + bus conversion and returns `AVG × (VBUSCT + VSHCT)` plus
//! the 10 % maximum tolerance; the device draws power-down current in
//! between. `read_measurement` returns [`Error::NotReady`] until CVRF is set.
//!
//! References: TI SBOS547C (INA226), SBOS601B (INA230), SBOS644D (INA231).

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Address with A1 = A0 = GND. A0/A1 (GND/VS/SDA/SCL) select 0x40..=0x4F.
pub const DEFAULT_ADDRESS: u8 = 0x40;

/// Register addresses.
pub mod reg {
    /// Configuration.
    pub const CONFIG: u8 = 0x00;
    /// Shunt voltage (2.5 µV LSB, signed).
    pub const SHUNT: u8 = 0x01;
    /// Bus voltage (1.25 mV LSB).
    pub const BUS: u8 = 0x02;
    /// Power (25 × Current_LSB).
    pub const POWER: u8 = 0x03;
    /// Current (Current_LSB, signed).
    pub const CURRENT: u8 = 0x04;
    /// Calibration.
    pub const CALIBRATION: u8 = 0x05;
    /// Mask/Enable (alert function select, flags).
    pub const MASK_ENABLE: u8 = 0x06;
    /// Alert limit.
    pub const ALERT_LIMIT: u8 = 0x07;
    /// Manufacturer ID (INA226 only).
    pub const MANUFACTURER_ID: u8 = 0xFE;
    /// Die ID (INA226; INA230 undocumented value).
    pub const DIE_ID: u8 = 0xFF;
}

/// Manufacturer ID register value ("TI").
pub const MANUFACTURER_ID: u16 = 0x5449;
/// INA226 die ID (bits 15..4 of register FFh; bits 3..0 are the revision).
pub const INA226_DIE_ID: u16 = 0x226;

const CONFIG_RESET: u16 = 1 << 15;
const MODE_POWER_DOWN: u16 = 0b000;
const MODE_SHUNT_BUS_TRIGGERED: u16 = 0b011;
const MODE_SHUNT_BUS_CONTINUOUS: u16 = 0b111;

/// Mask/Enable: conversion ready flag.
pub const MASK_CVRF: u16 = 1 << 3;
/// Mask/Enable: math overflow flag.
pub const MASK_OVF: u16 = 1 << 2;

/// Recovery time from power-down (40 µs in all three datasheets).
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
        match self {
            Self::X1 => 1,
            Self::X4 => 4,
            Self::X16 => 16,
            Self::X64 => 64,
            Self::X128 => 128,
            Self::X256 => 256,
            Self::X512 => 512,
            Self::X1024 => 1024,
        }
    }
}

/// ADC conversion time per sample (VBUSCT / VSHCT).
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

    const fn bits(self) -> u16 {
        self as u16
    }
}

/// Conversion mode used by the [`zigbee_sensor::blocking::Sensor`] impl.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Mode {
    /// One averaged shunt + bus conversion per `start_measurement`.
    #[default]
    Triggered,
    /// Free-running shunt + bus conversions.
    Continuous,
}

/// Averaging and conversion time (Configuration register 00h).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// Samples averaged per result.
    pub averaging: Averaging,
    /// Bus voltage conversion time.
    pub bus_conversion: ConversionTime,
    /// Shunt voltage conversion time.
    pub shunt_conversion: ConversionTime,
}

impl Config {
    /// Configuration register value with the given MODE bits (bit 14 is
    /// the fixed `1` of the reset value 4127h).
    pub const fn register(&self, mode_bits: u16) -> u16 {
        (1 << 14)
            | ((self.averaging as u16) << 9)
            | (self.bus_conversion.bits() << 6)
            | (self.shunt_conversion.bits() << 3)
            | (mode_bits & 0b111)
    }

    /// Worst-case time of one averaged shunt + bus result in µs (typical
    /// conversion times + 10 %, the maximum in the electrical
    /// characteristics).
    pub const fn conversion_us(&self) -> WaitUs {
        let typ = self.averaging.samples()
            * (self.bus_conversion.micros() + self.shunt_conversion.micros());
        typ + typ / 10
    }
}

/// Calibration register value and integer scale factors (Q24 µA and mW per
/// LSB) derived from it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Calibration {
    /// Value written to the Calibration register (05h).
    pub register: u16,
    /// µA per Current LSB, Q24.
    pub current_q24: i64,
    /// mW per Power LSB, Q24.
    pub power_q24: i64,
}

impl Calibration {
    /// Calibration for a shunt of `shunt_micro_ohm` µΩ and a Current LSB of
    /// `current_lsb_na` nA (equation 1: `CAL = 0.00512 / (Current_LSB ×
    /// R_SHUNT)`, truncated).
    ///
    /// Scale factors follow the truncated register value. Returns `None`
    /// when CAL is 0 or above 0x7FFF, or full-scale current / power would not
    /// fit `i32`.
    pub const fn new(shunt_micro_ohm: u32, current_lsb_na: u32) -> Option<Self> {
        if shunt_micro_ohm == 0 || current_lsb_na == 0 {
            return None;
        }
        // 0.00512 / (nA × 1e-9 × µΩ × 1e-6) = 5.12e12 / (nA × µΩ).
        let cal = 5_120_000_000_000 / (current_lsb_na as u64 * shunt_micro_ohm as u64);
        if cal == 0 || cal > 0x7FFF {
            return None;
        }
        let denominator = cal * shunt_micro_ohm as u64;
        // Effective Current_LSB in µA = 5.12e9 / (CAL × µΩ); power is 25×.
        let current_q24 = q24(5_120_000_000, denominator);
        let power_q24 = q24(128_000_000, denominator);
        if current_q24 >= 1 << 40 || power_q24 >= 1 << 39 {
            return None;
        }
        Some(Self {
            register: cal as u16,
            current_q24,
            power_q24,
        })
    }

    /// Calibration with the smallest current LSB that covers
    /// `max_current_ma` (equation 2: `I_max / 2^15`, rounded up to a nA).
    pub const fn for_max_current(shunt_micro_ohm: u32, max_current_ma: u32) -> Option<Self> {
        let lsb = (max_current_ma as u64 * 1_000_000).div_ceil(32_768);
        if lsb > u32::MAX as u64 {
            return None;
        }
        Self::new(shunt_micro_ohm, lsb as u32)
    }

    /// Current in µA from a raw Current register value.
    pub const fn current_ua(&self, raw: i16) -> i32 {
        ((raw as i64 * self.current_q24 + (1 << 23)) >> 24) as i32
    }

    /// Power in mW from a raw Power register value.
    pub const fn power_mw(&self, raw: u16) -> i32 {
        ((raw as i64 * self.power_q24 + (1 << 23)) >> 24) as i32
    }
}

const fn q24(num: u64, den: u64) -> i64 {
    (((num << 24) + den / 2) / den) as i64
}

/// One measurement.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Bus voltage in mV.
    pub bus_voltage: i32,
    /// Shunt voltage in µV.
    pub shunt_voltage: i32,
    /// Current in µA (positive from IN+ to IN−).
    pub current: i32,
    /// Power in mW.
    pub power: i32,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Voltage(self.bus_voltage));
        f(Quantity::Current(self.current));
        f(Quantity::Power(self.power));
    }
}

/// Bus voltage in mV from the raw Bus Voltage register (1.25 mV LSB).
pub const fn bus_millivolts(raw: u16) -> i32 {
    (raw as i32 * 5 + 2) / 4
}

/// Shunt voltage in µV from the raw Shunt Voltage register (2.5 µV LSB,
/// rounded half away from zero).
pub const fn shunt_microvolts(raw: i16) -> i32 {
    let x = raw as i32 * 5;
    if x >= 0 { (x + 1) / 2 } else { (x - 1) / 2 }
}

/// Decode the result registers. A math overflow (OVF in `mask_enable`)
/// gives [`Error::InvalidData`].
pub fn decode<E>(
    calibration: &Calibration,
    mask_enable: u16,
    shunt: u16,
    bus: u16,
    current: u16,
    power: u16,
) -> Result<Measurement, Error<E>> {
    if mask_enable & MASK_OVF != 0 {
        return Err(Error::InvalidData);
    }
    Ok(Measurement {
        bus_voltage: bus_millivolts(bus),
        shunt_voltage: shunt_microvolts(shunt as i16),
        current: calibration.current_ua(current as i16),
        power: calibration.power_mw(power),
    })
}

/// Check the INA226 ID registers (FEh, FFh).
pub fn check_id<E>(manufacturer: u16, die: u16) -> Result<(), Error<E>> {
    if manufacturer != MANUFACTURER_ID || die >> 4 != INA226_DIE_ID {
        return Err(Error::UnexpectedId(
            (u32::from(manufacturer) << 16) | u32::from(die),
        ));
    }
    Ok(())
}

macro_rules! constructors {
    () => {
        /// Create a driver; no bus traffic. Call `init` before measuring.
        pub const fn new(i2c: I2C, address: u8, calibration: Calibration) -> Self {
            Self {
                i2c,
                address,
                calibration,
                config: Config {
                    averaging: Averaging::X1,
                    bus_conversion: ConversionTime::Us1100,
                    shunt_conversion: ConversionTime::Us1100,
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

        /// Active calibration.
        pub const fn calibration(&self) -> &Calibration {
            &self.calibration
        }

        /// Give the bus back.
        pub fn release(self) -> I2C {
            self.i2c
        }

        const fn idle_config(&self) -> u16 {
            match self.mode {
                Mode::Triggered => self.config.register(MODE_POWER_DOWN),
                Mode::Continuous => self.config.register(MODE_SHUNT_BUS_CONTINUOUS),
            }
        }
    };
}

/// Blocking driver.
pub struct Ina226<I2C> {
    i2c: I2C,
    address: u8,
    calibration: Calibration,
    config: Config,
    mode: Mode,
}

impl<I2C> Ina226<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Ina226<I2C> {
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

    /// Verify the manufacturer and die ID (INA226 only; INA230/INA231 have
    /// no documented ID values).
    pub fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
        let manufacturer = self.read_register(reg::MANUFACTURER_ID)?;
        let die = self.read_register(reg::DIE_ID)?;
        check_id(manufacturer, die)
    }

    /// Write calibration and configuration (power-down in triggered mode).
    pub fn init(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_register(reg::CALIBRATION, self.calibration.register)?;
        self.write_register(reg::CONFIG, self.idle_config())
    }

    /// Reset all registers to defaults. Call `init` afterwards.
    pub fn reset(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_register(reg::CONFIG, CONFIG_RESET)
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Ina226<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        match self.mode {
            Mode::Triggered => {
                self.write_register(reg::CONFIG, self.config.register(MODE_SHUNT_BUS_TRIGGERED))?;
                Ok(self.config.conversion_us())
            }
            Mode::Continuous => Ok(0),
        }
    }

    /// Reading Mask/Enable clears CVRF, so each triggered result is read once.
    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        let mask = self.read_register(reg::MASK_ENABLE)?;
        if self.mode == Mode::Triggered && mask & MASK_CVRF == 0 {
            return Err(Error::NotReady);
        }
        let shunt = self.read_register(reg::SHUNT)?;
        let bus = self.read_register(reg::BUS)?;
        let current = self.read_register(reg::CURRENT)?;
        let power = self.read_register(reg::POWER)?;
        decode(&self.calibration, mask, shunt, bus, current, power)
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::LowPower for Ina226<I2C> {
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
        Averaging, CONFIG_RESET, Calibration, Config, ConversionTime, MASK_CVRF, MODE_POWER_DOWN,
        MODE_SHUNT_BUS_CONTINUOUS, MODE_SHUNT_BUS_TRIGGERED, Measurement, Mode,
        POWER_DOWN_RECOVERY_US, check_id, decode, reg,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Ina226`].
    pub struct Ina226<I2C> {
        i2c: I2C,
        address: u8,
        calibration: Calibration,
        config: Config,
        mode: Mode,
    }

    impl<I2C> Ina226<I2C> {
        constructors!();
    }

    impl<I2C: I2c> Ina226<I2C> {
        /// See [`super::Ina226::write_register`].
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

        /// See [`super::Ina226::read_register`].
        pub async fn read_register(&mut self, register: u8) -> Result<u16, Error<I2C::Error>> {
            let mut buf = [0u8; 2];
            self.i2c
                .write_read(self.address, &[register], &mut buf)
                .await
                .map_err(Error::bus)?;
            Ok(u16::from_be_bytes(buf))
        }

        /// See [`super::Ina226::probe`].
        pub async fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
            let manufacturer = self.read_register(reg::MANUFACTURER_ID).await?;
            let die = self.read_register(reg::DIE_ID).await?;
            check_id(manufacturer, die)
        }

        /// See [`super::Ina226::init`].
        pub async fn init(&mut self) -> Result<(), Error<I2C::Error>> {
            self.write_register(reg::CALIBRATION, self.calibration.register)
                .await?;
            self.write_register(reg::CONFIG, self.idle_config()).await
        }

        /// See [`super::Ina226::reset`].
        pub async fn reset(&mut self) -> Result<(), Error<I2C::Error>> {
            self.write_register(reg::CONFIG, CONFIG_RESET).await
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Ina226<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            match self.mode {
                Mode::Triggered => {
                    self.write_register(
                        reg::CONFIG,
                        self.config.register(MODE_SHUNT_BUS_TRIGGERED),
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
            let shunt = self.read_register(reg::SHUNT).await?;
            let bus = self.read_register(reg::BUS).await?;
            let current = self.read_register(reg::CURRENT).await?;
            let power = self.read_register(reg::POWER).await?;
            decode(&self.calibration, mask, shunt, bus, current, power)
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::LowPower for Ina226<I2C> {
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

    /// INA226 section 6.5.1: 2 mΩ shunt, 1 mA Current_LSB.
    fn example_calibration() -> Calibration {
        Calibration::new(2_000, 1_000_000).unwrap()
    }

    fn read(reg: u8, value: u16) -> Transaction {
        Transaction::write_read(ADDR, vec![reg], value.to_be_bytes().to_vec())
    }

    fn write(reg: u8, value: u16) -> Transaction {
        let [m, l] = value.to_be_bytes();
        Transaction::write(ADDR, vec![reg, m, l])
    }

    fn example_reads(mask: u16) -> [Transaction; 5] {
        [
            read(reg::MASK_ENABLE, mask),
            // 8000 × 2.5 µV = 20 mV; bus 2570h = 9584 → 11.98 V.
            read(reg::SHUNT, 8_000),
            read(reg::BUS, 0x2570),
            read(reg::CURRENT, 0x2710),
            read(reg::POWER, 0x12B8),
        ]
    }

    #[test]
    fn calibration_matches_datasheet_example() {
        let cal = example_calibration();
        assert_eq!(cal.register, 0x0A00);
        assert_eq!(cal.current_ua(0x2710), 10_000_000);
        // 4792 × 25 mW = 119.8 W.
        assert_eq!(cal.power_mw(0x12B8), 119_800);
        assert_eq!(Calibration::new(1, 1), None);
        assert_eq!(Calibration::new(1_000, 1), None);
        assert_eq!(
            Calibration::for_max_current(2_000, 15_000)
                .unwrap()
                .register,
            5_592
        );
    }

    #[test]
    fn conversions() {
        assert_eq!(bus_millivolts(0x2570), 11_980);
        assert_eq!(bus_millivolts(0x7FFF), 40_959);
        assert_eq!(shunt_microvolts(8_000), 20_000);
        // Section 7.1.2: −80 mV reads 8300h.
        assert_eq!(shunt_microvolts(0x8300_u16 as i16), -80_000);
        assert_eq!(shunt_microvolts(-1), -3);
        assert_eq!(Config::default().register(0b111), 0x4127);
        let config = Config {
            averaging: Averaging::X4,
            bus_conversion: ConversionTime::Us588,
            shunt_conversion: ConversionTime::Us588,
        };
        // Section 6.4.1: 588 µs + 588 µs, AVG 4 → 4.7 ms (+10 %).
        assert_eq!(config.conversion_us(), 5_174);
    }

    #[test]
    fn probe_checks_ids() {
        let expectations = [
            read(reg::MANUFACTURER_ID, 0x5449),
            read(reg::DIE_ID, 0x2260),
            read(reg::MANUFACTURER_ID, 0x5449),
            read(reg::DIE_ID, 0x2270),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut ina = Ina226::new(&mut i2c, ADDR, example_calibration());
        ina.probe().unwrap();
        assert_eq!(ina.probe(), Err(Error::UnexpectedId(0x5449_2270)));
        i2c.done();
    }

    #[test]
    fn triggered_measurement() {
        let mut expectations = vec![
            write(reg::CALIBRATION, 0x0A00),
            write(reg::CONFIG, 0x4120),
            write(reg::CONFIG, 0x4123),
            read(reg::MASK_ENABLE, 0x0000),
        ];
        expectations.extend(example_reads(MASK_CVRF));
        let mut i2c = Mock::new(&expectations);
        let mut ina = Ina226::new(&mut i2c, ADDR, example_calibration());
        ina.init().unwrap();
        assert_eq!(ina.start_measurement().unwrap(), 2_420);
        assert_eq!(ina.read_measurement(), Err(Error::NotReady));
        assert_eq!(
            ina.read_measurement().unwrap(),
            Measurement {
                bus_voltage: 11_980,
                shunt_voltage: 20_000,
                current: 10_000_000,
                power: 119_800,
            }
        );
        i2c.done();
    }

    #[test]
    fn continuous_overflow_and_power_down() {
        let mut expectations = vec![write(reg::CONFIG, 0x4127)];
        expectations.extend(example_reads(MASK_OVF));
        expectations.extend([write(reg::CONFIG, 0x4120), write(reg::CONFIG, 0x4127)]);
        let mut i2c = Mock::new(&expectations);
        let mut ina =
            Ina226::new(&mut i2c, ADDR, example_calibration()).with_mode(Mode::Continuous);
        ina.write_register(reg::CONFIG, ina.idle_config()).unwrap();
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

        let mut expectations = vec![
            read(reg::MANUFACTURER_ID, 0x5449),
            read(reg::DIE_ID, 0x2261),
            write(reg::CONFIG, 0x4123),
        ];
        expectations.extend(example_reads(MASK_CVRF));
        let mut i2c = Mock::new(&expectations);
        let mut ina = asynch::Ina226::new(&mut i2c, ADDR, example_calibration());
        let mut delay = RecordingDelay(0);
        block_on(ina.probe()).unwrap();
        let m = block_on(ina.measure(&mut delay)).unwrap();
        assert_eq!(m.current, 10_000_000);
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
