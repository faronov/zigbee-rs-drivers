//! Texas Instruments INA3221 triple-channel, high-side shunt and bus voltage
//! monitor (I2C).
//!
//! Each channel reports a shunt voltage (40 µV LSB, ±163.8 mV) and a bus
//! voltage (8 mV LSB, 0–26 V usable). The part has no current or power
//! registers: the driver computes current from the configured shunt
//! resistance of each channel (`I = V_shunt / R`, precomputed Q24 factor) and
//! power from bus voltage × current.
//!
//! In [`Mode::Triggered`] (default) `start_measurement` starts one single-shot
//! pass over all enabled channels and returns `AVG × channels × (VSHCT +
//! VBUSCT)` plus the 10 % maximum tolerance; afterwards the device powers
//! down by itself.
//!
//! Reference: TI "INA3221 Triple-Channel, High-Side Measurement, Shunt and
//! Bus Voltage Monitor", SBOS576C.

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Address with A0 = GND. A0 to GND/VS/SDA/SCL selects 0x40..=0x43.
pub const DEFAULT_ADDRESS: u8 = 0x40;

/// Number of channels.
pub const CHANNELS: usize = 3;

/// Register addresses.
pub mod reg {
    /// Configuration.
    pub const CONFIG: u8 = 0x00;
    /// Shunt voltage of channel `n` (0-based).
    pub const fn shunt(n: usize) -> u8 {
        0x01 + 2 * n as u8
    }
    /// Bus voltage of channel `n` (0-based).
    pub const fn bus(n: usize) -> u8 {
        0x02 + 2 * n as u8
    }
    /// Shunt-voltage sum.
    pub const SHUNT_SUM: u8 = 0x0D;
    /// Mask/Enable.
    pub const MASK_ENABLE: u8 = 0x0F;
    /// Manufacturer ID.
    pub const MANUFACTURER_ID: u8 = 0xFE;
    /// Die ID.
    pub const DIE_ID: u8 = 0xFF;
}

/// Manufacturer ID register value ("TI").
pub const MANUFACTURER_ID: u16 = 0x5449;
/// Die ID register value.
pub const DIE_ID: u16 = 0x3220;

const CONFIG_RESET: u16 = 1 << 15;
const MODE_POWER_DOWN: u16 = 0b000;
const MODE_SHUNT_BUS_SINGLE: u16 = 0b011;
const MODE_SHUNT_BUS_CONTINUOUS: u16 = 0b111;

/// Mask/Enable: conversion ready flag.
pub const MASK_CVRF: u16 = 1 << 0;

/// Recovery time from power-down (section 7.3.1: 40 µs).
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
}

/// Conversion mode used by the [`zigbee_sensor::blocking::Sensor`] impl.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Mode {
    /// One single-shot pass per `start_measurement`.
    #[default]
    Triggered,
    /// Free-running conversions.
    Continuous,
}

/// Channel enables, averaging and conversion times (register 00h).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Channel 1..3 enable.
    pub channels: [bool; CHANNELS],
    /// Samples averaged per result.
    pub averaging: Averaging,
    /// Bus voltage conversion time.
    pub bus_conversion: ConversionTime,
    /// Shunt voltage conversion time.
    pub shunt_conversion: ConversionTime,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            channels: [true; CHANNELS],
            averaging: Averaging::X1,
            bus_conversion: ConversionTime::Us1100,
            shunt_conversion: ConversionTime::Us1100,
        }
    }
}

impl Config {
    /// Configuration register value with the given MODE bits.
    pub const fn register(&self, mode_bits: u16) -> u16 {
        let mut enables = 0;
        let mut n = 0;
        while n < CHANNELS {
            if self.channels[n] {
                enables |= 1 << (14 - n);
            }
            n += 1;
        }
        enables
            | ((self.averaging as u16) << 9)
            | ((self.bus_conversion as u16) << 6)
            | ((self.shunt_conversion as u16) << 3)
            | (mode_bits & 0b111)
    }

    /// Number of enabled channels.
    pub const fn enabled_channels(&self) -> u32 {
        self.channels[0] as u32 + self.channels[1] as u32 + self.channels[2] as u32
    }

    /// Worst-case time of one pass over the enabled channels in µs
    /// (section 7.4.2: channels × (VSHCT + VBUSCT), × AVG; + 10 %).
    pub const fn conversion_us(&self) -> WaitUs {
        let typ = self.averaging.samples()
            * self.enabled_channels()
            * (self.bus_conversion.micros() + self.shunt_conversion.micros());
        typ + typ / 10
    }
}

/// Shunt resistances, as Q24 µA-per-LSB factors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Shunts {
    current_q24: [i64; CHANNELS],
}

impl Shunts {
    /// Shunt resistance per channel in µΩ.
    ///
    /// Returns `None` if a resistance is below 77 µΩ (full-scale
    /// 163.8 mV would exceed `i32` µA) or zero.
    pub const fn new(shunt_micro_ohm: [u32; CHANNELS]) -> Option<Self> {
        let mut current_q24 = [0; CHANNELS];
        let mut n = 0;
        while n < CHANNELS {
            let r = shunt_micro_ohm[n] as u64;
            if r == 0 {
                return None;
            }
            // 40 µV / R = 40e6 / R_µΩ µA per LSB.
            let q = ((40_000_000u64 << 24) + r / 2) / r;
            if q >= 1 << 43 {
                return None;
            }
            current_q24[n] = q as i64;
            n += 1;
        }
        Some(Self { current_q24 })
    }

    /// Current in µA of channel `n` from its raw Shunt Voltage register.
    pub const fn current_ua(&self, n: usize, raw: u16) -> i32 {
        ((shunt_lsb(raw) as i64 * self.current_q24[n] + (1 << 23)) >> 24) as i32
    }
}

const fn shunt_lsb(raw: u16) -> i32 {
    (raw as i16 >> 3) as i32
}

/// Shunt voltage in µV from a raw Shunt Voltage register (bits 15..3, 40 µV).
pub const fn shunt_microvolts(raw: u16) -> i32 {
    shunt_lsb(raw) * 40
}

/// Bus voltage in mV from a raw Bus Voltage register (bits 15..3, 8 mV).
pub const fn bus_millivolts(raw: u16) -> i32 {
    (raw as i16 >> 3) as i32 * 8
}

/// Power in mW from mV × µA.
///
/// `× 4295 >> 32` stands in for `/ 1e6` (relative error 7.6 ppm), avoiding a
/// 64-bit division; |mV| ≤ 32 760 and |µA| < 2^31 keep it inside `i64`.
pub const fn power_milliwatts(millivolts: i32, microamps: i32) -> i32 {
    ((millivolts as i64 * microamps as i64 * 4_295 + (1 << 31)) >> 32) as i32
}

/// Results of one channel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Channel {
    /// Bus voltage in mV.
    pub bus_voltage: i32,
    /// Shunt voltage in µV.
    pub shunt_voltage: i32,
    /// Current in µA.
    pub current: i32,
    /// Power in mW.
    pub power: i32,
}

/// One measurement; disabled channels are `None`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Channels 1..3.
    pub channels: [Option<Channel>; CHANNELS],
}

impl Readings for Measurement {
    /// Voltage, Current, Power for each enabled channel, in channel order.
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        for ch in self.channels.iter().flatten() {
            f(Quantity::Voltage(ch.bus_voltage));
            f(Quantity::Current(ch.current));
            f(Quantity::Power(ch.power));
        }
    }
}

/// Decode channel `n` from its raw shunt and bus registers.
pub const fn decode_channel(shunts: &Shunts, n: usize, shunt: u16, bus: u16) -> Channel {
    let bus_voltage = bus_millivolts(bus);
    let current = shunts.current_ua(n, shunt);
    Channel {
        bus_voltage,
        shunt_voltage: shunt_microvolts(shunt),
        current,
        power: power_milliwatts(bus_voltage, current),
    }
}

/// Check the ID registers (FEh, FFh).
pub fn check_id<E>(manufacturer: u16, die: u16) -> Result<(), Error<E>> {
    if manufacturer != MANUFACTURER_ID || die != DIE_ID {
        return Err(Error::UnexpectedId(
            (u32::from(manufacturer) << 16) | u32::from(die),
        ));
    }
    Ok(())
}

macro_rules! constructors {
    () => {
        /// Create a driver; no bus traffic. Call `init` before measuring.
        pub const fn new(i2c: I2C, address: u8, shunts: Shunts) -> Self {
            Self {
                i2c,
                address,
                shunts,
                config: Config {
                    channels: [true; CHANNELS],
                    averaging: Averaging::X1,
                    bus_conversion: ConversionTime::Us1100,
                    shunt_conversion: ConversionTime::Us1100,
                },
                mode: Mode::Triggered,
            }
        }

        /// Channel enables / averaging / conversion times written by `init`.
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
                Mode::Continuous => self.config.register(MODE_SHUNT_BUS_CONTINUOUS),
            }
        }
    };
}

/// Blocking driver.
pub struct Ina3221<I2C> {
    i2c: I2C,
    address: u8,
    shunts: Shunts,
    config: Config,
    mode: Mode,
}

impl<I2C> Ina3221<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Ina3221<I2C> {
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

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Ina3221<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        match self.mode {
            Mode::Triggered => {
                self.write_register(reg::CONFIG, self.config.register(MODE_SHUNT_BUS_SINGLE))?;
                Ok(self.config.conversion_us())
            }
            Mode::Continuous => Ok(0),
        }
    }

    /// Reading Mask/Enable clears CVRF (and the alert flags); in triggered
    /// mode returns [`Error::NotReady`] until CVRF is set.
    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        let mask = self.read_register(reg::MASK_ENABLE)?;
        if self.mode == Mode::Triggered && mask & MASK_CVRF == 0 {
            return Err(Error::NotReady);
        }
        let mut channels = [None; CHANNELS];
        for (n, slot) in channels.iter_mut().enumerate() {
            if self.config.channels[n] {
                let shunt = self.read_register(reg::shunt(n))?;
                let bus = self.read_register(reg::bus(n))?;
                *slot = Some(decode_channel(&self.shunts, n, shunt, bus));
            }
        }
        Ok(Measurement { channels })
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::LowPower for Ina3221<I2C> {
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
        Averaging, CHANNELS, CONFIG_RESET, Config, ConversionTime, MASK_CVRF, MODE_POWER_DOWN,
        MODE_SHUNT_BUS_CONTINUOUS, MODE_SHUNT_BUS_SINGLE, Measurement, Mode,
        POWER_DOWN_RECOVERY_US, Shunts, check_id, decode_channel, reg,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Ina3221`].
    pub struct Ina3221<I2C> {
        i2c: I2C,
        address: u8,
        shunts: Shunts,
        config: Config,
        mode: Mode,
    }

    impl<I2C> Ina3221<I2C> {
        constructors!();
    }

    impl<I2C: I2c> Ina3221<I2C> {
        /// See [`super::Ina3221::write_register`].
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

        /// See [`super::Ina3221::read_register`].
        pub async fn read_register(&mut self, register: u8) -> Result<u16, Error<I2C::Error>> {
            let mut buf = [0u8; 2];
            self.i2c
                .write_read(self.address, &[register], &mut buf)
                .await
                .map_err(Error::bus)?;
            Ok(u16::from_be_bytes(buf))
        }

        /// See [`super::Ina3221::probe`].
        pub async fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
            let manufacturer = self.read_register(reg::MANUFACTURER_ID).await?;
            let die = self.read_register(reg::DIE_ID).await?;
            check_id(manufacturer, die)
        }

        /// See [`super::Ina3221::init`].
        pub async fn init(&mut self) -> Result<(), Error<I2C::Error>> {
            self.write_register(reg::CONFIG, self.idle_config()).await
        }

        /// See [`super::Ina3221::reset`].
        pub async fn reset(&mut self) -> Result<(), Error<I2C::Error>> {
            self.write_register(reg::CONFIG, CONFIG_RESET).await
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Ina3221<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            match self.mode {
                Mode::Triggered => {
                    self.write_register(reg::CONFIG, self.config.register(MODE_SHUNT_BUS_SINGLE))
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
            let mut channels = [None; CHANNELS];
            for (n, slot) in channels.iter_mut().enumerate() {
                if self.config.channels[n] {
                    let shunt = self.read_register(reg::shunt(n)).await?;
                    let bus = self.read_register(reg::bus(n)).await?;
                    *slot = Some(decode_channel(&self.shunts, n, shunt, bus));
                }
            }
            Ok(Measurement { channels })
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::LowPower for Ina3221<I2C> {
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
    use std::vec::Vec;
    use zigbee_sensor::blocking::{LowPower, Sensor};

    const ADDR: u8 = DEFAULT_ADDRESS;

    fn shunts() -> Shunts {
        // 100 mΩ, 50 mΩ, 10 mΩ.
        Shunts::new([100_000, 50_000, 10_000]).unwrap()
    }

    fn read(reg: u8, value: u16) -> Transaction {
        Transaction::write_read(ADDR, vec![reg], value.to_be_bytes().to_vec())
    }

    fn write(reg: u8, value: u16) -> Transaction {
        let [m, l] = value.to_be_bytes();
        Transaction::write(ADDR, vec![reg, m, l])
    }

    #[test]
    fn conversions() {
        // Section 8.2.2: −80 mV reads C180h.
        assert_eq!(shunt_microvolts(0xC180), -80_000);
        assert_eq!(shunt_microvolts(0x7FF8), 163_800);
        assert_eq!(bus_millivolts(0x7FF8), 32_760);
        // 2000 LSB (80 mV) over 100 mΩ → 800 mA; over 10 mΩ → 8 A.
        let s = shunts();
        assert_eq!(s.current_ua(0, 2_000 << 3), 800_000);
        assert_eq!(s.current_ua(2, 0xC180), -8_000_000);
        assert_eq!(power_milliwatts(12_000, 800_000), 9_600);
        // Full-scale: within 7.6 ppm of −65 520 W.
        assert!((power_milliwatts(32_760, -2_000_000_000) + 65_520_000).abs() <= 500);
        assert_eq!(Shunts::new([100, 76, 100]), None);
        assert_eq!(Shunts::new([0, 100, 100]), None);
    }

    #[test]
    fn config_encoding_and_timing() {
        assert_eq!(Config::default().register(0b111), 0x7127);
        let config = Config {
            channels: [true, false, true],
            averaging: Averaging::X4,
            bus_conversion: ConversionTime::Us140,
            shunt_conversion: ConversionTime::Us588,
        };
        assert_eq!(config.register(MODE_SHUNT_BUS_SINGLE), 0x521B);
        // 4 × 2 × (140 + 588) = 5824 µs, +10 %.
        assert_eq!(config.conversion_us(), 6_406);
    }

    #[test]
    fn probe_checks_ids() {
        let expectations = [
            read(reg::MANUFACTURER_ID, 0x5449),
            read(reg::DIE_ID, 0x3220),
            read(reg::MANUFACTURER_ID, 0x5449),
            read(reg::DIE_ID, 0x2260),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut ina = Ina3221::new(&mut i2c, ADDR, shunts());
        ina.probe().unwrap();
        assert_eq!(ina.probe(), Err(Error::UnexpectedId(0x5449_2260)));
        i2c.done();
    }

    fn channel_reads() -> Vec<Transaction> {
        vec![
            read(reg::MASK_ENABLE, 0x0003),
            read(0x01, 2_000 << 3),
            read(0x02, 1_500 << 3),
            read(0x05, 0xC180),
            read(0x06, 625 << 3),
        ]
    }

    fn expected() -> Measurement {
        Measurement {
            channels: [
                Some(Channel {
                    bus_voltage: 12_000,
                    shunt_voltage: 80_000,
                    current: 800_000,
                    power: 9_600,
                }),
                None,
                Some(Channel {
                    bus_voltage: 5_000,
                    shunt_voltage: -80_000,
                    current: -8_000_000,
                    power: -40_000,
                }),
            ],
        }
    }

    #[test]
    fn triggered_two_channels() {
        let config = Config {
            channels: [true, false, true],
            ..Config::default()
        };
        let mut expectations = vec![
            write(reg::CONFIG, 0x5120),
            write(reg::CONFIG, 0x5123),
            read(reg::MASK_ENABLE, 0x0002),
        ];
        expectations.extend(channel_reads());
        let mut i2c = Mock::new(&expectations);
        let mut ina = Ina3221::new(&mut i2c, ADDR, shunts()).with_config(config);
        ina.init().unwrap();
        assert_eq!(ina.start_measurement().unwrap(), 4_840);
        assert_eq!(ina.read_measurement(), Err(Error::NotReady));
        let m = ina.read_measurement().unwrap();
        assert_eq!(m, expected());
        let mut quantities = Vec::new();
        m.for_each(&mut |q| quantities.push(q));
        assert_eq!(quantities.len(), 6);
        assert_eq!(quantities[3], Quantity::Voltage(5_000));
        i2c.done();
    }

    #[test]
    fn sleep_and_wake_continuous() {
        let expectations = [write(reg::CONFIG, 0x7120), write(reg::CONFIG, 0x7127)];
        let mut i2c = Mock::new(&expectations);
        let mut ina = Ina3221::new(&mut i2c, ADDR, shunts()).with_mode(Mode::Continuous);
        ina.sleep().unwrap();
        assert_eq!(ina.wake().unwrap(), 40);
        assert_eq!(ina.start_measurement().unwrap(), 0);
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

        let config = Config {
            channels: [true, false, true],
            ..Config::default()
        };
        let mut expectations = vec![write(reg::CONFIG, 0x5123)];
        expectations.extend(channel_reads());
        let mut i2c = Mock::new(&expectations);
        let mut ina = asynch::Ina3221::new(&mut i2c, ADDR, shunts()).with_config(config);
        let mut delay = RecordingDelay(0);
        assert_eq!(block_on(ina.measure(&mut delay)).unwrap(), expected());
        assert_eq!(delay.0, 4_840);
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
