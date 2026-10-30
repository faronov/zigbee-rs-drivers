//! Texas Instruments INA228 (I2C) and INA229 (SPI) 85 V, 20-bit power,
//! energy and charge monitors with die temperature sensor.
//!
//! Both parts share one register map; only the transport differs
//! ([`I2cInterface`], [`SpiInterface`]). The device computes current, power,
//! energy and charge from the SHUNT_CAL register; [`Calibration`] derives
//! that value and fixed-point scale factors from the shunt resistance, so
//! decoding needs only multiplies and shifts.
//!
//! Modes:
//!
//! - [`Mode::Triggered`] (default): `start_measurement` starts one averaged
//!   bus + shunt + temperature conversion and returns
//!   `AVG × (VBUSCT + VSHCT + VTCT)` + 10 % margin; the ADC shuts down
//!   afterwards. ENERGY and CHARGE are not valid in this mode (section
//!   7.3.1: the device does not track elapsed time), so
//!   [`Measurement::energy`] and [`Measurement::charge`] are `None`.
//! - [`Mode::Continuous`]: free-running conversions with energy/charge
//!   accumulation; `start_measurement` returns 0.
//!
//! References: TI SLYS021A (INA228, May 2022), SLYS023A (INA229, May 2022).

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address with A1 = A0 = GND. A0/A1 select 0x40..=0x4F.
pub const DEFAULT_ADDRESS: u8 = 0x40;

/// Register addresses.
pub mod reg {
    /// CONFIG (RST, RSTACC, CONVDLY, TEMPCOMP, ADCRANGE).
    pub const CONFIG: u8 = 0x00;
    /// ADC_CONFIG (MODE, conversion times, averaging).
    pub const ADC_CONFIG: u8 = 0x01;
    /// SHUNT_CAL.
    pub const SHUNT_CAL: u8 = 0x02;
    /// SHUNT_TEMPCO (ppm/°C).
    pub const SHUNT_TEMPCO: u8 = 0x03;
    /// VSHUNT (24 bit).
    pub const VSHUNT: u8 = 0x04;
    /// VBUS (24 bit).
    pub const VBUS: u8 = 0x05;
    /// DIETEMP (16 bit).
    pub const DIETEMP: u8 = 0x06;
    /// CURRENT (24 bit).
    pub const CURRENT: u8 = 0x07;
    /// POWER (24 bit).
    pub const POWER: u8 = 0x08;
    /// ENERGY (40 bit).
    pub const ENERGY: u8 = 0x09;
    /// CHARGE (40 bit).
    pub const CHARGE: u8 = 0x0A;
    /// DIAG_ALRT.
    pub const DIAG_ALRT: u8 = 0x0B;
    /// MANUFACTURER_ID.
    pub const MANUFACTURER_ID: u8 = 0x3E;
    /// DEVICE_ID.
    pub const DEVICE_ID: u8 = 0x3F;
}

/// Manufacturer ID register value ("TI").
pub const MANUFACTURER_ID: u16 = 0x5449;

const CONFIG_RST: u16 = 1 << 15;
const CONFIG_RSTACC: u16 = 1 << 14;

const MODE_SHUTDOWN: u16 = 0x0;
const MODE_TRIGGERED_ALL: u16 = 0x7;
const MODE_CONTINUOUS_ALL: u16 = 0xF;

/// DIAG_ALRT: math overflow (current/power invalid).
pub const DIAG_MATHOF: u16 = 1 << 9;
/// DIAG_ALRT: conversion ready flag (cleared by reading DIAG_ALRT).
pub const DIAG_CNVRF: u16 = 1 << 1;
/// DIAG_ALRT: trim memory OK (0 = checksum error).
pub const DIAG_MEMSTAT: u16 = 1 << 0;

/// Wake-up time from shutdown (electrical characteristics, TPOR: 60 µs).
pub const SHUTDOWN_RECOVERY_US: WaitUs = 60;

/// Start-up time after power-up or reset (TPOR: 300 µs).
pub const RESET_WAIT_US: WaitUs = 300;

/// Which part answered `probe`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Chip {
    /// INA228 (die ID 228h).
    Ina228,
    /// INA229 (die ID 229h).
    Ina229,
}

/// Register access over I2C (INA228) or SPI (INA229).
pub trait Interface {
    /// Transport error.
    type Error;
    /// Read a 16/24/40-bit register (big-endian, `buf.len()` bytes).
    fn read_reg(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error>;
    /// Write a 16-bit register.
    fn write_reg(&mut self, reg: u8, value: u16) -> Result<(), Self::Error>;
}

/// I2C transport (INA228).
#[derive(Debug)]
pub struct I2cInterface<I2C> {
    /// The bus.
    pub i2c: I2C,
    /// 7-bit address.
    pub address: u8,
}

/// SPI transport (INA229): mode 1 (CPOL 0, CPHA 1), up to 10 MHz. The
/// first byte is `address << 2 | R/W` (section 7.5.1.1).
#[derive(Debug)]
pub struct SpiInterface<SPI> {
    /// The device (bus + chip select).
    pub spi: SPI,
}

const fn spi_command(reg: u8, read: bool) -> u8 {
    (reg << 2) | read as u8
}

impl<I2C: embedded_hal::i2c::I2c> Interface for I2cInterface<I2C> {
    type Error = I2C::Error;

    fn read_reg(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, &[reg], buf)
    }

    fn write_reg(&mut self, reg: u8, value: u16) -> Result<(), Self::Error> {
        let [msb, lsb] = value.to_be_bytes();
        self.i2c.write(self.address, &[reg, msb, lsb])
    }
}

impl<SPI: embedded_hal::spi::SpiDevice> Interface for SpiInterface<SPI> {
    type Error = SPI::Error;

    fn read_reg(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        use embedded_hal::spi::Operation;
        self.spi.transaction(&mut [
            Operation::Write(&[spi_command(reg, true)]),
            Operation::Read(buf),
        ])
    }

    fn write_reg(&mut self, reg: u8, value: u16) -> Result<(), Self::Error> {
        let [msb, lsb] = value.to_be_bytes();
        self.spi.write(&[spi_command(reg, false), msb, lsb])
    }
}

/// Shunt full-scale range (CONFIG.ADCRANGE).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AdcRange {
    /// ±163.84 mV, 312.5 nV/LSB (default).
    #[default]
    Mv163,
    /// ±40.96 mV, 78.125 nV/LSB.
    Mv41,
}

/// Number of averaged samples (AVG).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Averaging {
    /// 1 sample (default).
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

/// Conversion time per sample (VBUSCT / VSHCT / VTCT).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ConversionTime {
    /// 50 µs.
    Us50,
    /// 84 µs.
    Us84,
    /// 150 µs.
    Us150,
    /// 280 µs.
    Us280,
    /// 540 µs.
    Us540,
    /// 1052 µs (default).
    #[default]
    Us1052,
    /// 2074 µs.
    Us2074,
    /// 4120 µs.
    Us4120,
}

impl ConversionTime {
    /// Conversion time in µs.
    pub const fn micros(self) -> u32 {
        match self {
            Self::Us50 => 50,
            Self::Us84 => 84,
            Self::Us150 => 150,
            Self::Us280 => 280,
            Self::Us540 => 540,
            Self::Us1052 => 1_052,
            Self::Us2074 => 2_074,
            Self::Us4120 => 4_120,
        }
    }
}

/// Conversion mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Mode {
    /// Single-shot bus + shunt + temperature per `start_measurement`.
    #[default]
    Triggered,
    /// Free-running conversions with energy/charge accumulation.
    Continuous,
}

/// ADC configuration (ADC_CONFIG register 01h).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// Samples averaged per result.
    pub averaging: Averaging,
    /// Bus voltage conversion time.
    pub bus_conversion: ConversionTime,
    /// Shunt voltage conversion time.
    pub shunt_conversion: ConversionTime,
    /// Die temperature conversion time.
    pub temperature_conversion: ConversionTime,
}

impl Config {
    /// ADC_CONFIG value with the given MODE nibble.
    pub const fn register(&self, mode: u16) -> u16 {
        ((mode & 0xF) << 12)
            | ((self.bus_conversion as u16) << 9)
            | ((self.shunt_conversion as u16) << 6)
            | ((self.temperature_conversion as u16) << 3)
            | self.averaging as u16
    }

    /// Time of one averaged bus + shunt + temperature result in µs, + 10 %
    /// margin (the datasheet gives no conversion-time tolerance).
    pub const fn conversion_us(&self) -> WaitUs {
        let typ = self.averaging.samples()
            * (self.bus_conversion.micros()
                + self.shunt_conversion.micros()
                + self.temperature_conversion.micros());
        typ + typ / 10
    }
}

/// SHUNT_CAL value, ADC range and fixed-point scale factors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Calibration {
    /// SHUNT_CAL register value.
    pub register: u16,
    /// Shunt range the register value was computed for.
    pub range: AdcRange,
    /// µA per CURRENT LSB, Q24.
    pub current_q24: i64,
    /// mW per POWER LSB, Q24.
    pub power_q24: i64,
    /// mWh per ENERGY LSB, Q40.
    pub energy_q40: u64,
    /// µAh per CHARGE LSB, Q40.
    pub charge_q40: u64,
}

impl Calibration {
    /// Calibration from a SHUNT_CAL register value; all scale factors are
    /// derived from it (`CURRENT_LSB = SHUNT_CAL / (13107.2e6 × R)`, ÷ 4
    /// with [`AdcRange::Mv41`]).
    ///
    /// Returns `None` for a zero/oversized register or resistance, or when
    /// the current LSB exceeds 4.096 mA (full scale would not fit `i32` µA).
    pub const fn from_register(
        shunt_micro_ohm: u32,
        register: u16,
        range: AdcRange,
    ) -> Option<Self> {
        if shunt_micro_ohm == 0 || register == 0 || register > 0x7FFF {
            return None;
        }
        let cal = register as u64;
        let r = shunt_micro_ohm as u64;
        let (r1, r9) = match range {
            AdcRange::Mv163 => (r, 9 * r),
            AdcRange::Mv41 => (4 * r, 36 * r),
        };
        // µA/LSB = cal × 1e7 / (2^17 × R_µΩ) → Q24: cal × 1.28e9 / R.
        let current_q24 = (cal * 1_280_000_000 + r1 / 2) / r1;
        if current_q24 >= 1 << 36 {
            return None;
        }
        Some(Self {
            register,
            range,
            current_q24: current_q24 as i64,
            // × 3.2 / 1000.
            power_q24: ((cal * 4_096_000 + r1 / 2) / r1) as i64,
            // × 51.2 / 3.6e6 → cal × 1.28e9 × 2^16 × 51.2 / 3.6e6 / R.
            energy_q40: (cal * 10_737_418_240 + r9 / 2) / r9,
            // / 3600 → cal × 1.28e9 × 2^16 / 3600 / R.
            charge_q40: (cal * 209_715_200_000 + r9 / 2) / r9,
        })
    }

    /// Calibration for a current LSB in nA (equation 2:
    /// `SHUNT_CAL = 13107.2e6 × CURRENT_LSB × R`, × 4 for ±40.96 mV).
    pub const fn new(shunt_micro_ohm: u32, current_lsb_na: u32, range: AdcRange) -> Option<Self> {
        // 13107.2e6 × 1e-15 = 128 / 9 765 625.
        let Some(p) = (current_lsb_na as u64).checked_mul(shunt_micro_ohm as u64) else {
            return None;
        };
        if p > 1 << 50 {
            return None;
        }
        let cal = match range {
            AdcRange::Mv163 => p * 128 / 9_765_625,
            AdcRange::Mv41 => p * 512 / 9_765_625,
        };
        if cal > 0x7FFF {
            return None;
        }
        Self::from_register(shunt_micro_ohm, cal as u16, range)
    }

    /// Calibration with `CURRENT_LSB = I_max / 2^19` (equation 3), i.e.
    /// `SHUNT_CAL = I_max[mA] × R[µΩ] / 40 000` (× 4 for ±40.96 mV).
    pub const fn for_max_current(
        shunt_micro_ohm: u32,
        max_current_ma: u32,
        range: AdcRange,
    ) -> Option<Self> {
        let p = max_current_ma as u64 * shunt_micro_ohm as u64;
        let cal = match range {
            AdcRange::Mv163 => p / 40_000,
            AdcRange::Mv41 => p / 10_000,
        };
        if cal > 0x7FFF {
            return None;
        }
        Self::from_register(shunt_micro_ohm, cal as u16, range)
    }

    /// Current in µA from the sign-extended 20-bit CURRENT value.
    pub const fn current_ua(&self, raw: i32) -> i32 {
        ((raw as i64 * self.current_q24 + (1 << 23)) >> 24) as i32
    }

    /// Power in mW from the 24-bit POWER value.
    pub const fn power_mw(&self, raw: u32) -> i32 {
        ((raw as i64 * self.power_q24 + (1 << 23)) >> 24) as i32
    }

    /// Energy in mWh from the 40-bit ENERGY value.
    pub const fn energy_mwh(&self, raw: u64) -> u64 {
        mul_q40(raw, self.energy_q40)
    }

    /// Charge in µAh from the sign-extended 40-bit CHARGE value.
    pub const fn charge_uah(&self, raw: i64) -> i64 {
        let magnitude = mul_q40(raw.unsigned_abs(), self.charge_q40) as i64;
        if raw < 0 { -magnitude } else { magnitude }
    }
}

/// `x × q >> 40` for `x < 2^40`, `q < 2^41` without a 128-bit multiply:
/// `x` is split into 20-bit halves.
const fn mul_q40(x: u64, q: u64) -> u64 {
    let hi = (x >> 20) & 0xF_FFFF;
    let lo = x & 0xF_FFFF;
    (hi * q + ((lo * q) >> 20) + (1 << 19)) >> 20
}

/// Sign-extend a 20-bit value stored in bits 23..4 of a 24-bit register.
pub const fn raw20(bytes: [u8; 3]) -> i32 {
    i32::from_be_bytes([bytes[0], bytes[1], bytes[2], 0]) >> 12
}

/// Shunt voltage in nV from the 20-bit VSHUNT value.
pub const fn shunt_nanovolts(raw: i32, range: AdcRange) -> i32 {
    match range {
        // 312.5 nV, 78.125 nV.
        AdcRange::Mv163 => (raw * 625 + 1) >> 1,
        AdcRange::Mv41 => (raw * 625 + 4) >> 3,
    }
}

/// Bus voltage in mV from the 20-bit VBUS value (195.3125 µV/LSB).
pub const fn bus_millivolts(raw: i32) -> i32 {
    (raw * 25 + 64) >> 7
}

/// Die temperature in 0.01 °C from DIETEMP (7.8125 m°C/LSB).
pub const fn temperature_centi(raw: i16) -> i32 {
    (raw as i32 * 25 + 16) >> 5
}

/// One measurement.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Bus voltage in mV.
    pub bus_voltage: i32,
    /// Shunt voltage in nV.
    pub shunt_voltage: i32,
    /// Current in µA.
    pub current: i32,
    /// Power in mW (magnitude).
    pub power: i32,
    /// Die temperature in 0.01 °C.
    pub temperature: i32,
    /// Accumulated energy in mWh (continuous mode only).
    pub energy: Option<u64>,
    /// Accumulated charge in µAh (continuous mode only).
    pub charge: Option<i64>,
}

impl Readings for Measurement {
    /// Voltage, Current, Power, Temperature, then Energy when available.
    /// Charge has no `Quantity`; read it from the struct.
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Voltage(self.bus_voltage));
        f(Quantity::Current(self.current));
        f(Quantity::Power(self.power));
        f(Quantity::Temperature(self.temperature));
        if let Some(e) = self.energy {
            f(Quantity::Energy(e));
        }
    }
}

/// Raw result registers.
#[derive(Debug, Clone, Copy, Default)]
pub struct RawResults {
    /// DIAG_ALRT.
    pub diag: u16,
    /// VSHUNT bytes.
    pub vshunt: [u8; 3],
    /// VBUS bytes.
    pub vbus: [u8; 3],
    /// DIETEMP.
    pub dietemp: u16,
    /// CURRENT bytes.
    pub current: [u8; 3],
    /// POWER bytes.
    pub power: [u8; 3],
    /// ENERGY and CHARGE bytes (continuous mode).
    pub accumulators: Option<([u8; 5], [u8; 5])>,
}

/// Decode raw results. MATHOF or a trim memory checksum error give
/// [`Error::InvalidData`].
pub fn decode<E>(cal: &Calibration, raw: &RawResults) -> Result<Measurement, Error<E>> {
    if raw.diag & DIAG_MATHOF != 0 || raw.diag & DIAG_MEMSTAT == 0 {
        return Err(Error::InvalidData);
    }
    let (energy, charge) = match raw.accumulators {
        Some((e, c)) => {
            let e = u64::from_be_bytes([0, 0, 0, e[0], e[1], e[2], e[3], e[4]]);
            let c = i64::from_be_bytes([c[0], c[1], c[2], c[3], c[4], 0, 0, 0]) >> 24;
            (Some(cal.energy_mwh(e)), Some(cal.charge_uah(c)))
        }
        None => (None, None),
    };
    let [p0, p1, p2] = raw.power;
    Ok(Measurement {
        bus_voltage: bus_millivolts(raw20(raw.vbus)),
        shunt_voltage: shunt_nanovolts(raw20(raw.vshunt), cal.range),
        current: cal.current_ua(raw20(raw.current)),
        power: cal.power_mw(u32::from_be_bytes([0, p0, p1, p2])),
        temperature: temperature_centi(raw.dietemp as i16),
        energy,
        charge,
    })
}

/// Check MANUFACTURER_ID / DEVICE_ID.
pub fn check_id<E>(manufacturer: u16, device: u16) -> Result<Chip, Error<E>> {
    match (manufacturer, device >> 4) {
        (MANUFACTURER_ID, 0x228) => Ok(Chip::Ina228),
        (MANUFACTURER_ID, 0x229) => Ok(Chip::Ina229),
        _ => Err(Error::UnexpectedId(
            (u32::from(manufacturer) << 16) | u32::from(device),
        )),
    }
}

macro_rules! constructors {
    ($name:ident) => {
        impl<IFACE> $name<IFACE> {
            /// Create a driver over any transport; no bus traffic.
            pub const fn new(iface: IFACE, calibration: Calibration) -> Self {
                Self {
                    iface,
                    calibration,
                    config: Config {
                        averaging: Averaging::X1,
                        bus_conversion: ConversionTime::Us1052,
                        shunt_conversion: ConversionTime::Us1052,
                        temperature_conversion: ConversionTime::Us1052,
                    },
                    mode: Mode::Triggered,
                }
            }

            /// ADC configuration written by `init`.
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

            /// Give the transport back.
            pub fn release(self) -> IFACE {
                self.iface
            }

            const fn idle_adc_config(&self) -> u16 {
                match self.mode {
                    Mode::Triggered => self.config.register(MODE_SHUTDOWN),
                    Mode::Continuous => self.config.register(MODE_CONTINUOUS_ALL),
                }
            }

            const fn config_register(&self) -> u16 {
                match self.calibration.range {
                    AdcRange::Mv163 => 0,
                    AdcRange::Mv41 => 1 << 4,
                }
            }
        }

        impl<I2C> $name<I2cInterface<I2C>> {
            /// INA228 on an I2C bus.
            pub const fn new_i2c(i2c: I2C, address: u8, calibration: Calibration) -> Self {
                Self::new(I2cInterface { i2c, address }, calibration)
            }
        }

        impl<SPI> $name<SpiInterface<SPI>> {
            /// INA229 on an SPI device.
            pub const fn new_spi(spi: SPI, calibration: Calibration) -> Self {
                Self::new(SpiInterface { spi }, calibration)
            }
        }
    };
}

/// Blocking driver.
#[derive(Debug)]
pub struct Ina228<IFACE> {
    iface: IFACE,
    calibration: Calibration,
    config: Config,
    mode: Mode,
}

constructors!(Ina228);

impl<IFACE: Interface> Ina228<IFACE> {
    /// Read a 16-bit register.
    pub fn read_register(&mut self, reg: u8) -> Result<u16, Error<IFACE::Error>> {
        let mut buf = [0; 2];
        self.iface.read_reg(reg, &mut buf).map_err(Error::bus)?;
        Ok(u16::from_be_bytes(buf))
    }

    /// Write a 16-bit register.
    pub fn write_register(&mut self, reg: u8, value: u16) -> Result<(), Error<IFACE::Error>> {
        self.iface.write_reg(reg, value).map_err(Error::bus)
    }

    /// Read MANUFACTURER_ID / DEVICE_ID and identify the part.
    pub fn probe(&mut self) -> Result<Chip, Error<IFACE::Error>> {
        let manufacturer = self.read_register(reg::MANUFACTURER_ID)?;
        let device = self.read_register(reg::DEVICE_ID)?;
        check_id(manufacturer, device)
    }

    /// Write ADCRANGE, SHUNT_CAL and ADC_CONFIG (shutdown in triggered
    /// mode, free-running in continuous mode).
    pub fn init(&mut self) -> Result<(), Error<IFACE::Error>> {
        self.write_register(reg::CONFIG, self.config_register())?;
        self.write_register(reg::SHUNT_CAL, self.calibration.register)?;
        self.write_register(reg::ADC_CONFIG, self.idle_adc_config())
    }

    /// System reset; wait [`RESET_WAIT_US`] and call `init` afterwards.
    pub fn reset(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        self.write_register(reg::CONFIG, CONFIG_RST)?;
        Ok(RESET_WAIT_US)
    }

    /// Clear ENERGY and CHARGE (RSTACC).
    pub fn reset_accumulators(&mut self) -> Result<(), Error<IFACE::Error>> {
        self.write_register(reg::CONFIG, self.config_register() | CONFIG_RSTACC)
    }

    fn read_bytes<const N: usize>(&mut self, reg: u8) -> Result<[u8; N], Error<IFACE::Error>> {
        let mut buf = [0; N];
        self.iface.read_reg(reg, &mut buf).map_err(Error::bus)?;
        Ok(buf)
    }
}

impl<IFACE: Interface> zigbee_sensor::blocking::Sensor for Ina228<IFACE> {
    type BusError = IFACE::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        match self.mode {
            Mode::Triggered => {
                self.write_register(reg::ADC_CONFIG, self.config.register(MODE_TRIGGERED_ALL))?;
                Ok(self.config.conversion_us())
            }
            Mode::Continuous => Ok(0),
        }
    }

    /// Reading DIAG_ALRT clears CNVRF; in triggered mode returns
    /// [`Error::NotReady`] until it is set.
    fn read_measurement(&mut self) -> Result<Measurement, Error<IFACE::Error>> {
        let diag = self.read_register(reg::DIAG_ALRT)?;
        if self.mode == Mode::Triggered && diag & DIAG_CNVRF == 0 {
            return Err(Error::NotReady);
        }
        let mut raw = RawResults {
            diag,
            vshunt: self.read_bytes(reg::VSHUNT)?,
            vbus: self.read_bytes(reg::VBUS)?,
            dietemp: self.read_register(reg::DIETEMP)?,
            current: self.read_bytes(reg::CURRENT)?,
            power: self.read_bytes(reg::POWER)?,
            accumulators: None,
        };
        if self.mode == Mode::Continuous {
            raw.accumulators = Some((self.read_bytes(reg::ENERGY)?, self.read_bytes(reg::CHARGE)?));
        }
        decode(&self.calibration, &raw)
    }
}

impl<IFACE: Interface> zigbee_sensor::blocking::LowPower for Ina228<IFACE> {
    type BusError = IFACE::Error;

    /// ADC shutdown (2.8 µA typ); accumulation stops.
    fn sleep(&mut self) -> Result<(), Error<IFACE::Error>> {
        self.write_register(reg::ADC_CONFIG, self.config.register(MODE_SHUTDOWN))
    }

    fn wake(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        self.write_register(reg::ADC_CONFIG, self.idle_adc_config())?;
        Ok(SHUTDOWN_RECOVERY_US)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        AdcRange, Averaging, CONFIG_RST, CONFIG_RSTACC, Calibration, Chip, Config, ConversionTime,
        DIAG_CNVRF, I2cInterface, MODE_CONTINUOUS_ALL, MODE_SHUTDOWN, MODE_TRIGGERED_ALL,
        Measurement, Mode, RESET_WAIT_US, RawResults, SHUTDOWN_RECOVERY_US, SpiInterface, check_id,
        decode, reg, spi_command,
    };
    use zigbee_sensor::{Error, WaitUs};

    /// Async register access.
    #[allow(async_fn_in_trait)]
    pub trait Interface {
        /// Transport error.
        type Error;
        /// Read a 16/24/40-bit register.
        async fn read_reg(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error>;
        /// Write a 16-bit register.
        async fn write_reg(&mut self, reg: u8, value: u16) -> Result<(), Self::Error>;
    }

    impl<I2C: embedded_hal_async::i2c::I2c> Interface for I2cInterface<I2C> {
        type Error = I2C::Error;

        async fn read_reg(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
            self.i2c.write_read(self.address, &[reg], buf).await
        }

        async fn write_reg(&mut self, reg: u8, value: u16) -> Result<(), Self::Error> {
            let [msb, lsb] = value.to_be_bytes();
            self.i2c.write(self.address, &[reg, msb, lsb]).await
        }
    }

    impl<SPI: embedded_hal_async::spi::SpiDevice> Interface for SpiInterface<SPI> {
        type Error = SPI::Error;

        async fn read_reg(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
            use embedded_hal_async::spi::Operation;
            self.spi
                .transaction(&mut [
                    Operation::Write(&[spi_command(reg, true)]),
                    Operation::Read(buf),
                ])
                .await
        }

        async fn write_reg(&mut self, reg: u8, value: u16) -> Result<(), Self::Error> {
            let [msb, lsb] = value.to_be_bytes();
            self.spi.write(&[spi_command(reg, false), msb, lsb]).await
        }
    }

    /// Async twin of [`super::Ina228`].
    #[derive(Debug)]
    pub struct Ina228<IFACE> {
        iface: IFACE,
        calibration: Calibration,
        config: Config,
        mode: Mode,
    }

    constructors!(Ina228);

    impl<IFACE: Interface> Ina228<IFACE> {
        /// See [`super::Ina228::read_register`].
        pub async fn read_register(&mut self, reg: u8) -> Result<u16, Error<IFACE::Error>> {
            let mut buf = [0; 2];
            self.iface
                .read_reg(reg, &mut buf)
                .await
                .map_err(Error::bus)?;
            Ok(u16::from_be_bytes(buf))
        }

        /// See [`super::Ina228::write_register`].
        pub async fn write_register(
            &mut self,
            reg: u8,
            value: u16,
        ) -> Result<(), Error<IFACE::Error>> {
            self.iface.write_reg(reg, value).await.map_err(Error::bus)
        }

        /// See [`super::Ina228::probe`].
        pub async fn probe(&mut self) -> Result<Chip, Error<IFACE::Error>> {
            let manufacturer = self.read_register(reg::MANUFACTURER_ID).await?;
            let device = self.read_register(reg::DEVICE_ID).await?;
            check_id(manufacturer, device)
        }

        /// See [`super::Ina228::init`].
        pub async fn init(&mut self) -> Result<(), Error<IFACE::Error>> {
            self.write_register(reg::CONFIG, self.config_register())
                .await?;
            self.write_register(reg::SHUNT_CAL, self.calibration.register)
                .await?;
            self.write_register(reg::ADC_CONFIG, self.idle_adc_config())
                .await
        }

        /// See [`super::Ina228::reset`].
        pub async fn reset(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
            self.write_register(reg::CONFIG, CONFIG_RST).await?;
            Ok(RESET_WAIT_US)
        }

        /// See [`super::Ina228::reset_accumulators`].
        pub async fn reset_accumulators(&mut self) -> Result<(), Error<IFACE::Error>> {
            self.write_register(reg::CONFIG, self.config_register() | CONFIG_RSTACC)
                .await
        }

        async fn read_bytes<const N: usize>(
            &mut self,
            reg: u8,
        ) -> Result<[u8; N], Error<IFACE::Error>> {
            let mut buf = [0; N];
            self.iface
                .read_reg(reg, &mut buf)
                .await
                .map_err(Error::bus)?;
            Ok(buf)
        }
    }

    impl<IFACE: Interface> zigbee_sensor::asynch::Sensor for Ina228<IFACE> {
        type BusError = IFACE::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
            match self.mode {
                Mode::Triggered => {
                    self.write_register(reg::ADC_CONFIG, self.config.register(MODE_TRIGGERED_ALL))
                        .await?;
                    Ok(self.config.conversion_us())
                }
                Mode::Continuous => Ok(0),
            }
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<IFACE::Error>> {
            let diag = self.read_register(reg::DIAG_ALRT).await?;
            if self.mode == Mode::Triggered && diag & DIAG_CNVRF == 0 {
                return Err(Error::NotReady);
            }
            let mut raw = RawResults {
                diag,
                vshunt: self.read_bytes(reg::VSHUNT).await?,
                vbus: self.read_bytes(reg::VBUS).await?,
                dietemp: self.read_register(reg::DIETEMP).await?,
                current: self.read_bytes(reg::CURRENT).await?,
                power: self.read_bytes(reg::POWER).await?,
                accumulators: None,
            };
            if self.mode == Mode::Continuous {
                raw.accumulators = Some((
                    self.read_bytes(reg::ENERGY).await?,
                    self.read_bytes(reg::CHARGE).await?,
                ));
            }
            decode(&self.calibration, &raw)
        }
    }

    impl<IFACE: Interface> zigbee_sensor::asynch::LowPower for Ina228<IFACE> {
        type BusError = IFACE::Error;

        async fn sleep(&mut self) -> Result<(), Error<IFACE::Error>> {
            self.write_register(reg::ADC_CONFIG, self.config.register(MODE_SHUTDOWN))
                .await
        }

        async fn wake(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
            self.write_register(reg::ADC_CONFIG, self.idle_adc_config())
                .await?;
            Ok(SHUTDOWN_RECOVERY_US)
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cT};
    use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiT};
    use std::vec;
    use std::vec::Vec;
    use zigbee_sensor::blocking::{LowPower, Sensor};

    const ADDR: u8 = DEFAULT_ADDRESS;

    /// Section 8.2.2: 16.2 mΩ shunt, 10 A maximum → SHUNT_CAL 4050.
    fn example_calibration() -> Calibration {
        Calibration::for_max_current(16_200, 10_000, AdcRange::Mv163).unwrap()
    }

    fn be24(v: u32) -> Vec<u8> {
        v.to_be_bytes()[1..].to_vec()
    }

    fn be40(v: u64) -> Vec<u8> {
        v.to_be_bytes()[3..].to_vec()
    }

    fn rd(reg: u8, data: Vec<u8>) -> I2cT {
        I2cT::write_read(ADDR, vec![reg], data)
    }

    fn wr(reg: u8, value: u16) -> I2cT {
        let [m, l] = value.to_be_bytes();
        I2cT::write(ADDR, vec![reg, m, l])
    }

    /// Table 8-4 returned values.
    fn example_reads(diag: u16) -> Vec<I2cT> {
        vec![
            rd(reg::DIAG_ALRT, diag.to_be_bytes().to_vec()),
            rd(reg::VSHUNT, be24(311_040 << 4)),
            rd(reg::VBUS, be24(245_760 << 4)),
            rd(reg::DIETEMP, 3_200u16.to_be_bytes().to_vec()),
            rd(reg::CURRENT, be24(314_572 << 4)),
            rd(reg::POWER, be24(4_718_604)),
        ]
    }

    const EXAMPLE: Measurement = Measurement {
        bus_voltage: 48_000,
        shunt_voltage: 97_200_000,
        current: 5_999_985,
        power: 288_001,
        temperature: 2_500,
        energy: None,
        charge: None,
    };

    #[test]
    fn calibration_matches_design_example() {
        let cal = example_calibration();
        assert_eq!(cal.register, 4050);
        // 10 A / 2^19 = 19.073486 µA/LSB.
        assert_eq!(cal.current_q24, 320_000_000);
        assert_eq!(cal.current_ua(314_572), 5_999_985);
        assert_eq!(cal.power_mw(4_718_604), 288_001);
        // 1 036 800 J = 288 Wh; 21 600 C = 6 Ah.
        assert_eq!(cal.energy_mwh(1_061_683_200), 288_000);
        assert_eq!(cal.charge_uah(1_132_462_080), 6_000_000);
        assert_eq!(cal.charge_uah(-1_132_462_080), -6_000_000);
        assert_eq!(
            Calibration::new(16_200, 19_073, AdcRange::Mv163).map(|c| c.register),
            Some(4049)
        );
        let narrow = Calibration::for_max_current(16_200, 2_500, AdcRange::Mv41).unwrap();
        assert_eq!(narrow.register, 4050);
        assert_eq!(narrow.current_ua(1 << 19), 2_500_000);
        assert_eq!(Calibration::from_register(1, 0x7FFF, AdcRange::Mv163), None);
        assert_eq!(
            Calibration::for_max_current(16_200, 100_000, AdcRange::Mv163),
            None
        );
    }

    #[test]
    fn raw_conversions() {
        assert_eq!(raw20([0xFF, 0xFF, 0xF0]), -1);
        assert_eq!(raw20([0x7F, 0xFF, 0xF0]), (1 << 19) - 1);
        assert_eq!(shunt_nanovolts(311_040, AdcRange::Mv163), 97_200_000);
        assert_eq!(shunt_nanovolts(-(1 << 19), AdcRange::Mv163), -163_840_000);
        assert_eq!(shunt_nanovolts(1 << 18, AdcRange::Mv41), 20_480_000);
        assert_eq!(bus_millivolts(245_760), 48_000);
        assert_eq!(temperature_centi(3_200), 2_500);
        assert_eq!(temperature_centi(-1_280), -1_000);
        let reset = Config::default();
        assert_eq!(reset.register(MODE_CONTINUOUS_ALL), 0xFB68);
        assert_eq!(reset.conversion_us(), 3_471);
        assert_eq!(spi_command(reg::DEVICE_ID, true), 0xFD);
        assert_eq!(spi_command(reg::ADC_CONFIG, false), 0x04);
    }

    #[test]
    fn probe_identifies_parts() {
        let expectations = [
            rd(reg::MANUFACTURER_ID, vec![0x54, 0x49]),
            rd(reg::DEVICE_ID, vec![0x22, 0x81]),
            rd(reg::MANUFACTURER_ID, vec![0x54, 0x49]),
            rd(reg::DEVICE_ID, vec![0x22, 0x60]),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = Ina228::new_i2c(&mut i2c, ADDR, example_calibration());
        assert_eq!(ina.probe(), Ok(Chip::Ina228));
        assert_eq!(ina.probe(), Err(Error::UnexpectedId(0x5449_2260)));
        i2c.done();
    }

    #[test]
    fn triggered_measurement_i2c() {
        let mut expectations = vec![
            wr(reg::CONFIG, 0),
            wr(reg::SHUNT_CAL, 4050),
            wr(reg::ADC_CONFIG, 0x0B68),
            wr(reg::ADC_CONFIG, 0x7B68),
            rd(reg::DIAG_ALRT, vec![0x00, 0x01]),
        ];
        expectations.extend(example_reads(DIAG_CNVRF | DIAG_MEMSTAT));
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = Ina228::new_i2c(&mut i2c, ADDR, example_calibration());
        ina.init().unwrap();
        assert_eq!(ina.start_measurement().unwrap(), 3_471);
        assert_eq!(ina.read_measurement(), Err(Error::NotReady));
        assert_eq!(ina.read_measurement().unwrap(), EXAMPLE);
        i2c.done();
    }

    #[test]
    fn continuous_reads_accumulators_and_mathof() {
        let mut expectations = example_reads(DIAG_MEMSTAT);
        expectations.extend([
            rd(reg::ENERGY, be40(1_061_683_200)),
            rd(
                reg::CHARGE,
                be40((-1_132_462_080i64) as u64 & 0xFF_FFFF_FFFF),
            ),
        ]);
        expectations.extend(example_reads(DIAG_MEMSTAT | DIAG_MATHOF));
        expectations.extend([
            rd(reg::ENERGY, be40(0)),
            rd(reg::CHARGE, be40(0)),
            wr(reg::CONFIG, CONFIG_RSTACC),
            wr(reg::ADC_CONFIG, 0x0B68),
            wr(reg::ADC_CONFIG, 0xFB68),
        ]);
        let mut i2c = I2cMock::new(&expectations);
        let mut ina =
            Ina228::new_i2c(&mut i2c, ADDR, example_calibration()).with_mode(Mode::Continuous);
        assert_eq!(ina.start_measurement().unwrap(), 0);
        let m = ina.read_measurement().unwrap();
        assert_eq!(m.energy, Some(288_000));
        assert_eq!(m.charge, Some(-6_000_000));
        let mut quantities = Vec::new();
        m.for_each(&mut |q| quantities.push(q));
        assert_eq!(quantities.last(), Some(&Quantity::Energy(288_000)));
        assert_eq!(ina.read_measurement(), Err(Error::InvalidData));
        ina.reset_accumulators().unwrap();
        ina.sleep().unwrap();
        assert_eq!(ina.wake().unwrap(), SHUTDOWN_RECOVERY_US);
        i2c.done();
    }

    #[test]
    fn ina229_spi_frames() {
        let expectations = [
            SpiT::transaction_start(),
            SpiT::write(0xF9),
            SpiT::read_vec(vec![0x54, 0x49]),
            SpiT::transaction_end(),
            SpiT::transaction_start(),
            SpiT::write(0xFD),
            SpiT::read_vec(vec![0x22, 0x91]),
            SpiT::transaction_end(),
            SpiT::transaction_start(),
            SpiT::write_vec(vec![0x04, 0x7B, 0x68]),
            SpiT::transaction_end(),
            SpiT::transaction_start(),
            SpiT::write(0x2D),
            SpiT::read_vec(vec![0x00, 0x03]),
            SpiT::transaction_end(),
            SpiT::transaction_start(),
            SpiT::write(0x11),
            SpiT::read_vec(be24(311_040 << 4)),
            SpiT::transaction_end(),
            SpiT::transaction_start(),
            SpiT::write(0x15),
            SpiT::read_vec(be24(245_760 << 4)),
            SpiT::transaction_end(),
            SpiT::transaction_start(),
            SpiT::write(0x19),
            SpiT::read_vec(vec![0x0C, 0x80]),
            SpiT::transaction_end(),
            SpiT::transaction_start(),
            SpiT::write(0x1D),
            SpiT::read_vec(be24(314_572 << 4)),
            SpiT::transaction_end(),
            SpiT::transaction_start(),
            SpiT::write(0x21),
            SpiT::read_vec(be24(4_718_604)),
            SpiT::transaction_end(),
        ];
        let mut spi = SpiMock::new(&expectations);
        let mut ina = Ina228::new_spi(&mut spi, example_calibration());
        assert_eq!(ina.probe(), Ok(Chip::Ina229));
        ina.start_measurement().unwrap();
        assert_eq!(ina.read_measurement().unwrap(), EXAMPLE);
        spi.done();
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

        let mut expectations = vec![wr(reg::ADC_CONFIG, 0x7B68)];
        expectations.extend(example_reads(DIAG_CNVRF | DIAG_MEMSTAT));
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = asynch::Ina228::new_i2c(&mut i2c, ADDR, example_calibration());
        let mut delay = RecordingDelay(0);
        assert_eq!(block_on(ina.measure(&mut delay)).unwrap(), EXAMPLE);
        assert_eq!(delay.0, 3_471);
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
