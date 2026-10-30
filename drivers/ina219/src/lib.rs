//! Texas Instruments INA219 zero-drift, bidirectional current/power monitor
//! (I2C).
//!
//! The INA219 measures the shunt voltage and the bus voltage (at IN−) and
//! computes current and power from a calibration value. The driver derives
//! the calibration register and all integer scale factors from the shunt
//! resistance and the chosen current LSB ([`Calibration`]).
//!
//! In [`Mode::Triggered`] (default) `start_measurement` writes a single-shot
//! shunt + bus conversion and returns the conversion time; between samples
//! the ADC is idle. [`Mode::Continuous`] keeps converting and
//! `start_measurement` returns 0.
//!
//! Reference: TI "INA219 Zerø-Drift, Bidirectional Current/Power Monitor With
//! I2C Interface", SBOS448G (revised December 2015).
//!
//! ```ignore
//! let cal = Calibration::new(100_000, 100_000).unwrap(); // 0.1 Ω, 100 µA/LSB
//! let mut ina = Ina219::new(i2c, DEFAULT_ADDRESS, cal);
//! ina.init()?;
//! let wait_us = ina.start_measurement()?;
//! // ... let the Zigbee stack run for wait_us ...
//! let m = ina.read_measurement()?;
//! ```

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Address with A1 = A0 = GND. A0/A1 select 0x40..=0x4F.
pub const DEFAULT_ADDRESS: u8 = 0x40;

const REG_CONFIG: u8 = 0x00;
const REG_SHUNT: u8 = 0x01;
const REG_BUS: u8 = 0x02;
const REG_POWER: u8 = 0x03;
const REG_CURRENT: u8 = 0x04;
const REG_CALIBRATION: u8 = 0x05;

const CONFIG_RESET: u16 = 1 << 15;
const MODE_POWER_DOWN: u16 = 0b000;
const MODE_SHUNT_BUS_TRIGGERED: u16 = 0b011;
const MODE_SHUNT_BUS_CONTINUOUS: u16 = 0b111;

/// Recovery time from power-down (datasheet section 8.4.1: 40 µs).
pub const POWER_DOWN_RECOVERY_US: WaitUs = 40;

/// Bus voltage full-scale range (BRNG).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BusRange {
    /// 0..16 V.
    V16,
    /// 0..32 V (power-on default; the input is still limited to 26 V).
    #[default]
    V32,
}

/// Shunt PGA gain and full-scale range (PG).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ShuntRange {
    /// ±40 mV (gain /1).
    Mv40,
    /// ±80 mV (gain /2).
    Mv80,
    /// ±160 mV (gain /4).
    Mv160,
    /// ±320 mV (gain /8, power-on default).
    #[default]
    Mv320,
}

/// ADC resolution / averaging for one channel (BADC / SADC, table 5).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Adc {
    /// 9 bit, 84 µs.
    Bits9,
    /// 10 bit, 148 µs.
    Bits10,
    /// 11 bit, 276 µs.
    Bits11,
    /// 12 bit, 532 µs (power-on default).
    #[default]
    Bits12,
    /// 12 bit, 2 samples averaged, 1.06 ms.
    Samples2,
    /// 4 samples, 2.13 ms.
    Samples4,
    /// 8 samples, 4.26 ms.
    Samples8,
    /// 16 samples, 8.51 ms.
    Samples16,
    /// 32 samples, 17.02 ms.
    Samples32,
    /// 64 samples, 34.05 ms.
    Samples64,
    /// 128 samples, 68.10 ms.
    Samples128,
}

impl Adc {
    const fn bits(self) -> u16 {
        match self {
            Self::Bits9 => 0b0000,
            Self::Bits10 => 0b0001,
            Self::Bits11 => 0b0010,
            Self::Bits12 => 0b0011,
            Self::Samples2 => 0b1001,
            Self::Samples4 => 0b1010,
            Self::Samples8 => 0b1011,
            Self::Samples16 => 0b1100,
            Self::Samples32 => 0b1101,
            Self::Samples64 => 0b1110,
            Self::Samples128 => 0b1111,
        }
    }

    /// Typical conversion time from table 5, in µs.
    pub const fn conversion_us(self) -> u32 {
        match self {
            Self::Bits9 => 84,
            Self::Bits10 => 148,
            Self::Bits11 => 276,
            Self::Bits12 => 532,
            Self::Samples2 => 1_060,
            Self::Samples4 => 2_130,
            Self::Samples8 => 4_260,
            Self::Samples16 => 8_510,
            Self::Samples32 => 17_020,
            Self::Samples64 => 34_050,
            Self::Samples128 => 68_100,
        }
    }
}

/// Conversion mode used by the [`zigbee_sensor::blocking::Sensor`] impl.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Mode {
    /// One shunt + bus conversion per `start_measurement` (lowest energy).
    #[default]
    Triggered,
    /// Free-running shunt + bus conversions.
    Continuous,
}

/// ADC and range configuration (Configuration register 00h).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// Bus voltage range.
    pub bus_range: BusRange,
    /// Shunt voltage range.
    pub shunt_range: ShuntRange,
    /// Bus ADC setting.
    pub bus_adc: Adc,
    /// Shunt ADC setting.
    pub shunt_adc: Adc,
}

impl Config {
    /// Configuration register value with the given MODE bits.
    pub const fn register(&self, mode_bits: u16) -> u16 {
        let brng = match self.bus_range {
            BusRange::V16 => 0,
            BusRange::V32 => 1 << 13,
        };
        let pg = match self.shunt_range {
            ShuntRange::Mv40 => 0b00,
            ShuntRange::Mv80 => 0b01,
            ShuntRange::Mv160 => 0b10,
            ShuntRange::Mv320 => 0b11,
        } << 11;
        brng | pg | (self.bus_adc.bits() << 7) | (self.shunt_adc.bits() << 3) | (mode_bits & 0b111)
    }

    /// Worst-case time of one shunt + bus conversion in µs: typical
    /// values of table 5 plus the 10 % tolerance of the electrical
    /// characteristics.
    pub const fn conversion_us(&self) -> WaitUs {
        let typ = self.bus_adc.conversion_us() + self.shunt_adc.conversion_us();
        typ + typ / 10
    }
}

/// Calibration register value and the integer scale factors derived from it.
///
/// `current_q24` is µA per Current-register LSB and `power_q24` mW per
/// Power-register LSB, both in Q8.24 fixed point, so decoding a sample
/// needs only a 64-bit multiply and a shift.
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
    /// Calibration for a shunt of `shunt_micro_ohm` µΩ and a Current
    /// register LSB of `current_lsb_na` nA (equation 1:
    /// `Cal = trunc(0.04096 / (Current_LSB × R_shunt))`).
    ///
    /// The scale factors are derived from the truncated register value, so
    /// they match what the device actually computes. Returns `None` when
    /// the calibration value is 0, does not fit the register (bit 0 is
    /// void, so the maximum is 0xFFFE) or full-scale current/power would
    /// not fit `i32` µA/mW.
    pub const fn new(shunt_micro_ohm: u32, current_lsb_na: u32) -> Option<Self> {
        if shunt_micro_ohm == 0 || current_lsb_na == 0 {
            return None;
        }
        // 0.04096 / (nA × 1e-9 × µΩ × 1e-6) = 4.096e13 / (nA × µΩ).
        let cal = 40_960_000_000_000 / (current_lsb_na as u64 * shunt_micro_ohm as u64);
        let cal = cal & !1;
        if cal == 0 || cal > 0xFFFE {
            return None;
        }
        let denominator = cal * shunt_micro_ohm as u64;
        // Effective Current_LSB in µA = 4.096e10 / (Cal × µΩ); power is 20×.
        let current_q24 = q24(40_960_000_000, denominator);
        let power_q24 = q24(819_200_000, denominator);
        // Full-scale current (2^15 LSB) and power (2^16 LSB) must fit i32.
        if current_q24 >= 1 << 40 || power_q24 >= 1 << 39 {
            return None;
        }
        Some(Self {
            register: cal as u16,
            current_q24,
            power_q24,
        })
    }

    /// Calibration with the smallest current LSB that still covers
    /// `max_current_ma` (equation 2: `Current_LSB = I_max / 2^15`, rounded up
    /// to a whole nA).
    pub const fn for_max_current(shunt_micro_ohm: u32, max_current_ma: u32) -> Option<Self> {
        let lsb = (max_current_ma as u64 * 1_000_000).div_ceil(32_768);
        if lsb > u32::MAX as u64 {
            return None;
        }
        Self::new(shunt_micro_ohm, lsb as u32)
    }

    /// Current in µA from a raw Current register value.
    pub const fn current_ua(&self, raw: i16) -> i32 {
        apply_q24(raw as i64, self.current_q24) as i32
    }

    /// Power in mW from a raw Power register value.
    pub const fn power_mw(&self, raw: u16) -> i32 {
        apply_q24(raw as i64, self.power_q24) as i32
    }
}

/// `round(num × 2^24 / den)`; `num` must stay below 2^39.
const fn q24(num: u64, den: u64) -> i64 {
    (((num << 24) + den / 2) / den) as i64
}

/// `round(raw × mul / 2^24)`.
const fn apply_q24(raw: i64, mul: i64) -> i64 {
    (raw * mul + (1 << 23)) >> 24
}

/// One measurement.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Bus voltage at IN− in mV.
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

/// Bus voltage register (02h): flags and voltage.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BusVoltage(pub u16);

impl BusVoltage {
    /// Bus voltage in mV (BD12..BD0 × 4 mV).
    pub const fn millivolts(self) -> i32 {
        (self.0 >> 3) as i32 * 4
    }

    /// CNVR: a conversion finished since the last Power register read.
    pub const fn conversion_ready(self) -> bool {
        self.0 & 0b10 != 0
    }

    /// OVF: current or power calculation overflowed.
    pub const fn overflow(self) -> bool {
        self.0 & 0b01 != 0
    }
}

/// Shunt voltage in µV from the raw Shunt Voltage register (10 µV LSB).
pub const fn shunt_microvolts(raw: i16) -> i32 {
    raw as i32 * 10
}

/// Decode the four result registers.
///
/// Returns [`Error::InvalidData`] on math overflow (current/power out of
/// range for the chosen calibration).
pub fn decode<E>(
    calibration: &Calibration,
    bus: BusVoltage,
    shunt: u16,
    current: u16,
    power: u16,
) -> Result<Measurement, Error<E>> {
    if bus.overflow() {
        return Err(Error::InvalidData);
    }
    Ok(Measurement {
        bus_voltage: bus.millivolts(),
        shunt_voltage: shunt_microvolts(shunt as i16),
        current: calibration.current_ua(current as i16),
        power: calibration.power_mw(power),
    })
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
                    bus_range: BusRange::V32,
                    shunt_range: ShuntRange::Mv320,
                    bus_adc: Adc::Bits12,
                    shunt_adc: Adc::Bits12,
                },
                mode: Mode::Triggered,
            }
        }

        /// Range and ADC configuration written by `init`.
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
pub struct Ina219<I2C> {
    i2c: I2C,
    address: u8,
    calibration: Calibration,
    config: Config,
    mode: Mode,
}

impl<I2C> Ina219<I2C> {
    constructors!();
}

impl<I2C: embedded_hal::i2c::I2c> Ina219<I2C> {
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

    /// Write calibration and configuration. In triggered mode the device is
    /// left powered down until the first `start_measurement`.
    pub fn init(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_register(REG_CALIBRATION, self.calibration.register)?;
        self.write_register(REG_CONFIG, self.idle_config())
    }

    /// Reset all registers to power-on defaults. Call `init` afterwards.
    pub fn reset(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_register(REG_CONFIG, CONFIG_RESET)
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Ina219<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        match self.mode {
            Mode::Triggered => {
                self.write_register(REG_CONFIG, self.config.register(MODE_SHUNT_BUS_TRIGGERED))?;
                Ok(self.config.conversion_us())
            }
            Mode::Continuous => Ok(0),
        }
    }

    /// In triggered mode returns [`Error::NotReady`] until CNVR is set.
    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        let bus = BusVoltage(self.read_register(REG_BUS)?);
        if self.mode == Mode::Triggered && !bus.conversion_ready() {
            return Err(Error::NotReady);
        }
        let shunt = self.read_register(REG_SHUNT)?;
        let current = self.read_register(REG_CURRENT)?;
        // Reading Power clears CNVR.
        let power = self.read_register(REG_POWER)?;
        decode(&self.calibration, bus, shunt, current, power)
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::LowPower for Ina219<I2C> {
    type BusError = I2C::Error;

    fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_register(REG_CONFIG, self.config.register(MODE_POWER_DOWN))
    }

    fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.write_register(REG_CONFIG, self.idle_config())?;
        Ok(POWER_DOWN_RECOVERY_US)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        Adc, BusRange, BusVoltage, CONFIG_RESET, Calibration, Config, MODE_POWER_DOWN,
        MODE_SHUNT_BUS_CONTINUOUS, MODE_SHUNT_BUS_TRIGGERED, Measurement, Mode,
        POWER_DOWN_RECOVERY_US, REG_BUS, REG_CALIBRATION, REG_CONFIG, REG_CURRENT, REG_POWER,
        REG_SHUNT, ShuntRange, decode,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Ina219`].
    pub struct Ina219<I2C> {
        i2c: I2C,
        address: u8,
        calibration: Calibration,
        config: Config,
        mode: Mode,
    }

    impl<I2C> Ina219<I2C> {
        constructors!();
    }

    impl<I2C: I2c> Ina219<I2C> {
        /// See [`super::Ina219::write_register`].
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

        /// See [`super::Ina219::read_register`].
        pub async fn read_register(&mut self, register: u8) -> Result<u16, Error<I2C::Error>> {
            let mut buf = [0u8; 2];
            self.i2c
                .write_read(self.address, &[register], &mut buf)
                .await
                .map_err(Error::bus)?;
            Ok(u16::from_be_bytes(buf))
        }

        /// See [`super::Ina219::init`].
        pub async fn init(&mut self) -> Result<(), Error<I2C::Error>> {
            self.write_register(REG_CALIBRATION, self.calibration.register)
                .await?;
            self.write_register(REG_CONFIG, self.idle_config()).await
        }

        /// See [`super::Ina219::reset`].
        pub async fn reset(&mut self) -> Result<(), Error<I2C::Error>> {
            self.write_register(REG_CONFIG, CONFIG_RESET).await
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Ina219<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            match self.mode {
                Mode::Triggered => {
                    self.write_register(REG_CONFIG, self.config.register(MODE_SHUNT_BUS_TRIGGERED))
                        .await?;
                    Ok(self.config.conversion_us())
                }
                Mode::Continuous => Ok(0),
            }
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let bus = BusVoltage(self.read_register(REG_BUS).await?);
            if self.mode == Mode::Triggered && !bus.conversion_ready() {
                return Err(Error::NotReady);
            }
            let shunt = self.read_register(REG_SHUNT).await?;
            let current = self.read_register(REG_CURRENT).await?;
            let power = self.read_register(REG_POWER).await?;
            decode(&self.calibration, bus, shunt, current, power)
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::LowPower for Ina219<I2C> {
        type BusError = I2C::Error;

        async fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
            self.write_register(REG_CONFIG, self.config.register(MODE_POWER_DOWN))
                .await
        }

        async fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.write_register(REG_CONFIG, self.idle_config()).await?;
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

    /// Section 9.2.2: 2 mΩ shunt, 1 mA current LSB.
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

    #[test]
    fn calibration_matches_datasheet_example() {
        let cal = example_calibration();
        assert_eq!(cal.register, 0x5000);
        // Table 8: Current 2710h → 10.0 A, Power 1766h → 119.8 W.
        assert_eq!(cal.current_ua(0x2710), 10_000_000);
        assert_eq!(cal.power_mw(0x1766), 119_800);
        assert_eq!(cal.current_ua(-1), -1_000);
        assert_eq!(Calibration::new(1, 1), None);
        assert_eq!(Calibration::new(0, 1_000), None);
        // 1 µΩ with a 1 A LSB: Cal = 40960 but full scale is beyond i32 µA.
        assert_eq!(Calibration::new(1, 1_000_000_000), None);
        // 15 A over 2 mΩ: minimum LSB 457.78 µA (section 9.2.2); 44739 with
        // the void bit 0 cleared.
        let cal = Calibration::for_max_current(2_000, 15_000).unwrap();
        assert_eq!(cal.register, 44_738);
    }

    #[test]
    fn config_register_encoding() {
        // Table 8 configuration 019Fh: 16 V, /1, 12-bit, continuous.
        let config = Config {
            bus_range: BusRange::V16,
            shunt_range: ShuntRange::Mv40,
            bus_adc: Adc::Bits12,
            shunt_adc: Adc::Bits12,
        };
        assert_eq!(config.register(MODE_SHUNT_BUS_CONTINUOUS), 0x019F);
        // Power-on default 399Fh.
        assert_eq!(Config::default().register(0b111), 0x399F);
        assert_eq!(config.conversion_us(), 1_170);
    }

    #[test]
    fn triggered_measurement_datasheet_example() {
        let config = Config {
            bus_range: BusRange::V16,
            shunt_range: ShuntRange::Mv40,
            bus_adc: Adc::Bits12,
            shunt_adc: Adc::Samples2,
        };
        let expectations = [
            write(REG_CALIBRATION, 0x5000),
            write(REG_CONFIG, 0x01C8),
            write(REG_CONFIG, 0x01CB),
            // Bus 5D98h with CNVR set.
            read(REG_BUS, 0x5D9A),
            read(REG_SHUNT, 0x07D0),
            read(REG_CURRENT, 0x2710),
            read(REG_POWER, 0x1766),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut ina = Ina219::new(&mut i2c, ADDR, example_calibration()).with_config(config);
        ina.init().unwrap();
        assert_eq!(ina.start_measurement().unwrap(), (532 + 1_060) * 11 / 10);
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
    fn not_ready_and_overflow() {
        let expectations = [
            read(REG_BUS, 0x5D98),
            read(REG_BUS, 0x5D9B),
            read(REG_SHUNT, 0x7FFF),
            read(REG_CURRENT, 0x7FFF),
            read(REG_POWER, 0xFFFF),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut ina = Ina219::new(&mut i2c, ADDR, example_calibration());
        assert_eq!(ina.read_measurement(), Err(Error::NotReady));
        assert_eq!(ina.read_measurement(), Err(Error::InvalidData));
        i2c.done();
    }

    #[test]
    fn continuous_mode_and_power_down() {
        let expectations = [
            write(REG_CONFIG, 0x399F),
            read(REG_BUS, 0x5D98),
            read(REG_SHUNT, 0xF830),
            read(REG_CURRENT, 0xD8F0),
            read(REG_POWER, 0x1766),
            write(REG_CONFIG, 0x3998),
            write(REG_CONFIG, 0x399F),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut ina =
            Ina219::new(&mut i2c, ADDR, example_calibration()).with_mode(Mode::Continuous);
        ina.write_register(REG_CONFIG, ina.idle_config()).unwrap();
        assert_eq!(ina.start_measurement().unwrap(), 0);
        let m = ina.read_measurement().unwrap();
        assert_eq!(m.shunt_voltage, -20_000);
        assert_eq!(m.current, -10_000_000);
        ina.sleep().unwrap();
        assert_eq!(ina.wake().unwrap(), POWER_DOWN_RECOVERY_US);
        i2c.done();
    }

    #[test]
    fn async_measure_awaits_conversion_time() {
        use zigbee_sensor::asynch::Sensor as _;

        struct RecordingDelay(u32);
        impl embedded_hal_async::delay::DelayNs for RecordingDelay {
            async fn delay_ns(&mut self, ns: u32) {
                self.0 += ns / 1_000;
            }
        }

        let expectations = [
            write(REG_CONFIG, 0x399B),
            read(REG_BUS, 0x5D9A),
            read(REG_SHUNT, 0x07D0),
            read(REG_CURRENT, 0x2710),
            read(REG_POWER, 0x1766),
        ];
        let mut i2c = Mock::new(&expectations);
        let mut ina = asynch::Ina219::new(&mut i2c, ADDR, example_calibration());
        let mut delay = RecordingDelay(0);
        let m = block_on(ina.measure(&mut delay)).unwrap();
        assert_eq!(m.power, 119_800);
        assert_eq!(delay.0, 1_170);
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
