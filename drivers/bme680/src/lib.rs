//! Bosch BME680 / BME688 gas, pressure, temperature and humidity sensor
//! over I2C or SPI, raw values only (no BSEC).
//!
//! The driver runs forced mode with one heater step (`res_heat_0`,
//! `gas_wait_0`, `nb_conv = 0`): [`start_measurement`] triggers one
//! T/P/RH conversion followed by a heated gas-resistance measurement and
//! returns the conversion time plus the heater duration. The part then
//! goes back to sleep (0.15 µA typ.) by itself.
//!
//! Gas resistance is reported in Ω together with the `gas_valid` and
//! `heat_stab` flags; [`Readings`] only publishes it when both are set.
//! BSEC (IAQ, eCO2, bVOC) is proprietary and not provided.
//!
//! Compensation uses the integer formulas of the datasheets (32-bit for
//! T/P/RH; the BME680 gas formula needs one 64-bit division, the BME688 one
//! is 32-bit). The integer pressure formula is up to ~8 Pa away from the
//! datasheet's floating-point one, below the ±12 Pa relative accuracy.
//! The variant is read from `variant_id` (0xF0).
//!
//! References:
//! * Bosch "BME680 Low power gas, pressure, temperature & humidity sensor"
//!   datasheet, BST-BME680-DS001 rev 1.9 (sections 3.2-3.4, 5).
//! * Bosch "BME688 Digital low power gas, pressure, temperature & humidity
//!   sensor with AI" datasheet, BST-BME688-DS000 rev 1.3 (sections 3, 5).
//! * Conversion-time budget cross-checked against the BSD-3-Clause Bosch
//!   BME68x SensorAPI (no code taken).
//!
//! [`start_measurement`]: zigbee_sensor::blocking::Sensor::start_measurement

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// I2C address with SDO low.
pub const PRIMARY_ADDRESS: u8 = 0x76;
/// I2C address with SDO high.
pub const SECONDARY_ADDRESS: u8 = 0x77;
/// `chip_id` of BME680 and BME688.
pub const CHIP_ID: u8 = 0x61;

const REG_CALIB3: u8 = 0x00;
const REG_FIELD0: u8 = 0x1D;
const REG_RES_HEAT_0: u8 = 0x5A;
const REG_GAS_WAIT_0: u8 = 0x64;
const REG_CTRL_GAS_1: u8 = 0x71;
const REG_CTRL_HUM: u8 = 0x72;
const REG_SPI_PAGE: u8 = 0x73;
const REG_CTRL_MEAS: u8 = 0x74;
const REG_CONFIG: u8 = 0x75;
const REG_CALIB1: u8 = 0x8A;
const REG_CHIP_ID: u8 = 0xD0;
const REG_RESET: u8 = 0xE0;
const REG_CALIB2: u8 = 0xE1;
const REG_VARIANT_ID: u8 = 0xF0;

const CMD_SOFT_RESET: u8 = 0xB6;
const NEW_DATA: u8 = 1 << 7;
const GAS_VALID: u8 = 1 << 5;
const HEAT_STAB: u8 = 1 << 4;
const SPI_MEM_PAGE: u8 = 1 << 4;

const LEN_CALIB1: usize = 23;
const LEN_CALIB2: usize = 14;
const LEN_CALIB3: usize = 5;
const LEN_FIELD: usize = 17;

/// Start-up time after power-on or soft reset (t_startup 2 ms).
pub const RESET_WAIT_US: WaitUs = 2_000;

/// Register access over I2C or SPI.
pub trait Interface {
    /// Transport error.
    type Error;
    /// Burst-read consecutive registers starting at `reg` (8-bit address).
    fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error>;
    /// Write one register (8-bit address).
    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error>;
}

/// I2C transport.
#[derive(Debug)]
pub struct I2cInterface<I2C> {
    /// The bus.
    pub i2c: I2C,
    /// 7-bit address ([`PRIMARY_ADDRESS`] or [`SECONDARY_ADDRESS`]).
    pub address: u8,
}

/// 4-wire SPI transport (mode 0/3, ≤ 10 MHz).
///
/// SPI uses 7-bit addresses and two memory pages selected by
/// `status.spi_mem_page`: page 0 maps registers 0x80..=0xFF, page 1 maps
/// 0x00..=0x7F. The interface switches pages as needed and caches the
/// current one (a soft reset written through it resets the cache).
#[derive(Debug)]
pub struct SpiInterface<SPI> {
    /// The device (bus + chip select).
    pub spi: SPI,
    page: Option<bool>,
}

impl<SPI> SpiInterface<SPI> {
    /// Wrap an SPI device; the page is unknown until the first access.
    pub const fn new(spi: SPI) -> Self {
        Self { spi, page: None }
    }

    /// Page bit value needed for `reg`, or `None` if the switch is cached.
    fn page_switch(&self, reg: u8) -> Option<bool> {
        let page1 = reg < 0x80;
        (reg != REG_SPI_PAGE && self.page != Some(page1)).then_some(page1)
    }

    fn after_write(&mut self, reg: u8, value: u8) {
        if reg == REG_RESET && value == CMD_SOFT_RESET {
            self.page = Some(false);
        }
    }
}

const fn page_value(status: u8, page1: bool) -> u8 {
    if page1 {
        status | SPI_MEM_PAGE
    } else {
        status & !SPI_MEM_PAGE
    }
}

impl<I2C: embedded_hal::i2c::I2c> Interface for I2cInterface<I2C> {
    type Error = I2C::Error;

    fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, &[reg], buf)
    }

    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
        self.i2c.write(self.address, &[reg, value])
    }
}

impl<SPI: embedded_hal::spi::SpiDevice> SpiInterface<SPI> {
    fn select(&mut self, reg: u8) -> Result<(), SPI::Error> {
        use embedded_hal::spi::Operation;
        if let Some(page1) = self.page_switch(reg) {
            let mut status = [0];
            self.spi.transaction(&mut [
                Operation::Write(&[REG_SPI_PAGE | 0x80]),
                Operation::Read(&mut status),
            ])?;
            self.spi
                .write(&[REG_SPI_PAGE, page_value(status[0], page1)])?;
            self.page = Some(page1);
        }
        Ok(())
    }
}

impl<SPI: embedded_hal::spi::SpiDevice> Interface for SpiInterface<SPI> {
    type Error = SPI::Error;

    fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        use embedded_hal::spi::Operation;
        self.select(reg)?;
        self.spi
            .transaction(&mut [Operation::Write(&[reg | 0x80]), Operation::Read(buf)])
    }

    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
        self.select(reg)?;
        self.spi.write(&[reg & 0x7F, value])?;
        self.after_write(reg, value);
        Ok(())
    }
}

/// Part variant (`variant_id`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// BME680 (`variant_id` 0x00).
    Bme680,
    /// BME688 (`variant_id` 0x01): different gas ADC and formula.
    Bme688,
}

/// Oversampling of one channel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Oversampling {
    /// Channel not measured.
    Skip = 0,
    /// ×1.
    X1 = 1,
    /// ×2.
    X2 = 2,
    /// ×4.
    X4 = 3,
    /// ×8.
    X8 = 4,
    /// ×16.
    X16 = 5,
}

impl Oversampling {
    /// Number of conversions (0 when skipped).
    pub const fn factor(self) -> u32 {
        match self {
            Self::Skip => 0,
            _ => 1 << (self as u32 - 1),
        }
    }
}

/// IIR filter coefficient for temperature and pressure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(missing_docs)]
pub enum Filter {
    Off = 0,
    C1 = 1,
    C3 = 2,
    C7 = 3,
    C15 = 4,
    C31 = 5,
    C63 = 6,
    C127 = 7,
}

/// Heater set-point for the gas measurement (heater step 0).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Heater {
    /// Target hot-plate temperature in °C (200..=400 typical).
    pub temperature: u16,
    /// Heating time in ms, 1..=4032 (encoded with a ×1/×4/×16/×64 factor).
    pub duration_ms: u16,
}

impl Heater {
    /// `gas_wait_x` encoding (datasheet 5.3.3.3): 6-bit value and a 2-bit
    /// multiplier ×1/×4/×16/×64. Durations are rounded down.
    pub const fn gas_wait(self) -> u8 {
        let mut d = self.duration_ms;
        if d >= 0xFC0 {
            return 0xFF;
        }
        let mut factor = 0;
        while d > 0x3F {
            d /= 4;
            factor += 1;
        }
        (d as u8) | (factor << 6)
    }

    /// Duration actually programmed by [`Self::gas_wait`], in ms.
    pub const fn programmed_ms(self) -> u32 {
        let w = self.gas_wait();
        ((w & 0x3F) as u32) << (2 * (w >> 6))
    }
}

/// Measurement configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Temperature oversampling (must not be skipped).
    pub temperature: Oversampling,
    /// Pressure oversampling.
    pub pressure: Oversampling,
    /// Humidity oversampling.
    pub humidity: Oversampling,
    /// IIR filter.
    pub filter: Filter,
    /// Heater profile; `None` disables the gas measurement.
    pub heater: Option<Heater>,
}

impl Config {
    /// Bosch example forced-mode setting: T ×2, P ×16, RH ×1, filter off,
    /// heater 300 °C for 100 ms.
    pub const DEFAULT: Self = Self {
        temperature: Oversampling::X2,
        pressure: Oversampling::X16,
        humidity: Oversampling::X1,
        filter: Filter::Off,
        heater: Some(Heater {
            temperature: 300,
            duration_ms: 100,
        }),
    };

    /// `ctrl_meas` (`osrs_t`, `osrs_p`, `mode`).
    pub const fn ctrl_meas(&self, forced: bool) -> u8 {
        ((self.temperature as u8) << 5) | ((self.pressure as u8) << 2) | forced as u8
    }

    /// `ctrl_gas_1`: `run_gas` (bit 4 on BME680, bit 5 on BME688) and
    /// `nb_conv = 0`.
    pub const fn ctrl_gas_1(&self, variant: Variant) -> u8 {
        match (self.heater, variant) {
            (None, _) => 0,
            (Some(_), Variant::Bme680) => 0x10,
            (Some(_), Variant::Bme688) => 0x20,
        }
    }

    /// Worst-case forced-mode duration in µs: 1.963 ms per oversampled
    /// conversion, 4 × 0.477 ms T/P/H switching, 5 × 0.477 ms gas ADC,
    /// 1 ms wake-up, plus the heater duration when gas is enabled.
    pub const fn max_conversion_us(&self) -> WaitUs {
        let cycles = self.temperature.factor() + self.pressure.factor() + self.humidity.factor();
        let mut us = cycles * 1_963 + 477 * 4 + 477 * 5 + 1_000;
        if let Some(h) = self.heater {
            us += h.programmed_ms() * 1_000;
        }
        us
    }
}

impl Default for Config {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Factory calibration (`par_*`, heater and gas range trims).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub struct Calibration {
    pub t1: u16,
    pub t2: i16,
    pub t3: i8,
    pub p1: u16,
    pub p2: i16,
    pub p3: i8,
    pub p4: i16,
    pub p5: i16,
    pub p6: i8,
    pub p7: i8,
    pub p8: i16,
    pub p9: i16,
    pub p10: u8,
    pub h1: u16,
    pub h2: u16,
    pub h3: i8,
    pub h4: i8,
    pub h5: i8,
    pub h6: u8,
    pub h7: i8,
    pub g1: i8,
    pub g2: i16,
    pub g3: i8,
    pub res_heat_range: u8,
    pub res_heat_val: i8,
    pub range_sw_err: i8,
}

/// BME680 gas range constants (datasheet 3.4.1, integer variant).
const GAS_K1: [u32; 16] = [
    2_147_483_647,
    2_147_483_647,
    2_147_483_647,
    2_147_483_647,
    2_147_483_647,
    2_126_008_810,
    2_147_483_647,
    2_130_303_777,
    2_147_483_647,
    2_147_483_647,
    2_143_188_679,
    2_136_746_228,
    2_147_483_647,
    2_126_008_810,
    2_147_483_647,
    2_147_483_647,
];
const GAS_K2: [u32; 16] = [
    4_096_000_000,
    2_048_000_000,
    1_024_000_000,
    512_000_000,
    255_744_255,
    127_110_228,
    64_000_000,
    32_258_064,
    16_016_016,
    8_000_000,
    4_000_000,
    2_000_000,
    1_000_000,
    500_000,
    250_000,
    125_000,
];

impl Calibration {
    const ZERO: Self = Self {
        t1: 0,
        t2: 0,
        t3: 0,
        p1: 0,
        p2: 0,
        p3: 0,
        p4: 0,
        p5: 0,
        p6: 0,
        p7: 0,
        p8: 0,
        p9: 0,
        p10: 0,
        h1: 0,
        h2: 0,
        h3: 0,
        h4: 0,
        h5: 0,
        h6: 0,
        h7: 0,
        g1: 0,
        g2: 0,
        g3: 0,
        res_heat_range: 0,
        res_heat_val: 0,
        range_sw_err: 0,
    };

    /// Parse blocks 0x8A..=0xA0, 0xE1..=0xEE and 0x00..=0x04
    /// (datasheet tables 11-14).
    pub fn parse(c1: &[u8; LEN_CALIB1], c2: &[u8; LEN_CALIB2], c3: &[u8; LEN_CALIB3]) -> Self {
        let u = |b: &[u8], i: usize| u16::from_le_bytes([b[i], b[i + 1]]);
        let s = |b: &[u8], i: usize| i16::from_le_bytes([b[i], b[i + 1]]);
        Self {
            t1: u(c2, 8),
            t2: s(c1, 0),
            t3: c1[2] as i8,
            p1: u(c1, 4),
            p2: s(c1, 6),
            p3: c1[8] as i8,
            p4: s(c1, 10),
            p5: s(c1, 12),
            p7: c1[14] as i8,
            p6: c1[15] as i8,
            p8: s(c1, 18),
            p9: s(c1, 20),
            p10: c1[22],
            h1: (u16::from(c2[2]) << 4) | u16::from(c2[1] & 0x0F),
            h2: (u16::from(c2[0]) << 4) | u16::from(c2[1] >> 4),
            h3: c2[3] as i8,
            h4: c2[4] as i8,
            h5: c2[5] as i8,
            h6: c2[6],
            h7: c2[7] as i8,
            g2: s(c2, 10),
            g1: c2[12] as i8,
            g3: c2[13] as i8,
            res_heat_val: c3[0] as i8,
            res_heat_range: (c3[2] >> 4) & 0x03,
            range_sw_err: (c3[4] as i8) >> 4,
        }
    }

    /// Temperature (datasheet 3.3.1, integer). Returns `(t_fine, 0.01 °C)`.
    pub fn compensate_temperature(&self, adc_t: u32) -> (i32, i32) {
        let var1 = ((adc_t as i32) >> 3) - (i32::from(self.t1) << 1);
        let var2 = var1.wrapping_mul(i32::from(self.t2)) >> 11;
        let var3 = (((var1 >> 1) * (var1 >> 1)) >> 12).wrapping_mul(i32::from(self.t3) << 4) >> 14;
        let t_fine = var2.wrapping_add(var3);
        (t_fine, t_fine.wrapping_mul(5).wrapping_add(128) >> 8)
    }

    /// Pressure in Pa (datasheet 3.3.2, integer). The intermediate is
    /// handled as `u32` as intended by the formula; the cubic `par_p10` term
    /// is evaluated in 64 bits because it can exceed `i32` for large
    /// `par_p10`.
    pub fn compensate_pressure(&self, adc_p: u32, t_fine: i32) -> u32 {
        let mut var1 = (t_fine >> 1) - 64_000;
        let sq = (var1 >> 2).wrapping_mul(var1 >> 2);
        let mut var2 = ((sq >> 11).wrapping_mul(i32::from(self.p6))) >> 2;
        var2 = var2.wrapping_add(var1.wrapping_mul(i32::from(self.p5)) << 1);
        var2 = (var2 >> 2).wrapping_add(i32::from(self.p4) << 16);
        var1 = (((sq >> 13).wrapping_mul(i32::from(self.p3) << 5)) >> 3)
            .wrapping_add(i32::from(self.p2).wrapping_mul(var1) >> 1);
        var1 >>= 18;
        var1 = (32_768 + var1).wrapping_mul(i32::from(self.p1)) >> 15;
        if var1 <= 0 {
            return 0;
        }
        let div = var1 as u32;
        let mut p = 1_048_576u32
            .wrapping_sub(adc_p)
            .wrapping_sub((var2 >> 12) as u32)
            .wrapping_mul(3_125);
        p = if p >= 1 << 30 {
            (p / div) << 1
        } else {
            (p << 1) / div
        };
        let var1 =
            i32::from(self.p9).wrapping_mul(((p >> 3).wrapping_mul(p >> 3) >> 13) as i32) >> 12;
        let var2 = ((p >> 2) as i32).wrapping_mul(i32::from(self.p8)) >> 13;
        let c = i64::from(p >> 8);
        let var3 = ((c * c * c * i64::from(self.p10)) >> 17) as i32;
        (p as i32).wrapping_add(
            var1.wrapping_add(var2)
                .wrapping_add(var3)
                .wrapping_add(i32::from(self.p7) << 7)
                >> 4,
        ) as u32
    }

    /// Relative humidity in 0.01 %RH (datasheet 3.3.3, integer, which
    /// yields 0.001 %RH), clamped to 0..=10000. `temperature` in 0.01 °C.
    pub fn compensate_humidity(&self, adc_h: u16, temperature: i32) -> u16 {
        let t = temperature;
        let var1 =
            i32::from(adc_h) - (i32::from(self.h1) << 4) - ((t * i32::from(self.h3) / 100) >> 1);
        let var2 = (i32::from(self.h2)
            * (t * i32::from(self.h4) / 100
                + ((t * (t * i32::from(self.h5) / 100)) >> 6) / 100
                + (1 << 14)))
            >> 10;
        let var3 = var1.wrapping_mul(var2);
        let var4 = ((i32::from(self.h6) << 7) + t * i32::from(self.h7) / 100) >> 4;
        let var5 = ((var3 >> 14).wrapping_mul(var3 >> 14)) >> 10;
        let var6 = var4.wrapping_mul(var5) >> 1;
        let milli = (var3.wrapping_add(var6) >> 10).wrapping_mul(1_000) >> 12;
        ((milli.clamp(0, 100_000) + 5) / 10) as u16
    }

    /// `res_heat_x` for a heater set-point (datasheet 3.3.5, integer).
    /// `ambient` in °C.
    pub fn res_heat(&self, target: u16, ambient: i32) -> u8 {
        let target = i32::from(target.min(400));
        let var1 = ((ambient * i32::from(self.g3)) / 10) << 8;
        let var2 = (i32::from(self.g1) + 784)
            * ((((i32::from(self.g2) + 154_009) * target * 5) / 100 + 3_276_800) / 10);
        let var3 = var1 + (var2 >> 1);
        let var4 = var3 / (i32::from(self.res_heat_range) + 4);
        let var5 = 131 * i32::from(self.res_heat_val) + 65_536;
        let x100 = (var4 / var5 - 250) * 34;
        ((x100 + 50) / 100).clamp(0, 255) as u8
    }

    /// BME680 gas resistance in Ω (datasheet 3.4.1, integer variant).
    pub fn gas_resistance_bme680(&self, gas_adc: u16, gas_range: u8) -> u32 {
        let r = usize::from(gas_range & 0x0F);
        let var1 = ((1_340 + 5 * i64::from(self.range_sw_err)) * i64::from(GAS_K1[r])) >> 16;
        let var2 = (i64::from(gas_adc) << 15) - (1 << 24) + var1;
        if var2 <= 0 {
            return u32::MAX;
        }
        let res = (((i64::from(GAS_K2[r]) * var1) >> 9) + (var2 >> 1)) / var2;
        res.clamp(0, i64::from(u32::MAX)) as u32
    }
}

/// BME688 gas resistance in Ω (BME688 datasheet, gas resistance
/// read-out): `1e6 · (262144 >> range) / (4096 + 3 · (adc − 512))`,
/// evaluated exactly in 32 bits.
pub fn gas_resistance_bme688(gas_adc: u16, gas_range: u8) -> u32 {
    let var1 = 262_144u32 >> (gas_range & 0x0F);
    let var2 = (4_096 + 3 * (i32::from(gas_adc & 0x3FF) - 512)) as u32;
    let q = 10_000 * var1 / var2;
    let r = 10_000 * var1 % var2;
    q * 100 + r * 100 / var2
}

/// Gas-resistance result of one measurement.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Gas {
    /// Heater resistance in Ω.
    pub resistance: u32,
    /// `gas_valid_r`: a gas conversion completed.
    pub valid: bool,
    /// `heat_stab_r`: the heater reached its target temperature.
    pub heat_stable: bool,
}

/// One compensated sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Temperature in 0.01 °C.
    pub temperature: i32,
    /// Pressure in Pa (0 if skipped).
    pub pressure: u32,
    /// Relative humidity in 0.01 %RH (0 if skipped).
    pub humidity: u16,
    /// Gas result when the heater is enabled.
    pub gas: Option<Gas>,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Temperature(self.temperature));
        if self.pressure != 0 {
            f(Quantity::Pressure(self.pressure));
        }
        f(Quantity::Humidity(self.humidity));
        if let Some(gas) = self.gas
            && gas.valid
            && gas.heat_stable
        {
            f(Quantity::GasResistance(gas.resistance));
        }
    }
}

/// Decode field 0 (registers 0x1D..=0x2D). Returns [`Error::NotReady`]
/// unless `new_data_0` is set.
pub fn decode<E>(
    field: &[u8; LEN_FIELD],
    cal: &Calibration,
    variant: Variant,
    config: &Config,
) -> Result<Measurement, Error<E>> {
    if field[0] & NEW_DATA == 0 {
        return Err(Error::NotReady);
    }
    let p20 = |i: usize| {
        (u32::from(field[i]) << 12) | (u32::from(field[i + 1]) << 4) | u32::from(field[i + 2] >> 4)
    };
    let (t_fine, temperature) = cal.compensate_temperature(p20(5));
    let pressure = if config.pressure == Oversampling::Skip {
        0
    } else {
        cal.compensate_pressure(p20(2), t_fine)
    };
    let humidity = if config.humidity == Oversampling::Skip {
        0
    } else {
        cal.compensate_humidity(u16::from_be_bytes([field[8], field[9]]), temperature)
    };
    let gas = config.heater.map(|_| {
        let i = match variant {
            Variant::Bme680 => 13,
            Variant::Bme688 => 15,
        };
        let adc = (u16::from(field[i]) << 2) | u16::from(field[i + 1] >> 6);
        let range = field[i + 1] & 0x0F;
        Gas {
            resistance: match variant {
                Variant::Bme680 => cal.gas_resistance_bme680(adc, range),
                Variant::Bme688 => gas_resistance_bme688(adc, range),
            },
            valid: field[i + 1] & GAS_VALID != 0,
            heat_stable: field[i + 1] & HEAT_STAB != 0,
        }
    });
    Ok(Measurement {
        temperature,
        pressure,
        humidity,
        gas,
    })
}

/// Device state shared by both drivers.
#[derive(Debug, Clone, Copy)]
struct State {
    variant: Option<Variant>,
    calibration: Calibration,
    config: Config,
    /// Ambient temperature (°C) used for the heater set-point.
    ambient: i32,
}

impl State {
    const fn new() -> Self {
        Self {
            variant: None,
            calibration: Calibration::ZERO,
            config: Config::DEFAULT,
            ambient: 25,
        }
    }

    /// Registers written by `set_config`, in order (`ctrl_hum` before
    /// `ctrl_meas`).
    fn config_writes(&self, variant: Variant) -> [(u8, u8); 6] {
        let c = &self.config;
        let heater = c.heater.unwrap_or(Heater {
            temperature: 0,
            duration_ms: 0,
        });
        [
            (REG_CTRL_MEAS, c.ctrl_meas(false)),
            (REG_CTRL_HUM, c.humidity as u8),
            (REG_CONFIG, (c.filter as u8) << 2),
            (REG_GAS_WAIT_0, heater.gas_wait()),
            (REG_RES_HEAT_0, self.res_heat()),
            (REG_CTRL_GAS_1, c.ctrl_gas_1(variant)),
        ]
    }

    fn res_heat(&self) -> u8 {
        self.config.heater.map_or(0, |h| {
            self.calibration.res_heat(h.temperature, self.ambient)
        })
    }
}

macro_rules! constructors {
    ($name:ident) => {
        impl<IFACE> $name<IFACE> {
            /// Create a driver over any transport; no bus traffic.
            pub const fn new(iface: IFACE) -> Self {
                Self {
                    iface,
                    state: State::new(),
                }
            }

            /// Select the configuration written by `init` / `set_config`.
            pub const fn with_config(mut self, config: Config) -> Self {
                self.state.config = config;
                self
            }

            /// Variant detected by `init`.
            pub const fn variant(&self) -> Option<Variant> {
                self.state.variant
            }

            /// Calibration read by `init`.
            pub const fn calibration(&self) -> &Calibration {
                &self.state.calibration
            }

            /// Give the transport back.
            pub fn release(self) -> IFACE {
                self.iface
            }
        }

        impl<I2C> $name<I2cInterface<I2C>> {
            /// Driver on an I2C bus.
            pub const fn new_i2c(i2c: I2C, address: u8) -> Self {
                Self::new(I2cInterface { i2c, address })
            }
        }

        impl<SPI> $name<SpiInterface<SPI>> {
            /// Driver on an SPI device.
            pub const fn new_spi(spi: SPI) -> Self {
                Self::new(SpiInterface::new(spi))
            }
        }
    };
}

/// Blocking driver.
#[derive(Debug)]
pub struct Bme680<IFACE> {
    iface: IFACE,
    state: State,
}

constructors!(Bme680);

impl<IFACE: Interface> Bme680<IFACE> {
    fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<IFACE::Error>> {
        self.iface.read_regs(reg, buf).map_err(Error::bus)
    }

    fn write(&mut self, reg: u8, value: u8) -> Result<(), Error<IFACE::Error>> {
        self.iface.write_reg(reg, value).map_err(Error::bus)
    }

    /// Check `chip_id` and read `variant_id`.
    pub fn probe(&mut self) -> Result<Variant, Error<IFACE::Error>> {
        let mut id = [0];
        self.read(REG_CHIP_ID, &mut id)?;
        if id[0] != CHIP_ID {
            return Err(Error::UnexpectedId(id[0].into()));
        }
        self.read(REG_VARIANT_ID, &mut id)?;
        Ok(if id[0] == 0x01 {
            Variant::Bme688
        } else {
            Variant::Bme680
        })
    }

    /// Soft reset. Wait [`RESET_WAIT_US`], then call [`Self::init`].
    pub fn soft_reset(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        self.write(REG_RESET, CMD_SOFT_RESET)?;
        self.state.variant = None;
        Ok(RESET_WAIT_US)
    }

    /// Identify the part, read the calibration and write the
    /// configuration. Called by `start_measurement` if needed.
    pub fn init(&mut self) -> Result<Variant, Error<IFACE::Error>> {
        let variant = self.probe()?;
        let mut c1 = [0; LEN_CALIB1];
        let mut c2 = [0; LEN_CALIB2];
        let mut c3 = [0; LEN_CALIB3];
        self.read(REG_CALIB1, &mut c1)?;
        self.read(REG_CALIB2, &mut c2)?;
        self.read(REG_CALIB3, &mut c3)?;
        self.state.calibration = Calibration::parse(&c1, &c2, &c3);
        self.state.variant = Some(variant);
        self.set_config(self.state.config)?;
        Ok(variant)
    }

    /// Write a new configuration (device left in sleep mode).
    pub fn set_config(&mut self, config: Config) -> Result<(), Error<IFACE::Error>> {
        self.state.config = config;
        if let Some(variant) = self.state.variant {
            for (reg, value) in self.state.config_writes(variant) {
                self.write(reg, value)?;
            }
        }
        Ok(())
    }
}

impl<IFACE: Interface> zigbee_sensor::blocking::Sensor for Bme680<IFACE> {
    type BusError = IFACE::Error;
    type Measurement = Measurement;

    /// Updates `res_heat_0` for the last measured ambient temperature, then
    /// triggers forced mode.
    fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        if self.state.variant.is_none() {
            self.init()?;
        } else if self.state.config.heater.is_some() {
            self.write(REG_RES_HEAT_0, self.state.res_heat())?;
        }
        self.write(REG_CTRL_MEAS, self.state.config.ctrl_meas(true))?;
        Ok(self.state.config.max_conversion_us())
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<IFACE::Error>> {
        let variant = self.state.variant.ok_or(Error::NotReady)?;
        let mut field = [0; LEN_FIELD];
        self.read(REG_FIELD0, &mut field)?;
        let m = decode(&field, &self.state.calibration, variant, &self.state.config)?;
        self.state.ambient = m.temperature / 100;
        Ok(m)
    }
}

impl<IFACE: Interface> zigbee_sensor::blocking::LowPower for Bme680<IFACE> {
    type BusError = IFACE::Error;

    /// Forced mode returns to sleep by itself; this forces `mode = 00`.
    fn sleep(&mut self) -> Result<(), Error<IFACE::Error>> {
        self.write(REG_CTRL_MEAS, self.state.config.ctrl_meas(false))
    }

    fn wake(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
        Ok(0)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        CHIP_ID, CMD_SOFT_RESET, Calibration, Config, I2cInterface, LEN_CALIB1, LEN_CALIB2,
        LEN_CALIB3, LEN_FIELD, Measurement, REG_CALIB1, REG_CALIB2, REG_CALIB3, REG_CHIP_ID,
        REG_CTRL_MEAS, REG_FIELD0, REG_RES_HEAT_0, REG_RESET, REG_SPI_PAGE, REG_VARIANT_ID,
        RESET_WAIT_US, SpiInterface, State, Variant, decode, page_value,
    };
    use zigbee_sensor::{Error, WaitUs};

    /// Async register access.
    #[allow(async_fn_in_trait)]
    pub trait Interface {
        /// Transport error.
        type Error;
        /// Burst-read consecutive registers starting at `reg`.
        async fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error>;
        /// Write one register.
        async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error>;
    }

    impl<I2C: embedded_hal_async::i2c::I2c> Interface for I2cInterface<I2C> {
        type Error = I2C::Error;

        async fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
            self.i2c.write_read(self.address, &[reg], buf).await
        }

        async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
            self.i2c.write(self.address, &[reg, value]).await
        }
    }

    impl<SPI: embedded_hal_async::spi::SpiDevice> SpiInterface<SPI> {
        async fn select_async(&mut self, reg: u8) -> Result<(), SPI::Error> {
            use embedded_hal_async::spi::Operation;
            if let Some(page1) = self.page_switch(reg) {
                let mut status = [0];
                self.spi
                    .transaction(&mut [
                        Operation::Write(&[REG_SPI_PAGE | 0x80]),
                        Operation::Read(&mut status),
                    ])
                    .await?;
                self.spi
                    .write(&[REG_SPI_PAGE, page_value(status[0], page1)])
                    .await?;
                self.page = Some(page1);
            }
            Ok(())
        }
    }

    impl<SPI: embedded_hal_async::spi::SpiDevice> Interface for SpiInterface<SPI> {
        type Error = SPI::Error;

        async fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
            use embedded_hal_async::spi::Operation;
            self.select_async(reg).await?;
            self.spi
                .transaction(&mut [Operation::Write(&[reg | 0x80]), Operation::Read(buf)])
                .await
        }

        async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
            self.select_async(reg).await?;
            self.spi.write(&[reg & 0x7F, value]).await?;
            self.after_write(reg, value);
            Ok(())
        }
    }

    /// Async twin of [`super::Bme680`].
    #[derive(Debug)]
    pub struct Bme680<IFACE> {
        iface: IFACE,
        state: State,
    }

    constructors!(Bme680);

    impl<IFACE: Interface> Bme680<IFACE> {
        async fn read(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<IFACE::Error>> {
            self.iface.read_regs(reg, buf).await.map_err(Error::bus)
        }

        async fn write(&mut self, reg: u8, value: u8) -> Result<(), Error<IFACE::Error>> {
            self.iface.write_reg(reg, value).await.map_err(Error::bus)
        }

        /// See [`super::Bme680::probe`].
        pub async fn probe(&mut self) -> Result<Variant, Error<IFACE::Error>> {
            let mut id = [0];
            self.read(REG_CHIP_ID, &mut id).await?;
            if id[0] != CHIP_ID {
                return Err(Error::UnexpectedId(id[0].into()));
            }
            self.read(REG_VARIANT_ID, &mut id).await?;
            Ok(if id[0] == 0x01 {
                Variant::Bme688
            } else {
                Variant::Bme680
            })
        }

        /// See [`super::Bme680::soft_reset`].
        pub async fn soft_reset(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
            self.write(REG_RESET, CMD_SOFT_RESET).await?;
            self.state.variant = None;
            Ok(RESET_WAIT_US)
        }

        /// See [`super::Bme680::init`].
        pub async fn init(&mut self) -> Result<Variant, Error<IFACE::Error>> {
            let variant = self.probe().await?;
            let mut c1 = [0; LEN_CALIB1];
            let mut c2 = [0; LEN_CALIB2];
            let mut c3 = [0; LEN_CALIB3];
            self.read(REG_CALIB1, &mut c1).await?;
            self.read(REG_CALIB2, &mut c2).await?;
            self.read(REG_CALIB3, &mut c3).await?;
            self.state.calibration = Calibration::parse(&c1, &c2, &c3);
            self.state.variant = Some(variant);
            self.set_config(self.state.config).await?;
            Ok(variant)
        }

        /// See [`super::Bme680::set_config`].
        pub async fn set_config(&mut self, config: Config) -> Result<(), Error<IFACE::Error>> {
            self.state.config = config;
            if let Some(variant) = self.state.variant {
                for (reg, value) in self.state.config_writes(variant) {
                    self.write(reg, value).await?;
                }
            }
            Ok(())
        }
    }

    impl<IFACE: Interface> zigbee_sensor::asynch::Sensor for Bme680<IFACE> {
        type BusError = IFACE::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
            if self.state.variant.is_none() {
                self.init().await?;
            } else if self.state.config.heater.is_some() {
                self.write(REG_RES_HEAT_0, self.state.res_heat()).await?;
            }
            self.write(REG_CTRL_MEAS, self.state.config.ctrl_meas(true))
                .await?;
            Ok(self.state.config.max_conversion_us())
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<IFACE::Error>> {
            let variant = self.state.variant.ok_or(Error::NotReady)?;
            let mut field = [0; LEN_FIELD];
            self.read(REG_FIELD0, &mut field).await?;
            let m = decode(&field, &self.state.calibration, variant, &self.state.config)?;
            self.state.ambient = m.temperature / 100;
            Ok(m)
        }
    }

    impl<IFACE: Interface> zigbee_sensor::asynch::LowPower for Bme680<IFACE> {
        type BusError = IFACE::Error;

        async fn sleep(&mut self) -> Result<(), Error<IFACE::Error>> {
            self.write(REG_CTRL_MEAS, self.state.config.ctrl_meas(false))
                .await
        }

        async fn wake(&mut self) -> Result<WaitUs, Error<IFACE::Error>> {
            Ok(0)
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTx};
    use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiTx};
    use std::vec;
    use std::vec::Vec;
    use zigbee_sensor::blocking::Sensor;

    /// A real BME680 calibration dump.
    fn cal_blocks() -> ([u8; LEN_CALIB1], [u8; LEN_CALIB2], [u8; LEN_CALIB3]) {
        let mut c1 = [0u8; LEN_CALIB1];
        let mut c2 = [0u8; LEN_CALIB2];
        let mut c3 = [0u8; LEN_CALIB3];
        let put = |b: &mut [u8], i: usize, v: i32| {
            b[i..i + 2].copy_from_slice(&(v as u16).to_le_bytes());
        };
        put(&mut c1, 0, 26_234); // t2
        c1[2] = 3; // t3
        put(&mut c1, 4, 36_217); // p1
        put(&mut c1, 6, -10_405); // p2
        c1[8] = 88; // p3
        put(&mut c1, 10, 6_763); // p4
        put(&mut c1, 12, -111); // p5
        c1[14] = 30; // p7
        c1[15] = 30; // p6
        put(&mut c1, 18, -6_485); // p8
        put(&mut c1, 20, -2_647); // p9
        c1[22] = 30; // p10
        // h2 = 1008 (0x3F0), h1 = 738 (0x2E2)
        c2[0] = 0x3F;
        c2[1] = 0x02;
        c2[2] = 0x2E;
        c2[3] = 0; // h3
        c2[4] = 45; // h4
        c2[5] = 20; // h5
        c2[6] = 120; // h6
        c2[7] = (-100i8) as u8; // h7
        put(&mut c2, 8, 26_559); // t1
        put(&mut c2, 10, -5_969); // g2
        c2[12] = (-30i8) as u8; // g1
        c2[13] = 18; // g3
        c3[0] = 46; // res_heat_val
        c3[2] = 0x10; // res_heat_range = 1
        c3[4] = 0xF0; // range_sw_err = -1
        (c1, c2, c3)
    }

    fn cal() -> Calibration {
        let (c1, c2, c3) = cal_blocks();
        Calibration::parse(&c1, &c2, &c3)
    }

    #[test]
    fn calibration_parsing() {
        let c = cal();
        assert_eq!((c.t1, c.t2, c.t3), (26_559, 26_234, 3));
        assert_eq!(
            (c.p1, c.p2, c.p6, c.p7, c.p10),
            (36_217, -10_405, 30, 30, 30)
        );
        assert_eq!((c.h1, c.h2, c.h7), (738, 1008, -100));
        assert_eq!((c.g1, c.g2, c.g3), (-30, -5_969, 18));
        assert_eq!(
            (c.res_heat_val, c.res_heat_range, c.range_sw_err),
            (46, 1, -1)
        );
    }

    // Floating-point reference formulas of the datasheet (BME680 3.3,
    // 3.4) to check the integer versions.
    #[allow(clippy::float_arithmetic)]
    mod reference {
        use super::Calibration;

        pub fn temperature(c: &Calibration, adc: u32) -> (f64, f64) {
            let adc = f64::from(adc);
            let var1 = (adc / 16_384.0 - f64::from(c.t1) / 1_024.0) * f64::from(c.t2);
            let d = adc / 131_072.0 - f64::from(c.t1) / 8_192.0;
            let var2 = d * d * f64::from(c.t3) * 16.0;
            let t_fine = var1 + var2;
            (t_fine, t_fine / 5_120.0)
        }

        pub fn pressure(c: &Calibration, adc: u32, t_fine: f64) -> f64 {
            let mut var1 = t_fine / 2.0 - 64_000.0;
            let mut var2 = var1 * var1 * (f64::from(c.p6) / 131_072.0);
            var2 += var1 * f64::from(c.p5) * 2.0;
            var2 = var2 / 4.0 + f64::from(c.p4) * 65_536.0;
            var1 = (f64::from(c.p3) * var1 * var1 / 16_384.0 + f64::from(c.p2) * var1) / 524_288.0;
            var1 = (1.0 + var1 / 32_768.0) * f64::from(c.p1);
            let mut p = 1_048_576.0 - f64::from(adc);
            p = (p - var2 / 4_096.0) * 6_250.0 / var1;
            let var1 = f64::from(c.p9) * p * p / 2_147_483_648.0;
            let var2 = p * (f64::from(c.p8) / 32_768.0);
            let var3 = (p / 256.0).powi(3) * (f64::from(c.p10) / 131_072.0);
            p + (var1 + var2 + var3 + f64::from(c.p7) * 128.0) / 16.0
        }

        pub fn humidity(c: &Calibration, adc: u16, t: f64) -> f64 {
            let var1 = f64::from(adc) - (f64::from(c.h1) * 16.0 + f64::from(c.h3) / 2.0 * t);
            let var2 = var1
                * (f64::from(c.h2) / 262_144.0
                    * (1.0
                        + f64::from(c.h4) / 16_384.0 * t
                        + f64::from(c.h5) / 1_048_576.0 * t * t));
            let var3 = f64::from(c.h6) / 16_384.0;
            let var4 = f64::from(c.h7) / 2_097_152.0;
            var2 + (var3 + var4 * t) * var2 * var2
        }

        pub fn res_heat(c: &Calibration, target: f64, amb: f64) -> f64 {
            let var1 = f64::from(c.g1) / 16.0 + 49.0;
            let var2 = f64::from(c.g2) / 32_768.0 * 0.0005 + 0.00235;
            let var3 = f64::from(c.g3) / 1_024.0;
            let var4 = var1 * (1.0 + var2 * target);
            let var5 = var4 + var3 * amb;
            3.4 * (var5
                * (4.0 / (4.0 + f64::from(c.res_heat_range)))
                * (1.0 / (1.0 + f64::from(c.res_heat_val) * 0.002))
                - 25.0)
        }

        pub fn gas_bme680(c: &Calibration, adc: u16, range: usize) -> f64 {
            const K1: [f64; 16] = [
                1.0, 1.0, 1.0, 1.0, 1.0, 0.99, 1.0, 0.992, 1.0, 1.0, 0.998, 0.995, 1.0, 0.99, 1.0,
                1.0,
            ];
            const K2: [f64; 16] = [
                8_000_000.0,
                4_000_000.0,
                2_000_000.0,
                1_000_000.0,
                499_500.499_5,
                248_262.164_8,
                125_000.0,
                63_004.032_26,
                31_281.281_28,
                15_625.0,
                7_812.5,
                3_906.25,
                1_953.125,
                976.5625,
                488.281_25,
                244.140_625,
            ];
            let var1 = (1_340.0 + 5.0 * f64::from(c.range_sw_err)) * K1[range];
            let k2 = K2[range];
            var1 * k2 / (f64::from(adc) - 512.0 + var1)
        }
    }

    #[test]
    #[allow(clippy::float_arithmetic)]
    fn tph_match_float_reference() {
        let c = cal();
        for adc_t in [450_000u32, 501_700, 550_000] {
            let (t_fine, t) = c.compensate_temperature(adc_t);
            let (rf_fine, rt) = reference::temperature(&c, adc_t);
            assert!((f64::from(t) - rt * 100.0).abs() <= 1.0, "T {t} vs {rt}");
            for adc_p in [300_000u32, 400_000, 500_000] {
                let p = c.compensate_pressure(adc_p, t_fine);
                let rp = reference::pressure(&c, adc_p, rf_fine);
                // The datasheet integer formula is coarser than the float
                // one: up to ~8 Pa apart, below the 12 Pa relative accuracy.
                assert!((f64::from(p) - rp).abs() <= 8.0, "P {p} vs {rp}");
            }
            for adc_h in [15_000u16, 20_000, 25_000] {
                let h = c.compensate_humidity(adc_h, t);
                let rh = reference::humidity(&c, adc_h, rt).clamp(0.0, 100.0);
                assert!((f64::from(h) - rh * 100.0).abs() <= 3.0, "H {h} vs {rh}");
            }
        }
        assert_eq!(c.compensate_humidity(0, 2_400), 0);
        assert_eq!(c.compensate_humidity(u16::MAX, 2_400), 10_000);
    }

    #[test]
    #[allow(clippy::float_arithmetic)]
    fn heater_and_gas_match_float_reference() {
        let c = cal();
        for (target, amb) in [(200u16, 25i32), (320, 25), (400, 10)] {
            let r = i32::from(c.res_heat(target, amb));
            let rf = reference::res_heat(&c, f64::from(target), f64::from(amb));
            // Integer and float heater formulas round differently (±2 LSB).
            assert!((f64::from(r) - rf).abs() <= 2.0, "{target}: {r} vs {rf}");
        }
        for range in 0..16u8 {
            for adc in [100u16, 512, 900] {
                let g = c.gas_resistance_bme680(adc, range);
                let gf = reference::gas_bme680(&c, adc, usize::from(range));
                assert!(
                    (f64::from(g) - gf).abs() <= gf * 0.001 + 1.0,
                    "{range}/{adc}: {g} vs {gf}"
                );
            }
        }
    }

    #[test]
    fn compensation_is_panic_free() {
        let c = cal();
        for adc_t in (0..=0xF_FFFF).step_by(40_009) {
            let (t_fine, t) = c.compensate_temperature(adc_t);
            for adc_p in (0..=0xF_FFFF).step_by(50_021) {
                let _ = c.compensate_pressure(adc_p, t_fine);
            }
            for adc_h in (0..=0xFFFF).step_by(4_001) {
                let _ = c.compensate_humidity(adc_h, t.clamp(-4_000, 8_500));
            }
        }
        for adc in [0, 1023] {
            for range in 0..16 {
                let _ = c.gas_resistance_bme680(adc, range);
                let _ = gas_resistance_bme688(adc, range);
            }
        }
    }

    #[test]
    fn bme688_gas_formula() {
        // adc 512, range 0: 1e6 × 262144 / 4096 = 64 MΩ.
        assert_eq!(gas_resistance_bme688(512, 0), 64_000_000);
        // adc 1023, range 4: 1e6 × 16384 / 5629 = 2 910 641 Ω.
        assert_eq!(gas_resistance_bme688(1023, 4), 2_910_641);
        assert_eq!(gas_resistance_bme688(0, 15), 3_125);
    }

    #[test]
    fn heater_encoding_and_wait() {
        let h = |ms| Heater {
            temperature: 300,
            duration_ms: ms,
        };
        assert_eq!(h(100).gas_wait(), 0x59); // 25 × 4
        assert_eq!(h(63).gas_wait(), 63);
        assert_eq!(h(4_032).gas_wait(), 0xFF);
        assert_eq!(h(150).programmed_ms(), 148);
        // T ×2 + P ×16 + H ×1 = 19 cycles: 19 × 1963 + 4293 + 1000 + 100 ms.
        assert_eq!(
            Config::DEFAULT.max_conversion_us(),
            19 * 1_963 + 5_293 + 100_000
        );
        let tph = Config {
            heater: None,
            ..Config::DEFAULT
        };
        assert_eq!(tph.max_conversion_us(), 19 * 1_963 + 5_293);
        assert_eq!(Config::DEFAULT.ctrl_meas(true), 0x55);
        assert_eq!(Config::DEFAULT.ctrl_gas_1(Variant::Bme680), 0x10);
        assert_eq!(Config::DEFAULT.ctrl_gas_1(Variant::Bme688), 0x20);
    }

    fn field(variant: Variant, new_data: bool) -> [u8; LEN_FIELD] {
        let mut f = [0u8; LEN_FIELD];
        f[0] = if new_data { 0x80 } else { 0x20 };
        // adc_P = 400000 = 0x61A80, adc_T = 501700 = 0x7A7C4, adc_H = 20000.
        f[2..5].copy_from_slice(&[0x61, 0xA8, 0x00]);
        f[5..8].copy_from_slice(&[0x7A, 0x7C, 0x40]);
        f[8..10].copy_from_slice(&20_000u16.to_be_bytes());
        // gas adc 512, range 4, valid + stable.
        let i = if variant == Variant::Bme680 { 13 } else { 15 };
        f[i] = 0x80;
        f[i + 1] = 0x34;
        f
    }

    #[test]
    fn decode_and_not_ready() {
        let c = cal();
        let cfg = Config::DEFAULT;
        assert_eq!(
            decode::<()>(&field(Variant::Bme680, false), &c, Variant::Bme680, &cfg),
            Err(Error::NotReady)
        );
        let m = decode::<()>(&field(Variant::Bme688, true), &c, Variant::Bme688, &cfg).unwrap();
        let (t_fine, t) = c.compensate_temperature(501_700);
        assert_eq!(m.temperature, t);
        assert_eq!(m.pressure, c.compensate_pressure(400_000, t_fine));
        assert_eq!(
            m.gas,
            Some(Gas {
                resistance: 4_000_000,
                valid: true,
                heat_stable: true
            })
        );
        let mut n = 0;
        m.for_each(&mut |_| n += 1);
        assert_eq!(n, 4);
    }

    fn init_tx(a: u8, variant: u8) -> Vec<I2cTx> {
        let (c1, c2, c3) = cal_blocks();
        let c = Calibration::parse(&c1, &c2, &c3);
        vec![
            I2cTx::write_read(a, vec![REG_CHIP_ID], vec![CHIP_ID]),
            I2cTx::write_read(a, vec![REG_VARIANT_ID], vec![variant]),
            I2cTx::write_read(a, vec![REG_CALIB1], c1.to_vec()),
            I2cTx::write_read(a, vec![REG_CALIB2], c2.to_vec()),
            I2cTx::write_read(a, vec![REG_CALIB3], c3.to_vec()),
            I2cTx::write(a, vec![REG_CTRL_MEAS, 0x54]),
            I2cTx::write(a, vec![REG_CTRL_HUM, 0x01]),
            I2cTx::write(a, vec![REG_CONFIG, 0x00]),
            I2cTx::write(a, vec![REG_GAS_WAIT_0, 0x59]),
            I2cTx::write(a, vec![REG_RES_HEAT_0, c.res_heat(300, 25)]),
            I2cTx::write(
                a,
                vec![REG_CTRL_GAS_1, if variant == 1 { 0x20 } else { 0x10 }],
            ),
        ]
    }

    #[test]
    fn i2c_forced_measurement_with_gas() {
        let a = SECONDARY_ADDRESS;
        let c = cal();
        let mut tx = init_tx(a, 0);
        tx.extend([
            I2cTx::write(a, vec![REG_CTRL_MEAS, 0x55]),
            I2cTx::write_read(a, vec![REG_FIELD0], field(Variant::Bme680, true).to_vec()),
            // Second sample: res_heat_0 recomputed for the measured ambient.
            I2cTx::write(a, vec![REG_RES_HEAT_0, c.res_heat(300, 24)]),
            I2cTx::write(a, vec![REG_CTRL_MEAS, 0x55]),
        ]);
        let mut i2c = I2cMock::new(&tx);
        let mut bme = Bme680::new_i2c(&mut i2c, a);
        assert_eq!(
            bme.start_measurement().unwrap(),
            Config::DEFAULT.max_conversion_us()
        );
        assert_eq!(bme.variant(), Some(Variant::Bme680));
        let m = bme.read_measurement().unwrap();
        assert_eq!(m.temperature / 100, 24);
        assert!(m.gas.unwrap().valid);
        bme.start_measurement().unwrap();
        i2c.done();
    }

    #[test]
    fn wrong_chip_id() {
        let tx = [I2cTx::write_read(
            PRIMARY_ADDRESS,
            vec![REG_CHIP_ID],
            vec![0x60],
        )];
        let mut i2c = I2cMock::new(&tx);
        let mut bme = Bme680::new_i2c(&mut i2c, PRIMARY_ADDRESS);
        assert_eq!(bme.probe(), Err(Error::UnexpectedId(0x60)));
        i2c.done();
    }

    #[test]
    fn spi_page_switching() {
        let tx = [
            // chip_id 0xD0 is in page 0: select page 0 first.
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![0xF3]),
            SpiTx::read_vec(vec![0x10]),
            SpiTx::transaction_end(),
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![0x73, 0x00]),
            SpiTx::transaction_end(),
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![0xD0]),
            SpiTx::read_vec(vec![CHIP_ID]),
            SpiTx::transaction_end(),
            // variant_id 0xF0: same page, no switch.
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![0xF0]),
            SpiTx::read_vec(vec![0x01]),
            SpiTx::transaction_end(),
            // ctrl_meas 0x74 is in page 1.
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![0xF3]),
            SpiTx::read_vec(vec![0x00]),
            SpiTx::transaction_end(),
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![0x73, 0x10]),
            SpiTx::transaction_end(),
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![0x74, 0x54]),
            SpiTx::transaction_end(),
        ];
        let mut spi = SpiMock::new(&tx);
        let mut bme = Bme680::new_spi(&mut spi);
        assert_eq!(bme.probe(), Ok(Variant::Bme688));
        zigbee_sensor::blocking::LowPower::sleep(&mut bme).unwrap();
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

        let a = PRIMARY_ADDRESS;
        let mut tx = init_tx(a, 1);
        tx.extend([
            I2cTx::write(a, vec![REG_CTRL_MEAS, 0x55]),
            I2cTx::write_read(a, vec![REG_FIELD0], field(Variant::Bme688, true).to_vec()),
        ]);
        let mut i2c = I2cMock::new(&tx);
        let mut bme = asynch::Bme680::new_i2c(&mut i2c, a);
        let mut delay = RecordingDelay(0);
        let m = block_on(bme.measure(&mut delay)).unwrap();
        assert_eq!(m.gas.unwrap().resistance, 4_000_000);
        assert_eq!(delay.0, Config::DEFAULT.max_conversion_us());
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
