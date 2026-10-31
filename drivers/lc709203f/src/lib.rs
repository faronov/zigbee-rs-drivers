//! onsemi LC709203F 1-cell Li-ion fuel gauge (HG-CVR, no sense resistor),
//! I2C address 0x0B.
//!
//! Every read and write carries a CRC-8-ATM (polynomial 0x07, init 0)
//! computed over the address bytes, command and data. Reads with a wrong
//! CRC return `Error::Crc`.
//!
//! The gauge tracks RSOC by itself (3 µA in operational mode), so
//! `start_measurement` returns 0 and `read_measurement` reads cell voltage,
//! ITE (0.1 %) and, in thermistor mode, the cell temperature.
//!
//! Start-up (data sheet Figures 20/21), split around the 90 ms
//! re-initialisation that a profile change triggers:
//!
//! ```text
//! wait(gauge.wake()?);          // only if it may be in sleep (XE parts after POR)
//! wait(gauge.start_init()?);    // operational mode, profile (90 ms if changed)
//! gauge.finish_init()?;         // APA, temperature mode / B, optional Initial RSOC
//! ```
//!
//! Reference: onsemi LC709203F/D Rev. 3 (2016).

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs, crc::crc8_msb};

/// Fixed I2C address (0x16/0x17 8-bit).
pub const ADDRESS: u8 = 0x0B;

/// Register (command) codes, 16-bit LSB first.
pub mod reg {
    /// Before RSOC (write 0xAA55).
    pub const BEFORE_RSOC: u8 = 0x04;
    /// Thermistor B constant (K).
    pub const THERMISTOR_B: u8 = 0x06;
    /// Initial RSOC (write 0xAA55).
    pub const INITIAL_RSOC: u8 = 0x07;
    /// Cell temperature, 0.1 K (written by the host in I2C mode).
    pub const CELL_TEMPERATURE: u8 = 0x08;
    /// Cell voltage, mV.
    pub const CELL_VOLTAGE: u8 = 0x09;
    /// Current direction (auto / charge / discharge).
    pub const CURRENT_DIRECTION: u8 = 0x0A;
    /// Adjustment Pack Application.
    pub const APA: u8 = 0x0B;
    /// Adjustment Pack Thermistor.
    pub const APT: u8 = 0x0C;
    /// RSOC, 1 %.
    pub const RSOC: u8 = 0x0D;
    /// Indicator to Empty, 0.1 %.
    pub const ITE: u8 = 0x0F;
    /// IC version.
    pub const IC_VERSION: u8 = 0x11;
    /// Change of the parameter (battery profile select).
    pub const CHANGE_OF_PARAMETER: u8 = 0x12;
    /// Alarm low RSOC, 1 % (0 = off).
    pub const ALARM_LOW_RSOC: u8 = 0x13;
    /// Alarm low cell voltage, mV (0 = off).
    pub const ALARM_LOW_VOLTAGE: u8 = 0x14;
    /// IC power mode.
    pub const IC_POWER_MODE: u8 = 0x15;
    /// Status bit (temperature source).
    pub const STATUS_BIT: u8 = 0x16;
    /// Number of the parameter (profile data file code).
    pub const NUMBER_OF_PARAMETER: u8 = 0x1A;
}

/// RSOC initialisation key for 0x04 / 0x07.
pub const INIT_RSOC_KEY: u16 = 0xAA55;
/// IC power mode: operational.
pub const POWER_OPERATIONAL: u16 = 0x0001;
/// IC power mode: sleep.
pub const POWER_SLEEP: u16 = 0x0002;
/// Re-initialisation time after a profile change (TINIT max).
pub const INIT_US: WaitUs = 90_000;
/// Wake-up time from sleep after the SDA falling edge (TWU max).
pub const WAKE_US: WaitUs = 400;

/// Current direction (RSOC reporting mode).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CurrentDirection {
    /// RSOC follows the battery (recommended).
    Auto = 0x0000,
    /// RSOC may only rise.
    Charge = 0x0001,
    /// RSOC may only fall.
    Discharge = 0xFFFF,
}

/// Start-up configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Adjustment Pack Application (data sheet Table 7, see
    /// [`typical_apa`]).
    pub apa: u8,
    /// Battery profile for register 0x12 (0 or 1, data sheet Table 8).
    pub profile: u16,
    /// Thermistor B constant in K for thermistor mode; `None` = I2C
    /// (host temperature) mode.
    pub thermistor_b: Option<u16>,
    /// Write Initial RSOC (0x07 = 0xAA55) in `finish_init`. Only useful
    /// right after a profile change or battery insertion at low load.
    pub initial_rsoc: bool,
}

impl Config {
    /// 500 mAh type-01 APA (0x10), profile 1 (3.7 V / 4.2 V cell on the
    /// -01/-03 parts), I2C temperature mode, no RSOC re-init.
    pub const fn new() -> Self {
        Self {
            apa: 0x10,
            profile: 1,
            thermistor_b: None,
            initial_rsoc: false,
        }
    }
}

impl Default for Config {
    fn default() -> Self {
        Self::new()
    }
}

/// Typical APA for a type-01/03 cell (data sheet Table 7), linearly
/// interpolated between 100 and 3000 mAh design capacity per cell.
pub const fn typical_apa(capacity_mah: u16) -> u8 {
    const T: [(u32, u32); 6] = [
        (100, 0x08),
        (200, 0x0B),
        (500, 0x10),
        (1_000, 0x19),
        (2_000, 0x2D),
        (3_000, 0x36),
    ];
    let c = capacity_mah as u32;
    if c <= T[0].0 {
        return T[0].1 as u8;
    }
    let mut i = 1;
    while i < T.len() {
        let (c1, a1) = T[i];
        if c <= c1 {
            let (c0, a0) = T[i - 1];
            return (a0 + ((a1 - a0) * (c - c0) + (c1 - c0) / 2) / (c1 - c0)) as u8;
        }
        i += 1;
    }
    T[T.len() - 1].1 as u8
}

/// CRC-8-ATM over `bytes`.
pub fn crc8_atm(bytes: &[u8]) -> u8 {
    crc8_msb(bytes, 0x07, 0x00)
}

/// Check a 3-byte read response (low, high, CRC) for `reg`.
pub fn decode_read(reg: u8, buf: [u8; 3]) -> Option<u16> {
    let crc = crc8_atm(&[ADDRESS << 1, reg, (ADDRESS << 1) | 1, buf[0], buf[1]]);
    (crc == buf[2]).then(|| u16::from_le_bytes([buf[0], buf[1]]))
}

/// Write frame (command, low, high, CRC) for `reg`.
pub fn encode_write(reg: u8, value: u16) -> [u8; 4] {
    let [l, h] = value.to_le_bytes();
    [reg, l, h, crc8_atm(&[ADDRESS << 1, reg, l, h])]
}

/// Cell temperature (0.1 K, 0 °C = 0x0AAC) in 0.01 °C.
pub const fn temperature_centi(raw: u16) -> i32 {
    raw as i32 * 10 - 27_320
}

/// 0.01 °C to the 0x08 register value, clamped to −20…+60 °C.
pub const fn temperature_raw(centi: i32) -> u16 {
    let c = if centi < -2_000 {
        -2_000
    } else if centi > 6_000 {
        6_000
    } else {
        centi
    };
    ((c + 27_320 + 5) / 10) as u16
}

/// ITE (0.1 %) in 0.5 % units, clamped to 200.
pub const fn ite_half_percent(raw: u16) -> u8 {
    let h = (raw as u32 + 2) / 5;
    if h > 200 { 200 } else { h as u8 }
}

/// One reading.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Cell voltage in mV.
    pub voltage: i32,
    /// Remaining charge (ITE) in 0.5 % units.
    pub percent: u8,
    /// Cell temperature in 0.01 °C (thermistor mode only).
    pub temperature: Option<i32>,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Voltage(self.voltage));
        f(Quantity::BatteryPercent(self.percent));
        if let Some(t) = self.temperature {
            f(Quantity::Temperature(t));
        }
    }
}

macro_rules! constructors {
    ($name:ident) => {
        impl<I2C> $name<I2C> {
            /// Create a driver with [`Config::new`]; no bus traffic.
            pub const fn new(i2c: I2C) -> Self {
                Self {
                    i2c,
                    config: Config::new(),
                    wake_pending: false,
                }
            }

            /// Start-up configuration used by `start_init` / `finish_init`.
            pub const fn with_config(mut self, config: Config) -> Self {
                self.config = config;
                self
            }

            /// Configuration.
            pub const fn config(&self) -> &Config {
                &self.config
            }

            /// Give the bus back.
            pub fn release(self) -> I2C {
                self.i2c
            }
        }
    };
}

/// Blocking driver.
#[derive(Debug)]
pub struct Lc709203f<I2C> {
    i2c: I2C,
    config: Config,
    wake_pending: bool,
}

constructors!(Lc709203f);

impl<I2C: embedded_hal::i2c::I2c> Lc709203f<I2C> {
    /// Read a register and check its CRC.
    pub fn read_register(&mut self, reg: u8) -> Result<u16, Error<I2C::Error>> {
        let mut buf = [0; 3];
        self.i2c
            .write_read(ADDRESS, &[reg], &mut buf)
            .map_err(Error::bus)?;
        decode_read(reg, buf).ok_or(Error::Crc)
    }

    /// Write a register (with CRC).
    pub fn write_register(&mut self, reg: u8, value: u16) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(ADDRESS, &encode_write(reg, value))
            .map_err(Error::bus)
    }

    /// Read the profile code (0x1A); the CRC check is the identity test.
    pub fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
        match self.read_register(reg::NUMBER_OF_PARAMETER)? {
            id @ (0x0000 | 0xFFFF) => Err(Error::UnexpectedId(id as u32)),
            _ => Ok(()),
        }
    }

    /// Operational mode and battery profile. Returns [`INIT_US`] if the
    /// profile changed (the IC re-initialises), else 0.
    pub fn start_init(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.write_register(reg::IC_POWER_MODE, POWER_OPERATIONAL)?;
        if self.read_register(reg::CHANGE_OF_PARAMETER)? == self.config.profile {
            return Ok(0);
        }
        self.write_register(reg::CHANGE_OF_PARAMETER, self.config.profile)?;
        Ok(INIT_US)
    }

    /// Operational mode, APA, optional Initial RSOC, temperature mode and
    /// thermistor B.
    pub fn finish_init(&mut self) -> Result<(), Error<I2C::Error>> {
        let c = self.config;
        self.write_register(reg::IC_POWER_MODE, POWER_OPERATIONAL)?;
        self.write_register(reg::APA, c.apa as u16)?;
        if c.initial_rsoc {
            self.write_register(reg::INITIAL_RSOC, INIT_RSOC_KEY)?;
        }
        match c.thermistor_b {
            Some(b) => {
                self.write_register(reg::STATUS_BIT, 1)?;
                self.write_register(reg::THERMISTOR_B, b)
            }
            None => self.write_register(reg::STATUS_BIT, 0),
        }
    }

    /// RSOC in 1 %.
    pub fn rsoc(&mut self) -> Result<u16, Error<I2C::Error>> {
        self.read_register(reg::RSOC)
    }

    /// IC version.
    pub fn ic_version(&mut self) -> Result<u16, Error<I2C::Error>> {
        self.read_register(reg::IC_VERSION)
    }

    /// Re-initialise RSOC from the present voltage (≤ 1.5 ms).
    pub fn initial_rsoc(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_register(reg::INITIAL_RSOC, INIT_RSOC_KEY)
    }

    /// Host-supplied cell temperature in 0.01 °C (I2C mode; update when it
    /// changes by more than 1 °C).
    pub fn set_temperature(&mut self, centi: i32) -> Result<(), Error<I2C::Error>> {
        self.write_register(reg::CELL_TEMPERATURE, temperature_raw(centi))
    }

    /// RSOC reporting mode.
    pub fn set_current_direction(
        &mut self,
        direction: CurrentDirection,
    ) -> Result<(), Error<I2C::Error>> {
        self.write_register(reg::CURRENT_DIRECTION, direction as u16)
    }

    /// ALARMB thresholds: RSOC in % and voltage in mV (0 disables).
    pub fn set_alarms(
        &mut self,
        rsoc_percent: u8,
        voltage_mv: u16,
    ) -> Result<(), Error<I2C::Error>> {
        self.write_register(reg::ALARM_LOW_RSOC, rsoc_percent.min(100) as u16)?;
        self.write_register(reg::ALARM_LOW_VOLTAGE, voltage_mv)
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Lc709203f<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    /// Returns 0; after `wake` this sets operational mode.
    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        if self.wake_pending {
            self.write_register(reg::IC_POWER_MODE, POWER_OPERATIONAL)?;
            self.wake_pending = false;
        }
        Ok(0)
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        let voltage = self.read_register(reg::CELL_VOLTAGE)? as i32;
        let percent = ite_half_percent(self.read_register(reg::ITE)?);
        let temperature = match self.config.thermistor_b {
            Some(_) => Some(temperature_centi(
                self.read_register(reg::CELL_TEMPERATURE)?,
            )),
            None => None,
        };
        Ok(Measurement {
            voltage,
            percent,
            temperature,
        })
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::LowPower for Lc709203f<I2C> {
    type BusError = I2C::Error;

    /// Sleep mode (1 µA typ vs 3 µA): only I2C works and RSOC tracking
    /// stops; charge moved while asleep is not counted.
    fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_register(reg::IC_POWER_MODE, POWER_SLEEP)
    }

    /// Send an operational-mode write as the SDA wake pulse (its NACK is
    /// ignored) and return TWU. The next `start_measurement` writes
    /// operational mode again.
    fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        let _ = self.i2c.write(
            ADDRESS,
            &encode_write(reg::IC_POWER_MODE, POWER_OPERATIONAL),
        );
        self.wake_pending = true;
        Ok(WAKE_US)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        ADDRESS, Config, CurrentDirection, INIT_RSOC_KEY, INIT_US, Measurement, POWER_OPERATIONAL,
        POWER_SLEEP, WAKE_US, decode_read, encode_write, ite_half_percent, reg, temperature_centi,
        temperature_raw,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Lc709203f`].
    #[derive(Debug)]
    pub struct Lc709203f<I2C> {
        i2c: I2C,
        config: Config,
        wake_pending: bool,
    }

    constructors!(Lc709203f);

    impl<I2C: I2c> Lc709203f<I2C> {
        /// See [`super::Lc709203f::read_register`].
        pub async fn read_register(&mut self, reg: u8) -> Result<u16, Error<I2C::Error>> {
            let mut buf = [0; 3];
            self.i2c
                .write_read(ADDRESS, &[reg], &mut buf)
                .await
                .map_err(Error::bus)?;
            decode_read(reg, buf).ok_or(Error::Crc)
        }

        /// See [`super::Lc709203f::write_register`].
        pub async fn write_register(
            &mut self,
            reg: u8,
            value: u16,
        ) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write(ADDRESS, &encode_write(reg, value))
                .await
                .map_err(Error::bus)
        }

        /// See [`super::Lc709203f::probe`].
        pub async fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
            match self.read_register(reg::NUMBER_OF_PARAMETER).await? {
                id @ (0x0000 | 0xFFFF) => Err(Error::UnexpectedId(id as u32)),
                _ => Ok(()),
            }
        }

        /// See [`super::Lc709203f::start_init`].
        pub async fn start_init(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.write_register(reg::IC_POWER_MODE, POWER_OPERATIONAL)
                .await?;
            if self.read_register(reg::CHANGE_OF_PARAMETER).await? == self.config.profile {
                return Ok(0);
            }
            self.write_register(reg::CHANGE_OF_PARAMETER, self.config.profile)
                .await?;
            Ok(INIT_US)
        }

        /// See [`super::Lc709203f::finish_init`].
        pub async fn finish_init(&mut self) -> Result<(), Error<I2C::Error>> {
            let c = self.config;
            self.write_register(reg::IC_POWER_MODE, POWER_OPERATIONAL)
                .await?;
            self.write_register(reg::APA, c.apa as u16).await?;
            if c.initial_rsoc {
                self.write_register(reg::INITIAL_RSOC, INIT_RSOC_KEY)
                    .await?;
            }
            match c.thermistor_b {
                Some(b) => {
                    self.write_register(reg::STATUS_BIT, 1).await?;
                    self.write_register(reg::THERMISTOR_B, b).await
                }
                None => self.write_register(reg::STATUS_BIT, 0).await,
            }
        }

        /// See [`super::Lc709203f::rsoc`].
        pub async fn rsoc(&mut self) -> Result<u16, Error<I2C::Error>> {
            self.read_register(reg::RSOC).await
        }

        /// See [`super::Lc709203f::ic_version`].
        pub async fn ic_version(&mut self) -> Result<u16, Error<I2C::Error>> {
            self.read_register(reg::IC_VERSION).await
        }

        /// See [`super::Lc709203f::initial_rsoc`].
        pub async fn initial_rsoc(&mut self) -> Result<(), Error<I2C::Error>> {
            self.write_register(reg::INITIAL_RSOC, INIT_RSOC_KEY).await
        }

        /// See [`super::Lc709203f::set_temperature`].
        pub async fn set_temperature(&mut self, centi: i32) -> Result<(), Error<I2C::Error>> {
            self.write_register(reg::CELL_TEMPERATURE, temperature_raw(centi))
                .await
        }

        /// See [`super::Lc709203f::set_current_direction`].
        pub async fn set_current_direction(
            &mut self,
            direction: CurrentDirection,
        ) -> Result<(), Error<I2C::Error>> {
            self.write_register(reg::CURRENT_DIRECTION, direction as u16)
                .await
        }

        /// See [`super::Lc709203f::set_alarms`].
        pub async fn set_alarms(
            &mut self,
            rsoc_percent: u8,
            voltage_mv: u16,
        ) -> Result<(), Error<I2C::Error>> {
            self.write_register(reg::ALARM_LOW_RSOC, rsoc_percent.min(100) as u16)
                .await?;
            self.write_register(reg::ALARM_LOW_VOLTAGE, voltage_mv)
                .await
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Lc709203f<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            if self.wake_pending {
                self.write_register(reg::IC_POWER_MODE, POWER_OPERATIONAL)
                    .await?;
                self.wake_pending = false;
            }
            Ok(0)
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let voltage = self.read_register(reg::CELL_VOLTAGE).await? as i32;
            let percent = ite_half_percent(self.read_register(reg::ITE).await?);
            let temperature = match self.config.thermistor_b {
                Some(_) => Some(temperature_centi(
                    self.read_register(reg::CELL_TEMPERATURE).await?,
                )),
                None => None,
            };
            Ok(Measurement {
                voltage,
                percent,
                temperature,
            })
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::LowPower for Lc709203f<I2C> {
        type BusError = I2C::Error;

        async fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
            self.write_register(reg::IC_POWER_MODE, POWER_SLEEP).await
        }

        async fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            let _ = self
                .i2c
                .write(
                    ADDRESS,
                    &encode_write(reg::IC_POWER_MODE, POWER_OPERATIONAL),
                )
                .await;
            self.wake_pending = true;
            Ok(WAKE_US)
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as T};
    use std::vec;
    use zigbee_sensor::blocking::{LowPower, Sensor};

    fn rd(reg: u8, value: u16) -> T {
        let [l, h] = value.to_le_bytes();
        let crc = crc8_atm(&[0x16, reg, 0x17, l, h]);
        T::write_read(ADDRESS, vec![reg], vec![l, h, crc])
    }

    fn wr(reg: u8, value: u16) -> T {
        T::write(ADDRESS, encode_write(reg, value).to_vec())
    }

    #[test]
    fn crc_and_conversions() {
        // Data sheet examples: 3778 mV read → 0x86; 0xAA55 write → 0x3B.
        assert_eq!(crc8_atm(&[0x16, 0x09, 0x17, 0xC2, 0x0E]), 0x86);
        assert_eq!(decode_read(0x09, [0xC2, 0x0E, 0x86]), Some(3_778));
        assert_eq!(decode_read(0x09, [0xC2, 0x0E, 0x87]), None);
        assert_eq!(encode_write(0x09, 0xAA55), [0x09, 0x55, 0xAA, 0x3B]);
        assert_eq!(temperature_centi(0x0AAC), 0);
        assert_eq!(temperature_centi(0x0BA6), 2_500);
        assert_eq!(temperature_raw(2_500), 0x0BA6);
        assert_eq!(temperature_raw(-5_000), 0x09E4);
        assert_eq!(temperature_raw(9_000), 0x0D04);
        assert_eq!(ite_half_percent(1_000), 200);
        assert_eq!(ite_half_percent(512), 102);
        assert_eq!(typical_apa(500), 0x10);
        assert_eq!(typical_apa(1_500), 0x23);
        assert_eq!(typical_apa(50), 0x08);
        assert_eq!(typical_apa(5_000), 0x36);
    }

    #[test]
    fn init_measure_sleep() {
        let config = Config {
            apa: 0x2D,
            profile: 1,
            thermistor_b: Some(3_435),
            initial_rsoc: true,
        };
        let expectations = [
            rd(reg::NUMBER_OF_PARAMETER, 0x0301),
            wr(reg::IC_POWER_MODE, POWER_OPERATIONAL),
            rd(reg::CHANGE_OF_PARAMETER, 0),
            wr(reg::CHANGE_OF_PARAMETER, 1),
            wr(reg::IC_POWER_MODE, POWER_OPERATIONAL),
            wr(reg::APA, 0x2D),
            wr(reg::INITIAL_RSOC, INIT_RSOC_KEY),
            wr(reg::STATUS_BIT, 1),
            wr(reg::THERMISTOR_B, 3_435),
            rd(reg::CELL_VOLTAGE, 3_778),
            rd(reg::ITE, 755),
            rd(reg::CELL_TEMPERATURE, 0x0BA6),
            wr(reg::IC_POWER_MODE, POWER_SLEEP),
            wr(reg::IC_POWER_MODE, POWER_OPERATIONAL)
                .with_error(embedded_hal::i2c::ErrorKind::Other),
            wr(reg::IC_POWER_MODE, POWER_OPERATIONAL),
            rd(reg::CELL_VOLTAGE, 3_700),
            rd(reg::ITE, 0),
            T::write_read(ADDRESS, vec![reg::CELL_TEMPERATURE], vec![0, 0, 0]),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut gauge = Lc709203f::new(&mut i2c).with_config(config);
        gauge.probe().unwrap();
        assert_eq!(gauge.start_init(), Ok(INIT_US));
        gauge.finish_init().unwrap();
        assert_eq!(gauge.start_measurement(), Ok(0));
        assert_eq!(
            gauge.read_measurement().unwrap(),
            Measurement {
                voltage: 3_778,
                percent: 151,
                temperature: Some(2_500),
            }
        );
        gauge.sleep().unwrap();
        assert_eq!(gauge.wake(), Ok(WAKE_US));
        assert_eq!(gauge.start_measurement(), Ok(0));
        // A zero CRC byte does not match this reading.
        assert_eq!(gauge.read_measurement(), Err(Error::Crc));
        i2c.done();
    }

    #[test]
    fn i2c_mode_and_settings() {
        let expectations = [
            wr(reg::IC_POWER_MODE, POWER_OPERATIONAL),
            rd(reg::CHANGE_OF_PARAMETER, 1),
            wr(reg::IC_POWER_MODE, POWER_OPERATIONAL),
            wr(reg::APA, 0x10),
            wr(reg::STATUS_BIT, 0),
            wr(reg::CELL_TEMPERATURE, 0x0B10),
            wr(reg::CURRENT_DIRECTION, 0xFFFF),
            wr(reg::ALARM_LOW_RSOC, 100),
            wr(reg::ALARM_LOW_VOLTAGE, 3_300),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut gauge = Lc709203f::new(&mut i2c);
        assert_eq!(gauge.start_init(), Ok(0));
        gauge.finish_init().unwrap();
        gauge.set_temperature(1_000).unwrap();
        gauge
            .set_current_direction(CurrentDirection::Discharge)
            .unwrap();
        gauge.set_alarms(150, 3_300).unwrap();
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

        let expectations = [rd(reg::CELL_VOLTAGE, 4_100), rd(reg::ITE, 999)];
        let mut i2c = I2cMock::new(&expectations);
        let mut gauge = asynch::Lc709203f::new(&mut i2c);
        let mut delay = RecordingDelay(0);
        let m = block_on(gauge.measure(&mut delay)).unwrap();
        assert_eq!(
            m,
            Measurement {
                voltage: 4_100,
                percent: 200,
                temperature: None,
            }
        );
        assert_eq!(delay.0, 0);
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
