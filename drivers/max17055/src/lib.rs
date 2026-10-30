//! Maxim (Analog Devices) MAX17055 1-cell ModelGauge m5 EZ fuel gauge,
//! I2C address 0x36.
//!
//! The gauge measures continuously (one task period is 175.8 ms in active
//! mode, 5.625 s in hibernate), so `start_measurement` returns 0 and
//! `read_measurement` reads VCell, Current, AvgCurrent, Temp, RepCap,
//! RepSOC, FullCapRep and TTE in three auto-increment word reads.
//!
//! Registers are 16 bit, least-significant byte first. Capacity and
//! current scale with the sense resistor, given in µΩ (default 10 mΩ,
//! accepted 100 µΩ…1 Ω).
//!
//! # POR handling / EZ configuration
//!
//! All configuration lives in RAM. After every power-on reset (Status.POR
//! set) the host runs the EZ sequence from the MAX17055 Software
//! Implementation Guide (UG6365):
//!
//! ```text
//! if gauge.por()? {
//!     let cfg = EzConfig::new(1_000, 50); // 1000 mAh, 50 mA termination
//!     let mut wait = gauge.start_ez_config(&cfg)?; // NotReady while FStat.DNR
//!     loop {
//!         delay.delay_us(wait);
//!         match gauge.finish_ez_config() {
//!             Err(Error::NotReady) => wait = EZ_POLL_US, // ModelCfg.Refresh
//!             r => break r?,
//!         }
//!     }
//! }
//! ```
//!
//! References: MAX17055 data sheet 19-8758 Rev 3; UG6365 Rev 2.

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Fixed I2C address (0x6C/0x6D 8-bit).
pub const ADDRESS: u8 = 0x36;

/// Register addresses.
pub mod reg {
    /// Status (POR = bit 1).
    pub const STATUS: u8 = 0x00;
    /// RepCap (capacity).
    pub const REP_CAP: u8 = 0x05;
    /// RepSOC (1/256 %).
    pub const REP_SOC: u8 = 0x06;
    /// Temp (1/256 °C).
    pub const TEMP: u8 = 0x08;
    /// VCell (78.125 µV).
    pub const VCELL: u8 = 0x09;
    /// Current (1.5625 µV / R).
    pub const CURRENT: u8 = 0x0A;
    /// AvgCurrent.
    pub const AVG_CURRENT: u8 = 0x0B;
    /// FullCapRep.
    pub const FULL_CAP_REP: u8 = 0x10;
    /// TTE (5.625 s).
    pub const TTE: u8 = 0x11;
    /// DesignCap.
    pub const DESIGN_CAP: u8 = 0x18;
    /// Config.
    pub const CONFIG: u8 = 0x1D;
    /// IChgTerm.
    pub const ICHG_TERM: u8 = 0x1E;
    /// DevName.
    pub const DEV_NAME: u8 = 0x21;
    /// VEmpty.
    pub const VEMPTY: u8 = 0x3A;
    /// FStat (DNR = bit 0).
    pub const FSTAT: u8 = 0x3D;
    /// dQAcc.
    pub const DQ_ACC: u8 = 0x45;
    /// dPAcc.
    pub const DP_ACC: u8 = 0x46;
    /// Command.
    pub const COMMAND: u8 = 0x60;
    /// HibCfg.
    pub const HIB_CFG: u8 = 0xBA;
    /// ModelCfg.
    pub const MODEL_CFG: u8 = 0xDB;
}

/// DevName of the MAX17055.
pub const DEV_NAME: u16 = 0x4010;
/// Status.POR.
pub const STATUS_POR: u16 = 1 << 1;
/// FStat.DNR (data not ready after power-up).
pub const FSTAT_DNR: u16 = 1 << 0;
/// ModelCfg.Refresh.
pub const MODEL_CFG_REFRESH: u16 = 1 << 15;
/// Config.SHDN.
pub const CONFIG_SHDN: u16 = 1 << 7;
/// Command: soft wake-up from hibernate.
pub const CMD_SOFT_WAKEUP: u16 = 0x0090;
/// HibCfg power-on value.
pub const HIB_CFG_DEFAULT: u16 = 0x870C;
/// Suggested poll interval while ModelCfg.Refresh is set (UG6365).
pub const EZ_POLL_US: WaitUs = 10_000;
/// One active-mode task period (175.8 ms), rounded up.
pub const TASK_PERIOD_US: WaitUs = 176_000;

/// EZ battery model (ModelCfg.ModelID).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Model {
    /// Lithium cobalt oxide (most cells).
    #[default]
    LiCoO2 = 0,
    /// Lithium NCR / NCA.
    Nca = 2,
    /// LiFePO4 (custom characterization recommended).
    LiFePo4 = 6,
}

/// EZ configuration (UG6365 option 1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EzConfig {
    /// Design capacity in mAh.
    pub design_capacity_mah: u32,
    /// Charge termination current in mA.
    pub termination_current_ma: u32,
    /// Empty voltage (VE) in mV, 10 mV steps (default 3300).
    pub empty_voltage_mv: u16,
    /// Recovery voltage (VR) in mV, 40 mV steps (default 3880).
    pub recovery_voltage_mv: u16,
    /// Charge voltage above 4.275 V (ModelCfg.VChg).
    pub high_charge_voltage: bool,
    /// Battery model.
    pub model: Model,
}

impl EzConfig {
    /// Configuration with data-sheet default VEmpty (3.3 V / 3.88 V), 4.2 V
    /// charging and the LiCoO2 model.
    pub const fn new(design_capacity_mah: u32, termination_current_ma: u32) -> Self {
        Self {
            design_capacity_mah,
            termination_current_ma,
            empty_voltage_mv: 3_300,
            recovery_voltage_mv: 3_880,
            high_charge_voltage: false,
            model: Model::LiCoO2,
        }
    }

    /// VEmpty register value.
    pub const fn vempty(&self) -> u16 {
        let ve = (self.empty_voltage_mv / 10) as u32;
        let vr = (self.recovery_voltage_mv / 40) as u32;
        let ve = if ve > 0x1FF { 0x1FF } else { ve };
        let vr = if vr > 0x7F { 0x7F } else { vr };
        ((ve << 7) | vr) as u16
    }

    /// ModelCfg register value (Refresh set).
    pub const fn model_cfg(&self) -> u16 {
        MODEL_CFG_REFRESH
            | if self.high_charge_voltage { 1 << 10 } else { 0 }
            | ((self.model as u16) << 4)
    }
}

/// Register values derived from an [`EzConfig`] and the sense resistor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EzRegisters {
    /// DesignCap.
    pub design_cap: u16,
    /// dQAcc = DesignCap / 32.
    pub dq_acc: u16,
    /// IChgTerm.
    pub ichg_term: u16,
    /// VEmpty.
    pub vempty: u16,
    /// dPAcc = dQAcc × 44138 (51200 with VChg) / DesignCap.
    pub dp_acc: u16,
    /// ModelCfg.
    pub model_cfg: u16,
}

const fn sat_u16(v: u64) -> u16 {
    if v > 0xFFFF { 0xFFFF } else { v as u16 }
}

const fn clamp_rsense(uohm: u32) -> u32 {
    if uohm < 100 {
        100
    } else if uohm > 1_000_000 {
        1_000_000
    } else {
        uohm
    }
}

/// Compute the EZ register values (configuration time only; uses u64
/// division).
pub const fn ez_registers(cfg: &EzConfig, rsense_uohm: u32) -> EzRegisters {
    let r = clamp_rsense(rsense_uohm) as u64;
    // Capacity LSB 5 µVh / R: raw = mAh × R[µΩ] / 5000.
    let design_cap = sat_u16(cfg.design_capacity_mah as u64 * r / 5_000);
    // Current LSB 1.5625 µV / R: raw = µA × R[µΩ] / 1_562_500.
    let ichg_term = sat_u16(cfg.termination_current_ma as u64 * 1_000 * r / 1_562_500);
    let dq_acc = design_cap / 32;
    let k = if cfg.high_charge_voltage {
        51_200
    } else {
        44_138
    };
    let dp_acc = if design_cap == 0 {
        0
    } else {
        sat_u16(dq_acc as u64 * k / design_cap as u64)
    };
    EzRegisters {
        design_cap,
        dq_acc,
        ichg_term,
        vempty: cfg.vempty(),
        dp_acc,
        model_cfg: cfg.model_cfg(),
    }
}

/// Pre-computed scale factors (Q16) for a sense resistor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Scale {
    current_q16: u32,
    capacity_q16: u64,
}

impl Scale {
    /// Scale for a sense resistor in µΩ (clamped to 100 µΩ…1 Ω).
    pub const fn new(rsense_uohm: u32) -> Self {
        let r = clamp_rsense(rsense_uohm) as u64;
        Self {
            current_q16: (((1_562_500u64 << 16) + r / 2) / r) as u32,
            capacity_q16: ((5_000_000u64 << 16) + r / 2) / r,
        }
    }

    /// Current register → µA.
    pub const fn current_ua(&self, raw: u16) -> i32 {
        ((raw as i16 as i64 * self.current_q16 as i64 + 0x8000) >> 16) as i32
    }

    /// Capacity register → µAh (saturating).
    pub const fn capacity_uah(&self, raw: u16) -> u32 {
        let v = (raw as u64 * self.capacity_q16 + 0x8000) >> 16;
        if v > u32::MAX as u64 {
            u32::MAX
        } else {
            v as u32
        }
    }
}

/// VCell (78.125 µV) → mV.
pub const fn vcell_millivolts(raw: u16) -> i32 {
    (raw as i32 * 625 + 4_000) / 8_000
}

/// Temp (1/256 °C) → 0.01 °C.
pub const fn temperature_centi(raw: u16) -> i32 {
    (raw as i16 as i32 * 25 + 32) >> 6
}

/// RepSOC (1/256 %) → 0.5 % units, clamped to 200.
pub const fn soc_half_percent(raw: u16) -> u8 {
    let half = (raw as u32 + 64) >> 7;
    if half > 200 { 200 } else { half as u8 }
}

/// TTE (5.625 s) → seconds; 0xFFFF (not discharging) → `None`.
pub const fn tte_seconds(raw: u16) -> Option<u32> {
    if raw == 0xFFFF {
        None
    } else {
        Some(raw as u32 * 45 / 8)
    }
}

/// One reading.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Cell voltage in mV.
    pub voltage: i32,
    /// Instant current in µA (positive = charging).
    pub current: i32,
    /// Average current in µA.
    pub average_current: i32,
    /// Temperature (thermistor or die, per Config.TSel) in 0.01 °C.
    pub temperature: i32,
    /// RepSOC in 0.5 % units.
    pub percent: u8,
    /// RepCap in µAh.
    pub remaining_capacity: u32,
    /// FullCapRep in µAh.
    pub full_capacity: u32,
    /// Time to empty in seconds (`None` when not discharging).
    pub time_to_empty: Option<u32>,
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Voltage(self.voltage));
        f(Quantity::Current(self.current));
        f(Quantity::BatteryPercent(self.percent));
        f(Quantity::Temperature(self.temperature));
    }
}

fn decode(scale: &Scale, a: &[u8; 8], b: &[u8; 4], c: &[u8; 4]) -> Measurement {
    let w = |s: &[u8], i: usize| u16::from_le_bytes([s[2 * i], s[2 * i + 1]]);
    Measurement {
        temperature: temperature_centi(w(a, 0)),
        voltage: vcell_millivolts(w(a, 1)),
        current: scale.current_ua(w(a, 2)),
        average_current: scale.current_ua(w(a, 3)),
        remaining_capacity: scale.capacity_uah(w(b, 0)),
        percent: soc_half_percent(w(b, 1)),
        full_capacity: scale.capacity_uah(w(c, 0)),
        time_to_empty: tte_seconds(w(c, 1)),
    }
}

macro_rules! constructors {
    ($name:ident) => {
        impl<I2C> $name<I2C> {
            /// Create a driver with a 10 mΩ sense resistor; no bus traffic.
            pub const fn new(i2c: I2C) -> Self {
                Self {
                    i2c,
                    rsense_uohm: 10_000,
                    scale: Scale::new(10_000),
                    hib_cfg: HIB_CFG_DEFAULT,
                }
            }

            /// Sense resistor in µΩ (100 µΩ…1 Ω).
            pub const fn with_sense_resistor(mut self, uohm: u32) -> Self {
                self.rsense_uohm = clamp_rsense(uohm);
                self.scale = Scale::new(uohm);
                self
            }

            /// HibCfg written by `LowPower::sleep` (default 0x870C); replaced
            /// by the value read in `start_ez_config`.
            pub const fn with_hib_cfg(mut self, hib_cfg: u16) -> Self {
                self.hib_cfg = hib_cfg;
                self
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
pub struct Max17055<I2C> {
    i2c: I2C,
    rsense_uohm: u32,
    scale: Scale,
    hib_cfg: u16,
}

constructors!(Max17055);

impl<I2C: embedded_hal::i2c::I2c> Max17055<I2C> {
    /// Read a 16-bit register.
    pub fn read_register(&mut self, reg: u8) -> Result<u16, Error<I2C::Error>> {
        let mut buf = [0; 2];
        self.i2c
            .write_read(ADDRESS, &[reg], &mut buf)
            .map_err(Error::bus)?;
        Ok(u16::from_le_bytes(buf))
    }

    /// Write a 16-bit register.
    pub fn write_register(&mut self, reg: u8, value: u16) -> Result<(), Error<I2C::Error>> {
        let [l, m] = value.to_le_bytes();
        self.i2c.write(ADDRESS, &[reg, l, m]).map_err(Error::bus)
    }

    /// Check DevName (0x4010).
    pub fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
        let id = self.read_register(reg::DEV_NAME)?;
        if id == DEV_NAME {
            Ok(())
        } else {
            Err(Error::UnexpectedId(id as u32))
        }
    }

    /// Status.POR: the gauge was reset and needs `start_ez_config`.
    pub fn por(&mut self) -> Result<bool, Error<I2C::Error>> {
        Ok(self.read_register(reg::STATUS)? & STATUS_POR != 0)
    }

    /// EZ configuration, first half: returns `NotReady` while FStat.DNR is
    /// set, saves HibCfg, exits hibernate and writes DesignCap, dQAcc,
    /// IChgTerm, VEmpty, dPAcc and ModelCfg (Refresh). Wait the returned
    /// time, then call `finish_ez_config`.
    pub fn start_ez_config(&mut self, cfg: &EzConfig) -> Result<WaitUs, Error<I2C::Error>> {
        if self.read_register(reg::FSTAT)? & FSTAT_DNR != 0 {
            return Err(Error::NotReady);
        }
        self.hib_cfg = self.read_register(reg::HIB_CFG)?;
        self.write_register(reg::COMMAND, CMD_SOFT_WAKEUP)?;
        self.write_register(reg::HIB_CFG, 0)?;
        self.write_register(reg::COMMAND, 0)?;
        let r = ez_registers(cfg, self.rsense_uohm);
        self.write_register(reg::DESIGN_CAP, r.design_cap)?;
        self.write_register(reg::DQ_ACC, r.dq_acc)?;
        self.write_register(reg::ICHG_TERM, r.ichg_term)?;
        self.write_register(reg::VEMPTY, r.vempty)?;
        self.write_register(reg::DP_ACC, r.dp_acc)?;
        self.write_register(reg::MODEL_CFG, r.model_cfg)?;
        Ok(EZ_POLL_US)
    }

    /// EZ configuration, second half: `NotReady` while ModelCfg.Refresh is
    /// set (poll every [`EZ_POLL_US`]); then restores HibCfg and clears
    /// Status.POR.
    pub fn finish_ez_config(&mut self) -> Result<(), Error<I2C::Error>> {
        if self.read_register(reg::MODEL_CFG)? & MODEL_CFG_REFRESH != 0 {
            return Err(Error::NotReady);
        }
        self.write_register(reg::HIB_CFG, self.hib_cfg)?;
        let status = self.read_register(reg::STATUS)?;
        self.write_register(reg::STATUS, status & !STATUS_POR)
    }

    /// Software shutdown (0.5 µA): HibCfg = 0, Config.SHDN = 1. The IC
    /// shuts down after ShdnTimer (default 45 s) and wakes on any SDA/SCL
    /// edge with all fuel-gauge state lost (Status.POR set → run the EZ
    /// configuration again).
    pub fn shutdown(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_register(reg::HIB_CFG, 0)?;
        let config = self.read_register(reg::CONFIG)?;
        self.write_register(reg::CONFIG, config | CONFIG_SHDN)
    }

    fn read_block<const N: usize>(&mut self, reg: u8) -> Result<[u8; N], Error<I2C::Error>> {
        let mut buf = [0; N];
        self.i2c
            .write_read(ADDRESS, &[reg], &mut buf)
            .map_err(Error::bus)?;
        Ok(buf)
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Max17055<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    /// The gauge runs autonomously; nothing to start.
    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        Ok(0)
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        let a = self.read_block::<8>(reg::TEMP)?;
        let b = self.read_block::<4>(reg::REP_CAP)?;
        let c = self.read_block::<4>(reg::FULL_CAP_REP)?;
        Ok(decode(&self.scale, &a, &b, &c))
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::LowPower for Max17055<I2C> {
    type BusError = I2C::Error;

    /// Allow hibernate (7 µA, 5.625 s updates) by writing the saved HibCfg.
    /// The gauge keeps tracking; for 0.5 µA shutdown see `shutdown`.
    fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_register(reg::HIB_CFG, self.hib_cfg)
    }

    /// Force active mode (18 µA): HibCfg = 0, soft wake-up, clear command.
    fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.write_register(reg::HIB_CFG, 0)?;
        self.write_register(reg::COMMAND, CMD_SOFT_WAKEUP)?;
        self.write_register(reg::COMMAND, 0)?;
        Ok(TASK_PERIOD_US)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        ADDRESS, CMD_SOFT_WAKEUP, CONFIG_SHDN, DEV_NAME, EZ_POLL_US, EzConfig, FSTAT_DNR,
        HIB_CFG_DEFAULT, MODEL_CFG_REFRESH, Measurement, STATUS_POR, Scale, TASK_PERIOD_US,
        clamp_rsense, decode, ez_registers, reg,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Max17055`].
    #[derive(Debug)]
    pub struct Max17055<I2C> {
        i2c: I2C,
        rsense_uohm: u32,
        scale: Scale,
        hib_cfg: u16,
    }

    constructors!(Max17055);

    impl<I2C: I2c> Max17055<I2C> {
        /// See [`super::Max17055::read_register`].
        pub async fn read_register(&mut self, reg: u8) -> Result<u16, Error<I2C::Error>> {
            let mut buf = [0; 2];
            self.i2c
                .write_read(ADDRESS, &[reg], &mut buf)
                .await
                .map_err(Error::bus)?;
            Ok(u16::from_le_bytes(buf))
        }

        /// See [`super::Max17055::write_register`].
        pub async fn write_register(
            &mut self,
            reg: u8,
            value: u16,
        ) -> Result<(), Error<I2C::Error>> {
            let [l, m] = value.to_le_bytes();
            self.i2c
                .write(ADDRESS, &[reg, l, m])
                .await
                .map_err(Error::bus)
        }

        /// See [`super::Max17055::probe`].
        pub async fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
            let id = self.read_register(reg::DEV_NAME).await?;
            if id == DEV_NAME {
                Ok(())
            } else {
                Err(Error::UnexpectedId(id as u32))
            }
        }

        /// See [`super::Max17055::por`].
        pub async fn por(&mut self) -> Result<bool, Error<I2C::Error>> {
            Ok(self.read_register(reg::STATUS).await? & STATUS_POR != 0)
        }

        /// See [`super::Max17055::start_ez_config`].
        pub async fn start_ez_config(
            &mut self,
            cfg: &EzConfig,
        ) -> Result<WaitUs, Error<I2C::Error>> {
            if self.read_register(reg::FSTAT).await? & FSTAT_DNR != 0 {
                return Err(Error::NotReady);
            }
            self.hib_cfg = self.read_register(reg::HIB_CFG).await?;
            self.write_register(reg::COMMAND, CMD_SOFT_WAKEUP).await?;
            self.write_register(reg::HIB_CFG, 0).await?;
            self.write_register(reg::COMMAND, 0).await?;
            let r = ez_registers(cfg, self.rsense_uohm);
            self.write_register(reg::DESIGN_CAP, r.design_cap).await?;
            self.write_register(reg::DQ_ACC, r.dq_acc).await?;
            self.write_register(reg::ICHG_TERM, r.ichg_term).await?;
            self.write_register(reg::VEMPTY, r.vempty).await?;
            self.write_register(reg::DP_ACC, r.dp_acc).await?;
            self.write_register(reg::MODEL_CFG, r.model_cfg).await?;
            Ok(EZ_POLL_US)
        }

        /// See [`super::Max17055::finish_ez_config`].
        pub async fn finish_ez_config(&mut self) -> Result<(), Error<I2C::Error>> {
            if self.read_register(reg::MODEL_CFG).await? & MODEL_CFG_REFRESH != 0 {
                return Err(Error::NotReady);
            }
            self.write_register(reg::HIB_CFG, self.hib_cfg).await?;
            let status = self.read_register(reg::STATUS).await?;
            self.write_register(reg::STATUS, status & !STATUS_POR).await
        }

        /// See [`super::Max17055::shutdown`].
        pub async fn shutdown(&mut self) -> Result<(), Error<I2C::Error>> {
            self.write_register(reg::HIB_CFG, 0).await?;
            let config = self.read_register(reg::CONFIG).await?;
            self.write_register(reg::CONFIG, config | CONFIG_SHDN).await
        }

        async fn read_block<const N: usize>(
            &mut self,
            reg: u8,
        ) -> Result<[u8; N], Error<I2C::Error>> {
            let mut buf = [0; N];
            self.i2c
                .write_read(ADDRESS, &[reg], &mut buf)
                .await
                .map_err(Error::bus)?;
            Ok(buf)
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Max17055<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            Ok(0)
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let a = self.read_block::<8>(reg::TEMP).await?;
            let b = self.read_block::<4>(reg::REP_CAP).await?;
            let c = self.read_block::<4>(reg::FULL_CAP_REP).await?;
            Ok(decode(&self.scale, &a, &b, &c))
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::LowPower for Max17055<I2C> {
        type BusError = I2C::Error;

        async fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
            self.write_register(reg::HIB_CFG, self.hib_cfg).await
        }

        async fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.write_register(reg::HIB_CFG, 0).await?;
            self.write_register(reg::COMMAND, CMD_SOFT_WAKEUP).await?;
            self.write_register(reg::COMMAND, 0).await?;
            Ok(TASK_PERIOD_US)
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as T};
    use std::vec;
    use std::vec::Vec;
    use zigbee_sensor::blocking::{LowPower, Sensor};

    fn rd(reg: u8, value: u16) -> T {
        T::write_read(ADDRESS, vec![reg], value.to_le_bytes().to_vec())
    }

    fn wr(reg: u8, value: u16) -> T {
        let [l, m] = value.to_le_bytes();
        T::write(ADDRESS, vec![reg, l, m])
    }

    fn rdn(reg: u8, words: &[u16]) -> T {
        let bytes: Vec<u8> = words.iter().flat_map(|w| w.to_le_bytes()).collect();
        T::write_read(ADDRESS, vec![reg], bytes)
    }

    fn measurement_reads() -> [T; 3] {
        [
            rdn(reg::TEMP, &[0x1980, 47_360, 0xFF9C, 0xFFC0]),
            rdn(reg::REP_CAP, &[2_000, 0x3200]),
            rdn(reg::FULL_CAP_REP, &[4_000, 640]),
        ]
    }

    const EXPECTED: Measurement = Measurement {
        voltage: 3_700,
        current: -15_625,
        average_current: -10_000,
        temperature: 2_550,
        percent: 100,
        remaining_capacity: 1_000_000,
        full_capacity: 2_000_000,
        time_to_empty: Some(3_600),
    };

    #[test]
    fn conversions() {
        let r = ez_registers(&EzConfig::new(1_000, 50), 10_000);
        assert_eq!(
            r,
            EzRegisters {
                design_cap: 2_000,
                dq_acc: 62,
                ichg_term: 320,
                vempty: 0xA561,
                dp_acc: 1_368,
                model_cfg: 0x8000,
            }
        );
        let mut cfg = EzConfig::new(3_000, 250);
        cfg.high_charge_voltage = true;
        cfg.model = Model::Nca;
        let r = ez_registers(&cfg, 5_000);
        assert_eq!(
            (r.design_cap, r.ichg_term, r.model_cfg),
            (3_000, 800, 0x8420)
        );
        assert_eq!(r.dp_acc, 1_587); // 93 × 51200 / 3000
        let s = Scale::new(5_000);
        assert_eq!(s.current_ua(1), 313);
        assert_eq!(s.capacity_uah(1), 1_000);
        assert_eq!(temperature_centi(0xFF00), -100);
        assert_eq!(tte_seconds(0xFFFF), None);
    }

    #[test]
    fn ez_config_sequence() {
        let expectations = [
            rd(reg::DEV_NAME, DEV_NAME),
            rd(reg::STATUS, 0x0002),
            rd(reg::FSTAT, 0x0001),
            rd(reg::FSTAT, 0x0000),
            rd(reg::HIB_CFG, 0x870C),
            wr(reg::COMMAND, 0x0090),
            wr(reg::HIB_CFG, 0),
            wr(reg::COMMAND, 0),
            wr(reg::DESIGN_CAP, 2_000),
            wr(reg::DQ_ACC, 62),
            wr(reg::ICHG_TERM, 320),
            wr(reg::VEMPTY, 0xA561),
            wr(reg::DP_ACC, 1_368),
            wr(reg::MODEL_CFG, 0x8000),
            rd(reg::MODEL_CFG, 0x8000),
            rd(reg::MODEL_CFG, 0x0000),
            wr(reg::HIB_CFG, 0x870C),
            rd(reg::STATUS, 0x0082),
            wr(reg::STATUS, 0x0080),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut gauge = Max17055::new(&mut i2c);
        gauge.probe().unwrap();
        assert_eq!(gauge.por(), Ok(true));
        let cfg = EzConfig::new(1_000, 50);
        assert_eq!(gauge.start_ez_config(&cfg), Err(Error::NotReady));
        assert_eq!(gauge.start_ez_config(&cfg), Ok(EZ_POLL_US));
        assert_eq!(gauge.finish_ez_config(), Err(Error::NotReady));
        gauge.finish_ez_config().unwrap();
        i2c.done();
    }

    #[test]
    fn measure_and_power_modes() {
        let mut expectations = measurement_reads().to_vec();
        expectations.extend([
            wr(reg::HIB_CFG, 0x1234),
            wr(reg::HIB_CFG, 0),
            wr(reg::COMMAND, 0x0090),
            wr(reg::COMMAND, 0),
            wr(reg::HIB_CFG, 0),
            rd(reg::CONFIG, 0x2210),
            wr(reg::CONFIG, 0x2290),
            rd(reg::DEV_NAME, 0x4031),
        ]);
        let mut i2c = I2cMock::new(&expectations);
        let mut gauge = Max17055::new(&mut i2c).with_hib_cfg(0x1234);
        assert_eq!(gauge.start_measurement(), Ok(0));
        assert_eq!(gauge.read_measurement(), Ok(EXPECTED));
        gauge.sleep().unwrap();
        assert_eq!(gauge.wake(), Ok(TASK_PERIOD_US));
        gauge.shutdown().unwrap();
        assert_eq!(gauge.probe(), Err(Error::UnexpectedId(0x4031)));
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

        let mut i2c = I2cMock::new(&measurement_reads());
        let mut gauge = asynch::Max17055::new(&mut i2c);
        let mut delay = RecordingDelay(0);
        assert_eq!(block_on(gauge.measure(&mut delay)), Ok(EXPECTED));
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
