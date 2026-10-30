//! Texas Instruments BQ27441-G1 / BQ27421-G1 / BQ27426 Impedance Track
//! 1-cell Li-ion fuel gauges, I2C address 0x55.
//!
//! The gauge measures voltage, average current and temperature once per
//! second by itself, so `start_measurement` returns 0 and
//! `read_measurement` does one incremental read of the standard commands
//! 0x02..=0x21 (one command, see the rate limit below).
//!
//! Configuration (design capacity, design energy, terminate voltage, taper
//! rate) lives in RAM and returns to ROM defaults after POR (Flags.ITPOR).
//! Update it with the TRM "Data Memory Parameter Update" sequence:
//!
//! ```text
//! gauge.unseal()?;                      // only if SEALED
//! wait(gauge.enter_config_update()?);   // then poll until true, ≤ 1 s
//! while !gauge.in_config_update()? { wait(CFG_UPDATE_POLL_US) }
//! gauge.configure_battery(&BatteryConfig::new(1200, 3200, 60))?;
//! wait(gauge.exit_config_update()?);    // SOFT_RESET, poll until false
//! while gauge.in_config_update()? { wait(CFG_UPDATE_POLL_US) }
//! gauge.seal()?;                        // if it was sealed
//! ```
//!
//! References: TI SLUUAC9A (bq27441-G1 TRM), SLUUAC5 (bq27421-G1 TRM),
//! SLUUBB0 (bq27426 TRM), SLUSBH1C (bq27441-G1 data sheet).

#![no_std]

use zigbee_sensor::{Error, Quantity, Readings, WaitUs};

/// Fixed I2C address (0xAA/0xAB 8-bit).
pub const ADDRESS: u8 = 0x55;

/// Standard and extended command codes (2-byte values are LSB first).
pub mod cmd {
    /// Control() (subcommand in, result out).
    pub const CONTROL: u8 = 0x00;
    /// Temperature(), 0.1 K.
    pub const TEMPERATURE: u8 = 0x02;
    /// Voltage(), mV.
    pub const VOLTAGE: u8 = 0x04;
    /// Flags().
    pub const FLAGS: u8 = 0x06;
    /// RemainingCapacity(), mAh.
    pub const REMAINING_CAPACITY: u8 = 0x0C;
    /// FullChargeCapacity(), mAh.
    pub const FULL_CHARGE_CAPACITY: u8 = 0x0E;
    /// AverageCurrent(), signed mA.
    pub const AVERAGE_CURRENT: u8 = 0x10;
    /// AveragePower(), signed mW.
    pub const AVERAGE_POWER: u8 = 0x18;
    /// StateOfCharge(), %.
    pub const STATE_OF_CHARGE: u8 = 0x1C;
    /// StateOfHealth() (percent, status byte).
    pub const STATE_OF_HEALTH: u8 = 0x20;
    /// DataClass().
    pub const DATA_CLASS: u8 = 0x3E;
    /// DataBlock().
    pub const DATA_BLOCK: u8 = 0x3F;
    /// BlockData() 0x40..=0x5F.
    pub const BLOCK_DATA: u8 = 0x40;
    /// BlockDataChecksum().
    pub const BLOCK_DATA_CHECKSUM: u8 = 0x60;
    /// BlockDataControl().
    pub const BLOCK_DATA_CONTROL: u8 = 0x61;
}

/// Control() subcommands.
pub mod ctrl {
    /// CONTROL_STATUS.
    pub const CONTROL_STATUS: u16 = 0x0000;
    /// DEVICE_TYPE.
    pub const DEVICE_TYPE: u16 = 0x0001;
    /// FW_VERSION.
    pub const FW_VERSION: u16 = 0x0002;
    /// DM_CODE.
    pub const DM_CODE: u16 = 0x0004;
    /// SET_HIBERNATE.
    pub const SET_HIBERNATE: u16 = 0x0011;
    /// CLEAR_HIBERNATE.
    pub const CLEAR_HIBERNATE: u16 = 0x0012;
    /// SET_CFGUPDATE (unsealed).
    pub const SET_CFGUPDATE: u16 = 0x0013;
    /// SHUTDOWN_ENABLE (unsealed).
    pub const SHUTDOWN_ENABLE: u16 = 0x001B;
    /// SHUTDOWN (unsealed, after SHUTDOWN_ENABLE).
    pub const SHUTDOWN: u16 = 0x001C;
    /// SEALED.
    pub const SEALED: u16 = 0x0020;
    /// RESET (unsealed): full reset to ROM defaults.
    pub const RESET: u16 = 0x0041;
    /// SOFT_RESET: leave CONFIG UPDATE mode and re-simulate.
    pub const SOFT_RESET: u16 = 0x0042;
    /// EXIT_CFGUPDATE: leave CONFIG UPDATE mode without re-simulation.
    pub const EXIT_CFGUPDATE: u16 = 0x0043;
    /// EXIT_RESIM: leave CONFIG UPDATE mode and re-simulate.
    pub const EXIT_RESIM: u16 = 0x0044;
    /// UNSEAL key word (sent twice).
    pub const UNSEAL_KEY: u16 = 0x8000;
}

/// Flags(): over-temperature.
pub const FLAGS_OT: u16 = 1 << 15;
/// Flags(): under-temperature.
pub const FLAGS_UT: u16 = 1 << 14;
/// Flags(): full charge.
pub const FLAGS_FC: u16 = 1 << 9;
/// Flags(): fast charging allowed.
pub const FLAGS_CHG: u16 = 1 << 8;
/// Flags(): OCV measurement taken.
pub const FLAGS_OCVTAKEN: u16 = 1 << 7;
/// Flags(): RAM configuration was reset to ROM defaults (POR or RESET).
pub const FLAGS_ITPOR: u16 = 1 << 5;
/// Flags(): CONFIG UPDATE mode active.
pub const FLAGS_CFGUPMODE: u16 = 1 << 4;
/// Flags(): battery detected.
pub const FLAGS_BAT_DET: u16 = 1 << 3;
/// Flags(): SOC below SOC1 Set Threshold.
pub const FLAGS_SOC1: u16 = 1 << 2;
/// Flags(): SOC below SOCF Set Threshold.
pub const FLAGS_SOCF: u16 = 1 << 1;
/// Flags(): discharging.
pub const FLAGS_DSG: u16 = 1 << 0;

/// CONTROL_STATUS: shutdown enabled.
pub const STATUS_SHUTDOWNEN: u16 = 1 << 15;
/// CONTROL_STATUS: SEALED.
pub const STATUS_SS: u16 = 1 << 13;
/// CONTROL_STATUS: initialisation complete.
pub const STATUS_INITCOMP: u16 = 1 << 7;
/// CONTROL_STATUS: hibernate requested.
pub const STATUS_HIBERNATE: u16 = 1 << 6;
/// CONTROL_STATUS: in SLEEP mode.
pub const STATUS_SLEEP: u16 = 1 << 4;

/// State subclass ID (design capacity, energy, terminate voltage, taper
/// rate).
pub const STATE_SUBCLASS: u8 = 82;

/// Poll interval while entering/leaving CONFIG UPDATE mode (≤ 1 s each).
/// The data sheet limits standard commands to two per second, so polling
/// Flags() faster risks a watchdog reset.
pub const CFG_UPDATE_POLL_US: WaitUs = 500_000;

/// Supported parts.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Variant {
    /// BQ27441-G1A/G1B.
    #[default]
    Bq27441,
    /// BQ27421-G1A/G1B/G1D.
    Bq27421,
    /// BQ27426.
    Bq27426,
}

impl Variant {
    /// State-subclass offsets of (Design Capacity, Design Energy,
    /// Terminate Voltage, Taper Rate).
    pub const fn state_offsets(self) -> [u8; 4] {
        match self {
            Self::Bq27441 | Self::Bq27421 => [10, 12, 16, 27],
            Self::Bq27426 => [6, 8, 10, 21],
        }
    }

    /// Whether `id` is a DEVICE_TYPE this variant reports.
    pub const fn accepts_device_type(self, id: u16) -> bool {
        match self {
            Self::Bq27441 | Self::Bq27421 => id == 0x0421,
            // SLUUBB0 lists 0x0426 in the command table but 0x0421 in the
            // subcommand summary; accept both.
            Self::Bq27426 => id == 0x0426 || id == 0x0421,
        }
    }
}

/// Battery parameters for the State subclass.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BatteryConfig {
    /// Design Capacity, mAh (0..=8000).
    pub design_capacity_mah: u16,
    /// Design Energy, mWh (0..=32767).
    pub design_energy_mwh: u16,
    /// Terminate Voltage, mV (2500..=3700).
    pub terminate_voltage_mv: u16,
    /// Taper Rate, 0.1 h rate (0..=2000).
    pub taper_rate: u16,
}

impl BatteryConfig {
    /// Parameters from capacity, empty voltage and the charger's
    /// termination (taper) current. Design Energy = capacity × 3.7 V;
    /// Taper Rate = capacity / (0.1 × taper current). Values are clamped
    /// to the TRM ranges.
    pub const fn new(capacity_mah: u16, terminate_voltage_mv: u16, taper_current_ma: u16) -> Self {
        let cap = if capacity_mah > 8_000 {
            8_000
        } else {
            capacity_mah
        };
        let energy = cap as u32 * 37 / 10;
        let taper = if taper_current_ma == 0 {
            2_000
        } else {
            let t = cap as u32 * 10 / taper_current_ma as u32;
            if t > 2_000 { 2_000 } else { t }
        };
        let tv = if terminate_voltage_mv < 2_500 {
            2_500
        } else if terminate_voltage_mv > 3_700 {
            3_700
        } else {
            terminate_voltage_mv
        };
        Self {
            design_capacity_mah: cap,
            design_energy_mwh: energy as u16,
            terminate_voltage_mv: tv,
            taper_rate: taper as u16,
        }
    }

    /// Write the parameters (big-endian) into a State block 0 image and
    /// return the touched byte range.
    pub fn apply(&self, variant: Variant, block: &mut [u8; 32]) -> (usize, usize) {
        let off = variant.state_offsets();
        let values = [
            self.design_capacity_mah,
            self.design_energy_mwh,
            self.terminate_voltage_mv,
            self.taper_rate,
        ];
        for (o, v) in off.iter().zip(values) {
            let o = *o as usize;
            block[o..o + 2].copy_from_slice(&v.to_be_bytes());
        }
        (off[0] as usize, off[3] as usize + 2)
    }
}

/// BlockDataChecksum: 255 − (byte sum mod 256).
pub fn block_checksum(block: &[u8; 32]) -> u8 {
    !block.iter().fold(0u8, |s, b| s.wrapping_add(*b))
}

/// Temperature() (0.1 K) in 0.01 °C.
pub const fn kelvin_tenths_to_centi(raw: u16) -> i32 {
    raw as i32 * 10 - 27_315
}

/// 0.01 °C to Temperature() (0.1 K), clamped at 0 K.
pub const fn centi_to_kelvin_tenths(centi: i32) -> u16 {
    let k = (centi + 27_315 + 5) / 10;
    if k < 0 {
        0
    } else if k > u16::MAX as i32 {
        u16::MAX
    } else {
        k as u16
    }
}

/// First command of the measurement burst.
const BURST_START: u8 = cmd::TEMPERATURE;
/// Bytes in the 0x02..=0x21 burst.
const BURST_LEN: usize = 32;

/// One reading.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Cell voltage in mV.
    pub voltage: i32,
    /// AverageCurrent in µA (positive = charging).
    pub current: i32,
    /// AveragePower in mW.
    pub power: i32,
    /// StateOfCharge in 0.5 % units (0..=200).
    pub percent: u8,
    /// Gauge temperature in 0.01 °C (internal sensor unless OpConfig.TEMPS
    /// and `write_temperature`).
    pub temperature: i32,
    /// RemainingCapacity in mAh.
    pub remaining_capacity_mah: u16,
    /// FullChargeCapacity in mAh.
    pub full_charge_capacity_mah: u16,
    /// StateOfHealth in % (None until the SOH status is valid).
    pub state_of_health: Option<u8>,
    /// Flags().
    pub flags: u16,
}

impl Measurement {
    /// Decode the 0x02..=0x21 burst.
    pub fn from_burst(b: &[u8; 32]) -> Self {
        let w = |cmd: u8| {
            let i = (cmd - BURST_START) as usize;
            u16::from_le_bytes([b[i], b[i + 1]])
        };
        let soc = w(cmd::STATE_OF_CHARGE).min(100) as u8;
        let soh = w(cmd::STATE_OF_HEALTH);
        Self {
            voltage: w(cmd::VOLTAGE) as i32,
            current: w(cmd::AVERAGE_CURRENT) as i16 as i32 * 1_000,
            power: w(cmd::AVERAGE_POWER) as i16 as i32,
            percent: soc * 2,
            temperature: kelvin_tenths_to_centi(w(cmd::TEMPERATURE)),
            remaining_capacity_mah: w(cmd::REMAINING_CAPACITY),
            full_charge_capacity_mah: w(cmd::FULL_CHARGE_CAPACITY),
            state_of_health: if soh >> 8 == 0 {
                None
            } else {
                Some((soh & 0xFF).min(100) as u8)
            },
            flags: w(cmd::FLAGS),
        }
    }
}

impl Readings for Measurement {
    fn for_each(&self, f: &mut dyn FnMut(Quantity)) {
        f(Quantity::Voltage(self.voltage));
        f(Quantity::Current(self.current));
        f(Quantity::Power(self.power));
        f(Quantity::BatteryPercent(self.percent));
        f(Quantity::Temperature(self.temperature));
    }
}

fn check_range(offset: u8, len: usize) -> bool {
    len != 0 && (offset % 32) as usize + len <= 32
}

macro_rules! constructors {
    ($name:ident) => {
        impl<I2C> $name<I2C> {
            /// Create a driver; no bus traffic.
            pub const fn new(i2c: I2C, variant: Variant) -> Self {
                Self {
                    i2c,
                    variant,
                    byte_writes: false,
                }
            }

            /// Split multi-byte writes into 1-byte writes, as the data sheet
            /// requires above 100 kHz SCL. The platform must still keep the
            /// 66 µs bus-free time between transactions.
            pub const fn with_byte_writes(mut self, enable: bool) -> Self {
                self.byte_writes = enable;
                self
            }

            /// Configured variant.
            pub const fn variant(&self) -> Variant {
                self.variant
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
pub struct Bq27441<I2C> {
    i2c: I2C,
    variant: Variant,
    byte_writes: bool,
}

constructors!(Bq27441);

impl<I2C: embedded_hal::i2c::I2c> Bq27441<I2C> {
    /// Write `data` starting at command `cmd` (≤ 32 bytes).
    pub fn write_bytes(&mut self, cmd: u8, data: &[u8]) -> Result<(), Error<I2C::Error>> {
        if data.len() > 32 {
            return Err(Error::InvalidData);
        }
        if self.byte_writes {
            for (i, b) in data.iter().enumerate() {
                self.i2c
                    .write(ADDRESS, &[cmd + i as u8, *b])
                    .map_err(Error::bus)?;
            }
            Ok(())
        } else {
            let mut buf = [0; 33];
            buf[0] = cmd;
            buf[1..=data.len()].copy_from_slice(data);
            self.i2c
                .write(ADDRESS, &buf[..=data.len()])
                .map_err(Error::bus)
        }
    }

    /// Incremental read starting at command `cmd`.
    pub fn read_bytes(&mut self, cmd: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write_read(ADDRESS, &[cmd], buf)
            .map_err(Error::bus)
    }

    /// Read a 2-byte standard command.
    pub fn read_word(&mut self, cmd: u8) -> Result<u16, Error<I2C::Error>> {
        let mut buf = [0; 2];
        self.read_bytes(cmd, &mut buf)?;
        Ok(u16::from_le_bytes(buf))
    }

    /// Send a Control() subcommand without reading the result.
    pub fn control_write(&mut self, sub: u16) -> Result<(), Error<I2C::Error>> {
        self.write_bytes(cmd::CONTROL, &sub.to_le_bytes())
    }

    /// Send a Control() subcommand and read its 2-byte result.
    pub fn control(&mut self, sub: u16) -> Result<u16, Error<I2C::Error>> {
        self.control_write(sub)?;
        self.read_word(cmd::CONTROL)
    }

    /// Check DEVICE_TYPE against the variant.
    pub fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
        let id = self.control(ctrl::DEVICE_TYPE)?;
        if self.variant.accepts_device_type(id) {
            Ok(())
        } else {
            Err(Error::UnexpectedId(id as u32))
        }
    }

    /// CONTROL_STATUS.
    pub fn control_status(&mut self) -> Result<u16, Error<I2C::Error>> {
        self.control(ctrl::CONTROL_STATUS)
    }

    /// FW_VERSION.
    pub fn fw_version(&mut self) -> Result<u16, Error<I2C::Error>> {
        self.control(ctrl::FW_VERSION)
    }

    /// DM_CODE (data memory configuration code).
    pub fn dm_code(&mut self) -> Result<u16, Error<I2C::Error>> {
        self.control(ctrl::DM_CODE)
    }

    /// Flags().
    pub fn flags(&mut self) -> Result<u16, Error<I2C::Error>> {
        self.read_word(cmd::FLAGS)
    }

    /// Flags.ITPOR: RAM configuration is at ROM defaults and needs
    /// `configure_battery` again.
    pub fn needs_config(&mut self) -> Result<bool, Error<I2C::Error>> {
        Ok(self.flags()? & FLAGS_ITPOR != 0)
    }

    /// Flags.CFGUPMODE.
    pub fn in_config_update(&mut self) -> Result<bool, Error<I2C::Error>> {
        Ok(self.flags()? & FLAGS_CFGUPMODE != 0)
    }

    /// Send the UNSEAL key (0x8000 twice).
    pub fn unseal(&mut self) -> Result<(), Error<I2C::Error>> {
        self.control_write(ctrl::UNSEAL_KEY)?;
        self.control_write(ctrl::UNSEAL_KEY)
    }

    /// Return to SEALED mode.
    pub fn seal(&mut self) -> Result<(), Error<I2C::Error>> {
        self.control_write(ctrl::SEALED)
    }

    /// SET_CFGUPDATE (unsealed). Poll `in_config_update` every returned
    /// interval until true (≤ 1 s).
    pub fn enter_config_update(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.control_write(ctrl::SET_CFGUPDATE)?;
        Ok(CFG_UPDATE_POLL_US)
    }

    /// SOFT_RESET: leave CONFIG UPDATE mode and apply the new data. Poll
    /// `in_config_update` every returned interval until false (≤ 1 s).
    pub fn exit_config_update(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.control_write(ctrl::SOFT_RESET)?;
        Ok(CFG_UPDATE_POLL_US)
    }

    /// Select data memory `class` / 32-byte `block` (BlockDataControl,
    /// DataClass, DataBlock).
    pub fn select_data_block(&mut self, class: u8, block: u8) -> Result<(), Error<I2C::Error>> {
        self.write_bytes(cmd::BLOCK_DATA_CONTROL, &[0x00])?;
        self.write_bytes(cmd::DATA_CLASS, &[class])?;
        self.write_bytes(cmd::DATA_BLOCK, &[block])
    }

    /// Read the selected 32-byte block (unsealed).
    pub fn read_data_block(&mut self) -> Result<[u8; 32], Error<I2C::Error>> {
        let mut block = [0; 32];
        self.read_bytes(cmd::BLOCK_DATA, &mut block)?;
        Ok(block)
    }

    /// Read `buf.len()` bytes of data memory at `offset` in `class`
    /// (unsealed; must not cross a 32-byte block).
    pub fn read_data_memory(
        &mut self,
        class: u8,
        offset: u8,
        buf: &mut [u8],
    ) -> Result<(), Error<I2C::Error>> {
        if !check_range(offset, buf.len()) {
            return Err(Error::InvalidData);
        }
        self.select_data_block(class, offset / 32)?;
        self.read_bytes(cmd::BLOCK_DATA + offset % 32, buf)
    }

    /// Write data memory (CONFIG UPDATE mode, `NotReady` otherwise): read
    /// the block, patch it, write the bytes, then the new checksum, which
    /// commits the block. Multi-byte parameters are big-endian.
    pub fn write_data_memory(
        &mut self,
        class: u8,
        offset: u8,
        data: &[u8],
    ) -> Result<(), Error<I2C::Error>> {
        if !check_range(offset, data.len()) {
            return Err(Error::InvalidData);
        }
        let lo = (offset % 32) as usize;
        self.update_block(class, offset / 32, |block| {
            block[lo..lo + data.len()].copy_from_slice(data);
            (lo, lo + data.len())
        })
    }

    /// Write Design Capacity, Design Energy, Terminate Voltage and Taper
    /// Rate in one block update (CONFIG UPDATE mode).
    pub fn configure_battery(&mut self, config: &BatteryConfig) -> Result<(), Error<I2C::Error>> {
        let variant = self.variant;
        self.update_block(STATE_SUBCLASS, 0, |block| config.apply(variant, block))
    }

    fn update_block(
        &mut self,
        class: u8,
        block_index: u8,
        patch: impl FnOnce(&mut [u8; 32]) -> (usize, usize),
    ) -> Result<(), Error<I2C::Error>> {
        if !self.in_config_update()? {
            return Err(Error::NotReady);
        }
        self.select_data_block(class, block_index)?;
        let mut block = self.read_data_block()?;
        let (lo, hi) = patch(&mut block);
        self.write_bytes(cmd::BLOCK_DATA + lo as u8, &block[lo..hi])?;
        self.write_bytes(cmd::BLOCK_DATA_CHECKSUM, &[block_checksum(&block)])
    }

    /// Host temperature for gauging (used only with OpConfig.TEMPS = 1).
    pub fn write_temperature(&mut self, centi: i32) -> Result<(), Error<I2C::Error>> {
        let raw = centi_to_kelvin_tenths(centi);
        self.write_bytes(cmd::TEMPERATURE, &raw.to_le_bytes())
    }

    /// RESET (unsealed): back to ROM defaults, sets Flags.ITPOR.
    pub fn reset(&mut self) -> Result<(), Error<I2C::Error>> {
        self.control_write(ctrl::RESET)
    }

    /// SHUTDOWN_ENABLE + SHUTDOWN (unsealed): 0.6 µA, RAM lost. Exit by
    /// driving GPOUT low → high for ≥ 200 µs.
    pub fn shutdown(&mut self) -> Result<(), Error<I2C::Error>> {
        self.control_write(ctrl::SHUTDOWN_ENABLE)?;
        self.control_write(ctrl::SHUTDOWN)
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::Sensor for Bq27441<I2C> {
    type BusError = I2C::Error;
    type Measurement = Measurement;

    /// The gauge runs autonomously; nothing to start.
    fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        Ok(0)
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        let mut b = [0; BURST_LEN];
        self.read_bytes(BURST_START, &mut b)?;
        Ok(Measurement::from_burst(&b))
    }
}

impl<I2C: embedded_hal::i2c::I2c> zigbee_sensor::blocking::LowPower for Bq27441<I2C> {
    type BusError = I2C::Error;

    /// SET_HIBERNATE: the gauge enters HIBERNATE (9 µA) once it has a
    /// valid OCV and |AverageCurrent| < Hibernate Current; SLEEP (21 µA)
    /// is automatic below Sleep Current.
    fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
        self.control_write(ctrl::SET_HIBERNATE)
    }

    /// CLEAR_HIBERNATE; returns one measurement period.
    fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
        self.control_write(ctrl::CLEAR_HIBERNATE)?;
        Ok(1_000_000)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        ADDRESS, BURST_LEN, BURST_START, BatteryConfig, CFG_UPDATE_POLL_US, FLAGS_CFGUPMODE,
        FLAGS_ITPOR, Measurement, STATE_SUBCLASS, Variant, block_checksum, centi_to_kelvin_tenths,
        check_range, cmd, ctrl,
    };
    use embedded_hal_async::i2c::I2c;
    use zigbee_sensor::{Error, WaitUs};

    /// Async twin of [`super::Bq27441`].
    #[derive(Debug)]
    pub struct Bq27441<I2C> {
        i2c: I2C,
        variant: Variant,
        byte_writes: bool,
    }

    constructors!(Bq27441);

    impl<I2C: I2c> Bq27441<I2C> {
        /// See [`super::Bq27441::write_bytes`].
        pub async fn write_bytes(&mut self, cmd: u8, data: &[u8]) -> Result<(), Error<I2C::Error>> {
            if data.len() > 32 {
                return Err(Error::InvalidData);
            }
            if self.byte_writes {
                for (i, b) in data.iter().enumerate() {
                    self.i2c
                        .write(ADDRESS, &[cmd + i as u8, *b])
                        .await
                        .map_err(Error::bus)?;
                }
                Ok(())
            } else {
                let mut buf = [0; 33];
                buf[0] = cmd;
                buf[1..=data.len()].copy_from_slice(data);
                self.i2c
                    .write(ADDRESS, &buf[..=data.len()])
                    .await
                    .map_err(Error::bus)
            }
        }

        /// See [`super::Bq27441::read_bytes`].
        pub async fn read_bytes(
            &mut self,
            cmd: u8,
            buf: &mut [u8],
        ) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write_read(ADDRESS, &[cmd], buf)
                .await
                .map_err(Error::bus)
        }

        /// See [`super::Bq27441::read_word`].
        pub async fn read_word(&mut self, cmd: u8) -> Result<u16, Error<I2C::Error>> {
            let mut buf = [0; 2];
            self.read_bytes(cmd, &mut buf).await?;
            Ok(u16::from_le_bytes(buf))
        }

        /// See [`super::Bq27441::control_write`].
        pub async fn control_write(&mut self, sub: u16) -> Result<(), Error<I2C::Error>> {
            self.write_bytes(cmd::CONTROL, &sub.to_le_bytes()).await
        }

        /// See [`super::Bq27441::control`].
        pub async fn control(&mut self, sub: u16) -> Result<u16, Error<I2C::Error>> {
            self.control_write(sub).await?;
            self.read_word(cmd::CONTROL).await
        }

        /// See [`super::Bq27441::probe`].
        pub async fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
            let id = self.control(ctrl::DEVICE_TYPE).await?;
            if self.variant.accepts_device_type(id) {
                Ok(())
            } else {
                Err(Error::UnexpectedId(id as u32))
            }
        }

        /// See [`super::Bq27441::control_status`].
        pub async fn control_status(&mut self) -> Result<u16, Error<I2C::Error>> {
            self.control(ctrl::CONTROL_STATUS).await
        }

        /// See [`super::Bq27441::fw_version`].
        pub async fn fw_version(&mut self) -> Result<u16, Error<I2C::Error>> {
            self.control(ctrl::FW_VERSION).await
        }

        /// See [`super::Bq27441::dm_code`].
        pub async fn dm_code(&mut self) -> Result<u16, Error<I2C::Error>> {
            self.control(ctrl::DM_CODE).await
        }

        /// See [`super::Bq27441::flags`].
        pub async fn flags(&mut self) -> Result<u16, Error<I2C::Error>> {
            self.read_word(cmd::FLAGS).await
        }

        /// See [`super::Bq27441::needs_config`].
        pub async fn needs_config(&mut self) -> Result<bool, Error<I2C::Error>> {
            Ok(self.flags().await? & FLAGS_ITPOR != 0)
        }

        /// See [`super::Bq27441::in_config_update`].
        pub async fn in_config_update(&mut self) -> Result<bool, Error<I2C::Error>> {
            Ok(self.flags().await? & FLAGS_CFGUPMODE != 0)
        }

        /// See [`super::Bq27441::unseal`].
        pub async fn unseal(&mut self) -> Result<(), Error<I2C::Error>> {
            self.control_write(ctrl::UNSEAL_KEY).await?;
            self.control_write(ctrl::UNSEAL_KEY).await
        }

        /// See [`super::Bq27441::seal`].
        pub async fn seal(&mut self) -> Result<(), Error<I2C::Error>> {
            self.control_write(ctrl::SEALED).await
        }

        /// See [`super::Bq27441::enter_config_update`].
        pub async fn enter_config_update(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.control_write(ctrl::SET_CFGUPDATE).await?;
            Ok(CFG_UPDATE_POLL_US)
        }

        /// See [`super::Bq27441::exit_config_update`].
        pub async fn exit_config_update(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.control_write(ctrl::SOFT_RESET).await?;
            Ok(CFG_UPDATE_POLL_US)
        }

        /// See [`super::Bq27441::select_data_block`].
        pub async fn select_data_block(
            &mut self,
            class: u8,
            block: u8,
        ) -> Result<(), Error<I2C::Error>> {
            self.write_bytes(cmd::BLOCK_DATA_CONTROL, &[0x00]).await?;
            self.write_bytes(cmd::DATA_CLASS, &[class]).await?;
            self.write_bytes(cmd::DATA_BLOCK, &[block]).await
        }

        /// See [`super::Bq27441::read_data_block`].
        pub async fn read_data_block(&mut self) -> Result<[u8; 32], Error<I2C::Error>> {
            let mut block = [0; 32];
            self.read_bytes(cmd::BLOCK_DATA, &mut block).await?;
            Ok(block)
        }

        /// See [`super::Bq27441::read_data_memory`].
        pub async fn read_data_memory(
            &mut self,
            class: u8,
            offset: u8,
            buf: &mut [u8],
        ) -> Result<(), Error<I2C::Error>> {
            if !check_range(offset, buf.len()) {
                return Err(Error::InvalidData);
            }
            self.select_data_block(class, offset / 32).await?;
            self.read_bytes(cmd::BLOCK_DATA + offset % 32, buf).await
        }

        /// See [`super::Bq27441::write_data_memory`].
        pub async fn write_data_memory(
            &mut self,
            class: u8,
            offset: u8,
            data: &[u8],
        ) -> Result<(), Error<I2C::Error>> {
            if !check_range(offset, data.len()) {
                return Err(Error::InvalidData);
            }
            let lo = (offset % 32) as usize;
            self.update_block(class, offset / 32, |block| {
                block[lo..lo + data.len()].copy_from_slice(data);
                (lo, lo + data.len())
            })
            .await
        }

        /// See [`super::Bq27441::configure_battery`].
        pub async fn configure_battery(
            &mut self,
            config: &BatteryConfig,
        ) -> Result<(), Error<I2C::Error>> {
            let variant = self.variant;
            self.update_block(STATE_SUBCLASS, 0, |block| config.apply(variant, block))
                .await
        }

        async fn update_block(
            &mut self,
            class: u8,
            block_index: u8,
            patch: impl FnOnce(&mut [u8; 32]) -> (usize, usize),
        ) -> Result<(), Error<I2C::Error>> {
            if !self.in_config_update().await? {
                return Err(Error::NotReady);
            }
            self.select_data_block(class, block_index).await?;
            let mut block = self.read_data_block().await?;
            let (lo, hi) = patch(&mut block);
            self.write_bytes(cmd::BLOCK_DATA + lo as u8, &block[lo..hi])
                .await?;
            self.write_bytes(cmd::BLOCK_DATA_CHECKSUM, &[block_checksum(&block)])
                .await
        }

        /// See [`super::Bq27441::write_temperature`].
        pub async fn write_temperature(&mut self, centi: i32) -> Result<(), Error<I2C::Error>> {
            let raw = centi_to_kelvin_tenths(centi);
            self.write_bytes(cmd::TEMPERATURE, &raw.to_le_bytes()).await
        }

        /// See [`super::Bq27441::reset`].
        pub async fn reset(&mut self) -> Result<(), Error<I2C::Error>> {
            self.control_write(ctrl::RESET).await
        }

        /// See [`super::Bq27441::shutdown`].
        pub async fn shutdown(&mut self) -> Result<(), Error<I2C::Error>> {
            self.control_write(ctrl::SHUTDOWN_ENABLE).await?;
            self.control_write(ctrl::SHUTDOWN).await
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::Sensor for Bq27441<I2C> {
        type BusError = I2C::Error;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            Ok(0)
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let mut b = [0; BURST_LEN];
            self.read_bytes(BURST_START, &mut b).await?;
            Ok(Measurement::from_burst(&b))
        }
    }

    impl<I2C: I2c> zigbee_sensor::asynch::LowPower for Bq27441<I2C> {
        type BusError = I2C::Error;

        async fn sleep(&mut self) -> Result<(), Error<I2C::Error>> {
            self.control_write(ctrl::SET_HIBERNATE).await
        }

        async fn wake(&mut self) -> Result<WaitUs, Error<I2C::Error>> {
            self.control_write(ctrl::CLEAR_HIBERNATE).await?;
            Ok(1_000_000)
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

    fn ctl(sub: u16) -> T {
        let [l, h] = sub.to_le_bytes();
        T::write(ADDRESS, vec![cmd::CONTROL, l, h])
    }

    fn rd(cmd: u8, value: u16) -> T {
        T::write_read(ADDRESS, vec![cmd], value.to_le_bytes().to_vec())
    }

    fn burst() -> Vec<u8> {
        let mut b = vec![0u8; 32];
        let mut put = |cmd: u8, v: u16| {
            let i = (cmd - 2) as usize;
            b[i..i + 2].copy_from_slice(&v.to_le_bytes());
        };
        put(cmd::TEMPERATURE, 2_982); // 25.05 °C
        put(cmd::VOLTAGE, 3_850);
        put(cmd::FLAGS, FLAGS_BAT_DET | FLAGS_DSG);
        put(cmd::REMAINING_CAPACITY, 600);
        put(cmd::FULL_CHARGE_CAPACITY, 1_180);
        put(cmd::AVERAGE_CURRENT, (-150i16) as u16);
        put(cmd::AVERAGE_POWER, (-577i16) as u16);
        put(cmd::STATE_OF_CHARGE, 51);
        put(cmd::STATE_OF_HEALTH, 0x0362);
        b
    }

    #[test]
    fn conversions() {
        assert_eq!(kelvin_tenths_to_centi(2_982), 2_505);
        assert_eq!(centi_to_kelvin_tenths(2_505), 2_982);
        assert_eq!(centi_to_kelvin_tenths(-30_000), 0);
        let c = BatteryConfig::new(1_200, 3_000, 60);
        assert_eq!(
            (c.design_energy_mwh, c.terminate_voltage_mv, c.taper_rate),
            (4_440, 3_000, 200)
        );
        assert_eq!(BatteryConfig::new(9_000, 2_000, 1).taper_rate, 2_000);
        // TRM example: Design Capacity 1000 → 1200 mAh at offset 10 gives
        // checksum 0x1F when the old checksum was 0xE8 (LSB = checksum).
        let mut block = [0u8; 32];
        block[10..12].copy_from_slice(&1_000u16.to_be_bytes());
        block[0] = 0x2C;
        assert_eq!(block_checksum(&block), 0xE8);
        block[10..12].copy_from_slice(&1_200u16.to_be_bytes());
        assert_eq!(block_checksum(&block), 0x1F);
    }

    #[test]
    fn probe_and_measure() {
        let expectations = [
            ctl(ctrl::DEVICE_TYPE),
            rd(cmd::CONTROL, 0x0421),
            T::write_read(ADDRESS, vec![cmd::TEMPERATURE], burst()),
            ctl(ctrl::SET_HIBERNATE),
            ctl(ctrl::CLEAR_HIBERNATE),
            ctl(ctrl::DEVICE_TYPE),
            rd(cmd::CONTROL, 0x0426),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut gauge = Bq27441::new(&mut i2c, Variant::Bq27421);
        gauge.probe().unwrap();
        assert_eq!(gauge.start_measurement(), Ok(0));
        assert_eq!(
            gauge.read_measurement().unwrap(),
            Measurement {
                voltage: 3_850,
                current: -150_000,
                power: -577,
                percent: 102,
                temperature: 2_505,
                remaining_capacity_mah: 600,
                full_charge_capacity_mah: 1_180,
                state_of_health: Some(98),
                flags: FLAGS_BAT_DET | FLAGS_DSG,
            }
        );
        gauge.sleep().unwrap();
        assert_eq!(gauge.wake(), Ok(1_000_000));
        assert_eq!(gauge.probe(), Err(Error::UnexpectedId(0x0426)));
        i2c.done();
    }

    #[test]
    fn design_capacity_update() {
        let mut block = vec![0u8; 32];
        block[10..12].copy_from_slice(&1_000u16.to_be_bytes());
        let mut new = [0u8; 32];
        new.copy_from_slice(&block);
        let cfg = BatteryConfig::new(1_200, 3_200, 60);
        let (lo, hi) = cfg.apply(Variant::Bq27441, &mut new);
        assert_eq!((lo, hi), (10, 29));
        let mut payload = vec![cmd::BLOCK_DATA + 10];
        payload.extend_from_slice(&new[10..29]);
        let expectations = [
            ctl(ctrl::UNSEAL_KEY),
            ctl(ctrl::UNSEAL_KEY),
            ctl(ctrl::SET_CFGUPDATE),
            rd(cmd::FLAGS, 0),
            rd(cmd::FLAGS, FLAGS_CFGUPMODE | FLAGS_ITPOR),
            T::write(ADDRESS, vec![cmd::BLOCK_DATA_CONTROL, 0x00]),
            T::write(ADDRESS, vec![cmd::DATA_CLASS, STATE_SUBCLASS]),
            T::write(ADDRESS, vec![cmd::DATA_BLOCK, 0x00]),
            T::write_read(ADDRESS, vec![cmd::BLOCK_DATA], block),
            T::write(ADDRESS, payload),
            T::write(
                ADDRESS,
                vec![cmd::BLOCK_DATA_CHECKSUM, block_checksum(&new)],
            ),
            ctl(ctrl::SOFT_RESET),
            rd(cmd::FLAGS, 0),
            ctl(ctrl::SEALED),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut gauge = Bq27441::new(&mut i2c, Variant::Bq27441);
        gauge.unseal().unwrap();
        assert_eq!(gauge.enter_config_update(), Ok(CFG_UPDATE_POLL_US));
        assert_eq!(gauge.configure_battery(&cfg), Err(Error::NotReady));
        gauge.configure_battery(&cfg).unwrap();
        assert_eq!(gauge.exit_config_update(), Ok(CFG_UPDATE_POLL_US));
        assert!(!gauge.in_config_update().unwrap());
        gauge.seal().unwrap();
        i2c.done();
    }

    #[test]
    fn byte_writes_and_bounds() {
        let expectations = [
            T::write(ADDRESS, vec![cmd::CONTROL, 0x13]),
            T::write(ADDRESS, vec![cmd::CONTROL + 1, 0x00]),
            T::write(ADDRESS, vec![cmd::TEMPERATURE, 0xA6]),
            T::write(ADDRESS, vec![cmd::TEMPERATURE + 1, 0x0B]),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut gauge = Bq27441::new(&mut i2c, Variant::Bq27426).with_byte_writes(true);
        gauge.enter_config_update().unwrap();
        gauge.write_temperature(2_505).unwrap();
        assert_eq!(
            gauge.write_data_memory(82, 30, &[1, 2, 3]),
            Err(Error::InvalidData)
        );
        let mut buf = [0; 0];
        assert_eq!(
            gauge.read_data_memory(82, 0, &mut buf),
            Err(Error::InvalidData)
        );
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

        let mut b = burst();
        b[30] = 0; // SOH status 0: not valid yet.
        b[31] = 0;
        let expectations = [T::write_read(ADDRESS, vec![cmd::TEMPERATURE], b)];
        let mut i2c = I2cMock::new(&expectations);
        let mut gauge = asynch::Bq27441::new(&mut i2c, Variant::Bq27426);
        let mut delay = RecordingDelay(0);
        let m = block_on(gauge.measure(&mut delay)).unwrap();
        assert_eq!(
            (m.voltage, m.percent, m.state_of_health),
            (3_850, 102, None)
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
