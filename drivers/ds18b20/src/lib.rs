//! DS18B20, DS18S20 (DS1820), DS1822 and MAX31820 1-Wire digital
//! thermometers.
//!
//! The driver is generic over the [`zigbee_onewire::OneWire`] bus trait
//! (blocking) or [`zigbee_onewire::asynch::OneWire`] (async), so it runs on
//! the bit-banged GPIO master as well as on a hardware 1-Wire master.
//!
//! Reference: Maxim DS18B20 datasheet rev 042208 (memory map, function
//! commands, conversion times table "tCONV"), DS18S20 datasheet (extended
//! resolution with COUNT_REMAIN / COUNT_PER_C), MAX31820 datasheet (same
//! family code 28h and scratchpad as DS18B20).
//!
//! | Family | Code | Resolution | Conversion time |
//! |---|---|---|---|
//! | DS18B20, MAX31820 | 28h | 9..12 bit | 94 / 188 / 375 / 750 ms |
//! | DS1822 | 22h | 9..12 bit | 94 / 188 / 375 / 750 ms |
//! | DS18S20, DS1820 | 10h | 9 bit (+ COUNT_REMAIN) | 750 ms |
//!
//! # Usage
//!
//! [`Sensor::start_measurement`](zigbee_sensor::blocking::Sensor) selects
//! the device (Match ROM, or Skip ROM for [`Ds18b20::new_skip_rom`]) and
//! issues Convert T; it returns the conversion time for the configured
//! resolution. `read_measurement` reads the 9-byte scratchpad and checks its
//! CRC. Several sensors on one bus can be converted at once with
//! [`Ds18b20::start_all`] and read one by one with [`Ds18b20::read_device`].
//!
//! # Power-on value
//!
//! The temperature register resets to +85 °C. Reading it before any
//! conversion completed (a brown-out reset during the conversion, or a read
//! that came too early) returns [`Error::InvalidData`]. A genuine reading of
//! exactly 85.0000 °C is therefore also rejected; this is the usual trade-off
//! for this part.
//!
//! # Parasite power
//!
//! A parasite-powered device (VDD tied to GND) draws its conversion current
//! from DQ, so the master must drive DQ high with a strong pull-up within
//! 10 µs after Convert T / Copy Scratchpad and hold it for the whole
//! conversion. Enable this with [`Ds18b20::with_parasite_power`]: the driver
//! then calls [`OneWire::strong_pullup`]`(true)` after those commands and
//! releases it before the next bus operation. The bus master must support
//! it (the plain `BitBang` master returns [`Error::Unsupported`]). Use
//! [`Ds18b20::read_power_supply`] to detect parasite devices; polling
//! with [`Ds18b20::conversion_done`] does not work in parasite mode.

#![no_std]

use zigbee_onewire::{OneWire, Rom};
use zigbee_sensor::{Error, Quantity, Readings, WaitUs, crc::crc8_maxim};

pub use zigbee_onewire;

/// Function command: start a temperature conversion.
pub const CMD_CONVERT_T: u8 = 0x44;
/// Function command: write TH, TL (and configuration on DS18B20/DS1822).
pub const CMD_WRITE_SCRATCHPAD: u8 = 0x4E;
/// Function command: read the 9-byte scratchpad.
pub const CMD_READ_SCRATCHPAD: u8 = 0xBE;
/// Function command: copy TH, TL, configuration to EEPROM.
pub const CMD_COPY_SCRATCHPAD: u8 = 0x48;
/// Function command: recall TH, TL, configuration from EEPROM.
pub const CMD_RECALL_E2: u8 = 0xB8;
/// Function command: parasite-powered devices answer with a 0 read slot.
pub const CMD_READ_POWER_SUPPLY: u8 = 0xB4;

/// EEPROM write time (tWR, 10 ms max).
pub const COPY_SCRATCHPAD_WAIT_US: WaitUs = 10_000;

/// Device family, from the first byte of the ROM code.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Family {
    /// DS18B20 or MAX31820 (28h), ±0.5 °C, 9..12-bit.
    Ds18b20,
    /// DS1822 (22h), ±2 °C, 9..12-bit.
    Ds1822,
    /// DS18S20 / DS1820 (10h), 9-bit with COUNT_REMAIN extension.
    Ds18s20,
}

impl Family {
    /// Family for a ROM family code.
    pub const fn from_code(code: u8) -> Option<Self> {
        match code {
            0x28 => Some(Self::Ds18b20),
            0x22 => Some(Self::Ds1822),
            0x10 => Some(Self::Ds18s20),
            _ => None,
        }
    }

    /// ROM family code.
    pub const fn code(self) -> u8 {
        match self {
            Self::Ds18b20 => 0x28,
            Self::Ds1822 => 0x22,
            Self::Ds18s20 => 0x10,
        }
    }

    /// Whether the resolution is configurable.
    pub const fn has_config(self) -> bool {
        !matches!(self, Self::Ds18s20)
    }

    fn from_rom<E>(rom: &Rom) -> Result<Self, Error<E>> {
        Self::from_code(rom.family()).ok_or(Error::UnexpectedId(u32::from(rom.family())))
    }
}

/// Conversion resolution (configuration register R1:R0).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Resolution {
    /// 0.5 °C, 93.75 ms.
    Bits9 = 0,
    /// 0.25 °C, 187.5 ms.
    Bits10 = 1,
    /// 0.125 °C, 375 ms.
    Bits11 = 2,
    /// 0.0625 °C, 750 ms (power-on default).
    Bits12 = 3,
}

impl Resolution {
    /// Configuration register value (`0 R1 R0 1 1 1 1 1`).
    pub const fn config_byte(self) -> u8 {
        ((self as u8) << 5) | 0x1F
    }

    /// Resolution encoded in a configuration register value.
    pub const fn from_config(config: u8) -> Self {
        match (config >> 5) & 3 {
            0 => Self::Bits9,
            1 => Self::Bits10,
            2 => Self::Bits11,
            _ => Self::Bits12,
        }
    }

    /// Maximum conversion time (tCONV) in µs.
    pub const fn conversion_us(self) -> WaitUs {
        match self {
            Self::Bits9 => 94_000,
            Self::Bits10 => 188_000,
            Self::Bits11 => 375_000,
            Self::Bits12 => 750_000,
        }
    }
}

/// The 9-byte scratchpad, CRC already verified.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Scratchpad(pub [u8; 9]);

impl Scratchpad {
    /// Check the CRC of a raw scratchpad read. An all-zero read (DQ shorted
    /// low, which passes the CRC) is [`Error::InvalidData`].
    pub fn check<E>(bytes: [u8; 9]) -> Result<Self, Error<E>> {
        if bytes == [0; 9] {
            return Err(Error::InvalidData);
        }
        if crc8_maxim(&bytes[..8]) != bytes[8] {
            return Err(Error::Crc);
        }
        Ok(Self(bytes))
    }

    /// Raw temperature register.
    pub const fn raw_temperature(&self) -> i16 {
        i16::from_le_bytes([self.0[0], self.0[1]])
    }

    /// TH alarm register (or user byte 1).
    pub const fn th(&self) -> i8 {
        self.0[2] as i8
    }

    /// TL alarm register (or user byte 2).
    pub const fn tl(&self) -> i8 {
        self.0[3] as i8
    }

    /// Configuration register (DS18B20 / DS1822).
    pub const fn config(&self) -> u8 {
        self.0[4]
    }

    /// Decode the temperature for a device family.
    pub fn temperature<E>(&self, family: Family) -> Result<Measurement, Error<E>> {
        decode(family, self)
    }
}

/// One temperature reading.
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

/// Signed division rounding half away from zero (`den > 0`).
const fn div_round(num: i32, den: i32) -> i32 {
    if num >= 0 {
        (num + den / 2) / den
    } else {
        (num - den / 2) / den
    }
}

/// Decode a scratchpad (datasheet "Temperature Register Format").
///
/// DS18B20 / DS1822: 1/16 °C two's complement, undefined low bits masked
/// according to the configured resolution. DS18S20: 0.5 °C with the
/// extended-resolution formula `T = TEMP_READ − 0.25 + (COUNT_PER_C −
/// COUNT_REMAIN) / COUNT_PER_C`. The +85 °C power-on value is
/// [`Error::InvalidData`].
pub fn decode<E>(family: Family, sp: &Scratchpad) -> Result<Measurement, Error<E>> {
    let raw = sp.raw_temperature();
    let temperature = match family {
        Family::Ds18b20 | Family::Ds1822 => {
            if raw == 0x0550 {
                return Err(Error::InvalidData);
            }
            let undefined = match Resolution::from_config(sp.config()) {
                Resolution::Bits9 => 0b111,
                Resolution::Bits10 => 0b11,
                Resolution::Bits11 => 0b1,
                Resolution::Bits12 => 0,
            };
            div_round(i32::from(raw & !undefined) * 25, 4)
        }
        Family::Ds18s20 => {
            if raw == 0x00AA {
                return Err(Error::InvalidData);
            }
            let count_remain = i32::from(sp.0[6]);
            let count_per_c = i32::from(sp.0[7]);
            if count_per_c == 0 || count_remain > count_per_c {
                i32::from(raw) * 50
            } else {
                i32::from(raw >> 1) * 100 - 25
                    + div_round((count_per_c - count_remain) * 100, count_per_c)
            }
        }
    };
    Ok(Measurement { temperature })
}

macro_rules! constructors {
    () => {
        /// Driver for the device with ROM code `rom` (Match ROM addressing).
        ///
        /// Fails with [`Error::Crc`] for a corrupted ROM code and
        /// [`Error::UnexpectedId`] (the family code) for a non-thermometer.
        pub fn new(bus: W, rom: Rom) -> Result<Self, Error<W::BusError>> {
            let rom = rom.check()?;
            let family = Family::from_rom(&rom)?;
            Ok(Self::build(bus, Some(rom), family))
        }

        /// Driver for the only device on the bus (Skip ROM addressing).
        pub const fn new_skip_rom(bus: W, family: Family) -> Self {
            Self::build(bus, None, family)
        }

        const fn build(bus: W, rom: Option<Rom>, family: Family) -> Self {
            Self {
                bus,
                rom,
                family,
                resolution: Resolution::Bits12,
                parasite: false,
                pullup_on: false,
            }
        }

        /// Resolution the device is configured for; used for the conversion
        /// wait. Default 12-bit (the power-on value, and the worst case). This
        /// does not write the device; see `set_resolution`.
        pub const fn with_resolution(mut self, resolution: Resolution) -> Self {
            self.resolution = resolution;
            self
        }

        /// Drive the strong pull-up during conversions and EEPROM writes.
        pub const fn with_parasite_power(mut self, parasite: bool) -> Self {
            self.parasite = parasite;
            self
        }

        /// ROM code (`None` with Skip ROM addressing).
        pub const fn rom(&self) -> Option<Rom> {
            self.rom
        }

        /// Device family.
        pub const fn family(&self) -> Family {
            self.family
        }

        /// Conversion time of `start_measurement` / `start_all`.
        pub const fn conversion_us(&self) -> WaitUs {
            match self.family {
                Family::Ds18s20 => Resolution::Bits12.conversion_us(),
                _ => self.resolution.conversion_us(),
            }
        }

        /// Give the bus back.
        pub fn release(self) -> W {
            self.bus
        }

        fn write_frame(&self, th: i8, tl: i8, config: u8) -> ([u8; 4], usize) {
            let frame = [CMD_WRITE_SCRATCHPAD, th as u8, tl as u8, config];
            (frame, if self.family.has_config() { 4 } else { 3 })
        }
    };
}

/// Blocking driver.
pub struct Ds18b20<W> {
    bus: W,
    rom: Option<Rom>,
    family: Family,
    resolution: Resolution,
    parasite: bool,
    pullup_on: bool,
}

impl<W: OneWire> Ds18b20<W> {
    constructors!();

    fn begin(&mut self, rom: Option<&Rom>) -> Result<(), Error<W::BusError>> {
        if self.pullup_on {
            self.pullup_on = false;
            self.bus.strong_pullup(false)?;
        }
        self.bus.select(rom)
    }

    fn command(&mut self, rom: Option<&Rom>, cmd: u8) -> Result<(), Error<W::BusError>> {
        self.begin(rom)?;
        self.bus.write_byte(cmd)
    }

    fn power_after(&mut self, cmd: u8) -> Result<(), Error<W::BusError>> {
        let rom = self.rom;
        self.command(rom.as_ref(), cmd)?;
        if self.parasite {
            self.bus.strong_pullup(true)?;
            self.pullup_on = true;
        }
        Ok(())
    }

    fn scratchpad_of(&mut self, rom: Option<&Rom>) -> Result<Scratchpad, Error<W::BusError>> {
        self.command(rom, CMD_READ_SCRATCHPAD)?;
        let mut buf = [0u8; 9];
        self.bus.read_bytes(&mut buf)?;
        Scratchpad::check(buf)
    }

    /// Read and CRC-check the scratchpad.
    pub fn read_scratchpad(&mut self) -> Result<Scratchpad, Error<W::BusError>> {
        let rom = self.rom;
        self.scratchpad_of(rom.as_ref())
    }

    fn write(&mut self, th: i8, tl: i8, config: u8) -> Result<(), Error<W::BusError>> {
        let (frame, len) = self.write_frame(th, tl, config);
        let rom = self.rom;
        self.begin(rom.as_ref())?;
        self.bus.write_bytes(&frame[..len])
    }

    /// Write the resolution to the scratchpad (TH/TL are preserved) and use
    /// it for the conversion wait. DS18S20: only [`Resolution::Bits9`]
    /// is accepted (no bus traffic), anything else is
    /// [`Error::Unsupported`]. Call [`Self::copy_scratchpad`] to keep it
    /// across power cycles.
    pub fn set_resolution(&mut self, resolution: Resolution) -> Result<(), Error<W::BusError>> {
        if !self.family.has_config() {
            return match resolution {
                Resolution::Bits9 => Ok(()),
                _ => Err(Error::Unsupported),
            };
        }
        let sp = self.read_scratchpad()?;
        self.write(sp.th(), sp.tl(), resolution.config_byte())?;
        self.resolution = resolution;
        Ok(())
    }

    /// Write the TH / TL alarm thresholds (whole °C) used by alarm search.
    pub fn set_alarm(&mut self, th: i8, tl: i8) -> Result<(), Error<W::BusError>> {
        let sp = self.read_scratchpad()?;
        self.write(th, tl, sp.config())
    }

    /// Copy TH, TL and configuration to EEPROM. Wait the returned time
    /// (strong pull-up held in parasite mode) before the next command.
    pub fn copy_scratchpad(&mut self) -> Result<WaitUs, Error<W::BusError>> {
        self.power_after(CMD_COPY_SCRATCHPAD)?;
        Ok(COPY_SCRATCHPAD_WAIT_US)
    }

    /// `true` if any addressed device is parasite-powered.
    pub fn read_power_supply(&mut self) -> Result<bool, Error<W::BusError>> {
        let rom = self.rom;
        self.command(rom.as_ref(), CMD_READ_POWER_SUPPLY)?;
        Ok(!self.bus.read_bit()?)
    }

    /// Poll the conversion status with one read slot. Valid only directly
    /// after `start_measurement` / `start_all` with no other bus traffic in
    /// between, and only for externally powered devices
    /// ([`Error::Unsupported`] in parasite mode).
    pub fn conversion_done(&mut self) -> Result<bool, Error<W::BusError>> {
        if self.parasite {
            return Err(Error::Unsupported);
        }
        self.bus.read_bit()
    }

    /// Start a conversion on every device of the bus (Skip ROM + Convert T).
    /// Returns the conversion time for this driver's resolution; configure
    /// the slowest resolution present on the bus.
    pub fn start_all(&mut self) -> Result<WaitUs, Error<W::BusError>> {
        let rom = self.rom.take();
        let result = self.power_after(CMD_CONVERT_T);
        self.rom = rom;
        result.map(|()| self.conversion_us())
    }

    /// Read the result of another device on the same bus (e.g. after
    /// [`Self::start_all`]). The family is taken from `rom`.
    pub fn read_device(&mut self, rom: &Rom) -> Result<Measurement, Error<W::BusError>> {
        let family = Family::from_rom(rom)?;
        self.scratchpad_of(Some(rom))?.temperature(family)
    }
}

impl<W: OneWire> zigbee_sensor::blocking::Sensor for Ds18b20<W> {
    type BusError = W::BusError;
    type Measurement = Measurement;

    fn start_measurement(&mut self) -> Result<WaitUs, Error<W::BusError>> {
        self.power_after(CMD_CONVERT_T)?;
        Ok(self.conversion_us())
    }

    fn read_measurement(&mut self) -> Result<Measurement, Error<W::BusError>> {
        let family = self.family;
        self.read_scratchpad()?.temperature(family)
    }
}

/// Async driver.
pub mod asynch {
    use super::{
        CMD_CONVERT_T, CMD_COPY_SCRATCHPAD, CMD_READ_POWER_SUPPLY, CMD_READ_SCRATCHPAD,
        CMD_WRITE_SCRATCHPAD, COPY_SCRATCHPAD_WAIT_US, Family, Measurement, Resolution, Scratchpad,
    };
    use zigbee_onewire::{Rom, asynch::OneWire};
    use zigbee_sensor::{Error, WaitUs};

    /// Async driver; same API as [`super::Ds18b20`].
    pub struct Ds18b20<W> {
        bus: W,
        rom: Option<Rom>,
        family: Family,
        resolution: Resolution,
        parasite: bool,
        pullup_on: bool,
    }

    impl<W: OneWire> Ds18b20<W> {
        constructors!();

        async fn begin(&mut self, rom: Option<&Rom>) -> Result<(), Error<W::BusError>> {
            if self.pullup_on {
                self.pullup_on = false;
                self.bus.strong_pullup(false).await?;
            }
            self.bus.select(rom).await
        }

        async fn command(&mut self, rom: Option<&Rom>, cmd: u8) -> Result<(), Error<W::BusError>> {
            self.begin(rom).await?;
            self.bus.write_byte(cmd).await
        }

        async fn power_after(&mut self, cmd: u8) -> Result<(), Error<W::BusError>> {
            let rom = self.rom;
            self.command(rom.as_ref(), cmd).await?;
            if self.parasite {
                self.bus.strong_pullup(true).await?;
                self.pullup_on = true;
            }
            Ok(())
        }

        async fn scratchpad_of(
            &mut self,
            rom: Option<&Rom>,
        ) -> Result<Scratchpad, Error<W::BusError>> {
            self.command(rom, CMD_READ_SCRATCHPAD).await?;
            let mut buf = [0u8; 9];
            self.bus.read_bytes(&mut buf).await?;
            Scratchpad::check(buf)
        }

        /// See [`super::Ds18b20::read_scratchpad`].
        pub async fn read_scratchpad(&mut self) -> Result<Scratchpad, Error<W::BusError>> {
            let rom = self.rom;
            self.scratchpad_of(rom.as_ref()).await
        }

        async fn write(&mut self, th: i8, tl: i8, config: u8) -> Result<(), Error<W::BusError>> {
            let (frame, len) = self.write_frame(th, tl, config);
            let rom = self.rom;
            self.begin(rom.as_ref()).await?;
            self.bus.write_bytes(&frame[..len]).await
        }

        /// See [`super::Ds18b20::set_resolution`].
        pub async fn set_resolution(
            &mut self,
            resolution: Resolution,
        ) -> Result<(), Error<W::BusError>> {
            if !self.family.has_config() {
                return match resolution {
                    Resolution::Bits9 => Ok(()),
                    _ => Err(Error::Unsupported),
                };
            }
            let sp = self.read_scratchpad().await?;
            self.write(sp.th(), sp.tl(), resolution.config_byte())
                .await?;
            self.resolution = resolution;
            Ok(())
        }

        /// See [`super::Ds18b20::set_alarm`].
        pub async fn set_alarm(&mut self, th: i8, tl: i8) -> Result<(), Error<W::BusError>> {
            let sp = self.read_scratchpad().await?;
            self.write(th, tl, sp.config()).await
        }

        /// See [`super::Ds18b20::copy_scratchpad`].
        pub async fn copy_scratchpad(&mut self) -> Result<WaitUs, Error<W::BusError>> {
            self.power_after(CMD_COPY_SCRATCHPAD).await?;
            Ok(COPY_SCRATCHPAD_WAIT_US)
        }

        /// See [`super::Ds18b20::read_power_supply`].
        pub async fn read_power_supply(&mut self) -> Result<bool, Error<W::BusError>> {
            let rom = self.rom;
            self.command(rom.as_ref(), CMD_READ_POWER_SUPPLY).await?;
            Ok(!self.bus.read_bit().await?)
        }

        /// See [`super::Ds18b20::conversion_done`].
        pub async fn conversion_done(&mut self) -> Result<bool, Error<W::BusError>> {
            if self.parasite {
                return Err(Error::Unsupported);
            }
            self.bus.read_bit().await
        }

        /// See [`super::Ds18b20::start_all`].
        pub async fn start_all(&mut self) -> Result<WaitUs, Error<W::BusError>> {
            let rom = self.rom.take();
            let result = self.power_after(CMD_CONVERT_T).await;
            self.rom = rom;
            result.map(|()| self.conversion_us())
        }

        /// See [`super::Ds18b20::read_device`].
        pub async fn read_device(&mut self, rom: &Rom) -> Result<Measurement, Error<W::BusError>> {
            let family = Family::from_rom(rom)?;
            self.scratchpad_of(Some(rom)).await?.temperature(family)
        }
    }

    impl<W: OneWire> zigbee_sensor::asynch::Sensor for Ds18b20<W> {
        type BusError = W::BusError;
        type Measurement = Measurement;

        async fn start_measurement(&mut self) -> Result<WaitUs, Error<W::BusError>> {
            self.power_after(CMD_CONVERT_T).await?;
            Ok(self.conversion_us())
        }

        async fn read_measurement(&mut self) -> Result<Measurement, Error<W::BusError>> {
            let family = self.family;
            self.read_scratchpad().await?.temperature(family)
        }
    }
}

#[cfg(test)]
mod tests;
