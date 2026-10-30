//! 1-Wire bus master.
//!
//! * [`OneWire`] — the bus operations (reset/presence, bit and byte I/O,
//!   search triplet, strong pull-up). Device drivers such as
//!   `zigbee-ds18b20` are generic over this trait, so a platform can plug in
//!   a hardware master (DS2482 bridge, UART-based 1-Wire, a vendor 1-Wire
//!   peripheral) instead of bit-banging. [`asynch::OneWire`] is the async
//!   twin.
//! * [`BitBang`] — a standard-speed master on one open-drain GPIO, using
//!   `DelayNs` for the microsecond slot timing. It implements both traits.
//! * ROM-level helpers ([`OneWire::select`], [`OneWire::read_rom`]) and the
//!   ROM / alarm search ([`Search`]) with CRC-8 checking.
//!
//! References: Maxim (Analog Devices) application note 126 "1-Wire
//! Communication Through Software" (standard-speed timing A..J), application
//! note 187 "1-Wire Search Algorithm", and the DS18B20 datasheet (rev
//! 042208) 1-Wire signalling section.
//!
//! # Timing and interrupts
//!
//! 1-Wire time slots are a few tens of microseconds and tolerate being
//! stretched only in some phases: the low pulse of a "write 1" must stay
//! below 15 µs and a read must sample within 15 µs of the falling edge. An
//! interrupt (radio, timer) inside these windows corrupts the bit. The
//! [`BitBang`] master therefore runs each timing-critical window (≤ 70 µs:
//! one write/read slot, or the presence-detect window of a reset) inside
//! [`InterruptMask::mask`]. **The caller provides the masking**, typically
//! with `critical_section::with`; the default [`NoMask`] does nothing and is
//! only safe when no interrupt can fire during a transaction. Recovery
//! times between slots run unmasked; stretching them is harmless.
//!
//! This is the one place in this repository where a driver uses a delay:
//! the guide allows µs bit timing for bit-banged protocols. Device-level
//! waits (temperature conversion, EEPROM copy) are still returned to the
//! caller as `WaitUs` by the device drivers.
//!
//! # Errors
//!
//! Every operation returns [`zigbee_sensor::Error`]:
//! * [`Error::Bus`] — the GPIO / hardware master failed;
//! * [`Error::Timeout`] — no presence pulse where a device was required;
//! * [`Error::Crc`] — a ROM code failed its CRC-8;
//! * [`Error::InvalidData`] — the bus stayed low after a reset (short
//!   circuit) or a device vanished in the middle of a search;
//! * [`Error::Unsupported`] — strong pull-up requested from a master that
//!   has none.

#![no_std]

mod bitbang;
mod search;

pub mod asynch;

pub use bitbang::{BitBang, InterruptMask, NoMask, timing};
pub use search::Search;

use zigbee_sensor::{Error, crc::crc8_maxim};

/// ROM (network-layer) command bytes, DS18B20 datasheet "ROM Commands".
pub mod command {
    /// Read the ROM code of the only device on the bus.
    pub const READ_ROM: u8 = 0x33;
    /// Address one device by its 64-bit ROM code.
    pub const MATCH_ROM: u8 = 0x55;
    /// Address all devices at once.
    pub const SKIP_ROM: u8 = 0xCC;
    /// Enumerate devices.
    pub const SEARCH_ROM: u8 = 0xF0;
    /// Enumerate devices with an alarm condition.
    pub const ALARM_SEARCH: u8 = 0xEC;
}

/// 64-bit 1-Wire ROM code: family code (byte 0), 48-bit serial number
/// (bytes 1..=6, LSB first) and CRC-8 (byte 7), in bus order.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Rom(pub [u8; 8]);

impl Rom {
    /// Build from the little-endian `u64` form (family code in the low byte),
    /// the way ROM codes are usually printed as `0xCC_SSSSSSSSSSSS_FF`.
    pub const fn from_u64(value: u64) -> Self {
        Self(value.to_le_bytes())
    }

    /// Little-endian `u64` form; see [`Self::from_u64`].
    pub const fn to_u64(self) -> u64 {
        u64::from_le_bytes(self.0)
    }

    /// Family code (device type), e.g. `0x28` for a DS18B20.
    pub const fn family(&self) -> u8 {
        self.0[0]
    }

    /// 48-bit serial number.
    pub const fn serial(&self) -> u64 {
        let b = self.0;
        u64::from_le_bytes([b[1], b[2], b[3], b[4], b[5], b[6], 0, 0])
    }

    /// The CRC-8 in byte 7 matches bytes 0..=6.
    pub fn crc_valid(&self) -> bool {
        crc8_maxim(&self.0[..7]) == self.0[7]
    }

    /// Check the CRC, mapping a mismatch to [`Error::Crc`].
    pub fn check<E>(self) -> Result<Self, Error<E>> {
        if self.crc_valid() {
            Ok(self)
        } else {
            Err(Error::Crc)
        }
    }
}

/// Result of one search step: two read slots, then one write slot.
///
/// This is the DS2482 "1-Wire Triplet" primitive; a hardware master can
/// override [`OneWire::triplet`] to do it in one command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Triplet {
    /// First read slot: AND of the current ROM bit of all participating
    /// devices.
    pub id_bit: bool,
    /// Second read slot: AND of the complemented bit.
    pub complement_bit: bool,
    /// Direction written in the third slot.
    pub direction: bool,
}

impl Triplet {
    /// Direction chosen for the read pair: the only bit present if the
    /// devices agree, otherwise `preferred`. Shared by the default blocking
    /// and async implementations.
    pub const fn direction_for(id_bit: bool, complement_bit: bool, preferred: bool) -> bool {
        if id_bit != complement_bit {
            id_bit
        } else {
            preferred
        }
    }
}

/// Blocking 1-Wire bus master.
///
/// Implementors provide [`reset`](Self::reset), [`write_bit`](Self::write_bit)
/// and [`read_bit`](Self::read_bit); everything else has a default built on
/// them that a hardware master may override. Bytes are sent LSB first.
pub trait OneWire {
    /// Transport error type.
    type BusError;

    /// Issue a reset pulse. Returns `true` if at least one device answered
    /// with a presence pulse.
    fn reset(&mut self) -> Result<bool, Error<Self::BusError>>;

    /// Write one time slot.
    fn write_bit(&mut self, bit: bool) -> Result<(), Error<Self::BusError>>;

    /// Read one time slot.
    fn read_bit(&mut self) -> Result<bool, Error<Self::BusError>>;

    /// Write one byte, LSB first.
    fn write_byte(&mut self, byte: u8) -> Result<(), Error<Self::BusError>> {
        for i in 0..8 {
            self.write_bit(byte >> i & 1 != 0)?;
        }
        Ok(())
    }

    /// Read one byte, LSB first.
    fn read_byte(&mut self) -> Result<u8, Error<Self::BusError>> {
        let mut byte = 0;
        for i in 0..8 {
            if self.read_bit()? {
                byte |= 1 << i;
            }
        }
        Ok(byte)
    }

    /// Write several bytes.
    fn write_bytes(&mut self, bytes: &[u8]) -> Result<(), Error<Self::BusError>> {
        for &b in bytes {
            self.write_byte(b)?;
        }
        Ok(())
    }

    /// Fill `buf` with read bytes.
    fn read_bytes(&mut self, buf: &mut [u8]) -> Result<(), Error<Self::BusError>> {
        for b in buf {
            *b = self.read_byte()?;
        }
        Ok(())
    }

    /// One search step: read the bit and its complement, then write the
    /// direction (the bit itself if all devices agree, else `preferred`).
    /// If both reads are `1` nobody participates and nothing is written.
    fn triplet(&mut self, preferred: bool) -> Result<Triplet, Error<Self::BusError>> {
        let id_bit = self.read_bit()?;
        let complement_bit = self.read_bit()?;
        let direction = Triplet::direction_for(id_bit, complement_bit, preferred);
        if !(id_bit && complement_bit) {
            self.write_bit(direction)?;
        }
        Ok(Triplet {
            id_bit,
            complement_bit,
            direction,
        })
    }

    /// Switch a strong pull-up on the data line on or off.
    ///
    /// Parasite-powered devices need it within 10 µs after a Convert T or
    /// Copy Scratchpad command and for the whole operation. The default
    /// returns [`Error::Unsupported`] when asked to enable it (a plain
    /// open-drain pin cannot source the current) and does nothing when
    /// asked to disable it.
    fn strong_pullup(&mut self, enable: bool) -> Result<(), Error<Self::BusError>> {
        if enable {
            Err(Error::Unsupported)
        } else {
            Ok(())
        }
    }

    /// Reset and require a presence pulse ([`Error::Timeout`] otherwise).
    fn reset_presence(&mut self) -> Result<(), Error<Self::BusError>> {
        if self.reset()? {
            Ok(())
        } else {
            Err(Error::Timeout)
        }
    }

    /// Reset, then address one device (Match ROM) or, with `None`, every
    /// device (Skip ROM). Follow with a function command.
    fn select(&mut self, rom: Option<&Rom>) -> Result<(), Error<Self::BusError>> {
        self.reset_presence()?;
        match rom {
            Some(rom) => {
                self.write_byte(command::MATCH_ROM)?;
                self.write_bytes(&rom.0)
            }
            None => self.write_byte(command::SKIP_ROM),
        }
    }

    /// Read the ROM code of the only device on the bus (Read ROM). With
    /// several devices the result is the AND of their codes and fails the
    /// CRC.
    fn read_rom(&mut self) -> Result<Rom, Error<Self::BusError>> {
        self.reset_presence()?;
        self.write_byte(command::READ_ROM)?;
        let mut rom = [0u8; 8];
        self.read_bytes(&mut rom)?;
        Rom(rom).check()
    }
}

impl<T: OneWire + ?Sized> OneWire for &mut T {
    type BusError = T::BusError;

    fn reset(&mut self) -> Result<bool, Error<Self::BusError>> {
        T::reset(self)
    }
    fn write_bit(&mut self, bit: bool) -> Result<(), Error<Self::BusError>> {
        T::write_bit(self, bit)
    }
    fn read_bit(&mut self) -> Result<bool, Error<Self::BusError>> {
        T::read_bit(self)
    }
    fn write_byte(&mut self, byte: u8) -> Result<(), Error<Self::BusError>> {
        T::write_byte(self, byte)
    }
    fn read_byte(&mut self) -> Result<u8, Error<Self::BusError>> {
        T::read_byte(self)
    }
    fn write_bytes(&mut self, bytes: &[u8]) -> Result<(), Error<Self::BusError>> {
        T::write_bytes(self, bytes)
    }
    fn read_bytes(&mut self, buf: &mut [u8]) -> Result<(), Error<Self::BusError>> {
        T::read_bytes(self, buf)
    }
    fn triplet(&mut self, preferred: bool) -> Result<Triplet, Error<Self::BusError>> {
        T::triplet(self, preferred)
    }
    fn strong_pullup(&mut self, enable: bool) -> Result<(), Error<Self::BusError>> {
        T::strong_pullup(self, enable)
    }
}

#[cfg(test)]
pub(crate) mod sim;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::SimBus;

    // DS18B20-style ROM with a valid CRC (core crate test vector).
    pub(crate) const ROM_A: Rom = Rom([0x28, 0xFF, 0x4C, 0x3C, 0x64, 0x16, 0x03, 0x00]);

    fn with_crc(mut bytes: [u8; 8]) -> Rom {
        bytes[7] = crc8_maxim(&bytes[..7]);
        Rom(bytes)
    }

    #[test]
    fn rom_fields() {
        let rom = with_crc(ROM_A.0);
        assert!(rom.crc_valid());
        assert_eq!(rom.family(), 0x28);
        assert_eq!(rom.serial(), 0x0316_643C_4CFF);
        assert_eq!(Rom::from_u64(rom.to_u64()), rom);
        let mut bad = rom;
        bad.0[3] ^= 0x10;
        assert_eq!(bad.check::<()>(), Err(Error::Crc));
    }

    #[test]
    fn read_rom_single_device() {
        let rom = with_crc(ROM_A.0);
        let mut bus = SimBus::new(&[rom]);
        assert_eq!(bus.read_rom(), Ok(rom));
    }

    #[test]
    fn read_rom_with_two_devices_fails_crc() {
        let a = with_crc(ROM_A.0);
        let b = with_crc([0x28, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0]);
        let mut bus = SimBus::new(&[a, b]);
        assert_eq!(bus.read_rom(), Err(Error::Crc));
    }

    #[test]
    fn select_without_devices_times_out() {
        let mut bus = SimBus::new(&[]);
        assert_eq!(bus.select(None), Err(Error::Timeout));
    }

    #[test]
    fn match_rom_addresses_one_device() {
        let a = with_crc(ROM_A.0);
        let b = with_crc([0x10, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0]);
        let mut bus = SimBus::new(&[a, b]);
        bus.select(Some(&b)).unwrap();
        assert_eq!(bus.selected(), [false, true]);
        bus.select(None).unwrap();
        assert_eq!(bus.selected(), [true, true]);
    }

    #[test]
    fn default_strong_pullup_is_unsupported() {
        let mut bus = SimBus::new(&[]);
        assert_eq!(bus.strong_pullup(true), Err(Error::Unsupported));
        assert_eq!(bus.strong_pullup(false), Ok(()));
    }
}
