//! ROM search (Maxim application note 187 "1-Wire Search Algorithm").

use crate::{OneWire, Rom, Triplet, asynch, command};
use zigbee_sensor::Error;

/// State of a Search ROM / Alarm Search enumeration.
///
/// Call [`Self::next`] (or [`Self::next_async`]) until it returns
/// `Ok(None)`; every `Ok(Some(rom))` is one device with a valid CRC.
///
/// ```ignore
/// let mut search = Search::new();
/// while let Some(rom) = search.next(&mut bus)? {
///     // rom.family() == 0x28 → DS18B20 ...
/// }
/// ```
#[derive(Debug, Clone)]
pub struct Search {
    rom: [u8; 8],
    /// 1-based bit position of the last unexplored 0-branch; 0 = none.
    last_discrepancy: u8,
    last_family_discrepancy: u8,
    done: bool,
    command: u8,
    family: Option<u8>,
}

enum Step {
    Continue,
    NoDevice,
    Vanished,
}

impl Default for Search {
    fn default() -> Self {
        Self::new()
    }
}

impl Search {
    /// Enumerate every device (Search ROM, F0h).
    pub const fn new() -> Self {
        Self {
            rom: [0; 8],
            last_discrepancy: 0,
            last_family_discrepancy: 0,
            done: false,
            command: command::SEARCH_ROM,
            family: None,
        }
    }

    /// Enumerate devices whose alarm flag is set (Alarm Search, ECh).
    pub const fn alarm() -> Self {
        let mut s = Self::new();
        s.command = command::ALARM_SEARCH;
        s
    }

    /// Enumerate only devices of one family (AN187 "target setup").
    pub const fn family(code: u8) -> Self {
        let mut s = Self::new();
        s.rom[0] = code;
        s.last_discrepancy = 64;
        s.family = Some(code);
        s
    }

    /// Skip the remaining devices of the family just returned (AN187
    /// "family skip setup").
    pub fn skip_family(&mut self) {
        self.last_discrepancy = self.last_family_discrepancy;
        self.last_family_discrepancy = 0;
        if self.last_discrepancy == 0 {
            self.done = true;
        }
    }

    /// The enumeration has finished.
    pub const fn is_done(&self) -> bool {
        self.done
    }

    fn preferred(&self, bit: u8) -> bool {
        if bit < self.last_discrepancy {
            let i = bit - 1;
            self.rom[usize::from(i / 8)] >> (i % 8) & 1 != 0
        } else {
            bit == self.last_discrepancy
        }
    }

    fn step(&mut self, bit: u8, t: Triplet, last_zero: &mut u8) -> Step {
        if t.id_bit && t.complement_bit {
            return if bit == 1 {
                Step::NoDevice
            } else {
                Step::Vanished
            };
        }
        if !t.id_bit && !t.complement_bit && !t.direction {
            *last_zero = bit;
            if bit < 9 {
                self.last_family_discrepancy = bit;
            }
        }
        let i = bit - 1;
        let mask = 1 << (i % 8);
        let byte = &mut self.rom[usize::from(i / 8)];
        if t.direction {
            *byte |= mask;
        } else {
            *byte &= !mask;
        }
        Step::Continue
    }

    fn stop<E>(&mut self, step: Step) -> Result<Option<Rom>, Error<E>> {
        self.done = true;
        match step {
            Step::Vanished => Err(Error::InvalidData),
            _ => Ok(None),
        }
    }

    fn finish<E>(&mut self, last_zero: u8) -> Result<Option<Rom>, Error<E>> {
        self.last_discrepancy = last_zero;
        if last_zero == 0 {
            self.done = true;
        }
        let rom = match Rom(self.rom).check() {
            Ok(rom) => rom,
            Err(e) => {
                self.done = true;
                return Err(e);
            }
        };
        if self.family.is_some_and(|f| f != rom.family()) {
            self.done = true;
            return Ok(None);
        }
        Ok(Some(rom))
    }

    /// Find the next device. `Ok(None)` when no (more) devices answer.
    ///
    /// Errors: [`Error::Crc`] for a corrupted ROM code and
    /// [`Error::InvalidData`] if a device disappeared mid-search; both end
    /// the enumeration.
    pub fn next<W: OneWire + ?Sized>(
        &mut self,
        bus: &mut W,
    ) -> Result<Option<Rom>, Error<W::BusError>> {
        if self.done {
            return Ok(None);
        }
        if !bus.reset()? {
            return self.stop(Step::NoDevice);
        }
        bus.write_byte(self.command)?;
        let mut last_zero = 0;
        for bit in 1..=64 {
            let t = bus.triplet(self.preferred(bit))?;
            match self.step(bit, t, &mut last_zero) {
                Step::Continue => {}
                other => return self.stop(other),
            }
        }
        self.finish(last_zero)
    }

    /// Async twin of [`Self::next`].
    pub async fn next_async<W: asynch::OneWire + ?Sized>(
        &mut self,
        bus: &mut W,
    ) -> Result<Option<Rom>, Error<W::BusError>> {
        if self.done {
            return Ok(None);
        }
        if !bus.reset().await? {
            return self.stop(Step::NoDevice);
        }
        bus.write_byte(self.command).await?;
        let mut last_zero = 0;
        for bit in 1..=64 {
            let t = bus.triplet(self.preferred(bit)).await?;
            match self.step(bit, t, &mut last_zero) {
                Step::Continue => {}
                other => return self.stop(other),
            }
        }
        self.finish(last_zero)
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use crate::sim::{SimBus, block_on};
    use std::vec::Vec;
    use zigbee_sensor::crc::crc8_maxim;

    fn rom(family: u8, serial: u64) -> Rom {
        let mut b = [0u8; 8];
        b[0] = family;
        b[1..7].copy_from_slice(&serial.to_le_bytes()[..6]);
        b[7] = crc8_maxim(&b[..7]);
        Rom(b)
    }

    fn roms() -> [Rom; 5] {
        [
            rom(0x28, 0x0000_0316_643C),
            rom(0x28, 0x0000_0316_643D),
            rom(0x10, 0x0000_0008_0281),
            rom(0x22, 0x0000_A000_0001),
            rom(0x28, 0x0000_FFFF_FFFF),
        ]
    }

    fn collect(bus: &mut SimBus, mut search: Search) -> Vec<Rom> {
        let mut found = Vec::new();
        while let Some(rom) = search.next(bus).unwrap() {
            found.push(rom);
        }
        found.sort();
        found
    }

    #[test]
    fn finds_every_device_once() {
        let mut expected = roms().to_vec();
        expected.sort();
        let mut bus = SimBus::new(&roms());
        assert_eq!(collect(&mut bus, Search::new()), expected);
        // One reset per device found.
        assert_eq!(bus.resets, 5);
    }

    #[test]
    fn single_device_and_empty_bus() {
        let r = roms()[2];
        let mut bus = SimBus::new(&[r]);
        assert_eq!(collect(&mut bus, Search::new()), [r]);
        let mut empty = SimBus::new(&[]);
        assert_eq!(collect(&mut empty, Search::new()), []);
    }

    #[test]
    fn alarm_search_only_returns_alarming_devices() {
        let mut bus = SimBus::new(&roms()).with_alarm(1).with_alarm(3);
        let mut expected = [roms()[1], roms()[3]];
        expected.sort();
        assert_eq!(collect(&mut bus, Search::alarm()), expected);
        let mut quiet = SimBus::new(&roms());
        assert_eq!(collect(&mut quiet, Search::alarm()), []);
    }

    #[test]
    fn family_search_and_skip() {
        let mut bus = SimBus::new(&roms());
        let found = collect(&mut bus, Search::family(0x28));
        assert_eq!(found.len(), 3);
        assert!(found.iter().all(|r| r.family() == 0x28));
        assert!(collect(&mut bus, Search::family(0x3B)).is_empty());

        // Skipping the family of the first hit leaves only other families.
        let mut search = Search::new();
        let first = search.next(&mut bus).unwrap().unwrap();
        search.skip_family();
        let mut rest = Vec::new();
        while let Some(r) = search.next(&mut bus).unwrap() {
            rest.push(r);
        }
        assert!(rest.iter().all(|r| r.family() != first.family()));
        let others = roms()
            .iter()
            .filter(|r| r.family() != first.family())
            .count();
        assert_eq!(rest.len(), others);
    }

    #[test]
    fn corrupted_rom_is_reported() {
        let mut bad = roms()[0];
        bad.0[7] ^= 0xFF;
        let mut bus = SimBus::new(&[bad]);
        let mut search = Search::new();
        assert_eq!(search.next(&mut bus), Err(Error::Crc));
        assert!(search.is_done());
    }

    #[test]
    fn async_search_matches_blocking() {
        let mut bus = SimBus::new(&roms());
        let mut search = Search::new();
        let mut found = Vec::new();
        while let Some(r) = block_on(search.next_async(&mut bus)).unwrap() {
            found.push(r);
        }
        found.sort();
        let mut expected = roms().to_vec();
        expected.sort();
        assert_eq!(found, expected);
    }
}
