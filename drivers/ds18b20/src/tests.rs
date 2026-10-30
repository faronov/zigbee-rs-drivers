extern crate std;

use super::*;
use std::collections::VecDeque;
use std::vec::Vec;
use zigbee_sensor::blocking::Sensor;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Ev {
    Reset(bool),
    W(u8),
    R(u8),
    RBit(bool),
    Pullup(bool),
}

/// Byte-level scripted bus.
struct Fake(VecDeque<Ev>);

impl Fake {
    fn new(script: Vec<Ev>) -> Self {
        Self(script.into())
    }

    fn next(&mut self) -> Ev {
        self.0.pop_front().expect("unexpected bus operation")
    }
}

impl Drop for Fake {
    fn drop(&mut self) {
        if !std::thread::panicking() {
            assert!(self.0.is_empty(), "unused script: {:?}", self.0);
        }
    }
}

impl OneWire for Fake {
    type BusError = ();
    fn reset(&mut self) -> Result<bool, Error<()>> {
        match self.next() {
            Ev::Reset(p) => Ok(p),
            e => panic!("reset, expected {e:?}"),
        }
    }
    fn write_bit(&mut self, _: bool) -> Result<(), Error<()>> {
        panic!("bit write");
    }
    fn read_bit(&mut self) -> Result<bool, Error<()>> {
        match self.next() {
            Ev::RBit(b) => Ok(b),
            e => panic!("read bit, expected {e:?}"),
        }
    }
    fn write_byte(&mut self, byte: u8) -> Result<(), Error<()>> {
        assert_eq!(self.next(), Ev::W(byte));
        Ok(())
    }
    fn read_byte(&mut self) -> Result<u8, Error<()>> {
        match self.next() {
            Ev::R(b) => Ok(b),
            e => panic!("read byte, expected {e:?}"),
        }
    }
    fn strong_pullup(&mut self, enable: bool) -> Result<(), Error<()>> {
        assert_eq!(self.next(), Ev::Pullup(enable));
        Ok(())
    }
}

impl zigbee_onewire::asynch::OneWire for Fake {
    type BusError = ();
    async fn reset(&mut self) -> Result<bool, Error<()>> {
        OneWire::reset(self)
    }
    async fn write_bit(&mut self, bit: bool) -> Result<(), Error<()>> {
        OneWire::write_bit(self, bit)
    }
    async fn read_bit(&mut self) -> Result<bool, Error<()>> {
        OneWire::read_bit(self)
    }
    async fn write_byte(&mut self, byte: u8) -> Result<(), Error<()>> {
        OneWire::write_byte(self, byte)
    }
    async fn read_byte(&mut self) -> Result<u8, Error<()>> {
        OneWire::read_byte(self)
    }
    async fn strong_pullup(&mut self, enable: bool) -> Result<(), Error<()>> {
        OneWire::strong_pullup(self, enable)
    }
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

fn rom(family: u8) -> Rom {
    let mut b = [family, 0xA2, 0x3C, 0x64, 0x16, 0x03, 0x00, 0];
    b[7] = crc8_maxim(&b[..7]);
    Rom(b)
}

/// Scratchpad bytes with a valid CRC.
fn sp(raw: u16, b4: u8, count_remain: u8, count_per_c: u8) -> [u8; 9] {
    let [lo, hi] = raw.to_le_bytes();
    let mut b = [lo, hi, 0x4B, 0x46, b4, 0xFF, count_remain, count_per_c, 0];
    b[8] = crc8_maxim(&b[..8]);
    b
}

fn select(rom: &Rom) -> Vec<Ev> {
    let mut v = std::vec![Ev::Reset(true), Ev::W(0x55)];
    v.extend(rom.0.iter().map(|&b| Ev::W(b)));
    v
}

fn read_sp(rom: &Rom, bytes: [u8; 9]) -> Vec<Ev> {
    let mut v = select(rom);
    v.push(Ev::W(0xBE));
    v.extend(bytes.iter().map(|&b| Ev::R(b)));
    v
}

fn b20(raw: u16, config: u8) -> Result<i32, Error<()>> {
    let sp = Scratchpad::check(sp(raw, config, 0x0C, 0x10))?;
    decode(Family::Ds18b20, &sp).map(|m| m.temperature)
}

fn s20(raw: u16, count_remain: u8, count_per_c: u8) -> Result<i32, Error<()>> {
    let sp = Scratchpad::check(sp(raw, 0xFF, count_remain, count_per_c))?;
    decode(Family::Ds18s20, &sp).map(|m| m.temperature)
}

#[test]
fn ds18b20_datasheet_table() {
    // DS18B20 datasheet table 1 "Temperature/Data Relationship".
    let table: [(u16, i32); 9] = [
        (0x07D0, 12_500),
        (0x0191, 2_506),
        (0x00A2, 1_013),
        (0x0008, 50),
        (0x0000, 0),
        (0xFFF8, -50),
        (0xFF5E, -1_013),
        (0xFE6F, -2_506),
        (0xFC90, -5_500),
    ];
    for (raw, centi) in table {
        assert_eq!(b20(raw, 0x7F), Ok(centi), "raw {raw:#06x}");
    }
    assert_eq!(b20(0x0550, 0x7F), Err(Error::InvalidData));
}

#[test]
fn low_resolution_masks_undefined_bits() {
    assert_eq!(b20(0x0191, Resolution::Bits9.config_byte()), Ok(2_500));
    assert_eq!(b20(0x0191, Resolution::Bits10.config_byte()), Ok(2_500));
    assert_eq!(b20(0x0193, Resolution::Bits11.config_byte()), Ok(2_513));
    assert_eq!(b20(0xFE6F, Resolution::Bits9.config_byte()), Ok(-2_550));
}

#[test]
fn ds18s20_table_and_extended_resolution() {
    // DS18S20 datasheet table 1, 9-bit values (COUNT_PER_C = 0 → plain).
    for (raw, centi) in [
        (0x0032, 2_500),
        (0x0001, 50),
        (0x0000, 0),
        (0xFFFF, -50),
        (0xFFCE, -2_500),
        (0xFF92, -5_500),
    ] {
        assert_eq!(s20(raw, 0, 0), Ok(centi), "raw {raw:#06x}");
    }
    assert_eq!(s20(0x00AA, 0x0C, 0x10), Err(Error::InvalidData));
    // T = TEMP_READ − 0.25 + (16 − COUNT_REMAIN) / 16.
    assert_eq!(s20(0x0032, 12, 16), Ok(2_500));
    assert_eq!(s20(0x0033, 4, 16), Ok(2_550));
    assert_eq!(s20(0xFFCE, 16, 16), Ok(-2_525));
    assert_eq!(s20(0xFFCF, 1, 16), Ok(-2_431));
}

#[test]
fn scratchpad_integrity() {
    let mut bytes = sp(0x0191, 0x7F, 0x0C, 0x10);
    bytes[1] ^= 1;
    assert_eq!(Scratchpad::check::<()>(bytes), Err(Error::Crc));
    assert_eq!(Scratchpad::check::<()>([0; 9]), Err(Error::InvalidData));
    assert_eq!(Scratchpad::check::<()>([0xFF; 9]), Err(Error::Crc));
}

#[test]
fn rejects_foreign_or_corrupted_rom() {
    let r = rom(0x3B);
    assert_eq!(
        Ds18b20::new(Fake::new(Vec::new()), r).err(),
        Some(Error::UnexpectedId(0x3B))
    );
    let mut bad = rom(0x28);
    bad.0[3] ^= 1;
    assert_eq!(
        Ds18b20::new(Fake::new(Vec::new()), bad).err(),
        Some(Error::Crc)
    );
}

#[test]
fn convert_and_read_by_rom() {
    let r = rom(0x28);
    let mut script = select(&r);
    script.push(Ev::W(0x44));
    script.extend(read_sp(&r, sp(0x0191, 0x7F, 0x0C, 0x10)));
    let mut dev = Ds18b20::new(Fake::new(script), r).unwrap();
    assert_eq!(dev.start_measurement(), Ok(750_000));
    assert_eq!(
        dev.read_measurement(),
        Ok(Measurement { temperature: 2_506 })
    );
}

#[test]
fn early_read_and_missing_device() {
    let r = rom(0x22);
    let mut script = read_sp(&r, sp(0x0550, 0x7F, 0x0C, 0x10));
    script.push(Ev::Reset(false));
    let mut dev = Ds18b20::new(Fake::new(script), r).unwrap();
    assert_eq!(dev.family(), Family::Ds1822);
    assert_eq!(dev.read_measurement(), Err(Error::InvalidData));
    assert_eq!(dev.start_measurement(), Err(Error::Timeout));
}

#[test]
fn parasite_power_holds_strong_pullup() {
    let r = rom(0x28);
    let mut script = select(&r);
    script.extend([Ev::W(0x44), Ev::Pullup(true), Ev::Pullup(false)]);
    script.extend(read_sp(&r, sp(0x07D0, 0x7F, 0x0C, 0x10)));
    let mut dev = Ds18b20::new(Fake::new(script), r)
        .unwrap()
        .with_parasite_power(true);
    assert_eq!(dev.start_measurement(), Ok(750_000));
    assert_eq!(dev.conversion_done(), Err(Error::Unsupported));
    assert_eq!(dev.read_measurement().unwrap().temperature, 12_500);
}

#[test]
fn set_resolution_keeps_alarm_bytes() {
    let r = rom(0x28);
    let mut script = read_sp(&r, sp(0x0191, 0x7F, 0x0C, 0x10));
    script.extend(select(&r));
    script.extend([Ev::W(0x4E), Ev::W(0x4B), Ev::W(0x46), Ev::W(0x3F)]);
    script.extend(select(&r));
    script.extend([Ev::W(0x48)]);
    script.extend(select(&r));
    script.extend([Ev::W(0x44), Ev::RBit(false), Ev::RBit(true)]);
    let mut dev = Ds18b20::new(Fake::new(script), r).unwrap();
    dev.set_resolution(Resolution::Bits10).unwrap();
    assert_eq!(dev.copy_scratchpad(), Ok(COPY_SCRATCHPAD_WAIT_US));
    assert_eq!(dev.start_measurement(), Ok(188_000));
    assert_eq!(dev.conversion_done(), Ok(false));
    assert_eq!(dev.conversion_done(), Ok(true));
}

#[test]
fn ds18s20_alarm_write_has_no_config_byte() {
    let r = rom(0x10);
    let mut script = read_sp(&r, sp(0x0032, 0xFF, 0x0C, 0x10));
    script.extend(select(&r));
    script.extend([Ev::W(0x4E), Ev::W(30), Ev::W(0xF6)]);
    let mut dev = Ds18b20::new(Fake::new(script), r)
        .unwrap()
        .with_resolution(Resolution::Bits9);
    assert_eq!(dev.conversion_us(), 750_000);
    assert_eq!(dev.set_resolution(Resolution::Bits9), Ok(()));
    assert_eq!(
        dev.set_resolution(Resolution::Bits12),
        Err(Error::Unsupported)
    );
    dev.set_alarm(30, -10).unwrap();
}

#[test]
fn broadcast_convert_then_read_each_device() {
    let a = rom(0x28);
    let b = rom(0x10);
    let mut script = std::vec![Ev::Reset(true), Ev::W(0xCC), Ev::W(0x44)];
    script.extend(read_sp(&a, sp(0xFF5E, 0x7F, 0x0C, 0x10)));
    script.extend(read_sp(&b, sp(0x0033, 0xFF, 4, 16)));
    script.extend(select(&a));
    script.extend([Ev::W(0xB4), Ev::RBit(false)]);
    let mut dev = Ds18b20::new(Fake::new(script), a)
        .unwrap()
        .with_resolution(Resolution::Bits11);
    assert_eq!(dev.start_all(), Ok(375_000));
    assert_eq!(dev.rom(), Some(a));
    assert_eq!(dev.read_device(&a).unwrap().temperature, -1_013);
    assert_eq!(dev.read_device(&b).unwrap().temperature, 2_550);
    assert_eq!(dev.read_power_supply(), Ok(true));
}

#[test]
fn skip_rom_single_device() {
    let mut script = std::vec![Ev::Reset(true), Ev::W(0xCC), Ev::W(0x44)];
    script.extend([Ev::Reset(true), Ev::W(0xCC), Ev::W(0xBE)]);
    script.extend(sp(0xFC90, 0x7F, 0x0C, 0x10).iter().map(|&b| Ev::R(b)));
    let mut dev = Ds18b20::new_skip_rom(Fake::new(script), Family::Ds18b20);
    assert_eq!(dev.start_measurement(), Ok(750_000));
    assert_eq!(dev.read_measurement().unwrap().temperature, -5_500);
}

#[test]
fn async_driver() {
    use zigbee_sensor::asynch::Sensor as _;
    let r = rom(0x28);
    let mut script = select(&r);
    script.extend([Ev::W(0x44), Ev::Pullup(true), Ev::Pullup(false)]);
    script.extend(read_sp(&r, sp(0xFE6F, 0x7F, 0x0C, 0x10)));
    script.extend(read_sp(&r, sp(0xFE6F, 0x7F, 0x0C, 0x10)));
    script.extend(select(&r));
    script.extend([Ev::W(0x4E), Ev::W(0x4B), Ev::W(0x46), Ev::W(0x1F)]);
    let mut dev = asynch::Ds18b20::new(Fake::new(script), r)
        .unwrap()
        .with_parasite_power(true);
    assert_eq!(block_on(dev.start_measurement()), Ok(750_000));
    assert_eq!(
        block_on(dev.read_measurement()).unwrap().temperature,
        -2_506
    );
    block_on(dev.set_resolution(Resolution::Bits9)).unwrap();
    assert_eq!(dev.conversion_us(), 94_000);
}
