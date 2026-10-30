# zigbee-onewire

1-Wire bus master for zigbee-rs sensor drivers.

* `OneWire` / `asynch::OneWire` traits: reset/presence, bit and byte I/O,
  search triplet, strong pull-up, Match/Skip/Read ROM helpers. Device
  drivers (`zigbee-ds18b20`) are generic over them, so a platform can use a
  hardware master (DS2482, UART-based 1-Wire, vendor peripheral) instead.
* `BitBang`: standard-speed master on one open-drain GPIO + `DelayNs`
  (timing from Maxim AN126, table 2).
* `Search`: Search ROM / Alarm Search / family-targeted search with
  CRC-8 check (Maxim AN187).

| | |
|---|---|
| Bus | 1-Wire standard speed (~15 kbit/s), overdrive not supported |
| Wiring | DQ to an open-drain GPIO, 4.7 kΩ pull-up to VDD (lower for long buses) |
| Slot timing | reset 480 + 70 + 410 µs, write/read slot 70 µs |
| Interrupts | each critical window (≤ 70 µs) runs inside `InterruptMask::mask`; provide one built on your critical section, or mask interrupts yourself |
| Async | slots run synchronously; `BitBang` yields to the executor after each byte and reset |
| Parasite power | `BitBang` has no strong pull-up (`strong_pullup(true)` → `Unsupported`); add a MOSFET and a custom `OneWire` impl, or power devices from VDD |
| Source | Maxim AN126 "1-Wire Communication Through Software", AN187 "1-Wire Search Algorithm", DS18B20 datasheet rev 042208 |
| Status | mock-tested |

```rust,ignore
let mut bus = BitBang::new(pin, delay).with_mask(Cs);
let mut search = Search::new();
while let Some(rom) = search.next(&mut bus)? {
    // rom.family(), rom.serial()
}
```
