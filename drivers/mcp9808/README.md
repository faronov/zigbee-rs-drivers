# zigbee-mcp9808

Microchip MCP9808 (±0.5 °C max) and generic JEDEC JC-42.4 digital
temperature sensors over I2C / SMBus, with window + critical Alert output.

| | |
|---|---|
| Address | 0x18–0x1F (A2..A0) |
| Conversion | no one-shot; after leaving shutdown 30 / 65 / 130 / 250 ms typ. for 0.5 / 0.25 / 0.125 / 0.0625 °C (driver waits typ. × 1.5); JC-42.4 generic: 125 ms |
| Resolution | 0.0625 °C result (MCP9808 resolution register), 0.25 °C limits |
| Shutdown current | 0.1 µA typ., 2 µA max (200 µA while converting) |
| Alert | T_UPPER/T_LOWER window + T_CRIT, hysteresis 0/1.5/3/6 °C, comparator or interrupt, polarity, critical-only; flags in T_A bits 15:13 |
| ID | Manufacturer ID 0x06 = 0x0054, Device ID 0x07 = 0x04xx |
| Output | 0.01 °C |
| Source | Microchip DS25095A (2011) |
| Status | mock-tested |

## Usage notes

* `Variant::Mcp9808` (default) checks the IDs in `probe()` and writes the
  resolution register; `Variant::Jc42` (e.g. TSE2002/TSE2004-class SPD
  sensors) skips both and returns the IDs it read.
* Call `configure()` once after power-up (the part starts converting). It
  enters shutdown. `start_measurement()` clears SHDN, and
  `read_measurement()` reads T_A and sets SHDN again (emulated one-shot).
  `LowPower::sleep` = shutdown, `wake` = continuous.
* The Alert output is neither asserted nor released while in shutdown, so
  with emulated one-shots it only reflects the window when awake. For a
  wake-up source, keep the part in continuous mode (`wake()`) at the cost
  of ~200 µA, or poll `read_flags()` after each sample.
* Interrupt-mode Alert is cleared with `clear_interrupt()`, which must run
  outside shutdown. T_CRIT is always comparator.
* Lock bits are never set by the driver; if set by other firmware,
  configuration writes are ignored until power-on reset.
* The data sheet is inconsistent about the resolution power-up default
  (0.25 °C in the T_A note, 0.0625 °C in register 5-7); `configure()`
  always writes it.
