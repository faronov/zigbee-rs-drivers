# zigbee-adt7410

Analog Devices ADT7410 (±0.5 °C) and ADT7420 (±0.25 °C) 16-bit digital
temperature sensors over I2C, with INT (window) and CT (critical) outputs.

| | |
|---|---|
| Address | 0x48 … 0x4B (`address(a1, a0)`) |
| Conversion | one-shot from shutdown (mode bits 6:5 = 01), 240 ms typ., no max given: driver waits 360 ms (1.5 × typ.); 1 SPS mode 60 ms typ. (waited 90 ms) |
| Resolution | 13 bit (0.0625 °C, chip default) or 16 bit (0.0078 °C, driver default) |
| Rates | continuous (back-to-back 240 ms), 1 SPS, one-shot |
| Shutdown current | 2 µA typ., 15 µA max at 3.3 V |
| Alert | INT: THIGH / TLOW window; CT: TCRIT; open-drain, programmable polarity, fault queue 1–4, interrupt (default) or comparator mode, common 0–15 °C hysteresis |
| ID | ID register 0x0B: bits 7:3 = 0b11001, bits 2:0 = revision (ADT7420 0xCB) |
| Output | 0.01 °C |
| Source | ADI ADT7410 Rev. C, ADT7420 Rev. A |
| Status | mock-tested |

## Usage notes

* Both parts share the register map and ID, so `Adt7420` is an alias of
  `Adt7410`; `probe()` returns the silicon revision.
* `configure()` writes resolution, fault queue, alert mode and polarities,
  and leaves the part in shutdown. `soft_reset()` restores the chip
  defaults (13 bit, continuous); wait the returned 200 µs, then call
  `configure()` again.
* Each `start_measurement()` writes the one-shot mode; the part shuts
  down after the conversion. `LowPower::sleep` = shutdown. `wake` starts
  continuous or 1 SPS mode (`Config::run_mode`).
* Setpoints always use the 16-bit 1/128 °C format, independent of the
  resolution. In 13-bit mode the three LSBs of the temperature carry the
  TCRIT / THIGH / TLOW flags; the decoder drops them.
* Interrupt mode (default) latches INT / CT until any register is read,
  including the temperature read, so one-shots can use INT as a wake-up
  source. In comparator mode INT holds through shutdown, and a one-shot
  write releases it once the temperature is back inside the hysteresis
  band. `read_status()` clears the limit flags.
