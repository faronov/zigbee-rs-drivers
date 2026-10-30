# zigbee-stts751

ST STTS751 2.25 V digital temperature sensor over SMBus, with a latched
EVENT output and an Addr/THERM comparator output.

| | |
|---|---|
| Address | STTS751-0: 0x48 / 0x49 / 0x38 / 0x39; STTS751-1: 0x4A / 0x4B / 0x3A / 0x3B (7.5 k / 12 k / 20 k / 33 kΩ pull-up on Addr/THERM; GND = 0x39 / 0x3B, THERM unusable) |
| Conversion | one-shot from standby (register 0x0F); max 14 / 28 / 56 / 112 ms at 9 / 10 / 11 / 12 bit |
| Resolution | 9 / 10 (default) / 11 / 12 bit, 0.5 … 0.0625 °C |
| Rates | 0.0625 … 32 conversions/s (16/s needs ≤ 11 bit, 32/s needs ≤ 10 bit) |
| Shutdown current | standby 3 µA typ., 8 µA max |
| Alert | EVENT (open-drain, active low): above high or ≤ low limit, latched until SMBus ARA; Addr/THERM comparator with whole-degree limit and hysteresis |
| ID | Manufacturer ID 0xFE = 0x53; Product ID 0xFD = 0x00 (-0) / 0x01 (-1) |
| Output | 0.01 °C |
| Source | ST DocID16483 Rev 7 |
| Status | mock-tested |

## Usage notes

* `configure()` writes resolution, EVENT mask, conversion rate and the SMBus
  timeout, and leaves the part in standby. It rejects rate / resolution pairs
  that the data sheet forbids with `Error::Unsupported`.
* The one-shot register only works in standby, so `start_measurement()`
  enters standby first when the part is running (after power-up or `wake`).
* The part supports only SMBus byte protocols, so `read_measurement()` reads
  the high byte (0x00) and the low byte (0x02) in two transfers.
* `LowPower::sleep` = standby; `wake` starts continuous conversion at the
  configured rate.
* EVENT stays latched, including in standby, until `acknowledge_alert()`
  sends an Alert Response (address 0x0C). That makes it usable as a wake-up
  source with one-shots. If the condition persists, the next conversion
  asserts it again. `read_status()` clears THIGH / TLOW once the condition
  has gone.
* Limits use 0.0625 °C steps and saturate to −64 … +127.9375 °C. THERM
  limit and hysteresis are whole degrees.
