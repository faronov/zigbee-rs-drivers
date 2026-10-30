# zigbee-ld2450

Hi-Link HLK-LD2450 24 GHz multi-target (up to 3) motion-tracking radar
(UART protocol).

| | |
|---|---|
| Interface | UART 256000 8N1 default (`embedded_io::Read + ReadReady + Write` / `embedded_io_async::Read + Write`) |
| Frames | reports `AA FF 03 00` + 3 × (x, y, speed, resolution) + `55 CC`; commands/ACKs `FD FC FB FA … 04 03 02 01` |
| Conversion | free running, 10 reports/s; `start_measurement` returns 0 |
| Output | `Quantity::Occupancy` (any target), `Quantity::Distance` to the nearest target (mm, √(x² + y²)); raw targets in `Report::targets` |
| Commands | config enable/end, single/multi-target tracking, query mode, firmware version, baud rate, factory reset, restart, Bluetooth, MAC, zone filter query/set |
| Current | > 200 mA supply capability required (5 V) |
| Source | Hi-Link "HLK-LD2450 Serial Communication Protocol" V1.03 (2023-10-17) |
| Status | mock-tested |

Notes:

* Report X/Y/speed use sign-magnitude with bit 15 set = **positive**
  (`decode_signed`); zone-filter coordinates are ordinary two's complement.
* An all-zero target slot means "no target".
* Bracket commands with `enable_config()` / `end_config()`. Zone filters are
  persistent and apply immediately; baud rate, Bluetooth and factory reset
  apply after `Command::Restart`.
* The blocking `read_measurement` drains the UART and returns the newest
  report or `NotReady`; command helpers block on a silent UART — use `send()`
  + `poll_ack()` for a bounded wait. The LD2450 has no OUT pin.
* The document describes the MAC as 3 bytes but its example has 6 (6 are
  decoded).
