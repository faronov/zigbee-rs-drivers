# zigbee-s3km1110

Iclegend S3KM1110 24 GHz human-presence radar on the Ai-Thinker Rd-03 and
Waveshare HMMD modules (UART protocol of the module firmware).

| | |
|---|---|
| Interface | UART 115200 8N1 (`embedded_io::Read + ReadReady + Write` / `embedded_io_async::Read + Write`) |
| Frames | reports `F4 F3 F2 F1, len, detected, distance cm, 16 × u16 gate energy, F8 F7 F6 F5`; commands/ACKs `FD FC FB FA … 04 03 02 01` |
| Conversion | free running, one report per 100 ms; `start_measurement` returns 0 |
| Output | `Quantity::Occupancy`, plus `Quantity::Distance` (mm) when a target is present; raw gate energies |
| Commands | open/close command mode, write/read parameter (min/max gate, disappearance delay, per-gate trigger/hold thresholds, optional flash persistence), auto-threshold + progress, system mode, firmware version, serial number |
| Current | ≈ 50 mA average (3.3 V) |
| Source | Ai-Thinker "Rd-03 Serial communication protocol" V1.0.1 (2024-11-28); Waveshare HMMD wiki |
| Status | mock-tested |

Notes:

* The module boots in text ("normal") mode. Call `enable_report_mode()`
  once after power-up; the mode is not persistent.
* Gates are 0.7 m; thresholds are linear power ratios (`10^(dB/10)`).
* Writes are RAM-only unless `persist` is set: the driver then appends the
  documented "ABD" parameter (word 0x002F, value 0x64). Because 0x002F is
  also the hold-threshold word of gate 15, that gate's hold threshold is not
  addressable.
* If the first `open()` is not acknowledged (the module may still be
  sending output), the vendor recommends flushing the RX buffer after
  ~100 ms and sending it again.
* No Iclegend-authored protocol document is public; the command set is the
  one published by both module vendors. Debug-mode range-Doppler frames and
  the undocumented register/factory-test commands are not supported.
* OT2 / OUT (high = presence) can be read with `zigbee-digital`.
