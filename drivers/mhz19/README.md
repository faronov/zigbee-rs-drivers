# zigbee-mhz19

Winsen MH-Z19B, MH-Z19C and MH-Z19E NDIR CO2 sensors over UART: read CO2
(0x86), automatic baseline correction (ABC) on/off, zero and span
calibration and detection range.

| | |
|---|---|
| Bus | UART 9600 8N1, 3.3 V TTL (5 V tolerant) |
| Frame | 9 bytes `FF 01 cmd b3..b7 cs`, cs = (!(b1+…+b7)) + 1 |
| ID | none |
| Response | answer to 0x86 within `RESPONSE_US` (100 ms, assumed; 9 bytes take 9.4 ms) |
| Pre-heat | 3 min |
| Current | B: < 60 mA average, 150 mA peak at 5 V; no sleep command |
| Supply | B: 4.5–5.5 V (C/E: see their manuals) |
| Source | Winsen MH-Z19B User's Manual v1.0; MH-Z19C/E use the same commands |
| Status | mock-tested |

## Usage

The sensor only answers queries. `start_measurement` discards stale RX
bytes, sends `FF 01 86 00 00 00 00 00 79` and returns `RESPONSE_US`;
`read_measurement` parses the available bytes (never blocks — the UART must
implement `embedded_io::ReadReady`) and returns `NotReady` until the
response is complete or `Crc` if only a corrupt frame arrived.

Calibration and configuration commands have no response. ABC is on at
delivery; disable it (`set_abc(false)`) for spaces that never see fresh air.
Zero calibration needs ≥ 20 min in 400 ppm air; alternatively pull HD low
for ≥ 7 s.

## Errata / notes

- **0x99 detection range byte position**: the B manual puts the range in
  bytes 3–4, while the C/E manuals (and B units in the field) use bytes
  6–7 (`FF 01 99 00 00 00 13 88 CB` = 5000 ppm). `RangeLayout::Bytes6And7`
  is the default; select the other with `with_range_layout`.
- Byte 4 of the 0x86 response is often read as "temperature + 40"; it is
  undocumented and not reported.

## Wiring

| MH-Z19 | MCU |
|---|---|
| Vin | 5 V |
| GND | GND |
| Tx | UART RX |
| Rx | UART TX |
| HD | optional GPIO (low ≥ 7 s = zero calibration) |
