# zigbee-cm1106

Cubic CM1106 / CM1106-C / CM1106SL-NS NDIR CO2 sensor modules: UART protocol
(read CO2 with status, ABC on/off and cycle, calibration, serial number,
software version, CM1106SL-NS working mode / period / ABC store), the
CM1106-C I2C command protocol and the CM1106SL-NS I2C register map.

| | |
|---|---|
| Bus | UART 9600 8N1 (`Cm1106`); I2C `0x31` (CM1106-C, `Cm1106C`, 10–400 kHz); I2C `0x34` (CM1106SL-NS, `Cm1106Sl`, ≤ 100 kHz, clock stretching) |
| ID | no ID register; serial number (5 × u16) and ASCII version via commands |
| Timing | CM1106/-C: 1 s sampling, first reading ≤ 30 s (550 ppm + preheating flag until then). CM1106SL-NS: ≈ 730 ms from EN high to RDY low |
| Current | CM1106/-C < 45 mA at 4.5–5.5 V; CM1106SL-NS ≤ 74 µA average at a 1 min cycle, 3.3–5.5 V |
| Output | `Co2` in ppm (0–5000) |
| Source | Cubic CM1106-C Specification V0.6 (2022-12-22); CM1106SL-NS Specification V0.7 (2021-11-05) |
| Status | mock-tested |

## Usage

UART (all variants): the sensor only answers requests.
`start_measurement` discards stale input, sends `11 01 01 ED` and returns
`RESPONSE_US` (100 ms, assumed); `read_measurement` parses the reply or
returns `NotReady`. Other commands: `send_command(&…_command())`, then
`read_reply(cmd::…)` and the `Frame` decoders (`abc`, `period`,
`working_mode`, `serial_number`, `version`, `is_ack`). `set_abc` and
`calibrate` (400–1500 ppm) validate their arguments (`InvalidArgument`).

CM1106-C I2C: `start_measurement` writes command `0x01` and returns
`I2C_RESPONSE_US` (20 ms, assumed); `read_measurement` reads
`01 DF0 DF1 DF2 CS` and verifies the checksum. Other commands via
`send(&[cmd, …])` + `read_reply(cmd, &mut buf)`.

CM1106SL-NS I2C: in single measurement mode (default) `start_measurement`
writes `1` to register `0x93` and returns `SL_CYCLE_US` (730 ms; use the RDY
pin to be exact); in continuous mode (`with_mode` / `set_mode`) it returns 0.
`read_measurement` reads registers `0x00..=0x0E` (error status, CO2, chip
temperature, counter).

`Readings` omit CO2 while the preheating or sensor-error status bit is set
(UART / CM1106-C) or on a fatal error (CM1106SL-NS I2C).

## Notes / Errata

- No sleep command on any variant, so no `LowPower`. The CM1106SL-NS is
  powered per measurement through **EN** (high = on); after EN high wait for
  **RDY** low (≈ 730 ms), communicate, then pull EN low. Do not remove power
  during an EEPROM write (< 25 ms).
- CM1106SL-NS working mode encodings are opposite: UART command `0x51` uses
  0 = single, 1 = continuous; I2C register `0x95` uses 0 = continuous,
  1 = single (`WorkingMode` vs `SlMode`).
- In single measurement mode the CM1106SL-NS needs `store_abc_command()`
  every 30 min (or after each measurement if powered less often) for ABC.
- Datasheet errata: the CM1106SL-NS "set continuous mode" example
  `11 01 51 01 98` has a wrong length byte and checksum (correct:
  `11 02 51 01 9B`; `16 01 51 98` is the reply). The CM1106-C version example
  `16 0C 1E 43 4D 20 56 30 2E 30 2E 32 30 00 97` has a wrong checksum (bytes
  as printed sum to `9C`).
- Status byte bits (UART DF3 / I2C DF2) are reserved on the CM1106SL-NS.
- The original CM1106 datasheet was not available; the CM1106-C
  specification documents the same UART protocol.

## Wiring

| CM1106 / CM1106-C | MCU |
|---|---|
| CON4-1 +5V | 4.5–5.5 V |
| CON4-2 GND | GND |
| CON5-2 RX/SDA | UART TX / SDA |
| CON5-3 TX/SCL | UART RX / SCL |
| CON5-4 R/T | high/floating = UART, low = I2C |

| CM1106SL-NS | MCU |
|---|---|
| CON5-1 GND | GND |
| CON5-2 VBB | 3.3–5.5 V |
| CON5-3 VDDIO | MCU I/O voltage |
| CON5-4 RX/SDA | UART TX / SDA |
| CON5-5 TX/SCL | UART RX / SCL |
| CON4-1 EN | GPIO (high = powered) |
| CON4-3 RDY | GPIO input (low = data ready) |
| CON4-4 COMSEL | high/floating = UART, low = I2C |
