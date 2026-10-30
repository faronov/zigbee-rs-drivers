# zigbee-ze08

Winsen ZE08-CH2O and ZE07-CH2O electrochemical formaldehyde modules over
UART: active upload and Q&A communication types, checksum verification and
µg/m³ ↔ ppb conversion.

| | |
|---|---|
| Bus | UART 9600 8N1, 3.3 V levels |
| Frame | 9 bytes `FF b1..b7 cs`, cs = (!(b1+…+b7)) + 1 |
| ID | none |
| Timing | active upload every 1 s; Q&A reply within `RESPONSE_US` (100 ms, assumed) |
| Warm-up | ≤ 3 min (≥ 5 min on first use); response ≤ 60 s |
| Range | 0–5 ppm (5000 ppb), resolution ≤ 10 ppb |
| Supply | 3.7–9 V; no sleep command |
| Output | `Formaldehyde` in ppb |
| Source | Winsen ZE08-CH2O User's Manual v1.0; ZE07-CH2O User's Manual |
| Status | mock-tested |

## Usage

At power-on the module is in **active upload** mode and sends
`FF 17 04 00 CH CL FH FL CS` (concentration and full range, ppb) every
second. In this mode `start_measurement` returns 0 and `read_measurement`
returns the newest complete frame in the RX buffer (or `NotReady`). Size
the UART RX buffer for at least one frame per poll interval, or the
newest frame may be stale.

`set_mode(Mode::QuestionAnswer)` sends `FF 01 78 41 00 00 00 00 46`; then
`start_measurement` sends `FF 01 86 00 00 00 00 00 79` and returns
`RESPONSE_US`, and the reply carries both µg/m³ and ppb. The mode is not
documented as persistent: re-send it after power-up.

`ug_m3_to_ppb` / `ppb_to_ug_m3` convert at 25 °C and 1013 hPa
(24.45 L/mol, M = 30.03 g/mol). The module computes its own ppb value
(manual example: 42 µg/m³ → 32 ppb), which is what the driver reports.

## Wiring

| ZE08 pin | MCU |
|---|---|
| 4 Vin | 3.7–9 V |
| 3 GND | GND |
| 6 TXD | UART RX |
| 5 RXD | UART TX |
| 2 DAC, 7 PWM | unused (analog/PWM outputs) |
| 1 HD | leave open (factory zero calibration) |
