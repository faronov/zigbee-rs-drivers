# zigbee-pmsx003

Plantower PMS5003, PMS7003 and PMSA003 laser particulate matter sensors
over UART (active/passive modes, sleep/wake) and PMSA003I over I2C.
Decodes standard (CF=1) and atmospheric PM1.0/PM2.5/PM10 plus particle
counts.

| | |
|---|---|
| Bus | UART 9600 8N1, 3.3 V levels; PMSA003I: I2C ≤ 100 kHz |
| Address | PMSA003I `0x12` |
| Frame | 32 bytes `42 4D 00 1C` + 13 BE words + 16-bit sum |
| ID | none (PMSA003I: version byte in data 13) |
| Timing | active: 2.3 s (stable) / 200–800 ms (fast); total response ≤ 10 s; stable ≥ 30 s after wake |
| Current | ≤ 100 mA active, ≤ 200 µA standby, 4.5–5.5 V supply (fan) |
| Output | `Pm1`, `Pm25`, `Pm10` (atmospheric) in 0.1 µg/m³; resolution 1 µg/m³ |
| Source | Plantower PMS5003 data manual V2.3; PMSA003I data manual V2.6 |
| Status | mock-tested |

## Usage

UART parts start in **active** mode after power-up (not retained):
`start_measurement` returns 0 and `read_measurement` returns the newest
valid frame in the RX buffer, or `NotReady`. Size the UART RX buffer for
at least one 32-byte frame per poll.

`set_mode(Mode::Passive)` sends `42 4D E1 00 00 01 70`; afterwards
`start_measurement` sends `42 4D E2 00 00 01 71` and returns
`RESPONSE_US` (100 ms, assumed — the manual gives no answer time).

`sleep()` / `wake()` send `42 4D E4 00 00/01 ..`; `wake` returns 30 s
(fan spin-up). Pulling SET low has the same effect without UART traffic.

`Pmsa003i` reads the frame from register 0x00; the sensor is free-running,
so `start_measurement` returns 0.

## Notes

- `Measurement::standard` (CF=1) is meant for factory environments;
  `atmospheric` is reported as quantities. Particle counts per 0.1 L and
  data 13 are available on `Measurement` (no matching `Quantity`).
- Command acknowledgements and other non-28-byte frames are skipped by the
  parser.
- PMS1003/PMS3003 (24-byte frames) are not supported.

## Wiring

| PMS5003 pin | MCU |
|---|---|
| 1 VCC | 5 V |
| 2 GND | GND |
| 3 SET | optional GPIO (low = sleep; pulled up inside) |
| 4 RX | UART TX |
| 5 TX | UART RX |
| 6 RESET | optional GPIO (low = reset; pulled up inside) |

PMSA003I: pins 1–2 VCC 5 V, 3–4 GND, 7 SCL, 9 SDA (4.7 kΩ pull-ups to
3.3 V), 5 RESET, 10 SET.
