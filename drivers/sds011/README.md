# zigbee-sds011

Nova Fitness SDS011 and SDS021 laser PM2.5/PM10 sensors over UART: active
and query reporting, sleep/work, working period, device ID and firmware
version, with checksum verification.

| | |
|---|---|
| Bus | UART 9600 8N1, 3.3 V TTL |
| Frames | sensor → host 10 bytes `AA C0/C5 d1..d6 cs AB`; host → sensor 19 bytes `AA B4 cmd .. id1 id2 cs AB` |
| ID | 16-bit device ID in every frame (`FFFF` = all sensors) |
| Timing | 1 Hz data; query ≥ 3 s apart; stable 30 s after work starts |
| Current | 70 mA ± 10 mA working, < 4 mA sleeping (laser + fan off) at 5 V |
| Supply | 4.7–5.3 V, > 1 W |
| Output | `Pm25`, `Pm10` in 0.1 µg/m³ (range 0–999.9 µg/m³) |
| Source | Nova Fitness Laser Dust Sensor Control Protocol V1.3; SDS011 specification V1.3 |
| Status | mock-tested |

## Usage

The reporting mode is retained over power cycles (factory default
**active**), so tell the driver which mode the sensor is in
(`with_reporting_mode`) or set it once with `set_reporting_mode`.

- Active: `start_measurement` returns 0; `read_measurement` returns the
  newest data frame in the RX buffer or `NotReady`.
- Query: `start_measurement` sends "query data" and returns `RESPONSE_US`
  (100 ms, assumed — the protocol gives no reply time).

`sleep()` / `wake()` send the sleep/work command; `wake` returns 30 s
(`STABILIZE_US`). The sleep state is **not** retained: the sensor works
after power-on. `set_working_period(n)` (1–30 min, persistent) makes the
sensor itself work 30 s and sleep `n × 60 − 30` s; 0 = continuous.

Other commands: `send_command(&packet)` with any `*_command` encoder, then
`read_reply(cmd::…)` and the `Reply` decoders; firmware version via
`request_firmware_version` / `read_firmware_version`. `with_device_id`
addresses one sensor and ignores frames from others on a shared bus.

## Errata

- The protocol's "query working mode" example reply
  `AA C5 06 00 00 00 A1 60 07 AB` is labelled "working", but its data byte
  (0) means sleep per the command table; `Reply::working` follows the table.
- The laser lifetime is ~8000 h: prefer query mode with sleep, or a working
  period, for long-term monitoring.

SDS021 uses the same UART protocol and frames; electrical data and the
pinout above are for SDS011 — check the SDS021 datasheet.

## Wiring

| SDS011 pin | MCU |
|---|---|
| 3 5V | 5 V |
| 5 GND | GND |
| 7 TXD | UART RX |
| 6 RXD | UART TX |
| 2, 4 | PM2.5 / PM10 PWM outputs (unused) |
