# zigbee-explorir

GSS / SST Sensing ExplorIR (-M, -W), SprintIR and CozIR NDIR CO2 sensors
(also MISIR, MinIR) over the common ASCII UART protocol: streaming, polling
and command modes, filtered/unfiltered CO2 with the `.` multiplier,
optional temperature/humidity, digital filter, output-field mask, zero-point
calibration and autocalibration settings.

| | |
|---|---|
| Bus | UART 9600 8N1, 3 V levels (Rx accepts 3–5 V) |
| ID | none (`Y` returns firmware version and serial number in command mode) |
| Timing | 2 readings/s streaming (SprintIR 20/s); up to 100 ms command latency; warm-up ≈ filter setting in s (1.2 s at filter 1, 32 s at the default 32) |
| Power | 3.3–5.5 V (3.3 V recommended, linear regulator); ≈ 3.5 mW streaming, 3 mW polling, < 3.5 mW command mode; 50 ms inrush up to 60 mA (budget 100 mA peaks) |
| Output | `Co2` in ppm; `Temperature` (0.01 °C) and `Humidity` (0.01 %RH) on CozIR with the T/H option |
| Source | GSS Sensor User's Manual (COZIR, SprintIR, MISIR, MinIR) Rev. I, Aug 2015; SST Quick Start Guide QS-003 Rev 1 (CozIR, ExplorIR, SprintIR) |
| Status | mock-tested |

## Usage

The sensor powers up streaming (`K 1`, factory default) or in polling mode
(`K 2`), whichever was used last; tell the driver with `with_mode`, or call
`set_mode`.

- Streaming: `start_measurement` returns 0; `read_measurement` returns the
  newest reading line or `NotReady`.
- Polling: `start_measurement` sends `Q` and returns `RESPONSE_US` (150 ms).

Readings are multiplied by the ppm multiplier: 1 for ambient (≤ 2 %)
sensors, 10 or 100 for wide-range ones. Use `with_multiplier`, or
`request_multiplier` + `read_multiplier` (`.` command). Enable T/H output
with `send_command(&set_fields_command(field::HUMIDITY | field::TEMPERATURE
| field::CO2_FILTERED))` (`M 4164`).

`sleep()` sends `K 0` (command mode: measurement stopped);
`wake()` restores the previous streaming/polling mode and returns the
warm-up for the configured filter (`with_filter` / `set_filter`).

Other commands: `send_command(&…_command())` or `Command::new(b'X').arg(n)`,
then `read_reply(b'X')` → `Line` (`value(i)`, `autocalibration()`);
a `?` reply maps to `Unsupported`.

## Notes

- There is no checksum; lines are validated syntactically and lines longer
  than 64 characters are discarded.
- Command mode is not retained over a power cycle. `Y` and `@` settings
  changes should be done in command mode; measurement and zero-point
  commands are disabled in command mode.
- Settings are stored in non-volatile memory: configure once, not at every
  power-up.
- `X` / `F` concentrations are in sensor units (ppm ÷ multiplier).
- Autocalibration history is lost on power-off; the sensor must stay
  powered for a whole autocalibration period.
- Without the T/H option the sensor reports `T 01000` (0 °C) and
  `H 00000`; do not enable those fields in that case.
- SprintIR: select at most two output fields.

## Wiring

Only GND, 3V3, Rx and Tx are needed; leave all other pins unconnected.
CozIR-A 10-way 0.1" header (other models: see their datasheet; SST
adaptor cable colours black GND, red 3V3, orange Rx, yellow Tx):

| CozIR-A pin | MCU |
|---|---|
| 1 GND | GND |
| 3 3.3V | 3.3 V |
| 5 Rx | UART TX |
| 7 Tx | UART RX |
| 8 Zero, 10 Ambient | not connected (calibration inputs) |
