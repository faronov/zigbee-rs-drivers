# zigbee-ccs811

ams (now ScioSense) CCS811 metal-oxide gas sensor: equivalent CO2 and TVOC
computed on-chip, with temperature/humidity compensation input, baseline
save/restore, eCO2 threshold interrupts and raw data.

| | |
|---|---|
| Bus | I2C 0x5A (ADDR low), 0x5B (ADDR high), 10–400 kHz |
| ID | HW_ID 0x81, HW_VERSION 0x1X |
| Start-up | 20 ms after power-on, 2 ms after reset; boot loader → `start_app()` |
| Sampling | drive mode 1: 1 s (46 mW), 2: 10 s (7 mW), 3: 60 s (1.2 mW), 4: raw only 250 ms |
| Current | 26 mA while heating at 1.8 V; idle mode 0.034 mW; sleep (nWAKE high) 19 µA |
| Output | eCO2 400–8192 ppm, TVOC 0–1187 ppb |
| Source | ams CCS811 datasheet v1-00 (2016-Dec-23) |
| Status | mock-tested |

## Usage

1. `probe()` checks HW_ID; `start_app()` leaves the boot loader (wait the
   returned `APP_START_US`, an assumption: the datasheet gives no figure).
2. `start_measurement()` writes the active drive mode (`with_active_mode`,
   default 1 s) and returns its interval, or `0` while running.
   `read_measurement()` returns `NotReady` until DATA_READY and
   `InvalidData` on the STATUS error bit (`error_flags()` tells why).
3. `LowPower::sleep` selects drive mode 0, `wake` restores the active mode.

Readings need 20 min of conditioning after entering a mode; new sensors
need a 48 h burn-in. Restore a saved baseline only after conditioning.
When moving to a slower drive mode, stay idle ≥ 10 min first.

## Wiring

| CCS811 | MCU |
|---|---|
| VDD | 1.8–3.6 V |
| SDA, SCL | I2C with pull-ups |
| ADDR | GND (0x5A) / VDD (0x5B) |
| nWAKE | GND, or a GPIO driven low ≥ 50 µs before and during every I2C transaction (driver does not manage it) |
| nRESET | VDD or GPIO |
| nINT | optional data-ready / threshold GPIO |
