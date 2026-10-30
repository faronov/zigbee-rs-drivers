# zigbee-stts22h

ST STTS22H ±0.5 °C (−10…+60 °C) digital temperature sensor over I2C /
SMBus, with 16-bit 0.01 °C output and a latched ALERT/INT pin.

| | |
|---|---|
| Address | 0x38 (Addr = VDD), 0x3C (15 kΩ pull-up), 0x3E (56 kΩ pull-up), 0x3F (GND) |
| Conversion | one-shot (default mode, ONE_SHOT bit), BUSY flag; time not specified: driver waits 1.5 × freerun period = 60 / 30 / 15 / 7.5 ms for 8 / 4 / 2 / 1 averages |
| Resolution | 0.01 °C/LSB (16 bit); averaging 8 / 4 / 2 / 1 (0.02 … 0.055 °C RMS) |
| Rates | low-ODR 1 Hz (2 µA), freerun 25 / 50 / 100 / 200 Hz |
| Shutdown current | power-down 0.5 µA typ. (1.75 µA averaged at one one-shot per second) |
| Alert | ALERT/INT open-drain, active low; 8-bit high/low thresholds `(reg − 63) × 0.64 °C` (−39.68…122.88 °C, 0 = off); latched until STATUS read or ARA |
| ID | WHOAMI 0x01 = 0xA0 |
| Output | 0.01 °C |
| Source | ST DS12606 Rev 8 (March 2026) |
| Status | mock-tested |

## Usage notes

* Wait 12 ms after power-up (`BOOT_TIME_US`). The part starts in
  one-shot/power-down mode, so `configure()` is optional but writes
  averaging, BDU, IF_ADD_INC and the timeout bit.
* Every CTRL write sets IF_ADD_INC (the data sheet disagrees about its
  default), so `read_measurement()` reads TEMP_L_OUT and TEMP_H_OUT in one
  transfer.
* `LowPower::sleep` = power-down. `wake` writes power-down first (the
  data sheet requires it before any mode change), then starts low-ODR
  (default) or freerun mode.
* ALERT is latched, so it survives power-down and works as a wake-up
  source with one-shots. `read_status()` and `conversion_busy()` read
  STATUS, which clears the limit flags and releases ALERT.
* `set_limits` rounds to the 0.64 °C grid and clamps to 1..=255; `None`
  disables a limit.
