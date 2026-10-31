# zigbee-mmc56x3

MEMSIC MMC5603NJ and MMC5633NJL 3-axis AMR magnetometers (same register
map). Use cases: door/window contact with a magnet, utility meter dial
reading, orientation.

| | |
|---|---|
| Bus | I2C 0x30 (fixed), up to 400 kHz (I3C not used) |
| ID | `Product_ID` (0x39) = 0x10 (both parts) |
| Output | magnetic field in nT (`Vector3`), 20-bit, 16384 counts/gauss ≈ 6.1 nT/LSB, ±30 gauss |
| Default | bandwidth 00 (lowest noise), automatic SET/RESET on, on-demand measurements |
| Sampling | one measurement per `start_measurement()`; `read_measurement()` returns `NotReady` until `Status1.Meas_m_done`; optional internal continuous mode 1–255 Hz (1 kHz with `hpower`), periodic SET every 1–2000 samples |
| Conversion | `tTM` 6.6 / 3.5 / 2.0 / 1.2 ms for bandwidth 00 / 01 / 10 / 11 |
| Offset | automatic SET/RESET (default); manual `do_set()` / `do_reset()` (1 ms `tSR`) + `set_reset_offset()` |
| Threshold interrupt | none (no interrupt pin) |
| Current | ≈1 µA between measurements; 3.4 / 1.8 / 1.3 / 0.75 mA at 100 Hz for bandwidth 00 / 01 / 10 / 11, scaling linearly with rate (e.g. ≈13 µA at 1 Hz, bandwidth 10) |
| Sleep | `LowPower::sleep` stops continuous mode (`Cmm_en` = 0); on-demand mode idles by itself |
| Source | MEMSIC MMC5603NJ datasheet Rev. B (2022-01-17); MMC5633 registers checked against the Linux `mmc5633` IIO driver |
| Status | mock-tested |

Notes

* The control and ODR registers are write-only; the driver keeps the
  configuration and always writes full register values. Call
  `configure()` after power-up or `soft_reset()` (20 ms OTP reload).
* `Meas_m_done` is bit 6 (0x40) of `Status1` per the register table; the
  datasheet's measurement example says "bit 1". The Linux and Adafruit
  drivers also use 0x40.
* Pick the bandwidth by noise vs energy: 1.5 / 2.0 / 3.0 / 4.0 mG RMS.
  With automatic SET/RESET, continuous mode is limited to 75 Hz at
  bandwidth 00 and 150 Hz at bandwidth 01.
* The MMC5633 datasheet was not available; the product ID and register
  map come from the Linux driver and another open-source driver.
* Temperature output (0.8 °C/LSB) and self-test are not used.
