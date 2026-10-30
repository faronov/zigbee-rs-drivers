# zigbee-bmm350

Bosch BMM350 3-axis TMR magnetometer. Use cases: door/window contact with
a magnet, utility meter dial reading, compass heading.

| | |
|---|---|
| Bus | I2C 0x14 (ADSEL low) / 0x15 (ADSEL high), standard/fast mode; every read returns 2 dummy bytes, handled by the driver (blocking + async). I3C is not supported |
| ID | `CHIP_ID` (0x00) = 0x33 |
| Output | magnetic field in nT (`Vector3`), ~0.1 µT resolution, ±2000 µT all axes; compensated die temperature in 0.01 °C (`Measurement::die_temperature`, not reported as a reading) |
| Default | averaging 2 ("regular power"), forced (single) measurements with `FM` |
| Sampling | one forced measurement per `start_measurement()`; `read_measurement()` returns `NotReady` until `INT_STATUS.drdy`; optional normal mode 1.5625–400 Hz (averaging limited at 100/200/400 Hz, otherwise `Unsupported`) |
| Conversion | forced `FM` 15 / 17 / 20 / 28 ms, `FM_FAST` 4 / 5 / 9 / 16 ms for averaging 1 / 2 / 4 / 8 |
| Offset | factory OTP offset/sensitivity/TCO/TCS/cross-axis compensation on the host (integer port of Bosch SensorAPI); bit reset + flux-guide reset via `magnetic_reset()` |
| Threshold interrupt | none on chip; data ready can be routed to the INT pin (latched, push-pull/open-drain, polarity) |
| Current | 1.8 µA suspend; 200 µA at 100 Hz averaging 2, 350 µA at 100 Hz averaging 4; 3.5 mA peak |
| Sleep | `LowPower::sleep` = suspend (registers retained); `wake()` re-enters normal mode (70 ms) or does nothing in forced mode |
| Source | Bosch Sensortec BMM350 datasheet BST-BMM350-DS001-27 (February 2025); BMM350 SensorAPI v1.10.0 (OTP layout, reset sequence, delays, compensation) |
| Status | mock-tested; compensation matches the SensorAPI fixed-point (`BMM350_USE_FIXED_POINT`) code on golden vectors |

Notes

* After power-on wait 3 ms, then `probe()`. `configure()` is a multi-step
  sequence: it returns a wait, then call `continue_configure()` after each
  wait until it returns 0. It reads the 12 OTP compensation words
  (300 µs each), powers the OTP off, runs a bit reset (14 ms) and a
  flux-guide reset (18 ms), then writes ODR/averaging, axis enable and
  `INT_CTRL`. `start_measurement()` runs pending steps itself and
  `read_measurement()` returns `NotReady` until configuration is done.
* The datasheet makes host compensation mandatory but only via the API
  and gives no formulas. `src/compensation.rs` is a fixed-point port of
  Bosch's BMM350 SensorAPI (BSD-3-Clause) and carries its notice.
* Forced mode is only reachable from suspend, so the chip returns to
  suspend (1.8 µA) between single measurements without a `sleep()` call.
  `FM_FAST` (`Config::fast_forced`) is only allowed when triggering at
  ≥ 25 Hz (datasheet 5.1.4), hence `FM` by default.
* Suspend → normal uses the datasheet's 70 ms; SensorAPI waits 38 ms.
* After a bit reset / flux-guide reset SensorAPI checks the command
  value in `PMU_CMD_STATUS_0[7:5]`; the datasheet does not document these
  bits, so the driver does not verify them.
* Data ready in forced mode is assumed to set `INT_STATUS.drdy` like in
  normal mode; it is not yet verified on hardware.
* Self-test (X/Y only, multi-step in SensorAPI) is not implemented.
