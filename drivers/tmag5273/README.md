# zigbee-tmag5273

Texas Instruments TMAG5273 low-power linear 3D hall-effect sensor with I2C.

| | |
|---|---|
| Parts | TMAG5273A1/B1/C1/D1 (±40/±80 mT), A2/B2/C2/D2 (±133/±266 mT) |
| Address | A 0x35, B 0x22, C 0x78, D 0x44 (re-assignable at run time) |
| Output | `Quantity::MagneticField` (X/Y/Z nT), optional `Quantity::Temperature` (0.01 °C) and `Quantity::Angle` (0.01°, when angle calculation is on); magnitude in `Measurement` |
| Conversion | 70 µs + 25 µs + 25 µs × averages × channels (1×–32× averaging) |
| Current | 2.3 mA active (3.0 mA low-noise), 0.45 mA standby, 5 nA sleep |
| Source | TI datasheet SLYS045C |
| Status | mock-tested |

Notes:

* `probe()` checks the "TI" manufacturer ID (0x5449) and reads DEVICE_ID
  to pick the ±40 or ±133 mT scaling, so call it before reading.
* The default mode is triggered standby. `start_measurement` sets the
  trigger bit in the register-address byte (`0x98`) and returns the
  conversion time. One 12-byte burst then reads the temperature, X/Y/Z,
  CONV_STATUS, angle and magnitude. It returns `NotReady` while
  `RESULT_STATUS` is clear.
* `Mode::Continuous` returns 0 from `start_measurement`.
  `Mode::WakeUpAndSleep` duty-cycles at `SleepTime` and drives INT with the
  `set_thresholds` limits.
* `sleep()` selects sleep mode. The part does not acknowledge its address
  while asleep, so `wake()` ignores the NACK, restores the configured mode
  and returns 50 µs. Results are not retained across sleep.
* CRC and the reduced 1-byte/8-bit read modes are not used.
* The angle (1/16° register) is emitted as `Quantity::Angle` in 0.01°. The
  raw value and the vector magnitude are in `Measurement`.
