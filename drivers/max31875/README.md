# zigbee-max31875

Analog Devices (Maxim) MAX31875 low-power ±1 °C temperature sensor in a
0.84 mm × 0.84 mm WLP, over I2C / SMBus with optional PEC.

| | |
|---|---|
| Address | 0x48–0x4F, factory-programmed by ordering option R0–R7 (`address(n)`) |
| Conversion | one-shot from shutdown (D0); 10-bit max 50 / 70 / 150 ms at VDD ≥ 3 V / 2.5–3 V / ≤ 2.5 V, ×2 per extra bit |
| Resolution | 8 / 9 / 10 / 12 bit (1 … 0.0625 °C), normal or extended (+150 °C) format |
| Rates | 0.25 / 1 / 4 / 8 conversions/s (automatic mode) |
| Shutdown current | 0.5 µA typ., 3 µA max (timeout disabled); 0.7 / 3.5 µA with timeout |
| Alert | **no pin**: T_OS / T_HYST comparator or interrupt, fault queue 1/2/4/6, OT status bit D15 |
| ID | none |
| Output | 0.01 °C |
| Source | Maxim 19-100110 Rev 8 (5/21) |
| Status | mock-tested |

## Usage notes

* `configure()` enters shutdown; `start_measurement()` writes D8 = D0 = 1
  and the part returns to shutdown after the conversion.
  `LowPower::sleep` = shutdown, `wake` = automatic conversions.
* The wait is the 10-bit maximum for `Config::supply`, scaled by
  resolution, plus the data sheet's first-conversion overhead
  (10 / 15 / 30 ms). The default assumes VDD ≤ 2.5 V (180 ms at 10 bit);
  set `Supply::Above3V` for a regulated 3.3 V rail (60 ms).
* Without an ALERT pin the threshold logic cannot wake the MCU, and
  entering shutdown resets OT status. Compare readings in firmware, or
  run automatic mode and poll `over_temperature()`.
* After toggling `Config::extended` the temperature format changes only
  after the next conversion (allow two conversion times). T_OS / T_HYST
  are not reformatted; `set_limits()` encodes them in the configured
  format, so call it again after switching.
* PEC: the configuration write that sets the PEC bit is sent without a
  PEC byte; afterwards all transfers carry one and read errors return
  `Error::Crc`. If only the MCU resets, build the driver with
  `with_pec_active(true)`.
