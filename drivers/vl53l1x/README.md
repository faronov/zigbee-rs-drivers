# zigbee-vl53l1x

ST VL53L1X (up to 4 m) and VL53L4CD (up to 1.3 m) time-of-flight ranging
sensors, following ST's Ultra Lite Drivers.

| | |
|---|---|
| Parts | VL53L1X (model ID 0xEACC), VL53L4CD (0xEBAA) |
| Address | 0x29 (changeable with `set_address`, volatile) |
| Output | `Quantity::Distance` (mm) when the range status is 0; status, signal/ambient rates, SPAD count and (VL53L4CD) sigma in `Measurement` |
| Timing | VL53L1X budget 15 (short mode)/20/33/50/100/200/500 ms; VL53L4CD 10–200 ms |
| Current | tens of mA while ranging, µA-level in software standby (see the datasheets) |
| Source | ST VL53L1X ULD (STSW-IMG009) and VL53L4CD ULD (STSW-IMG026), stm32duino ports, BSD-3-Clause |
| Status | mock-tested |

Notes:

* The default configuration blob, register map, VL53L1X timing tables and
  range-status mapping come from ST's BSD-3-Clause ULD sources. The ST
  copyright notice is kept in `src/lib.rs`.
* Initialisation has two steps, with no waiting inside the driver.
  `begin_init` returns `NotReady` until the firmware has booted (register
  0xE5 = 3), loads the blob and starts the VHV calibration ranging.
  `finish_init` returns `NotReady` until that ranging ends, then stops it
  and applies the post-init settings. After init, the VL53L1X runs long
  mode with about 100 ms; the VL53L4CD runs 50 ms continuous.
* `start_measurement` starts ranging. `read_measurement` returns
  `NotReady` until a result is ready, then reads the result block in one
  transfer, clears the interrupt and stops ranging. For continuous use,
  call `start_ranging`, `read_result` and `stop_ranging` yourself.
* Like the VL53L4CD ULD, the driver drops the first VL53L4CD result after
  each start. The wait returned for the VL53L4CD therefore covers two
  periods.
* `set_timing(budget, inter)`: `inter = 0` ranges back-to-back. Otherwise
  `inter` must exceed the budget on the VL53L4CD (autonomous low-power
  mode) and be at least the budget on the VL53L1X. The ULD's float
  factors (1.075 and 1.055) are applied in integer arithmetic.
* The distance mode and the ROI (4–16 SPADs) are VL53L1X only. On the
  VL53L4CD they return `Unsupported`. Changing the mode re-applies the
  timing budget; the 15 ms budget exists only in short mode.
* `set_thresholds` restricts the interrupt to a distance window. On the
  VL53L1X, the ULD ORs the new window into the old bits; this driver
  replaces them instead.
* `set_offset`, `set_xtalk`, `set_signal_threshold` and
  `set_sigma_threshold` follow the ULD register formats. ST's offset and
  crosstalk calibration procedures (ranging a target) are left to the
  application.
