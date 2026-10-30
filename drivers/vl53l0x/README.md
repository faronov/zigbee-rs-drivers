# zigbee-vl53l0x

ST VL53L0X time-of-flight ranging sensor (up to ≈2 m), following the
minimal path of ST's VL53L0X API.

| | |
|---|---|
| Parts | VL53L0X (model ID 0xEE at 0xC0) |
| Address | 0x29 (changeable with `set_address`, volatile) |
| Output | `Quantity::Distance` (mm) when the API range status is 0; status, signal/ambient rates and effective SPAD count in `Measurement` |
| Timing | timing budget ≥ 20 ms, ≈33 ms after init; single-shot wait = budget + 10 % |
| Current | ≈19 mA while ranging, ≈5 µA in software standby between single shots (datasheet) |
| Source | ST VL53L0X API (STSW-IMG005), stm32duino `VL53L0X` port, BSD-3-Clause |
| Status | mock-tested |

Notes:

* ST publishes no register map for the VL53L0X. The init sequences,
  default tuning table (80 single-byte writes), reference-SPAD selection,
  timing-budget arithmetic and range-status mapping come from ST's
  BSD-3-Clause API sources. Both ST notices are kept in `src/lib.rs`.
* Initialisation has two steps, with no waiting inside the driver.
  `begin_init` runs `DataInit` and `StaticInit` and starts the VHV
  calibration. `finish_init` returns `NotReady` until the VHV and then
  the phase calibration have finished; call it again after
  `CAL_WAIT_US`. The calibration durations are not documented, so that
  wait is an estimate. Power-up boot (≈1.2 ms after XSHUT) is the
  application's job.
* Reference SPADs are taken from the NVM data. If that data is out of
  range, `begin_init` returns `Unsupported`, because ST's reference SPAD
  management procedure is not implemented. The NVM strobe poll is
  bounded to 2000 reads with no delay (API `VL53L0X_DEFAULT_MAX_LOOP`)
  and returns `Timeout` if it never completes. ST inserts an OS delay
  before the poll; this driver does not.
* The API limit checks are applied as their net register effect: the
  MSRC and pre-range signal-rate checks are disabled, and the final-range
  minimum signal rate is 0.25 MCPS. The sigma check is computed on the
  host in the API; it is not implemented, so range status 1 never
  occurs.
* After init, TCC and MSRC are disabled (sequence 0xE8) and the final
  range timeout is recalculated to keep the budget read at startup, as
  `StaticInit` does. `set_timing_budget` recomputes the final range
  timeout from the current sequence and VCSEL periods. VCSEL period
  changes (long-range mode) are not exposed.
* `start_measurement` starts one single-shot ranging. ST then polls
  SYSRANGE_START until it clears; this driver relies on the interrupt
  status instead. `read_measurement` returns `NotReady` until the
  interrupt status is set, reads the 12-byte result block and clears the
  interrupt (up to three attempts, else `Timeout`).
* The I/O pads default to 1.8 V. Boards with 2.8–3.3 V I2C pull-ups
  usually need `enable_io_2v8()` before `begin_init` (API `USE_I2C_2V8`).
* Offset and crosstalk calibration, continuous modes and the
  interrupt-threshold modes are left out.
