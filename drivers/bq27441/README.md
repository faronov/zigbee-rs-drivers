# zigbee-bq27441

TI BQ27441-G1 / BQ27421-G1 / BQ27426 Impedance Track 1-cell Li-ion fuel
gauges over I2C.

| | |
|---|---|
| Address | 0x55 (fixed) |
| Conversion | autonomous: voltage, average current and temperature once per second |
| Power-down current | hibernate 9 µA, sleep 21 µA (automatic), normal 93 µA; shutdown 0.6 µA (RAM lost) |
| Output | voltage mV, average current µA, average power mW, SOC 0.5 %, temperature 0.01 °C; remaining / full-charge capacity mAh, SOH %, Flags |
| Calibration | `BatteryConfig` (design capacity, design energy, terminate voltage, taper rate) written to the State subclass (82) in CONFIG UPDATE mode |
| Source | TI SLUUAC9A (bq27441-G1 TRM), SLUUAC5 (bq27421-G1 TRM), SLUUBB0 (bq27426 TRM), SLUSBH1C (bq27441-G1 data sheet) |
| Status | mock-tested |

Notes:

- `read_measurement` reads standard commands 0x02..=0x21 in one
  incremental read (the reserved 0x16 and 0x1A words are skipped when
  decoding). The data sheet allows at most two standard commands per second.
  Faster polling can trigger a watchdog reset, so do not read more often
  than needed. Data refreshes once per second anyway.
- Bus timing: the data sheet requires ≥ 66 µs bus-free time between
  packets. Above 100 kHz SCL it also requires 1-byte writes; enable them
  with `with_byte_writes(true)`. The driver issues the Control() write and
  read back-to-back, which is fine at 100 kHz. At 400 kHz the platform's I2C
  must provide t(BUF).
- POR handling: configuration is RAM only. Flags.ITPOR (`needs_config()`)
  means the ROM defaults are back. Design-capacity update (TRM 4.1
  example):
  1. `unseal()` (0x8000 twice).
  2. `enter_config_update()`, then poll `in_config_update()` every 500 ms
     until true (≤ 1 s).
  3. `configure_battery(&BatteryConfig::new(capacity_mah,
     terminate_mv, taper_current_ma))`. It reads State block 0, patches the
     big-endian parameters and writes them. It then writes the new
     checksum `255 − Σbytes`, which commits the block.
  4. `exit_config_update()` (SOFT_RESET), then poll until false.
  5. `seal()`.
  `write_data_memory(class, offset, bytes)` and `read_data_memory` reach
  any other parameter (OpConfig, Hibernate Current, …).
- `BatteryConfig::new` sets Design Energy = capacity × 3.7 V and
  Taper Rate = capacity / (0.1 × taper current) (TI quick-start formula).
  All values are clamped to the TRM ranges.
- State-subclass offsets differ by part: BQ27441/BQ27421 use 10/12/16/27,
  and BQ27426 uses 6/8/10/21 for the same four parameters. The BQ27426
  update example reuses 0x4A (offset 10) from the BQ27441 TRM. The driver
  uses the offsets from the parameter table, which the Linux bq27xxx driver
  also uses.
- `probe` checks DEVICE_TYPE: 0x0421 for BQ27441/BQ27421. For the BQ27426
  it accepts 0x0426 or 0x0421, because SLUUBB0 lists both.
- `LowPower::sleep` sends SET_HIBERNATE. The gauge only hibernates after a
  valid OCV and below Hibernate Current. `wake` sends CLEAR_HIBERNATE and
  returns one measurement period. `shutdown()` (unsealed) is the 0.6 µA
  mode; to exit it, raise GPOUT from low to high for ≥ 200 µs.
- Temperature is the internal sensor unless OpConfig.TEMPS = 1, in which
  case the host supplies it with `write_temperature`.
- Remaining and full-charge capacity and SOH are reported in the struct
  only. Core `Quantity` has no capacity variant.
