# zigbee-pac194x

Microchip PAC1941-1/-2, PAC1942-1/-2, PAC1943-1 and PAC1944-1: one to four
channel 9 V power monitors with on-chip power product and 56-bit energy
accumulators, I2C/SMBus.

| | |
|---|---|
| Address | 0x10..=0x1F (ADDRSEL resistor; GND = 0x10) |
| Sampling | continuous 1024/256/64/8 SPS (default 8 SPS: 12 µA typ with 4 channels); REFRESH snapshot stable after 1 ms |
| Sleep current | 5 µA typ (Sleep sample mode); 1 ms band-gap start on wake |
| Output | per enabled channel: bus voltage mV, current µA, power mW, energy mWh |
| Calibration | `ChannelConfig::new(shunt_µΩ)` / `ChannelConfig::with_ranges(shunt_µΩ, vbus, vsense)` (unipolar, bipolar ±FSR, bipolar ±FSR/2); shunt ≥ 50 µΩ |
| Source | Microchip DS20006543C |
| Status | mock-tested |

Notes:

- `start_measurement` sends REFRESH_V (snapshot, accumulators kept) and
  returns 1 ms; `read_measurement` reads VBUS/VSENSE (8-sample rolling
  averages by default, `with_averaging(false)` for the latest sample),
  VPOWER and VACC of each enabled channel.
- Single-shot mode is not used: its results need a second REFRESH 1 ms
  after the conversion. Continuous 8 SPS costs about as much as sleeping.
- `init` writes the ranges, ACCUM_CONFIG = VPOWER and CTRL (disabled
  channels set in CHANNEL_N_OFF), then REFRESH (clears accumulators); wait
  the returned 1 ms, the part NACKs commands meanwhile.
- Energy uses Equation 5-9 with the fixed sample rate (a power of two, so
  the division is a shift; 128-bit multiply). It is valid only while the
  SLOW pin stays low (SLOW forces 8 SPS) and the rate is the internal
  oscillator rate (no tolerance given). The accumulator overflows after
  2^26 full-scale samples (18 h at 1024 SPS, 97 days at 8 SPS);
  `reset_accumulators` sends REFRESH. `accumulator_count` reads ACC_COUNT.
- Power uses the datasheet denominators (2^30, 2^29 if VBUS or VSENSE is
  bipolar ±FSR). Bipolar energy can be negative; `Readings` then omits it.
- `sleep`/`wake` switch CTRL between Sleep and the sample rate with
  REFRESH_V; `wake` returns band-gap + one sample period + 1 ms.
- Alert limits, GPIO and SMBus settings are not used by the driver.
