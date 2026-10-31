# zigbee-lc709203f

onsemi LC709203F 1-cell Li-ion fuel gauge (HG-CVR, no sense resistor)
over I2C with CRC-8.

| | |
|---|---|
| Address | 0x0B (fixed) |
| Conversion | autonomous; the sampling rate adapts to the charge/discharge rate |
| Power-down current | sleep 1 µA typ (2 µA max); operational 3 µA typ |
| Output | cell voltage mV, ITE 0.5 %, cell temperature 0.01 °C (thermistor mode); RSOC 1 % via `rsoc()` |
| Calibration | `Config`: APA (`typical_apa(capacity_mah)` from Table 7), battery profile (0x12), thermistor B constant or host temperature, optional Initial RSOC |
| Source | onsemi LC709203F/D Rev. 3 |
| Status | mock-tested |

Notes:

- Every transfer carries CRC-8-ATM (poly 0x07, init 0) over the address
  byte(s), command and data. Reads are verified (`Error::Crc`), and writes
  without a valid CRC are ignored by the IC. Both data sheet examples are
  tests.
- Start-up follows Figures 20/21:
  1. `start_init()` sets operational mode and writes the profile only if
     it differs. A profile change re-initialises the IC and returns
     `INIT_US` (90 ms).
  2. `finish_init()` writes operational mode again, APA, optional Initial
     RSOC (0x07 = 0xAA55), and Status Bit (thermistor or I2C mode) with
     the B constant.
  Default `Config`: APA 0x10 (500 mAh), profile 1 (3.7 V / 4.2 V on the
  -01/-03 parts, UR18650ZY on -04), I2C temperature mode.
- The WLCSP (-xMH) parts go to sleep by themselves 1.2 s after power-on
  and after a profile change. In sleep, I2C needs a wake pulse first.
  `LowPower::wake` sends an operational-mode write as the SDA pulse (its
  NACK is ignored) and returns TWU (400 µs). The next `start_measurement`
  writes operational mode again. If the part may be asleep, call `wake()`
  before `start_init()`.
- `LowPower::sleep` writes sleep mode. RSOC tracking stops and charge moved
  while asleep is lost. Operational mode is only 3 µA, so many designs
  never sleep it.
- Temperature: in I2C mode, the host writes `set_temperature` (−20…+60 °C
  clamp) and the reading omits temperature. In thermistor mode, 0x08 is
  measured and reported. The register uses 0 °C = 0x0AAC (273.2 K).
- `probe` reads Number of the Parameter (0x1A). The CRC check is the
  identity test because the part has no fixed ID; 0x0000 and 0xFFFF are
  rejected.
- I2C clock stretching and a 10 s bus time-out apply. Shared buses must
  wake the gauge before other traffic (data sheet note).
