# zigbee-xgzp68xxd

CFSensor XGZP6847D and XGZP6897D calibrated gauge / differential pressure
sensors (vacuum to 1 MPa ranges, factory compensated). Two digital
interfaces exist, each with its own driver:

* `Xgzp`: command protocol at 0x6D (XGZP6897D, and XGZP6847D part numbers
  without a `C`); pressure = signed 24-bit / K, where K depends on the range.
* `XgzpC`: register protocol at 0x58 (`XGZP6847DC…`); pressure =
  raw / 2^21 · (PMAX − PMIN), with temperature calibration registers.

| | |
|---|---|
| Bus | I2C only; 0x6D (`Xgzp`) or 0x58 (`XgzpC`) |
| Mode | `Xgzp`: combined T + P command; `XgzpC`: one-shot, two conversions per sample (temperature, then pressure) |
| Conversion | `Xgzp`: 20 ms (datasheet); `XgzpC`: 20 ms per conversion (assumed, see notes) |
| Standby current | XGZP6897D < 100 nA (5 µA per single measurement); XGZP6847D 100 nA (1.7-1.8 mA operating) |
| Output | 0.01 °C; signed Pa and 0.01 Pa |
| Source | XGZP6897D datasheet V2.7 (2023-02); XGZP6847D datasheets V2.8 (2024-09, 0x6D) and V3.2 (2026-08, 0x58) |
| Status | mock-tested |

Notes:

* Pressure is signed (vacuum and differential ranges). `Readings` emits it
  as `Quantity::GaugePressure` (Pa, may be negative), never as barometric
  `Quantity::Pressure`; `Measurement::pressure_centi` keeps 0.01 Pa.
* `Xgzp::new(i2c, k)` (`k = 0` gives `InvalidArgument` at start): take K from `k_for_full_scale(largest absolute
  pressure of the range in Pa)` (−100…100, −100…0 and 0…100 kPa all use
  64; up to 1.5 MPa) or from the part's datasheet. Below 1 kPa it returns
  the XGZP6897D value 8192 (the XGZP6847D table stops at 1 kPa). There is
  no ID register. `read_measurement`
  reads the five data registers one at a time as in the datasheet and does
  not poll `Sco` (the datasheet notes polling may not respond and allows a
  20 ms wait instead); `is_busy()` reads `Sco` for callers who want it.
* `XgzpC::new(i2c, pmin, pmax)` with the range in Pa, then `init()`: checks
  the ID register (it holds the address in bits 7:1), reads the
  temperature calibration (0x20/0x21) and switches from the power-on
  normal mode to sleep, keeping the factory oversampling.
  `start_measurement()` returns `InvalidArgument` if `pmax <= pmin` or the
  span exceeds `MAX_SPAN_PA` (5 MPa).
* `XgzpC` two-phase cycle, as in `zigbee-bmp180`: `start_measurement()`
  starts the temperature conversion; the first `read_measurement()` reads
  it, starts the pressure conversion and returns `Error::Pending(wait_us)`;
  wait that long and read again. The `measure()` helpers follow this phase.
* `XgzpC` assumptions: the datasheet gives no conversion time and no ready
  flag, so each phase waits 20 ms (the vendor sample's wait). `ONE_PT` = 1
  is taken to select pressure ("measure the temperature or pressure signal
  by clearing or setting"). The temperature divisor `2^(reg 0x21 / 10)` is
  evaluated with a fixed-point table, so non-multiples of 10 match the
  vendor's `pow()` code.
* Wiring: VDD 2.5–5.5 V (XGZP6897D, XGZP6847D) or 3.0–5.5 V (XGZP6847DC); I2C pull-ups
  to VDD (4.7 kΩ typ.). On a 3.3 V MCU, power the sensor from 3.3 V or use
  a level shifter.
