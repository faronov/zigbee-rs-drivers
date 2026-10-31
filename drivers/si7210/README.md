# zigbee-si7210

Silicon Labs Si7210 I2C hall-effect magnetic position and temperature
sensor. All I2C variants are covered. The part is marked "Not Recommended for
New Designs".

| | |
|---|---|
| Address | 0x30–0x33, fixed by part number |
| Conversion | single burst: 11 µs + 8.8 µs per extra averaged sample (max 4096 samples ≈ 36 ms) |
| Output | `Quantity::MagneticFlux` in µT (nT in `Measurement`); optional `Quantity::Temperature` in 0.01 °C |
| Scale | ±20.47 mT (1.25 µT/LSB) or ±204.7 mT (12.5 µT/LSB) |
| Current | 50 nA full sleep, ≈ 0.4 µA with the sleep timer, 360–1000 µA idle, 3.5–8.5 mA converting |
| Output pin | threshold/hysteresis switch, omnipolar/unipolar/latch (`set_switch`, kept across sleep via `Usestore`) |
| Source | Silicon Labs Si7210 datasheet Rev. 1.3 |
| Status | mock-tested |

Notes:

* Call `init()` once. It checks the chip ID and loads the OTP temperature
  offset and gain.
* Each `start_measurement` wakes the part with a bare address write. It then
  rewrites the cached scale coefficients and filter, because sleep reloads
  them from OTP, and triggers one burst. Afterwards `read_measurement` enters
  `AfterRead`:
  * `Periodic` (default): the factory sleep-timer loop, which keeps the
    output pin working;
  * `Sleep`: about 50 nA;
  * `Idle`.
* The temperature comes from the same burst. The OTP gain and offset are
  applied, along with the −0.222 °C/V × VDD term (`set_vdd_mv`, default
  3.3 V).
* The OTP coefficient sets used by `set_scale` read about 0.1 mT high when
  averaging is off (datasheet §4.1.2).
* Datasheet errata:
  * the Bop/Brp formulas quote 0.05 mT/bit, but the 13-bit comparator LSB is
    5 µT (20 mT scale);
  * the tamper formula lacks the ×32 factor that its own examples use.
    Tamper and the sleep timer interval are factory-set and not exposed.
