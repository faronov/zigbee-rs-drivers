# zigbee-opt3001

TI OPT3001, OPT3004 and OPT3006 ambient light sensors (human-eye response,
0.01–83 865 lx) over I2C. The three parts share the register map, the
addresses and the device ID; they differ in package and filter.

| | |
|---|---|
| Address | 0x44 (ADDR=GND), 0x45 (VDD), 0x46 (SDA), 0x47 (SCL) |
| ID | manufacturer 0x5449, device 0x3001 (`probe()`) |
| Mode | single shot with automatic full-scale range; shuts down by itself |
| Conversion | 100 ms (110 max) or 800 ms (880 max) integration + 3 ms; auto-range adds a 10 ms assessment → driver waits 126 / 896 ms |
| Shutdown current | 0.3 µA typ., 0.47 µA max |
| Output | milli-lux + raw E/M word, OVF and limit flags |
| Interrupt | low/high limits (milli-lux, encoded to E/M), latched or hysteresis, fault count, polarity, end-of-conversion mode |
| Source | TI SBOS681C (OPT3001), SBOS929A (OPT3004), SBOS698 (OPT3006) |
| Status | mock-tested |

Wiring: INT is open drain (pull-up). Reading the configuration register
(done by `read_measurement`) clears the conversion-ready flag and latched
limit flags.
