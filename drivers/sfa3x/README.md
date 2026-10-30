# zigbee-sfa3x

Driver for the Sensirion SFA30 formaldehyde sensor module over I2C. The SFA30 reports HCHO in ppb, plus relative humidity and temperature from its on-board SHT sensor.

The driver supports:

- start and stop of continuous measurement
- reading the measured values
- device marking (`probe` checks that it is printable ASCII)
- soft reset

| | |
|---|---|
| Address | 0x5D |
| Conversion | updates internally every 0.5 s (2 Hz); the first data arrives 0.5 s after start. A read returns the average since the previous read: the arithmetic mean below 60 s, exponential smoothing beyond that. Read every 0.5–60 s. Waits: read 5 ms, start 1 ms, stop 50 ms, reset 1 s. |
| Output | HCHO in ppb (rounded from 0.2 ppb; negative values clamp to 0); RH 0.01 %RH; T 0.01 °C (sensor resolution 0.005 °C, truncated) |
| Warm-up | HCHO reads 0 for the first 10 s after power-up |
| Current | measuring 1.0 mA avg (2.0 mA max; 2.5–5 mA peak); idle 0.5 mA (1.0 mA max); **no sleep mode** |
| Source | Sensirion "Description of the SFA30 I2C Interface", V1.0 (July 2022, D1, preliminary); "Data Sheet SFA30", V1.3 (January 2025, D1). The start, stop and reset times come from Sensirion's `embedded-i2c-sfa3x` driver, because the I2C note does not specify them. |
| Status | mock-tested |

## Wiring

- **Connector:** Molex Micro-Lock Plus, 7 pins. Pin 1 is VDD, pin 2 GND, pin 3 SDA (RX in UART mode), pin 4 SCL (TX in UART mode), and pin 5 SEL. **Tie SEL to GND to select I2C**; left floating, the module uses UART.
- **Supply:** VDD is 3.15–5.5 V.
- **Pull-ups:** use external pull-ups, e.g. 10 kΩ.
- **Bus:** up to 100 kHz. Keep cables under 10 cm.

## Measurement cycle

`start_measurement` starts continuous measurement on its first call and returns 500 ms; the read that follows is `NotReady`. Each later call sends `Read Measured Values` and returns 5 ms. If the sensor NACKs the read because it is busy or has no data yet, the driver returns `NotReady`.

## Power

There is no sleep mode, so the crate has no `LowPower` implementation. To save power, stop measuring (idle 0.5 mA), or switch the supply off. Switching it off restarts the 10 s HCHO warm-up.
