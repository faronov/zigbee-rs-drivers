# zigbee-sps30

Sensirion SPS30 optical particulate matter sensor, used over I2C. The driver uses the unsigned 16-bit integer output format. It covers:

- start and stop measurement, data-ready flag, measured values
- PM1.0, PM2.5, PM4.0 and PM10 mass concentration in 0.1 µg/m³ (1 µg/m³ resolution)
- number concentrations (#/cm³) and typical particle size (nm)
- sleep and wake-up (`LowPower`)
- manual fan cleaning and the auto-cleaning interval
- device status register (read and clear)
- product type (`probe`), serial number, firmware version, reset

| | |
|---|---|
| Address | 0x69 |
| Conversion | free-running, a new sample every 1 s; stable after 8–30 s depending on concentration |
| Current | measuring 55 mA (80 mA during the first 200 ms of fan start-up); idle 330 µA; **sleep 38 µA** (max 50 µA) |
| Commands | start/stop 20 ms, sleep/wake-up 5 ms, reset 100 ms |
| Source | Sensirion Datasheet SPS30, version 2.0 (June 2023, D1) |
| Status | mock-tested |

## Wiring

- **Supply:** VDD is 4.5–5.5 V.
- **Interface select:** to select I2C, tie SEL (pin 4) to GND before or at power-up.
- **Pull-ups:** the bus needs external pull-ups, e.g. 10 kΩ. VIH min is 2.31 V, so 3.3 V pull-ups work.
- **Speed:** up to 100 kHz, with no clock stretching.
- **Cable length:** keep I2C cables under 10 cm. Sensirion recommends UART (not supported here) for longer runs.

## Firmware requirements

- Firmware ≥ 2.0: the integer output format and sleep / wake-up.
- Firmware ≥ 2.2: the device status register.

Check with `firmware_version()`.

## Sleep and wake

- **Sleep:** allowed only from Idle. While measuring, `sleep()` returns `NotReady` without bus traffic. Call `stop_measurement()` first and wait 20 ms.
- **Wake:** `wake()` sends the wake-up command twice, as the datasheet allows. The first one only re-enables the I2C interface, and its NACK is ignored.
- **Reset:** to reset the sensor while it sleeps, wake it first.

## Fan cleaning

Auto-cleaning runs every 168 h by default, but the counter restarts at power-off. If the sensor is power-cycled often, call `start_fan_cleaning()` (Measurement-Mode only) at least once a week.

## Number concentrations and particle size

These are decoded into `Measurement`, but `Readings` emits only the four PM mass quantities. The core has no quantity for number concentration or particle size.
