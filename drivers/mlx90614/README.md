# zigbee-mlx90614

Melexis MLX90614 and MLX90615 infrared (non-contact) thermometers over
SMBus with PEC. They report the object temperature and the ambient
(package) temperature.

| | MLX90614 | MLX90615 |
|---|---|---|
| Address | 0x5A (EEPROM-programmable) | 0x5B (EEPROM-programmable) |
| Conversion | continuous, EEPROM IIR/FIR filter; first data 250 ms typ. after wake (driver: 375 ms) | continuous, EEPROM IIR; first data 300 ms typ. after wake (driver: 450 ms) |
| Resolution | 0.02 K | 0.02 K |
| Rates | free-running, set by the EEPROM filter configuration | same |
| Shutdown current | sleep 2 µA typ., 6 µA max (3 V parts only; no sleep on the 5 V MLX90614Axx) | sleep 1.1 µA typ., 3 µA max (SCL and SDA high) |
| Alert | none (the PWM / thermal-relay modes are not supported) | none |
| ID | factory ID number at EEPROM 0x1C–0x1F; `probe` checks the stored SMBus address (0x0E) | ID at EEPROM 0x0E–0x0F; SMBus address at 0x00 |
| Output | object + ambient, 0.01 °C | object + ambient, 0.01 °C |
| Source | MLX90614 data sheet rev 021 | MLX90615 data sheet rev 001 |
| Status | mock-tested | mock-tested |

## Usage notes

* PEC is CRC-8 with polynomial 0x07 over every byte, including the address
  bytes. Reads with a bad PEC return `Error::Crc`. A RAM word with the
  error flag (bit 15) set returns `Error::InvalidData`.
* `Readings` emits `Quantity::ObjectTemperature` for the object, then
  `Quantity::Temperature` for the ambient (package) temperature.
* The sensor converts continuously. `start_measurement()` issues no bus
  traffic and returns 0; `read_measurement()` reads the latest object and
  ambient values. `read_object2()` reads zone 2 of dual-zone MLX90614
  parts.
* `read_emissivity()` returns ε × 10 000. Emissivity is not written:
  changing it means an EEPROM erase/write, and the data sheets warn
  against altering calibration cells.
* **Sleep / wake:** `LowPower::sleep` sends the Enter SLEEP command
  (0xFF on MLX90614, 0xC6 on MLX90615). Waking needs the bus pins
  driven as GPIOs, which the driver cannot do:
  * MLX90614: hold SDA low for more than 33 ms while SCL stays high.
  * MLX90615: hold SCL low for at least 50 ms. Keep SCL high during sleep.

  `Variant::wake_pulse_us()` gives the pulse length. After generating the
  pulse, call `LowPower::wake`: it issues no bus traffic and returns the
  wait until data is valid.
* The SMBus clock must be 10–100 kHz. The parts support only Read Word
  and Write Word, not byte protocols.
