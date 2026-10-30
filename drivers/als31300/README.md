# zigbee-als31300

Allegro ALS31300 3D linear hall-effect sensor with I2C and low-power modes.

| | |
|---|---|
| Parts | ALS31300EEJASR-500 / -1000 / -2000 (4/2/1 LSB/G), -JOY (X/Y 1, Z 0.25 LSB/G) |
| Address | 96–110 via the ADR0/ADR1 dividers (EEPROM address when both pins are at VCC) |
| Output | `Quantity::MagneticField` (X/Y/Z nT) and `Quantity::Temperature` (0.01 °C) |
| Update | free-running; 160/330/495 µs for 1/2/3 channels at BW 0 |
| Current | 3.4 mA active, 14 nA sleep, 12 µA–2 mA duty-cycled (LPDCM) |
| Source | Allegro ALS31300 datasheet Rev. 14 |
| Status | mock-tested |

Notes:

* The part has no ID register, so there is no probe. Choose `Variant` from
  the part-number suffix. It sets the nT/LSB scaling and cannot be read back.
* `start_measurement` returns 0. `read_measurement` reads 0x28/0x29 in one
  8-byte transfer and returns `NotReady` while New Data is clear.
* Writing anything except the sleep bits needs the customer access code
  (0x35 ← 0x2C413534). The driver sends it before its first write. After a
  power cycle, call `unlock()` again.
* `set_measurement` (bandwidth, channel enables, INT latch) and
  `set_interrupt` write EEPROM, which is rated for 1000 cycles. They
  read-modify-write, skip unchanged values and return the 50 ms tEEP to
  wait.
* `sleep()` enters 14 nA sleep without the access code. `wake()` returns
  1.1 ms: tPOD (600 µs) plus one update. `set_power_mode(DutyCycle(..))`
  selects LPDCM with the inactive time.
* I2C CRC and the fast/full loop read modes are not supported. Keep EEPROM
  0x02 bit 18 cleared.
* The interrupt threshold equations lost their superscripts in the
  datasheet. `(code + 1) × 2⁵ − 1` and `× 2⁶` are the readings consistent
  with a 12-bit range.
