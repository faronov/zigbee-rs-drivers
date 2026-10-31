# zigbee-icp101xx

TDK InvenSense ICP-10100, ICP-10101, ICP-10110 and ICP-10111 barometric
pressure (300-1100 hPa) and temperature sensors. All four share the die
and command set (ICP-10100/10110: 3-hole IPx8 lid; ICP-10101/10111:
1-hole lid; 10100/10101 2×2 mm LGA-10, 10110/10111 2×2.5 mm LGA-8).

| | |
|---|---|
| Bus | I2C only, fixed address 0x63, ≤ 400 kHz |
| Mode | command per measurement; the part idles automatically afterwards |
| Conversion (max) | Low Power 1.8 ms, Normal 6.3 ms (default), Low Noise 23.8 ms, Ultra Low Noise 94.5 ms |
| Supply current | 1.3 / 2.6 / 5.2 / 10.4 µA at 1 Hz per mode; 1.0 µA idle |
| Output | 0.01 °C, Pa |
| Source | ICP-101xx datasheet DS-000186 revision 1.2 |
| Status | mock-tested |

Notes:

* `init()` reads the ID (product code bits 5:0 = 0x08, else
  `UnexpectedId`) and the four OTP constants; every word is CRC-8 checked
  (`Error::Crc`).
* Pressure uses the datasheet's three-point formula `P = A + B/(C + p)`,
  re-derived as a closed integer form (see the crate docs). Host tests
  compare it with the datasheet float algorithm over 20 000 random OTP
  sets: worst difference < 0.52 Pa (the rounding to 1 Pa). Uses one u64
  division per sample; no float.
* The sensor NACKs all I2C traffic while converting, so a
  `read_measurement()` before the returned wait fails with a bus error, not
  `NotReady`.
* Wiring: VDD is 1.8 V ±5 % (1.71-1.89 V); SCL/SDA must not exceed VDD,
  so a 3.3 V MCU needs a level shifter. Power-up and soft reset take
  170 µs.
