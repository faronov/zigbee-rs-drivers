# zigbee-p3t1755

NXP P3T1755 ±0.5 °C (−20…+85 °C) digital temperature sensor, used in I2C
mode with ALERT output.

| | |
|---|---|
| Address | 0x40–0x5F via tri-state A2/A1/A0 (GND/VCC/SDA/SCL, data sheet table 4); 0x48 with all pins at GND |
| Conversion | one-shot from shutdown (OS bit): 7.8 ms typ., 12 ms max |
| Resolution | 12 bit, 0.0625 °C (fixed) |
| Rates | continuous period 27.5 / 55 / 110 / 220 ms |
| Shutdown current | 0.2 µA typ., 1 µA max (1.8 V, bus idle) |
| Alert | ALERT pin, T_LOW / T_HIGH 12-bit limits, comparator or interrupt, polarity, fault queue 1/2/4/6, SMBus alert response |
| ID | none |
| Output | 0.01 °C |
| Source | NXP P3T1755 data sheet Rev. 1.3 (28 August 2025) |
| Status | mock-tested |

## Usage notes

* `configure()` enters shutdown; `start_measurement()` writes OS = SD = 1
  and the part returns to shutdown after the conversion.
  `LowPower::sleep` = shutdown, `wake` = continuous.
* The configuration register is a single byte; the driver writes and
  reads one byte. OS always reads 0, so completion cannot be polled—wait
  the returned 12 ms.
* For a wake-up ALERT with one-shots, use comparator mode: interrupt-mode
  ALERT is cleared by entering shutdown (and by any register read). The
  power-on fault queue is 2, so the default `Config` needs two
  consecutive out-of-limit one-shots; use `FaultQueue::One` for immediate
  alerts.
* Wait 20 ms after VCC ≥ VPOR before the first transaction. The address
  pins are latched at the first START condition.
* I3C (dynamic address, IBI) is not used by this driver.
