# zigbee-dht

Aosong DHT11 and DHT22 / AM2302: temperature and relative humidity over the
single-wire protocol, bit-banged on one open-drain GPIO.

| | |
|---|---|
| Interface | one GPIO (`InputPin + OutputPin`, open-drain, 4.7–10 kΩ pull-up) + `DelayNs` for bit timing |
| Start pulse | host low 18 ms (DHT11) / 1.1 ms (DHT22), returned as `WaitUs` |
| Answer | ~5 ms bit-banged in `read_measurement`; mask interrupts around it |
| Interval | ≥ 2 s between reads, ≥ 1 s after power-up |
| Output | 0.01 °C, 0.01 %RH (DHT11: 1 °C / 1 %RH, newer parts 0.1 °C) |
| Features | checksum, pure `decode_frame` (5 bytes) and `decode_pulses` (40 high times, for input capture / RMT) |
| Source | Aosong DHT11 manual, Aosong AM2302 manual |
| Status | simulated-line tested |

Bits are decided by comparing each bit's high time with its 50 µs low
preamble, so the polling loop speed does not need calibrating. An
interrupt during the ~5 ms answer corrupts the frame (`Error::Crc`) or
loses an edge (`Error::Timeout`); retry after the 2 s interval.

The async driver awaits the start pulse; bit timing still needs a blocking
`DelayNs`.

DHT11 data sheets differ on the start pulse (Aosong: ≥ 1–10 ms, older
D-Robotics: ≥ 18 ms); the driver uses 18 ms. Newer DHT11 parts report
negative temperatures with bit 7 of the temperature tenths byte, which the
decoder honours.
