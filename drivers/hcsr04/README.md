# zigbee-hcsr04

HC-SR04, RCWL-1601 and pin-compatible ultrasonic rangers (TRIG/ECHO).

| | |
|---|---|
| Interface | TRIG output pin, ECHO input pin (+ `Wait` for async), µs clock |
| Range | 2 cm – 4 m (HC-SR04), 2 cm – 4.5 m (RCWL-1601) |
| Measurement | 10 µs trigger, echo pulse = round-trip time (≈ 5.8 ms per metre); ≥ 60 ms between measurements |
| Output | `Quantity::Distance` in mm, temperature-compensated speed of sound (331.3 + 0.606·T m/s) |
| Current | HC-SR04 15 mA working; RCWL-1601 ≈ 2 mA. No sleep mode: switch the supply with a GPIO/MOSFET on battery nodes |
| Source | Elecfreaks "Ultrasonic Ranging Module HC-SR04" datasheet; RCWL-1601: no factory datasheet, reseller specs (HC-SR04 compatible) |
| Status | mock-tested |

Timing exception: the 10 µs trigger pulse uses `DelayNs`, and the
bit-banged `Sensor` path busy-polls ECHO for up to the timeout (default
30 ms). On a Zigbee node prefer `trigger()` + a timer input capture +
`echo_to_mm()` / `feed_echo()`, or the async driver.

Wiring notes:

* HC-SR04 is a 5 V part: ECHO is 5 V, use a divider (e.g. 10 k / 20 k) into
  a 3.3 V MCU. TRIG accepts 3.3 V logic.
* RCWL-1601 runs from 3 – 5.5 V with logic at the supply level; use it
  directly on 3.3 V MCUs.
* The 38 ms "no echo" pulse often quoted for HC-SR04 is not in the
  datasheet; clones vary, so the driver uses its own timeout.
