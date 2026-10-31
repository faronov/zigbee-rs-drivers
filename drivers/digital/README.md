# zigbee-digital

Debounced GPIO detectors: PIR motion modules (HC-SR501, AM312, microwave
modules with a digital output), reed/door contacts, water-leak probes,
digital hall switches and vibration/tilt switches.

| | |
|---|---|
| Interface | one GPIO input (`embedded_hal::digital::InputPin`, async: `+ Wait`) |
| Core | pure state machine fed with `(level, timestamp_ms)`; no internal timers |
| Debounce | configurable per input (presets: contact 20 ms, hall 10 ms, leak 500 ms, PIR 50 ms glitch filter) |
| Hold | occupancy/vibration stays active for `hold_ms` after the input goes inactive |
| Output | `Quantity::Occupancy` (PIR), `Quantity::Contact` (reed, hall switch; `true` = closed / magnet present), `Quantity::Alarm` (water leak, vibration); typed `Reading { kind, active }` |
| Status | mock-tested |

Wiring notes:

* Contacts, leak probes and vibration switches: switch to GND with the
  MCU pull-up enabled (presets assume active low). A reed switch with a 1 MΩ
  external pull-up draws 3 µA when closed; prefer a large pull-up on
  battery nodes and wake on both edges.
* PIR modules: HC-SR501 needs 5 V (its output is 3.3 V); AM312 runs from
  3 V and draws ~15 µA. Put the module's own retrigger time short and let
  `hold_ms` define the occupancy clear time.
* Use the pin interrupt to call `sample(now_ms)` and a timer at
  `next_deadline()` to call `poll(now_ms)`; between events the MCU can sleep.
