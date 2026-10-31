# zigbee-ld2410

Hi-Link HLK-LD2410, LD2410B and LD2410C 24 GHz FMCW human-presence radars
(UART protocol).

| | |
|---|---|
| Interface | UART 256000 8N1 default (`embedded_io::Read + ReadReady + Write` / `embedded_io_async::Read + Write`) |
| Frames | reports `F4 F3 F2 F1 … F8 F7 F6 F5` (basic / engineering), commands and ACKs `FD FC FB FA … 04 03 02 01` |
| Conversion | free running, ~10 reports/s; `start_measurement` returns 0 |
| Output | `Quantity::Occupancy`, plus `Quantity::Distance` (mm) when a target is present; raw energies in engineering mode |
| Commands | config enable/end, max gates + unmanned duration, gate sensitivities, read parameters, engineering mode, firmware version, baud rate, factory reset, restart; B/C: Bluetooth, MAC, distance resolution, light control, noise detection |
| Current | ≈ 79 mA average (module supply 5 V) |
| Source | Hi-Link serial protocol: LD2410 V1.02, LD2410B V1.06, LD2410C V1.00 / V1.07 |
| Status | mock-tested |

Notes:

* Bracket every configuration command with `enable_config()` / `end_config()`;
  reports stop while configuration mode is active. Baud rate, resolution,
  Bluetooth and factory reset take effect after `Command::Restart`.
* The blocking `read_measurement` never waits: it drains the UART and returns
  the newest report or `NotReady`. At 256000 baud a report arrives every
  ~100 ms; poll often or give the UART a receive buffer of ≥ 64 bytes.
* Command helpers read until the matching ACK and give up with `Timeout` after
  ~1 KiB of other traffic, but block/await on a silent UART. For a bounded
  wait use `send()` and poll `poll_ack()` from a timer (blocking driver), or
  wrap the future in a timeout (async driver).
* Document caveats: the two trailing engineering-frame bytes are light value +
  OUT state on the LD2410B but "reserved" for LD2410 / LD2410C (exposed raw as
  `Engineering::aux`); the MAC ACK is described as 3 bytes but the example has
  6 (6 are decoded); the LD2410C document gives both 0.75 m and 0.2 m as the
  factory resolution. The OUT pin (high = present) can be read with
  `zigbee-digital` instead of the UART.
* Bluetooth permission / password commands (0x00A8 / 0x00A9) are not
  implemented: the module answers them over BLE only.
