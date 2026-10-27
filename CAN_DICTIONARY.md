# VOLTIQ CAN Dictionary — bench bus, v1

500 kbit/s, standard 11-bit IDs. Simulated bus for Phase 0 —
frames are built by the services and written to `can.log`.
Same IDs and layouts carry over to the ESP32 CAN hardware later.

## Frames

### 0x100 — BMS status (every 100 ms)
| Byte | Content |
|---|---|
| 0–1 | pack voltage, u16, 0.1 V steps |
| 2–3 | pack current, i16, 0.1 A steps (+ = discharging) |
| 4 | SOC, u8, percent |
| 5 | max cell temp, i8, °C |

Example: `100#01E2012C480F` = 48.2 V, 30.0 A, 72 %, 15 °C.

### 0x200 — Motor status (every 100 ms)
| Byte | Content |
|---|---|
| 0–1 | RPM, u16 |
| 2–3 | torque, i16, 0.1 Nm steps |
| 4 | controller temp, i8, °C |
| 5 | throttle, u8, percent |

### 0x300 — Charger status (every 200 ms, in CHARGE)
| Byte | Content |
|---|---|
| 0 | state: 0 idle, 1 charging, 2 complete |
| 1–2 | power, u16, 0.1 kW steps |
| 3 | SOC, u8, percent |

### 0x3F0 — Supervisor heartbeat / fault (every 100 ms)
| Byte | Content |
|---|---|
| 0–1 | heartbeat counter, u16 (rolls over, that is fine) |
| 2 | vehicle state: 0 OFF, 1 ACC, 2 RUN, 3 CHARGE, 4 FAULT |
| 3 | fault code: 0 none, 1 over-temp, 2 over-current, 3 CAN-silence |

## Notes
- No frame is safety-critical by itself: the supervisor reads the
  shared state directly, the bus is for telemetry and logging.
- Cell temps (plan's 0x110) and throttle request (0x210) are not
  separate frames yet — temps ride in 0x100, throttle in 0x200.
  They get their own IDs in Phase 1 when real sensors arrive.
