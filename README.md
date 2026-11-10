# Volt-IQ 0.1.0 — EV OS core, Phase 0 (laptop simulation)

Volt-IQ is Deepesh's EV operating system plan: one platform that runs
the battery service (CellSight), motor service (TorqueTwin) and
charging service (GridSync) together, with a Safety Supervisor that
can veto anything. Phase 0 is the skeleton that breathes — on a
laptop, in Rust, with simulated signals. No hardware, no ESP32 yet.

## What it does

- Vehicle state machine: `OFF → ACC → RUN ⇄ CHARGE`, and from any
  state → `FAULT` (latched, needs a deliberate `clear`).
- Services as threads, like the plan's FreeRTOS task map: BMS,
  motor, charge, plant sim, safety supervisor, logger, dashboard.
- A message bus carrying real CAN frames (IDs from the plan:
  0x100 BMS, 0x200 motor, 0x300 charger, 0x3F0 supervisor).
  Frames are logged to `can.log`. Dictionary: `CAN_DICTIONARY.md`.
- Safety Supervisor: over-temp (>60 °C), over-current (>150 A) and
  CAN-silence (>1 s) force FAULT, open the contactor, and log the
  event. Motor spins only in RUN with a healthy pack; charging locks
  the motor out.
- Logger writes `volt-iq-log.csv` (telemetry every 500 ms),
  `events.log` (faults/state changes) and `can.log` (bus frames).
- Terminal dashboard refreshes every 500 ms.

## Architecture (and the HAL line)

```
stdin commands ──▶ main ──▶ Vehicle (state.rs, core)
                       │
   ┌──────────┬────────┼────────┬──────────┬──────────┐
   BMS      motor   charge   plant    supervisor   logger/dashboard
   thread   thread  thread   (sim)    thread       threads
        \      |       |        |          |           /
         └─────┴───────┴────────┴──────────┴──────────┘
              Hal trait (hal.rs) + shared Vehicle state
```

The core — `state.rs` (state machine + supervisor rules),
`can.rs` (encode/decode), and the service *logic* — never touches
hardware. Everything hardware-ish goes through the `Hal` trait:

```rust
trait Hal {
    fn now_ms(&self) -> u128;
    fn read_pack(&self) -> PackReading;      // V, A, max cell temp
    fn read_soc(&self) -> f32;               // BMS state of charge
    fn read_controller_temp(&self) -> f32;
    fn read_rpm(&self) -> f32;
    fn set_motor_torque(&self, torque_nm: f32);
    fn set_contactor(&self, closed: bool);
    fn set_charger(&self, on: bool);
}
```

Each service is also split into a plain `*_tick` function plus a
thin thread loop around it (`bms_tick`, `motor_tick`,
`charge_tick`, `supervisor_tick` in `services.rs`). The tick
functions are what the tests call directly — and on ESP32 they
become the FreeRTOS task bodies, unchanged.

Today the only implementation is `SimHal`: a fake 48 V, 2 Ah bench
pack with simple physics (SOC by coulomb counting, temps rising
with load, RPM chasing the torque command). Core modules contain no
`println!`, no `std::time` — time is passed in as milliseconds.

### Moving to ESP32 later (the porting path)

1. Write `EspHal: Hal` — ADC reads for pack V/I and temps, GPIO
   for the contactor relay, PWM/CAN driver for the motor, charger
   relay control, `esp_timer` for `now_ms`.
2. Delete the plant sim thread (real sensors answer instead).
3. Swap the dashboard thread for the LVGL display task and the
   file logger for an SD/flash logger.
4. **Nothing changes** in `state.rs`, `can.rs`, or the supervisor
   rules. If a rule needs changing for hardware, it changes once,
   in the core, and the sim still tests it.

No esp-rs toolchain is involved in this phase — that comes when
hardware arrives (Phase 1 in the plan).

## Run it

```bash
cargo run
```

Commands (type them, Enter after each):
```
key                 # OFF -> ACC -> RUN -> OFF
throttle 0-100      # only spins the motor in RUN
plug                # -> CHARGE (motor locked out)
unplug              # CHARGE -> ACC
fault temp          # force cell temp to 75 C (supervisor trips)
fault current       # force pack current to 180 A
fault can           # BMS stops talking (trips after 1 s of silence)
clear               # deliberate fault reset -> OFF
status              # one-line state summary
wait 1000           # pause, mostly for scripted demos
quit                # stop, logs stay in this folder
```

Scripted demo (the Phase 0 exit test):
```bash
cargo run < demo.txt
```
It does: key on → RUN → throttle 60 → inject over-temp → FAULT
+ log entry → clear → key on → plug in → charge → unplug → quit.
Then check: `events.log` should show the OVER-TEMP fault,
`volt-iq-log.csv` should show the state walk OFF → ACC → RUN →
FAULT → OFF → ACC → RUN → CHARGE → ACC.

## Honest limitations

- Everything is simulated. SOC is schoolbook coulomb counting on
  a 2 Ah bench pack — educational, not lab-grade.
- No hardware, no automotive certification, no road use. Bench
  scale only, exactly like the plan says.
- The dashboard and stdin commands share one terminal, so the
  dashboard redraw can overlap what you type. It is a demo
  limitation, not a vehicle bug.
- CAN here is an in-memory channel with the plan's IDs and
  layouts — there is no real bus until Phase 1 hardware.
- Verification note: on the build VM the sandbox freezes the
  process for seconds at a time, so the full `demo.txt` run could
  not complete in one go there — but every step of it is verified:
  `cargo test` covers the whole drive → fault → clear → charge
  cycle through the real tick functions (19/19), and live runs
  showed the dashboard, RPM rising with throttle, the OVER-TEMP
  fault tripping + latching + logging, and CAN frames flowing.
  On a normal laptop `demo.txt` runs in ~12 seconds.
