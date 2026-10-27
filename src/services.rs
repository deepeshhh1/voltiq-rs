// Vehicle services. Each one is a thread with a fixed period,
// like the FreeRTOS task map in the plan.
//
// Services talk to the vehicle only through the Hal trait and the
// shared Vehicle state. They never print the dashboard and never
// touch files — the logger thread owns files.

use std::fs::OpenOptions;
use std::io::Write;
use std::sync::mpsc::{Receiver, Sender};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use crate::can;
use crate::hal::{Hal, SimHal};
use crate::state::{FaultCode, Vehicle, VehicleState};

// anything a service wants logged or put on the bus
pub enum Msg {
    Can(can::Frame),
    Event(String),
}

pub struct Shared {
    pub vehicle: Mutex<Vehicle>,
    pub hal: SimHal,
    pub stop: std::sync::atomic::AtomicBool,
    pub last_msg: Mutex<String>,
}

impl Shared {
    pub fn new() -> Arc<Shared> {
        Arc::new(Shared {
            vehicle: Mutex::new(Vehicle::new()),
            hal: SimHal::new(),
            stop: std::sync::atomic::AtomicBool::new(false),
            last_msg: Mutex::new(String::from("booted. type: key")),
        })
    }
    pub fn stopped(&self) -> bool {
        self.stop.load(std::sync::atomic::Ordering::Relaxed)
    }
}

fn sleep_ms(ms: u64) {
    thread::sleep(Duration::from_millis(ms));
}

// BMS service, 100 ms. Reads the pack, updates the shared state,
// puts a 0x100 frame on the bus. The tick is a plain function so
// tests (and later the ESP32 task body) can call it directly.
pub fn bms_tick(s: &Shared, bus: &Sender<Msg>) {
    let silenced = {
        let v = s.vehicle.lock().unwrap();
        v.injected.can_silence
    };
    if !silenced {
        let r = s.hal.read_pack();
        let mut v = s.vehicle.lock().unwrap();
        v.pack_v = r.volts;
        v.pack_i = r.amps;
        v.max_cell_temp = r.max_cell_temp;
        v.soc = s.hal.read_soc();
        v.controller_temp = s.hal.read_controller_temp();
        let soc = v.soc as u8;
        let temp = v.max_cell_temp as i8;
        let _ = bus.send(Msg::Can(can::encode_bms(
            v.pack_v, v.pack_i, soc, temp,
        )));
    }
}

pub fn bms_thread(s: Arc<Shared>, bus: Sender<Msg>) {
    loop {
        if s.stopped() {
            break;
        }
        bms_tick(&s, &bus);
        sleep_ms(100);
    }
}

// motor service, 100 ms. Throttle -> torque request, but only
// when the supervisor's rule allows the motor to spin.
pub fn motor_tick(s: &Shared, bus: &Sender<Msg>) {
    let (allowed, throttle) = {
        let v = s.vehicle.lock().unwrap();
        (v.motor_allowed(), v.throttle)
    };
    let torque = if allowed {
        throttle as f32 * 0.15 // 15 Nm max, bench motor
    } else {
        0.0
    };
    s.hal.set_motor_torque(torque);
    let rpm = s.hal.read_rpm();
    let mut v = s.vehicle.lock().unwrap();
    v.rpm = rpm;
    v.torque = if allowed { torque } else { 0.0 };
    let _ = bus.send(Msg::Can(can::encode_motor(
        rpm as u16,
        v.torque,
        v.controller_temp as i8,
        throttle,
    )));
}

pub fn motor_thread(s: Arc<Shared>, bus: Sender<Msg>) {
    loop {
        if s.stopped() {
            break;
        }
        motor_tick(&s, &bus);
        sleep_ms(100);
    }
}

// charge service, 200 ms. Only does anything in CHARGE.
pub fn charge_tick(s: &Shared, bus: &Sender<Msg>) {
    let (in_charge, soc) = {
        let v = s.vehicle.lock().unwrap();
        (v.state == VehicleState::Charge, v.soc)
    };
    if in_charge {
        s.hal.set_charger(true);
        let mut v = s.vehicle.lock().unwrap();
        v.charger_kw = 0.5;
        v.charging = true;
        if soc >= 100.0 {
            v.charge_done = true;
            v.charging = false;
            v.charger_kw = 0.0;
        }
        let state = if v.charge_done { 2 } else { 1 };
        let _ = bus.send(Msg::Can(can::encode_charger(
            state,
            v.charger_kw,
            soc as u8,
        )));
    } else {
        s.hal.set_charger(false);
        let mut v = s.vehicle.lock().unwrap();
        v.charger_kw = 0.0;
        v.charging = false;
    }
}

pub fn charge_thread(s: Arc<Shared>, bus: Sender<Msg>) {
    loop {
        if s.stopped() {
            break;
        }
        charge_tick(&s, &bus);
        sleep_ms(200);
    }
}

// the fake physics. Host only — on ESP32 this thread does not
// exist, real sensors answer instead.
pub fn plant_thread(s: Arc<Shared>) {
    loop {
        if s.stopped() {
            break;
        }
        let injected = {
            let v = s.vehicle.lock().unwrap();
            v.injected
        };
        s.hal.tick(100, &injected);
        sleep_ms(100);
    }
}

// safety supervisor, 100 ms, highest priority in spirit.
// Checks the rules, holds the veto, sends the 0x3F0 heartbeat.
pub fn supervisor_tick(s: &Shared, bus: &Sender<Msg>) {
    let now = s.hal.now_ms();
    // contactor follows the core's decision, every cycle
    let closed = s.vehicle.lock().unwrap().contactor_closed;
    s.hal.set_contactor(closed);
    let raised = {
        let mut v = s.vehicle.lock().unwrap();
        v.supervisor_check(now)
    };
    if let Some(f) = raised {
        let _ = bus.send(Msg::Event(format!(
            "FAULT {} - contactor opened, state latched",
            f.name()
        )));
    }
    let (state, fault, hb) = {
        let mut v = s.vehicle.lock().unwrap();
        v.heartbeat = v.heartbeat.wrapping_add(1);
        (v.state as u8, v.fault as u8, v.heartbeat)
    };
    let _ = bus.send(Msg::Can(can::encode_supervisor(hb, state, fault)));
}

pub fn supervisor_thread(s: Arc<Shared>, bus: Sender<Msg>) {
    loop {
        if s.stopped() {
            break;
        }
        supervisor_tick(&s, &bus);
        sleep_ms(100);
    }
}

// logger: CAN frames -> can.log, events -> events.log,
// telemetry snapshot -> voltiq-log.csv every 500 ms.
pub fn logger_thread(s: Arc<Shared>, rx: Receiver<Msg>) {
    let mut csv = OpenOptions::new()
        .create(true)
        .append(true)
        .open("voltiq-log.csv")
        .expect("cannot open voltiq-log.csv");
    let mut can_log = OpenOptions::new()
        .create(true)
        .append(true)
        .open("can.log")
        .expect("cannot open can.log");
    let mut ev_log = OpenOptions::new()
        .create(true)
        .append(true)
        .open("events.log")
        .expect("cannot open events.log");
    // header only if the file is new/empty, not on every restart
    let fresh = csv.metadata().map(|m| m.len() == 0).unwrap_or(true);
    if fresh {
        let _ = writeln!(
            csv,
            "t_ms,state,soc,pack_v,pack_i,max_temp,rpm,torque,charger_kw,fault,contactor"
        );
    }
    // telemetry on a schedule, even when the bus is busy:
    // the old code only wrote a row when recv timed out, which
    // never happens while frames keep arriving.
    let mut last_telemetry_ms = 0u128;
    loop {
        if s.stopped() {
            break;
        }
        match rx.recv_timeout(Duration::from_millis(100)) {
            Ok(Msg::Can(f)) => {
                let _ = writeln!(can_log, "{} {}", s.hal.now_ms(), f.to_hex());
            }
            Ok(Msg::Event(e)) => {
                let _ = writeln!(ev_log, "[{} ms] {}", s.hal.now_ms(), e);
            }
            Err(_) => {}
        }
        let now = s.hal.now_ms();
        if now - last_telemetry_ms >= 500 {
            last_telemetry_ms = now;
            let (t, st, soc, v, i, temp, rpm, tq, kw, fault, cont) = {
                let v = s.vehicle.lock().unwrap();
                (
                    s.hal.now_ms(),
                    v.state.name(),
                    v.soc,
                    v.pack_v,
                    v.pack_i,
                    v.max_cell_temp,
                    v.rpm,
                    v.torque,
                    v.charger_kw,
                    v.fault.name(),
                    v.contactor_closed,
                )
            };
            let _ = writeln!(
                csv,
                "{},{},{:.1},{:.1},{:.1},{:.1},{:.0},{:.1},{:.1},{},{}",
                t, st, soc, v, i, temp, rpm, tq, kw, fault, cont
            );
            let _ = csv.flush();
        }
    }
}

// dashboard, 500 ms. Plain text, no TUI crate.
pub fn dashboard_thread(s: Arc<Shared>) {
    loop {
        if s.stopped() {
            break;
        }
        let (st, soc, v, i, temp, ctrl, rpm, tq, thr, kw, charging, done, fault, cont, plugged, msg) = {
            let v = s.vehicle.lock().unwrap();
            let msg = s.last_msg.lock().unwrap().clone();
            (
                v.state.name(),
                v.soc,
                v.pack_v,
                v.pack_i,
                v.max_cell_temp,
                v.controller_temp,
                v.rpm,
                v.torque,
                v.throttle,
                v.charger_kw,
                v.charging,
                v.charge_done,
                v.fault,
                v.contactor_closed,
                v.charger_plugged,
                msg,
            )
        };
        // SOC bar, 20 blocks
        let filled = ((soc / 100.0 * 20.0) as usize).min(20);
        let bar: String = std::iter::repeat('#').take(filled).collect();
        let empty: String = std::iter::repeat('-').take(20 - filled).collect();
        print!("\x1b[2J\x1b[H"); // clear screen, cursor home
        println!("VOLTIQ 0.1.0 — EV OS sim (SIMULATED signals, no hardware)");
        println!("============================================================");
        println!("STATE: {:<8}  contactor: {}", st, if cont { "CLOSED" } else { "OPEN" });
        if fault != FaultCode::None {
            println!("!!! FAULT LATCHED: {} — type 'clear' to reset !!!", fault.name());
        }
        println!("SOC:   [{}{}] {:.0}%", bar, empty, soc);
        println!(
            "PACK:  {:.1} V   {:.1} A   max cell temp {:.0} C   controller {:.0} C",
            v, i, temp, ctrl
        );
        println!(
            "MOTOR: {:.0} rpm   {:.1} Nm   throttle {}%   {}",
            rpm,
            tq,
            thr,
            if st == "CHARGE" { "LOCKED OUT" } else { "" }
        );
        println!(
            "CHARGE: plugged={}  {}  {:.1} kW  {}",
            plugged,
            if charging { "CHARGING" } else { "idle" },
            kw,
            if done { "COMPLETE" } else { "" }
        );
        println!("------------------------------------------------------------");
        println!("> {}", msg);
        println!("commands: key | throttle 0-100 | plug | unplug | fault temp/current/can | clear | status | quit");
        let _ = std::io::stdout().flush();
        sleep_ms(500);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc;
    use std::time::Duration;

    fn setup() -> (Arc<Shared>, mpsc::Sender<Msg>, mpsc::Receiver<Msg>) {
        let (tx, rx) = mpsc::channel();
        (Shared::new(), tx, rx)
    }

    fn drain(rx: &mpsc::Receiver<Msg>) -> Vec<Msg> {
        let mut out = vec![];
        while let Ok(m) = rx.try_recv() {
            out.push(m);
        }
        out
    }

    fn has_frame(msgs: &[Msg], id: u16) -> bool {
        msgs.iter().any(|m| match m {
            Msg::Can(f) => f.id == id,
            _ => false,
        })
    }

    #[test]
    fn bms_tick_sends_0x100_and_updates_readings() {
        let (s, tx, rx) = setup();
        bms_tick(&s, &tx);
        let msgs = drain(&rx);
        assert!(has_frame(&msgs, 0x100));
        let v = s.vehicle.lock().unwrap();
        assert!((v.pack_v - 48.0).abs() < 0.1);
    }

    #[test]
    fn bms_tick_stays_silent_when_can_silenced() {
        let (s, tx, rx) = setup();
        s.vehicle.lock().unwrap().inject("can", 0).unwrap();
        bms_tick(&s, &tx);
        assert!(drain(&rx).is_empty());
    }

    #[test]
    fn motor_tick_spins_only_in_run() {
        let (s, tx, rx) = setup();
        s.vehicle.lock().unwrap().set_throttle(80).unwrap();
        motor_tick(&s, &tx); // still OFF: no torque
        assert_eq!(s.vehicle.lock().unwrap().torque, 0.0);
        s.vehicle.lock().unwrap().key().unwrap();
        s.vehicle.lock().unwrap().key().unwrap(); // RUN
        motor_tick(&s, &tx);
        assert!((s.vehicle.lock().unwrap().torque - 12.0).abs() < 0.01);
        assert!(has_frame(&drain(&rx), 0x200));
    }

    #[test]
    fn motor_tick_locked_out_while_charging() {
        let (s, tx, _rx) = setup();
        s.vehicle.lock().unwrap().key().unwrap();
        s.vehicle.lock().unwrap().key().unwrap();
        s.vehicle.lock().unwrap().plug_charger().unwrap();
        s.vehicle.lock().unwrap().set_throttle(80).unwrap();
        motor_tick(&s, &tx);
        assert_eq!(s.vehicle.lock().unwrap().torque, 0.0);
    }

    #[test]
    fn charge_tick_turns_charger_on_only_in_charge() {
        let (s, tx, rx) = setup();
        charge_tick(&s, &tx); // OFF: charger stays off
        assert_eq!(s.vehicle.lock().unwrap().charger_kw, 0.0);
        assert!(!has_frame(&drain(&rx), 0x300));
        s.vehicle.lock().unwrap().key().unwrap();
        s.vehicle.lock().unwrap().key().unwrap();
        s.vehicle.lock().unwrap().plug_charger().unwrap();
        charge_tick(&s, &tx);
        assert!((s.vehicle.lock().unwrap().charger_kw - 0.5).abs() < 0.01);
        assert!(s.vehicle.lock().unwrap().charging);
        assert!(has_frame(&drain(&rx), 0x300));
    }

    #[test]
    fn supervisor_tick_trips_and_reports_fault() {
        let (s, tx, rx) = setup();
        s.vehicle.lock().unwrap().key().unwrap();
        s.vehicle.lock().unwrap().key().unwrap();
        s.vehicle.lock().unwrap().inject("temp", 0).unwrap();
        supervisor_tick(&s, &tx);
        let v = s.vehicle.lock().unwrap();
        assert_eq!(v.state, VehicleState::Fault);
        assert!(!v.contactor_closed);
        drop(v);
        let msgs = drain(&rx);
        assert!(msgs.iter().any(|m| matches!(m, Msg::Event(e) if e.contains("OVER-TEMP"))));
        assert!(has_frame(&msgs, 0x3F0));
    }

    #[test]
    fn full_drive_charge_cycle_through_ticks() {
        let (s, tx, rx) = setup();
        // drive a bit (supervisor thread normally mirrors the
        // contactor into the HAL every cycle; do it by hand here)
        s.vehicle.lock().unwrap().key().unwrap();
        s.vehicle.lock().unwrap().key().unwrap();
        s.hal.set_contactor(true);
        s.vehicle.lock().unwrap().set_throttle(60).unwrap();
        for _ in 0..5 {
            s.hal.tick(100, &s.vehicle.lock().unwrap().injected);
            motor_tick(&s, &tx);
            bms_tick(&s, &tx);
        }
        assert!(s.vehicle.lock().unwrap().rpm > 0.0);
        // park, plug in, charge ticks raise soc
        s.vehicle.lock().unwrap().set_throttle(0).unwrap();
        s.vehicle.lock().unwrap().plug_charger().unwrap();
        let soc_before = s.vehicle.lock().unwrap().soc;
        for _ in 0..20 {
            s.hal.tick(100, &s.vehicle.lock().unwrap().injected);
            charge_tick(&s, &tx);
            bms_tick(&s, &tx);
        }
        assert!(s.vehicle.lock().unwrap().soc > soc_before);
        assert!(has_frame(&drain(&rx), 0x300));
        // unplug, back to ACC, motor still locked out until RUN
        s.vehicle.lock().unwrap().unplug_charger().unwrap();
        assert_eq!(s.vehicle.lock().unwrap().state, VehicleState::Acc);
        let _ = rx.recv_timeout(Duration::from_millis(10));
    }
}
