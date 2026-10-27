// VOLTIQ Phase 0 — EV OS core as a laptop simulation.
// Wires the services together and reads commands from stdin.
// No hardware here: sensors, contactor and motor are all SimHal.

mod can;
mod hal;
mod services;
mod state;

use crate::hal::Hal;

use std::io::{BufRead, Write};
use std::sync::mpsc;
use std::thread;

use services::Shared;

fn set_msg(s: &Shared, text: &str) {
    let mut m = s.last_msg.lock().unwrap();
    *m = text.to_string();
}

fn main() {
    let s = Shared::new();
    let (tx, rx) = mpsc::channel();

    let mut handles = vec![];
    let mk = |f: fn(std::sync::Arc<Shared>, mpsc::Sender<services::Msg>)| {
        let s2 = s.clone();
        let tx2 = tx.clone();
        thread::spawn(move || f(s2, tx2))
    };
    handles.push(mk(services::bms_thread));
    handles.push(mk(services::motor_thread));
    handles.push(mk(services::charge_thread));
    handles.push(mk(services::supervisor_thread));
    {
        let s2 = s.clone();
        handles.push(thread::spawn(move || services::plant_thread(s2)));
    }
    {
        let s2 = s.clone();
        handles.push(thread::spawn(move || services::dashboard_thread(s2)));
    }
    {
        let s2 = s.clone();
        handles.push(thread::spawn(move || services::logger_thread(s2, rx)));
    }
    drop(tx); // services hold the other senders now

    println!("VOLTIQ booting... type 'key' to turn on. 'quit' to stop.");
    let stdin = std::io::stdin();
    for line in stdin.lock().lines() {
        let line = match line {
            Ok(l) => l,
            Err(_) => break,
        };
        let parts: Vec<&str> = line.trim().split_whitespace().collect();
        if parts.is_empty() {
            continue;
        }
        let cmd = parts[0];
        match cmd {
            "quit" | "exit" => break,
            "key" => {
                let r = s.vehicle.lock().unwrap().key();
                set_msg(&s, &show(r));
            }
            "plug" => {
                let r = s.vehicle.lock().unwrap().plug_charger();
                set_msg(&s, &show(r));
            }
            "unplug" => {
                let r = s.vehicle.lock().unwrap().unplug_charger();
                set_msg(&s, &show(r));
            }
            "throttle" => {
                if parts.len() < 2 {
                    set_msg(&s, "usage: throttle 0-100");
                    continue;
                }
                match parts[1].parse::<u8>() {
                    Ok(v) => {
                        let r = s.vehicle.lock().unwrap().set_throttle(v);
                        set_msg(&s, &show(r));
                    }
                    Err(_) => set_msg(&s, "throttle needs a number 0-100"),
                }
            }
            "fault" => {
                if parts.len() < 2 {
                    set_msg(&s, "usage: fault temp/current/can");
                    continue;
                }
                let now = s.hal.now_ms();
                let r = s.vehicle.lock().unwrap().inject(parts[1], now);
                set_msg(&s, &show(r));
            }
            "clear" => {
                let r = s.vehicle.lock().unwrap().clear_fault();
                set_msg(&s, &show(r));
            }
            "wait" => {
                if let Some(ms) = parts.get(1).and_then(|x| x.parse::<u64>().ok()) {
                    thread::sleep(std::time::Duration::from_millis(ms));
                    set_msg(&s, &format!("waited {} ms", ms));
                }
            }
            "status" => {
                let v = s.vehicle.lock().unwrap();
                set_msg(
                    &s,
                    &format!(
                        "state={} soc={:.0}% pack={:.1}V {:.1}A rpm={:.0} fault={}",
                        v.state.name(),
                        v.soc,
                        v.pack_v,
                        v.pack_i,
                        v.rpm,
                        v.fault.name()
                    ),
                );
            }
            _ => set_msg(&s, "unknown command. try: key, throttle, plug, fault, clear, quit"),
        }
    }

    s.stop.store(true, std::sync::atomic::Ordering::Relaxed);
    // give the threads a moment to see the flag and exit
    thread::sleep(std::time::Duration::from_millis(600));
    println!("VOLTIQ stopped. Logs: voltiq-log.csv, events.log, can.log");
    // threads are daemon-ish; process exits here.
    let _ = std::io::stdout().flush();
}

fn show(r: Result<&str, &str>) -> String {
    match r {
        Ok(m) => m.to_string(),
        Err(m) => format!("no: {}", m),
    }
}
