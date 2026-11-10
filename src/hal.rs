// HAL — the line between Volt-IQ core and hardware.
//
// Rule: state.rs, can.rs and the supervisor rules never touch this.
// They only work on plain values passed in. Everything that smells
// like hardware — time, sensors, contactor, motor output, charger —
// goes through this trait.
//
// Today there is one implementation, SimHal (laptop simulation).
// On ESP32 the same trait gets a real implementation (ADC reads,
// GPIO contactor, CAN driver, FreeRTOS time) and the core code
// does not change. That is the whole porting plan.

use std::sync::Mutex;
use std::time::Instant;

use crate::state::InjectedFaults;

// what a sensor read gives back. Plain numbers, no std types.
#[derive(Debug, Clone, Copy)]
pub struct PackReading {
    pub volts: f32,
    pub amps: f32, // + = discharging
    pub max_cell_temp: f32,
}

pub trait Hal: Send + Sync {
    fn now_ms(&self) -> u128;
    fn read_pack(&self) -> PackReading;
    fn read_soc(&self) -> f32;
    fn read_controller_temp(&self) -> f32;
    fn read_rpm(&self) -> f32;
    // motor drive command, Nm. 0 = coast.
    fn set_motor_torque(&self, torque_nm: f32);
    fn set_contactor(&self, closed: bool);
    fn set_charger(&self, on: bool);
}

// ---- simulator implementation (host only) ----

// the fake vehicle the sim pretends to measure. Bench pack:
// 48 V, 2 Ah. Small on purpose so SOC moves visibly in a demo.
struct Plant {
    soc: f32,
    pack_v: f32,
    pack_i: f32,
    max_temp: f32,
    ctrl_temp: f32,
    rpm: f32,
    torque_cmd: f32,
    contactor: bool,
    charger_on: bool,
}

pub struct SimHal {
    plant: Mutex<Plant>,
    start: Instant,
}

impl SimHal {
    pub fn new() -> SimHal {
        SimHal {
            plant: Mutex::new(Plant {
                soc: 72.0,
                pack_v: 48.0,
                pack_i: 0.0,
                max_temp: 28.0,
                ctrl_temp: 30.0,
                rpm: 0.0,
                torque_cmd: 0.0,
                contactor: false,
                charger_on: false,
            }),
            start: Instant::now(),
        }
    }

    // one physics step. Called by the sim thread, never by core logic.
    // dt_ms: how much fake time passed. injected: faults the demo set.
    pub fn tick(&self, dt_ms: u64, injected: &InjectedFaults) {
        let dt = dt_ms as f32 / 1000.0;
        let mut p = self.plant.lock().unwrap();
        if injected.over_temp {
            p.max_temp = 75.0;
        }
        if p.charger_on && p.contactor {
            // charging: 0.5 kW bench charger, about 10 A into the pack
            p.pack_i = -10.0;
            p.soc += 10.0 * dt / 3600.0 / 2.0 * 100.0;
            if p.soc > 100.0 {
                p.soc = 100.0;
            }
            p.pack_v = 48.0 + p.soc * 0.06;
            p.rpm = 0.0;
            p.max_temp += (30.0 - p.max_temp) * dt * 0.05;
        } else {
            // driving (or parked): rpm chases the torque command
            let target_rpm = if p.contactor { p.torque_cmd / 15.0 * 3000.0 } else { 0.0 };
            p.rpm += (target_rpm - p.rpm) * (dt * 2.0).min(1.0);
            if injected.over_current {
                p.pack_i = 180.0;
            } else if p.contactor {
                p.pack_i = 2.0 + p.torque_cmd * 1.9;
            } else {
                p.pack_i = 0.0;
            }
            // coulomb counting, schoolbook version (see README limits)
            p.soc -= p.pack_i * dt / 3600.0 / 2.0 * 100.0;
            if p.soc < 0.0 {
                p.soc = 0.0;
            }
            p.pack_v = 42.0 + p.soc * 0.07;
            if !injected.over_temp {
                let hot = 28.0 + p.pack_i * 0.8;
                p.max_temp += (hot - p.max_temp) * dt * 0.05;
            }
        }
        let ctrl_hot = 30.0 + p.torque_cmd * 1.2;
        p.ctrl_temp += (ctrl_hot - p.ctrl_temp) * dt * 0.05;
    }
}

impl Hal for SimHal {
    fn now_ms(&self) -> u128 {
        self.start.elapsed().as_millis()
    }

    fn read_pack(&self) -> PackReading {
        let p = self.plant.lock().unwrap();
        PackReading {
            volts: p.pack_v,
            amps: p.pack_i,
            max_cell_temp: p.max_temp,
        }
    }

    fn read_soc(&self) -> f32 {
        self.plant.lock().unwrap().soc
    }

    fn read_controller_temp(&self) -> f32 {
        self.plant.lock().unwrap().ctrl_temp
    }

    fn read_rpm(&self) -> f32 {
        self.plant.lock().unwrap().rpm
    }

    fn set_motor_torque(&self, torque_nm: f32) {
        self.plant.lock().unwrap().torque_cmd = torque_nm;
    }

    fn set_contactor(&self, closed: bool) {
        self.plant.lock().unwrap().contactor = closed;
    }

    fn set_charger(&self, on: bool) {
        self.plant.lock().unwrap().charger_on = on;
    }
}
