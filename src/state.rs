// Vehicle state machine + supervisor rules.
// This is the spine of VOLTIQ Phase 0. Everything else (services,
// dashboard, logger) just reads this or asks it to change.

// numbers match the supervisor CAN frame (0x3F0)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VehicleState {
    Off = 0,
    Acc = 1,
    Run = 2,
    Charge = 3,
    Fault = 4,
}

impl VehicleState {
    pub fn name(&self) -> &'static str {
        match self {
            VehicleState::Off => "OFF",
            VehicleState::Acc => "ACC",
            VehicleState::Run => "RUN",
            VehicleState::Charge => "CHARGE",
            VehicleState::Fault => "FAULT",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaultCode {
    None = 0,
    OverTemp = 1,
    OverCurrent = 2,
    CanSilence = 3,
}

impl FaultCode {
    pub fn name(&self) -> &'static str {
        match self {
            FaultCode::None => "none",
            FaultCode::OverTemp => "OVER-TEMP",
            FaultCode::OverCurrent => "OVER-CURRENT",
            FaultCode::CanSilence => "CAN-SILENCE",
        }
    }
}

// limits from the plan. Bench pack, so the numbers are small on purpose.
pub const MAX_CELL_TEMP: f32 = 60.0; // deg C
pub const MAX_PACK_CURRENT: f32 = 150.0; // A
pub const CAN_SILENCE_LIMIT_MS: u128 = 1000;

// faults the demo can inject
#[derive(Debug, Default, Clone, Copy)]
pub struct InjectedFaults {
    pub over_temp: bool,
    pub over_current: bool,
    pub can_silence: bool,
}

// everything the services share. One lock guards it.
pub struct Vehicle {
    pub state: VehicleState,
    pub fault: FaultCode,
    pub contactor_closed: bool,
    pub soc: f32,          // percent
    pub pack_v: f32,       // volts
    pub pack_i: f32,       // amps, + = discharging
    pub max_cell_temp: f32,
    pub controller_temp: f32,
    pub rpm: f32,
    pub torque: f32, // Nm
    pub throttle: u8, // 0-100, driver request
    pub charger_plugged: bool,
    pub charger_kw: f32,
    pub charging: bool,
    pub charge_done: bool,
    pub injected: InjectedFaults,
    pub can_silence_since_ms: Option<u128>, // set when BMS stops talking
    pub heartbeat: u16,
}

impl Vehicle {
    pub fn new() -> Vehicle {
        Vehicle {
            state: VehicleState::Off,
            fault: FaultCode::None,
            contactor_closed: false,
            soc: 72.0,
            pack_v: 48.0,
            pack_i: 0.0,
            max_cell_temp: 28.0,
            controller_temp: 30.0,
            rpm: 0.0,
            torque: 0.0,
            throttle: 0,
            charger_plugged: false,
            charger_kw: 0.0,
            charging: false,
            charge_done: false,
            injected: InjectedFaults::default(),
            can_silence_since_ms: None,
            heartbeat: 0,
        }
    }

    pub fn healthy(&self) -> bool {
        self.fault == FaultCode::None
    }

    // supervisor rule: motor spins only in RUN, healthy, contactor closed
    pub fn motor_allowed(&self) -> bool {
        self.state == VehicleState::Run && self.healthy() && self.contactor_closed
    }

    // key press: OFF -> ACC -> RUN -> OFF. Fault or Charge: does nothing.
    pub fn key(&mut self) -> Result<&'static str, &'static str> {
        match self.state {
            VehicleState::Off => {
                self.state = VehicleState::Acc;
                Ok("key on: OFF -> ACC")
            }
            VehicleState::Acc => {
                self.state = VehicleState::Run;
                self.contactor_closed = true;
                Ok("key on: ACC -> RUN, contactor closed")
            }
            VehicleState::Run => {
                self.state = VehicleState::Off;
                self.contactor_closed = false;
                self.throttle = 0;
                Ok("key off: RUN -> OFF")
            }
            VehicleState::Charge => Err("unplug the charger first"),
            VehicleState::Fault => Err("clear the fault first"),
        }
    }

    // plug in: only from ACC or RUN. Motor gets locked out in CHARGE.
    pub fn plug_charger(&mut self) -> Result<&'static str, &'static str> {
        match self.state {
            VehicleState::Acc | VehicleState::Run => {
                self.state = VehicleState::Charge;
                self.charger_plugged = true;
                self.charging = true;
                self.charge_done = false;
                self.throttle = 0;
                Ok("charger plugged: -> CHARGE, motor locked out")
            }
            VehicleState::Off => Err("turn the key on first"),
            VehicleState::Charge => Err("already charging"),
            VehicleState::Fault => Err("clear the fault first"),
        }
    }

    pub fn unplug_charger(&mut self) -> Result<&'static str, &'static str> {
        if self.state != VehicleState::Charge {
            return Err("not charging");
        }
        self.state = VehicleState::Acc;
        self.charger_plugged = false;
        self.charging = false;
        self.charger_kw = 0.0;
        Ok("charger unplugged: CHARGE -> ACC")
    }

    pub fn set_throttle(&mut self, v: u8) -> Result<&'static str, &'static str> {
        if v > 100 {
            return Err("throttle is 0-100");
        }
        self.throttle = v;
        if self.state == VehicleState::Charge {
            return Ok("throttle noted, but motor is locked out while charging");
        }
        if self.state != VehicleState::Run {
            return Ok("throttle noted, motor only spins in RUN");
        }
        Ok("throttle set")
    }

    // deliberate fault injection for the demo
    pub fn inject(&mut self, kind: &str, now_ms: u128) -> Result<&'static str, &'static str> {
        match kind {
            "temp" => {
                self.injected.over_temp = true;
                self.max_cell_temp = 75.0;
                Ok("injected: cell temp forced to 75C")
            }
            "current" => {
                self.injected.over_current = true;
                self.pack_i = 180.0;
                Ok("injected: pack current forced to 180A")
            }
            "can" => {
                self.injected.can_silence = true;
                self.can_silence_since_ms = Some(now_ms);
                Ok("injected: BMS stopped talking (CAN silence)")
            }
            _ => Err("fault kinds: temp, current, can"),
        }
    }

    // supervisor check, runs every cycle. Latches the fault.
    // returns the fault it raised, if any.
    pub fn supervisor_check(&mut self, now_ms: u128) -> Option<FaultCode> {
        if self.fault != FaultCode::None {
            return None; // already latched, stays latched
        }
        let raised = if self.max_cell_temp > MAX_CELL_TEMP {
            Some(FaultCode::OverTemp)
        } else if self.pack_i > MAX_PACK_CURRENT {
            Some(FaultCode::OverCurrent)
        } else if let Some(since) = self.can_silence_since_ms {
            if now_ms - since > CAN_SILENCE_LIMIT_MS {
                Some(FaultCode::CanSilence)
            } else {
                None
            }
        } else {
            None
        };
        if let Some(f) = raised {
            self.trip(f);
        }
        raised
    }

    fn trip(&mut self, f: FaultCode) {
        self.fault = f;
        self.state = VehicleState::Fault;
        self.contactor_closed = false;
        self.throttle = 0;
        self.charging = false;
        self.charger_kw = 0.0;
    }

    // deliberate clear only. Also clears whatever was injected.
    pub fn clear_fault(&mut self) -> Result<&'static str, &'static str> {
        if self.state != VehicleState::Fault {
            return Err("no fault to clear");
        }
        self.fault = FaultCode::None;
        self.injected = InjectedFaults::default();
        self.can_silence_since_ms = None;
        self.max_cell_temp = 35.0;
        self.pack_i = 0.0;
        self.rpm = 0.0;
        self.torque = 0.0;
        self.state = VehicleState::Off;
        Ok("fault cleared: -> OFF. Key on to drive again.")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn key_walks_off_acc_run_off() {
        let mut v = Vehicle::new();
        assert_eq!(v.state, VehicleState::Off);
        v.key().unwrap();
        assert_eq!(v.state, VehicleState::Acc);
        v.key().unwrap();
        assert_eq!(v.state, VehicleState::Run);
        assert!(v.contactor_closed);
        v.key().unwrap();
        assert_eq!(v.state, VehicleState::Off);
        assert!(!v.contactor_closed);
    }

    #[test]
    fn plug_from_off_rejected() {
        let mut v = Vehicle::new();
        assert!(v.plug_charger().is_err());
        v.key().unwrap();
        assert!(v.plug_charger().is_ok());
        assert_eq!(v.state, VehicleState::Charge);
        // motor locked out while charging
        assert!(!v.motor_allowed());
        v.unplug_charger().unwrap();
        assert_eq!(v.state, VehicleState::Acc);
    }

    #[test]
    fn motor_blocked_unless_run_and_healthy() {
        let mut v = Vehicle::new();
        assert!(!v.motor_allowed()); // OFF
        v.key().unwrap();
        assert!(!v.motor_allowed()); // ACC, not RUN
        v.key().unwrap();
        assert!(v.motor_allowed()); // RUN
        v.inject("temp", 0).unwrap();
        v.supervisor_check(100);
        assert_eq!(v.state, VehicleState::Fault);
        assert!(!v.motor_allowed());
    }

    #[test]
    fn over_temp_trips_and_latches() {
        let mut v = Vehicle::new();
        v.key().unwrap();
        v.key().unwrap();
        v.inject("temp", 0).unwrap();
        let f = v.supervisor_check(100);
        assert_eq!(f, Some(FaultCode::OverTemp));
        assert_eq!(v.state, VehicleState::Fault);
        assert!(!v.contactor_closed);
        // stays latched even though temp is now fine
        v.max_cell_temp = 30.0;
        assert!(v.supervisor_check(200).is_none());
        assert_eq!(v.state, VehicleState::Fault);
        // deliberate clear only
        v.clear_fault().unwrap();
        assert_eq!(v.state, VehicleState::Off);
        assert_eq!(v.fault, FaultCode::None);
    }

    #[test]
    fn over_current_trips() {
        let mut v = Vehicle::new();
        v.inject("current", 0).unwrap();
        let f = v.supervisor_check(0);
        assert_eq!(f, Some(FaultCode::OverCurrent));
    }

    #[test]
    fn can_silence_needs_a_full_second() {
        let mut v = Vehicle::new();
        v.inject("can", 1000).unwrap();
        assert!(v.supervisor_check(1500).is_none()); // 500ms, not yet
        let f = v.supervisor_check(2100); // 1100ms of silence
        assert_eq!(f, Some(FaultCode::CanSilence));
    }

    #[test]
    fn clear_when_no_fault_is_an_error() {
        let mut v = Vehicle::new();
        assert!(v.clear_fault().is_err());
        assert!(v.set_throttle(101).is_err());
    }
}
