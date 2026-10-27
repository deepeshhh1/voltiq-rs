// CAN frames, VOLTIQ bench bus (500 kbit/s, 11-bit IDs).
// Full table lives in CAN_DICTIONARY.md. Everything here is
// simulated — no real bus, no hardware.

pub const ID_BMS_STATUS: u16 = 0x100;
pub const ID_MOTOR_STATUS: u16 = 0x200;
pub const ID_CHARGER_STATUS: u16 = 0x300;
pub const ID_SUPERVISOR: u16 = 0x3F0;

// one frame: id + up to 8 data bytes
#[derive(Debug, Clone, PartialEq)]
pub struct Frame {
    pub id: u16,
    pub data: Vec<u8>,
}

impl Frame {
    pub fn new(id: u16, data: Vec<u8>) -> Frame {
        Frame { id, data }
    }
    // "100#AB12003C" style, handy for the log
    pub fn to_hex(&self) -> String {
        let bytes: Vec<String> = self.data.iter().map(|b| format!("{:02X}", b)).collect();
        format!("{:03X}#{}", self.id, bytes.join(""))
    }
}

// BMS status 0x100, 6 bytes:
//   pack voltage, 0.1 V steps, u16
//   pack current, 0.1 A steps, i16 (+ = discharging)
//   SOC, percent, u8
//   max cell temp, deg C, i8
pub fn encode_bms(pack_v: f32, pack_i: f32, soc: u8, max_temp: i8) -> Frame {
    let v = (pack_v * 10.0).round() as u16;
    let i = (pack_i * 10.0).round() as i16;
    Frame::new(
        ID_BMS_STATUS,
        vec![
            (v >> 8) as u8,
            v as u8,
            (i >> 8) as u8,
            i as u8,
            soc,
            max_temp as u8,
        ],
    )
}

#[allow(dead_code)]
pub fn decode_bms(f: &Frame) -> Option<(f32, f32, u8, i8)> {
    if f.id != ID_BMS_STATUS || f.data.len() < 6 {
        return None;
    }
    let v = ((f.data[0] as u16) << 8 | f.data[1] as u16) as f32 / 10.0;
    let i = ((f.data[2] as i16) << 8 | f.data[3] as i16) as f32 / 10.0;
    Some((v, i, f.data[4], f.data[5] as i8))
}

// motor status 0x200, 6 bytes:
//   rpm, u16
//   torque, 0.1 Nm steps, i16
//   controller temp, deg C, i8
//   throttle, percent, u8
pub fn encode_motor(rpm: u16, torque: f32, ctrl_temp: i8, throttle: u8) -> Frame {
    let t = (torque * 10.0).round() as i16;
    Frame::new(
        ID_MOTOR_STATUS,
        vec![
            (rpm >> 8) as u8,
            rpm as u8,
            (t >> 8) as u8,
            t as u8,
            ctrl_temp as u8,
            throttle,
        ],
    )
}

#[allow(dead_code)]
pub fn decode_motor(f: &Frame) -> Option<(u16, f32, i8, u8)> {
    if f.id != ID_MOTOR_STATUS || f.data.len() < 6 {
        return None;
    }
    let rpm = (f.data[0] as u16) << 8 | f.data[1] as u16;
    let t = ((f.data[2] as i16) << 8 | f.data[3] as i16) as f32 / 10.0;
    Some((rpm, t, f.data[4] as i8, f.data[5]))
}

// charger status 0x300, 4 bytes:
//   state: 0 idle, 1 charging, 2 complete
//   power, 0.1 kW steps, u16
//   soc, percent, u8
pub fn encode_charger(state: u8, power_kw: f32, soc: u8) -> Frame {
    let p = (power_kw * 10.0).round() as u16;
    Frame::new(
        ID_CHARGER_STATUS,
        vec![state, (p >> 8) as u8, p as u8, soc],
    )
}

#[allow(dead_code)]
pub fn decode_charger(f: &Frame) -> Option<(u8, f32, u8)> {
    if f.id != ID_CHARGER_STATUS || f.data.len() < 4 {
        return None;
    }
    let p = ((f.data[1] as u16) << 8 | f.data[2] as u16) as f32 / 10.0;
    Some((f.data[0], p, f.data[3]))
}

// supervisor 0x3F0, 4 bytes:
//   heartbeat counter, u16 (rolls over, that is fine)
//   vehicle state, u8 (same numbers as VehicleState)
//   fault code, u8 (0 = none)
pub fn encode_supervisor(heartbeat: u16, state: u8, fault: u8) -> Frame {
    Frame::new(
        ID_SUPERVISOR,
        vec![(heartbeat >> 8) as u8, heartbeat as u8, state, fault],
    )
}

#[allow(dead_code)]
pub fn decode_supervisor(f: &Frame) -> Option<(u16, u8, u8)> {
    if f.id != ID_SUPERVISOR || f.data.len() < 4 {
        return None;
    }
    let hb = (f.data[0] as u16) << 8 | f.data[1] as u16;
    Some((hb, f.data[2], f.data[3]))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bms_round_trip() {
        let f = encode_bms(48.2, -10.5, 72, 34);
        let (v, i, soc, t) = decode_bms(&f).unwrap();
        assert!((v - 48.2).abs() < 0.11);
        assert!((i + 10.5).abs() < 0.11);
        assert_eq!(soc, 72);
        assert_eq!(t, 34);
    }

    #[test]
    fn motor_round_trip() {
        let f = encode_motor(1800, 9.5, 41, 60);
        let (rpm, t, temp, thr) = decode_motor(&f).unwrap();
        assert_eq!(rpm, 1800);
        assert!((t - 9.5).abs() < 0.11);
        assert_eq!(temp, 41);
        assert_eq!(thr, 60);
    }

    #[test]
    fn charger_round_trip() {
        let f = encode_charger(1, 0.5, 55);
        let (s, p, soc) = decode_charger(&f).unwrap();
        assert_eq!(s, 1);
        assert!((p - 0.5).abs() < 0.11);
        assert_eq!(soc, 55);
    }

    #[test]
    fn supervisor_round_trip() {
        let f = encode_supervisor(1234, 2, 0);
        let (hb, s, fault) = decode_supervisor(&f).unwrap();
        assert_eq!(hb, 1234);
        assert_eq!(s, 2);
        assert_eq!(fault, 0);
    }

    #[test]
    fn wrong_id_rejected() {
        let f = Frame::new(0x200, vec![0, 0, 0, 0, 0, 0]);
        assert!(decode_bms(&f).is_none());
        let short = Frame::new(ID_BMS_STATUS, vec![1, 2]);
        assert!(decode_bms(&short).is_none());
    }
}
