/// Binary telemetry encoding for high-speed MQTT publishing
/// Reduces payload from ~300 bytes JSON to ~74 bytes binary
/// Target: 20 Hz (50ms intervals) sustainable over MQTT
///
/// Protocol Version: 2 (added lap timer fields)
use core::mem;

/// Current protocol version
pub const PROTOCOL_VERSION: u8 = 2;

/// Telemetry packet structure (74 bytes total)
///
/// Layout:
/// - Header (7 bytes): version, magic, timestamp
/// - IMU data (24 bytes): ax, ay, az, wz, roll, pitch
/// - EKF state (25 bytes): yaw, x, y, vx, vy, speed_kmh, mode
/// - GPS data (9 bytes): lat, lon, gps_valid
/// - Lap timer (7 bytes): lap_time_ms, lap_count, lap_flags
/// - Checksum (2 bytes)
#[repr(C, packed)]
#[derive(Copy, Clone)]
pub struct TelemetryPacket {
    pub version: u8,       // Protocol version (2)
    pub header: u16,       // 0xAA55 magic
    pub timestamp_ms: u32, // milliseconds

    // IMU data (24 bytes)
    pub ax: f32, // m/s²
    pub ay: f32,
    pub az: f32,
    pub wz: f32,   // rad/s
    pub roll: f32, // radians
    pub pitch: f32,

    // EKF state (25 bytes)
    pub yaw: f32, // radians
    pub x: f32,   // meters
    pub y: f32,
    pub vx: f32, // m/s
    pub vy: f32,
    pub speed_kmh: f32,
    pub mode: u8, // 0=IDLE, 1=ACCEL, 2=BRAKE, 4=CORNER, 5=ACCEL+CORNER, 6=BRAKE+CORNER

    // GPS data (9 bytes)
    pub lat: f32, // degrees (f32 for size, ~1cm accuracy)
    pub lon: f32,
    pub gps_valid: u8, // 0/1

    // Lap timer (7 bytes)
    pub lap_time_ms: u32, // Current lap time in ms (0 if not timing)
    pub lap_count: u16,   // Completed lap count
    pub lap_flags: u8,    // Flags: 1=crossed_start, 2=crossed_finish, 4=new_lap, 8=new_best, 16=invalid

    pub checksum: u16, // Simple checksum
}

impl TelemetryPacket {
    pub fn new() -> Self {
        Self {
            version: PROTOCOL_VERSION,
            header: 0xAA55,
            timestamp_ms: 0,
            ax: 0.0,
            ay: 0.0,
            az: 0.0,
            wz: 0.0,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            x: 0.0,
            y: 0.0,
            vx: 0.0,
            vy: 0.0,
            speed_kmh: 0.0,
            mode: 0,
            lat: 0.0,
            lon: 0.0,
            gps_valid: 0,
            lap_time_ms: 0,
            lap_count: 0,
            lap_flags: 0,
            checksum: 0,
        }
    }

    /// Compute checksum (simple sum of all bytes except checksum field)
    fn compute_checksum(&self) -> u16 {
        let bytes = unsafe {
            core::slice::from_raw_parts(
                self as *const Self as *const u8,
                mem::size_of::<Self>() - 2, // exclude checksum field
            )
        };

        bytes
            .iter()
            .fold(0u16, |acc, &b| acc.wrapping_add(b as u16))
    }

    /// Serialize to bytes with checksum
    #[allow(clippy::wrong_self_convention)] // Needs &mut to update checksum
    pub fn to_bytes(&mut self) -> &[u8] {
        self.checksum = self.compute_checksum();

        unsafe {
            core::slice::from_raw_parts(self as *const Self as *const u8, mem::size_of::<Self>())
        }
    }
}

/// Helper to convert mode enum to u8 (legacy, now using Mode::as_u8())
#[allow(dead_code)]
pub fn mode_to_u8(mode_str: &str) -> u8 {
    match mode_str {
        "IDLE" => 0,
        "ACCEL" => 1,
        "BRAKE" => 2,
        "CORNER" => 3,
        _ => 0,
    }
}

/// Publish binary telemetry (optimized for 20 Hz)
/// COMMENTED OUT FOR SPEED TEST - requires full sensor modules
// pub fn publish_telemetry_binary(
// mqtt: &mut crate::mqtt::MqttClient,
// imu: &crate::imu::Wt901Parser,
// gps: &crate::gps::NmeaParser,
// ekf: &crate::ekf::Ekf,
// mode_classifier: &crate::mode::ModeClassifier,
// ) -> Result<(), Box<dyn std::error::Error>> {
// let (ax, ay, az) = imu.get_accel_corrected();
// let (ekf_x, ekf_y) = ekf.position();
// let (mut ekf_vx, mut ekf_vy) = ekf.velocity();
// let mut ekf_speed_kmh = mode_classifier.get_speed_kmh();
//
// Clamp small values to zero
// if ekf_speed_kmh < 0.5 {
// ekf_speed_kmh = 0.0;
// ekf_vx = 0.0;
// ekf_vy = 0.0;
// }
//
// let mut packet = TelemetryPacket::new();
// packet.timestamp_ms = unsafe {
// (esp_idf_svc::sys::esp_timer_get_time() / 1000) as u32
// };
//
// packet.ax = ax;
// packet.ay = ay;
// packet.az = az;
// packet.wz = imu.data.wz;
// packet.roll = imu.data.roll.to_radians();
// packet.pitch = imu.data.pitch.to_radians();
// packet.yaw = ekf.yaw();
//
// packet.x = ekf_x;
// packet.y = ekf_y;
// packet.vx = ekf_vx;
// packet.vy = ekf_vy;
// packet.speed_kmh = ekf_speed_kmh;
// packet.mode = mode_to_u8(mode_classifier.get_mode().as_str());
//
// if gps.last_fix.valid {
// packet.lat = gps.last_fix.lat as f32;
// packet.lon = gps.last_fix.lon as f32;
// packet.gps_valid = 1;
// } else {
// packet.gps_valid = 0;
// }
//
// let bytes = packet.to_bytes();
// mqtt.publish_binary("car/telemetry_bin", bytes, false)?;
//
// Ok(())
// }

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_packet_size() {
        assert_eq!(mem::size_of::<TelemetryPacket>(), 74);
    }

    #[test]
    fn test_version_field() {
        let packet = TelemetryPacket::new();
        assert_eq!(packet.version, PROTOCOL_VERSION);
        assert_eq!(packet.version, 2);
    }

    #[test]
    fn test_lap_timer_fields() {
        let mut packet = TelemetryPacket::new();
        packet.lap_time_ms = 65432;
        packet.lap_count = 5;
        packet.lap_flags = 0x0F;

        assert_eq!(packet.lap_time_ms, 65432);
        assert_eq!(packet.lap_count, 5);
        assert_eq!(packet.lap_flags, 0x0F);
    }
}
