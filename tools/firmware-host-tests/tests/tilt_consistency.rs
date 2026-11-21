use blackbox_host_tests::imu_evidence::{angular_rates, motion_evidence};
use sensor_fusion::{FusionConfig, SensorFusion};
use wt901::ImuData;

const G: f64 = 9.80665;
const NOW: u32 = 1000;
const PACKETS: [u64; 3] = [1_000_000; 3];

// Independent f64 oracle: elementary rotations, applied one at a time.
fn rx([x, y, z]: [f64; 3], a: f64) -> [f64; 3] {
    [x, a.cos() * y - a.sin() * z, a.sin() * y + a.cos() * z]
}
fn ry([x, y, z]: [f64; 3], a: f64) -> [f64; 3] {
    [a.cos() * x + a.sin() * z, y, -a.sin() * x + a.cos() * z]
}
fn rz([x, y, z]: [f64; 3], a: f64) -> [f64; 3] {
    [a.cos() * x - a.sin() * y, a.sin() * x + a.cos() * y, z]
}
fn body(v: [f64; 3], pose: [f64; 3]) -> [f64; 3] {
    rx(ry(rz(v, -pose[2]), -pose[1]), -pose[0])
}
fn sample(pose_deg: [f64; 3], euler_rates: [f64; 3], linear: [f64; 3]) -> (ImuData, f32) {
    let p = pose_deg.map(f64::to_radians);
    // Euler rotations' instantaneous axes expressed in earth coordinates.
    let roll_axis = rz(ry([1.0, 0.0, 0.0], p[1]), p[2]);
    let pitch_axis = rz([0.0, 1.0, 0.0], p[2]);
    let omega = std::array::from_fn(|i| {
        roll_axis[i] * euler_rates[0]
            + pitch_axis[i] * euler_rates[1]
            + if i == 2 { euler_rates[2] } else { 0.0 }
    });
    let gyro = body(omega, p);
    let a = body([linear[0], linear[1], linear[2] + G], p);
    (
        ImuData {
            ax: a[0] as f32,
            ay: a[1] as f32,
            az: a[2] as f32,
            wx: gyro[0] as f32,
            wy: gyro[1] as f32,
            wz: gyro[2] as f32,
            roll: pose_deg[0] as f32,
            pitch: pose_deg[1] as f32,
            yaw: pose_deg[2] as f32,
            ..Default::default()
        },
        omega[2] as f32,
    )
}
fn accel(d: &ImuData) -> (f32, f32, f32) {
    (d.ax, d.ay, d.az)
}

#[test]
fn tilted_stationary_gravity_is_not_motion() {
    for pose in [[0.0, 12.0, 0.0], [15.0, -17.0, 123.0], [0.0, 0.0, 0.0]] {
        let (d, _) = sample(pose, [0.0; 3], [0.0; 3]);
        let e = motion_evidence(&d, accel(&d), NOW, PACKETS).unwrap();
        assert!(e.stationary_candidate(0.0), "{pose:?}");
        assert!(!e.stationary_candidate(2.0));
    }
}

#[test]
fn tilted_turn_preserves_planar_lateral_acceleration() {
    for pose in [[8.0, 12.0, 0.0], [-17.0, 19.0, -150.0], [0.0, 0.0, 50.0]] {
        let (d, _) = sample(pose, [0.0, 0.0, 0.3], [0.0; 3]);
        let e = motion_evidence(&d, accel(&d), NOW, PACKETS).unwrap();
        let mut f = SensorFusion::new(FusionConfig::default());
        let (_, lateral) = f.process_imu(
            d.ax,
            d.ay,
            d.az,
            d.roll,
            d.pitch,
            0.0,
            20.0,
            e.rates.earth_vertical,
            0.033,
            false,
        );
        assert!((lateral - 6.0).abs() < 1e-5, "{lateral}");
        assert!((e.rates.heading.unwrap() - 0.3).abs() < 1e-6);
    }
}

#[test]
fn heading_and_vertical_rotation_have_distinct_contracts() {
    for pose in [[0.0, 0.0, 0.0], [17.0, 23.0, 41.0], [-31.0, -37.0, -120.0]] {
        for rates in [[0.4, -0.2, 0.3], [0.0, 0.5, 0.0], [0.5, 0.0, 0.0]] {
            let (d, vertical) = sample(pose, rates, [0.0; 3]);
            let a = angular_rates(&d, NOW, [PACKETS[1], PACKETS[2]]).unwrap();
            assert!((a.heading.unwrap() - rates[2] as f32).abs() < 1e-6);
            assert!((a.earth_vertical - vertical).abs() < 1e-6);
            let mut ekf = sensor_fusion::Ekf::new();
            ekf.predict(0.0, 0.0, a.heading.unwrap(), 0.1);
            assert!((ekf.yaw() - rates[2] as f32 * 0.1).abs() < 1e-6);
        }
    }
}

#[test]
fn horizontal_axis_rotation_and_vertical_acceleration_reject_stationarity() {
    for rates in [[0.4, 0.0, 0.0], [0.0, 0.4, 0.0], [0.0, 0.0, 0.4]] {
        let (d, _) = sample([10.0, 15.0, 80.0], rates, [0.0; 3]);
        assert!(!motion_evidence(&d, accel(&d), NOW, PACKETS)
            .unwrap()
            .stationary_candidate(0.0));
    }
    for axis in 0..3 {
        let mut linear = [0.0; 3];
        linear[axis] = 2.0;
        let (d, _) = sample([10.0, 15.0, 80.0], [0.0; 3], linear);
        assert!(!motion_evidence(&d, accel(&d), NOW, PACKETS)
            .unwrap()
            .stationary_candidate(0.0));
    }
}

#[test]
fn stale_missing_nonfinite_and_singular_evidence_is_explicit() {
    let (d, _) = sample([8.0, 12.0, 0.0], [0.0; 3], [0.0; 3]);
    for i in 0..3 {
        for at in [0, 799_000] {
            let mut times = PACKETS;
            times[i] = at;
            assert!(motion_evidence(&d, accel(&d), NOW, times).is_none());
        }
    }
    for field in 0..5 {
        let mut invalid = d;
        match field {
            0 => invalid.wx = f32::NAN,
            1 => invalid.wy = f32::INFINITY,
            2 => invalid.wz = f32::NAN,
            3 => invalid.roll = f32::NAN,
            _ => invalid.pitch = f32::NAN,
        }
        assert!(angular_rates(&invalid, NOW, [1_000_000; 2]).is_none());
    }
    assert!(motion_evidence(&d, (f32::NAN, d.ay, d.az), NOW, PACKETS).is_none());
    assert!(angular_rates(&d, NOW, [1_000_000; 2]).is_some());
    for pitch in [89.0, 90.0, -90.0] {
        let (d, _) = sample([10.0, pitch, 30.0], [0.2, 0.1, 0.3], [0.0; 3]);
        let a = angular_rates(&d, NOW, [1_000_000; 2]).unwrap();
        assert!(a.heading.is_none());
        assert!(a.earth_vertical.is_finite());
    }
    let wrap_now = 4;
    let at = (u32::MAX as u64 - 5) * 1000;
    assert!(motion_evidence(&d, accel(&d), wrap_now, [at; 3]).is_some());
}

#[test]
fn imu_loss_preserves_finite_packet_and_healthy_gps_timing_json() {
    use blackbox_host_tests::{
        binary_telemetry::TelemetryPacket,
        telemetry_http::{telemetry_json, TimingTelemetry},
    };
    let mut f = SensorFusion::new(FusionConfig::default());
    f.process_imu(0.0, 0.0, G as f32, 0.0, 0.0, 0.0, 20.0, 0.3, 0.033, false);
    let before = (
        f.get_lon_blended(),
        f.get_lat_centripetal(),
        f.get_orientation_correction(),
    );
    for _ in 0..100 {
        f.pause_imu(0.033);
    }
    assert_eq!(
        before,
        (
            f.get_lon_blended(),
            f.get_lat_centripetal(),
            f.get_orientation_correction()
        )
    );
    let mut packet = TelemetryPacket::new();
    packet.ax = -f.get_lon_blended();
    packet.ay = -f.get_lat_centripetal();
    packet.wz = 0.123; // v4 retains body-Z, independent of derived rates.
    packet.gps_valid = 1;
    packet.lap_time_ms = 4321;
    let bytes = packet.to_bytes();
    assert_eq!(bytes.len(), 82);
    assert_eq!(f32::from_le_bytes(bytes[19..23].try_into().unwrap()), 0.123);
    assert!(f32::from_le_bytes(bytes[7..11].try_into().unwrap()).is_finite());
    let timing = TimingTelemetry {
        gps_valid: true,
        gps_age_ms: Some(0),
        imu_age_ms: None,
        state: "timing",
        last_lap_ms: Some(4321),
        ..Default::default()
    };
    let json: serde_json::Value =
        serde_json::from_str(&telemetry_json(9, "packet", &timing, None)).unwrap();
    assert!(json["timing"]["imu_age_ms"].is_null());
    assert_eq!(json["timing"]["gps_valid"], true);
    assert_eq!(json["timing"]["state"], "timing");
    assert_eq!(json["timing"]["last_lap_ms"], 4321);
}

#[test]
fn gps_only_gap_requires_five_new_stationary_fixes_even_without_a_poll() {
    use blackbox_host_tests::imu_evidence::StationaryTracker;
    for start in [1000_u32, u32::MAX - 100] {
        let mut tracker = StationaryTracker::default();
        for i in 0..5 {
            tracker.record_gps_fix(start.wrapping_add(i * 40));
            tracker.observe_candidate(true);
        }
        assert!(tracker.confirmed());
        // IMU is healthy throughout. Recovery can happen before another polling
        // callback observes stale GPS (e.g. a stalled loop or buffered fixes).
        let recovered = start.wrapping_add(1661);
        assert!(!tracker.gps_is_fresh(recovered));
        tracker.record_gps_fix(recovered);
        tracker.observe_candidate(true);
        assert!(!tracker.confirmed(), "recovery reused parked evidence");
        for i in 1..4 {
            tracker.record_gps_fix(recovered.wrapping_add(i * 40));
            tracker.observe_candidate(true);
            assert!(!tracker.confirmed());
        }
        tracker.record_gps_fix(recovered.wrapping_add(160));
        tracker.observe_candidate(true);
        assert!(tracker.confirmed());
        tracker.invalidate();
        assert!(!tracker.confirmed());
    }
}
