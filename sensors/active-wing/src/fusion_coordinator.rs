/// Fusion Coordinator - Centralized sensor fusion from multiple distributed sensors
///
/// Architecture:
/// - Each sensor publishes to its own MQTT topic
/// - Coordinator subscribes to all sensor topics
/// - Coordinator runs multi-sensor EKF
/// - Coordinator publishes fused state estimate
///
/// Benefits:
/// - Sensors can be on different ESP32 boards
/// - Easy to add/remove sensors dynamically
/// - Sensor failures don't crash the system
/// - Can fuse data from heterogeneous sources

use crate::sensor_framework::*;
use crate::ekf::Ekf;
use crate::transforms::{body_to_earth, remove_gravity};
use std::collections::HashMap;

/// Multi-sensor fusion coordinator
pub struct FusionCoordinator {
    /// Extended Kalman Filter for state estimation
    ekf: Ekf,

    /// Registered sensor capabilities
    sensors: HashMap<SensorId, SensorCapabilities>,

    /// Last update time per sensor (for timeout detection)
    last_updates: HashMap<SensorId, u64>,

    /// Sensor timeout threshold (microseconds)
    timeout_us: u64,

    /// IMU orientation for gravity removal
    current_roll: f32,
    current_pitch: f32,

    /// Last IMU timestamp for dt calculation
    last_imu_us: u64,
}

impl FusionCoordinator {
    pub fn new() -> Self {
        Self {
            ekf: Ekf::new(),
            sensors: HashMap::new(),
            last_updates: HashMap::new(),
            timeout_us: 1_000_000,  // 1 second timeout
            current_roll: 0.0,
            current_pitch: 0.0,
            last_imu_us: 0,
        }
    }

    /// Register a sensor with the coordinator
    pub fn register_sensor(&mut self, caps: SensorCapabilities) {
        log::info!("Fusion: Registered sensor {} ({}) @ {}Hz",
                   caps.id.0, caps.name, caps.update_rate_hz);
        self.sensors.insert(caps.id, caps);
    }

    /// Process incoming sensor reading
    pub fn process_reading(&mut self, reading: SensorReading) {
        let now_us = unsafe { esp_idf_svc::sys::esp_timer_get_time() as u64 };
        self.last_updates.insert(reading.sensor_id, now_us);

        match reading.data {
            SensorData::Imu(imu) => self.process_imu(imu, reading.timestamp_us),
            SensorData::Gps(gps) => self.process_gps(gps, reading.timestamp_us),
            SensorData::WheelSpeed(wheels) => self.process_wheel_speed(wheels, reading.timestamp_us),
            SensorData::Can(can) => self.process_can(can, reading.timestamp_us),
            _ => {
                log::warn!("Fusion: Unsupported sensor data type");
            }
        }
    }

    /// Process IMU reading
    fn process_imu(&mut self, imu: ImuReading, timestamp_us: u64) {
        // Store orientation for gravity removal
        // In a real implementation, you'd extract this from the IMU data
        // For now, assume orientation is in a separate field or computed

        // Remove gravity from accelerometer
        let (ax_b, ay_b, az_b) = remove_gravity(
            imu.accel[0], imu.accel[1], imu.accel[2],
            self.current_roll, self.current_pitch,
        );

        // Transform to earth frame
        let (ax_e, ay_e) = body_to_earth(
            ax_b, ay_b, az_b,
            self.current_roll, self.current_pitch,
            self.ekf.yaw(),
        );

        // Compute dt from last IMU update
        if self.last_imu_us == 0 {
            self.last_imu_us = timestamp_us;
            return;
        }
        let dt = (timestamp_us - self.last_imu_us) as f32 * 1e-6;
        self.last_imu_us = timestamp_us;

        if dt > 0.0005 && dt < 0.05 {
            // EKF prediction step
            self.ekf.predict(ax_e, ay_e, imu.gyro[2], dt);
        }
    }

    /// Process GPS reading
    fn process_gps(&mut self, gps: GpsReading, _timestamp_us: u64) {
        // For simplicity, assume GPS provides local coordinates
        // In reality, you'd convert lat/lon to local frame

        // Update position (would need reference point setup)
        // self.ekf.update_position(x, y);

        // Update velocity from GPS course and speed
        let vx = gps.speed * gps.course.cos();
        let vy = gps.speed * gps.course.sin();
        self.ekf.update_velocity(vx, vy);
        self.ekf.update_speed(gps.speed);
    }

    /// Process wheel speed reading
    fn process_wheel_speed(&mut self, wheels: WheelSpeedReading, _timestamp_us: u64) {
        // Wheel speed provides excellent velocity estimate
        // Average of all wheels (assuming no-slip)
        const WHEEL_RADIUS: f32 = 0.3;  // meters (adjust for your car)

        let v_fl = wheels.fl * WHEEL_RADIUS;
        let v_fr = wheels.fr * WHEEL_RADIUS;
        let v_rl = wheels.rl * WHEEL_RADIUS;
        let v_rr = wheels.rr * WHEEL_RADIUS;

        let avg_speed = (v_fl + v_fr + v_rl + v_rr) / 4.0;

        // Update speed (higher weight than GPS due to better accuracy)
        self.ekf.update_speed(avg_speed);

        // Could also estimate yaw rate from wheel speed differential
        // wz = (v_right - v_left) / track_width
    }

    /// Process CAN bus reading
    fn process_can(&mut self, can: CanReading, _timestamp_us: u64) {
        // Parse CAN message based on ID
        match can.can_id {
            0x100 => {
                // Example: Throttle position
                // let throttle = can.data[0] as f32 / 255.0;
            }
            0x200 => {
                // Example: Brake pressure
                // let brake = u16::from_be_bytes([can.data[0], can.data[1]]) as f32;
            }
            _ => {}
        }

        // CAN data can provide additional constraints:
        // - If brake is applied, expect deceleration
        // - If throttle is 0, expect no positive acceleration
    }

    /// Check for sensor timeouts
    pub fn check_timeouts(&mut self, now_us: u64) -> Vec<SensorId> {
        let mut timed_out = Vec::new();

        for (id, last_update) in &self.last_updates {
            if now_us - last_update > self.timeout_us {
                timed_out.push(*id);
                log::warn!("Sensor {} timed out!", id.0);
            }
        }

        timed_out
    }

    /// Get current fused state estimate
    pub fn get_state(&self) -> FusedState {
        let (x, y) = self.ekf.position();
        let (vx, vy) = self.ekf.velocity();
        let yaw = self.ekf.yaw();
        let speed = self.ekf.speed();

        FusedState {
            timestamp_us: unsafe { esp_idf_svc::sys::esp_timer_get_time() as u64 },
            x, y,
            vx, vy,
            yaw,
            speed,
            active_sensors: self.last_updates.len(),
        }
    }
}

/// Fused state output
#[derive(Debug, Clone)]
pub struct FusedState {
    pub timestamp_us: u64,
    pub x: f32,
    pub y: f32,
    pub vx: f32,
    pub vy: f32,
    pub yaw: f32,
    pub speed: f32,
    pub active_sensors: usize,
}

/// MQTT-based fusion coordinator (for distributed deployment)
pub struct MqttFusionCoordinator {
    coordinator: FusionCoordinator,
    // mqtt_client: MqttClient,  // Would be injected
}

impl MqttFusionCoordinator {
    pub fn new() -> Self {
        Self {
            coordinator: FusionCoordinator::new(),
        }
    }

    /// Subscribe to all registered sensor topics
    pub fn subscribe_to_sensors(&mut self) {
        // In a full implementation:
        // for sensor in self.coordinator.sensors.values() {
        //     self.mqtt_client.subscribe(sensor.mqtt_topic);
        // }
    }

    /// Handle incoming MQTT message from a sensor
    pub fn on_sensor_message(&mut self, _topic: &str, _payload: &[u8]) {
        // Deserialize sensor reading
        // let reading: SensorReading = serde_json::from_slice(payload).unwrap();
        // self.coordinator.process_reading(reading);
    }

    /// Publish fused state
    pub fn publish_fused_state(&mut self) {
        let state = self.coordinator.get_state();
        // self.mqtt_client.publish("car/fusion/state", serialize(state));
        log::info!("Fused state: x={:.2} y={:.2} v={:.2}m/s yaw={:.2}° sensors={}",
                   state.x, state.y, state.speed, state.yaw.to_degrees(), state.active_sensors);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fusion_coordinator() {
        let mut coordinator = FusionCoordinator::new();

        // Register sensors
        coordinator.register_sensor(SensorCapabilities {
            id: SensorId(1),
            name: "IMU",
            measurement_types: &[MeasurementType::Acceleration3D],
            update_rate_hz: 200.0,
            latency_ms: 5.0,
            accuracy: 0.1,
            mqtt_topic: "car/sensors/imu",
        });

        coordinator.register_sensor(SensorCapabilities {
            id: SensorId(2),
            name: "GPS",
            measurement_types: &[MeasurementType::Velocity3D],
            update_rate_hz: 5.0,
            latency_ms: 200.0,
            accuracy: 2.5,
            mqtt_topic: "car/sensors/gps",
        });

        // Simulate GPS reading
        let gps_reading = SensorReading {
            sensor_id: SensorId(2),
            timestamp_us: 1000000,
            data: SensorData::Gps(GpsReading {
                lat: 37.7749,
                lon: -122.4194,
                alt: None,
                speed: 10.0,
                course: 0.0,
                hdop: Some(1.2),
                satellites: Some(8),
            }),
        };

        coordinator.process_reading(gps_reading);

        let state = coordinator.get_state();
        assert!(state.speed > 0.0);
    }
}
