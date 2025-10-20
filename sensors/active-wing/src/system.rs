#![allow(dead_code)] // System API for future use

/// System-level architecture for telemetry system
/// Implements proper separation of concerns and dependency inversion
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::uart::UartDriver;
use log::info;
// Import from framework crate
use motorsport_telemetry::ekf::Ekf;
use motorsport_telemetry::transforms::{body_to_earth, remove_gravity};

use crate::{
    binary_telemetry,
    gps::NmeaParser,
    imu::{ImuBias, ImuCalibrator, PacketType, Wt901Parser},
    mode::ModeClassifier,
    mqtt::MqttClient,
    rgb_led::RgbLed,
    tcp_stream::TcpTelemetryStream,
};

const CALIB_SAMPLES: usize = 500;

/// System-level errors
#[derive(Debug)]
pub enum SystemError {
    SensorError(&'static str),
    CommunicationError(&'static str),
    CalibrationFailed,
}

/// Manages sensor data acquisition
pub struct SensorManager {
    pub imu_parser: Wt901Parser,
    pub gps_parser: NmeaParser,
    imu_uart: UartDriver<'static>,
    gps_uart: UartDriver<'static>,
    last_imu_us: u64,
    stationary_count: u32,
}

impl SensorManager {
    pub fn new(imu_uart: UartDriver<'static>, gps_uart: UartDriver<'static>) -> Self {
        Self {
            imu_parser: Wt901Parser::new(),
            gps_parser: NmeaParser::new(),
            imu_uart,
            gps_uart,
            last_imu_us: unsafe { esp_idf_svc::sys::esp_timer_get_time() as u64 },
            stationary_count: 0,
        }
    }

    /// Get IMU data (convenience method)
    pub fn imu_data(&self) -> &wt901::ImuData {
        self.imu_parser.data()
    }

    /// Get GPS fix (convenience method)
    pub fn gps_fix(&self) -> &neo6m::GpsFix {
        self.gps_parser.last_fix()
    }

    /// Calibrate IMU (must be stationary)
    pub fn calibrate_imu(
        &mut self,
        led: &mut RgbLed,
        mut mqtt: Option<&mut MqttClient>,
    ) -> Result<ImuBias, SystemError> {
        let mut calibrator = ImuCalibrator::new(CALIB_SAMPLES);
        let mut buf = [0u8; 1];
        let mut last_progress = 0;

        while !calibrator.is_complete() {
            if self.imu_uart.read(&mut buf, 0).unwrap_or(0) > 0 {
                let timestamp_us = unsafe { esp_idf_svc::sys::esp_timer_get_time() as u64 };
                if let Some(packet_type) = self.imu_parser.feed_byte(buf[0], timestamp_us) {
                    if packet_type == PacketType::Accel {
                        let data = self.imu_parser.data();
                        calibrator.add_sample(data.ax, data.ay, data.az);

                        let progress = (calibrator.progress() * 100.0) as u32;
                        if progress > last_progress && progress.is_multiple_of(10) {
                            if let Some(ref mut mqtt_client) = mqtt {
                                mqtt_client
                                    .publish_status("calib_running", Some(calibrator.progress()))
                                    .ok();
                            }
                            last_progress = progress;
                            led.yellow().ok();
                            FreeRtos::delay_ms(50);
                            led.set_low().ok();
                        }
                    }
                }
            }
        }

        calibrator
            .compute_bias()
            .ok_or(SystemError::CalibrationFailed)
    }

    /// Poll IMU and return dt if new accel data available
    pub fn poll_imu(&mut self) -> Option<(f32, u64)> {
        let now_us = unsafe { esp_idf_svc::sys::esp_timer_get_time() as u64 };
        let mut buf = [0u8; 1];

        if self.imu_uart.read(&mut buf, 0).unwrap_or(0) > 0 {
            if let Some(packet_type) = self.imu_parser.feed_byte(buf[0], now_us) {
                if packet_type == PacketType::Accel {
                    let dt = (now_us - self.last_imu_us) as f32 * 1e-6;
                    self.last_imu_us = now_us;

                    if dt > 5e-4 && dt < 0.05 {
                        return Some((dt, now_us));
                    }
                }
            }
        }
        None
    }

    /// Poll GPS and return true if new fix available
    pub fn poll_gps(&mut self) -> bool {
        let mut buf = [0u8; 1];

        if self.gps_uart.read(&mut buf, 0).unwrap_or(0) > 0 {
            return self.gps_parser.feed_byte(buf[0]);
        }
        false
    }

    /// Check if vehicle is stationary
    pub fn is_stationary(&mut self, ax: f32, ay: f32, wz: f32) -> bool {
        const ACC_THR: f32 = 0.18 * 9.80665;
        const WZ_THR: f32 = 12.0 * 0.017453293;
        const GPS_SPEED_THR: f32 = 3.5;
        const POS_SPEED_THR: f32 = 5.0;

        let low_inertial = ax.abs() < ACC_THR && ay.abs() < ACC_THR && wz.abs() < WZ_THR;
        let low_speed = self.gps_parser.last_fix().speed < GPS_SPEED_THR
            && self.gps_parser.position_based_speed() < POS_SPEED_THR;

        let stationary = low_inertial && low_speed;

        if stationary {
            self.stationary_count += 1;
        } else {
            self.stationary_count = 0;
        }

        stationary && self.stationary_count >= 5
    }
}

/// Manages state estimation (sensor fusion)
pub struct StateEstimator {
    pub ekf: Ekf,
    pub mode_classifier: ModeClassifier,
}

impl StateEstimator {
    pub fn new() -> Self {
        Self {
            ekf: Ekf::new(),
            mode_classifier: ModeClassifier::new(),
        }
    }

    /// Update EKF prediction with IMU data
    pub fn predict(&mut self, ax_earth: f32, ay_earth: f32, wz: f32, dt: f32) {
        self.ekf.predict(ax_earth, ay_earth, wz, dt);
    }

    /// Update EKF with GPS position
    pub fn update_position(&mut self, x: f32, y: f32) {
        self.ekf.update_position(x, y);
    }

    /// Update EKF with GPS velocity
    pub fn update_velocity(&mut self, vx: f32, vy: f32) {
        self.ekf.update_velocity(vx, vy);
    }

    /// Update EKF with GPS speed
    pub fn update_speed(&mut self, speed: f32) {
        self.ekf.update_speed(speed);
    }

    /// Update with magnetometer yaw
    pub fn update_yaw(&mut self, yaw: f32) {
        self.ekf.update_yaw(yaw);
    }

    /// Perform ZUPT (zero velocity update)
    pub fn zupt(&mut self) {
        self.ekf.zupt();
    }

    /// Update accelerometer bias
    pub fn update_bias(&mut self, ax: f32, ay: f32) {
        self.ekf.update_bias(ax, ay);
    }

    /// Update mode classifier
    pub fn update_mode(&mut self, ax_e: f32, ay_e: f32, yaw: f32, wz: f32, vx: f32, vy: f32) {
        self.mode_classifier.update(ax_e, ay_e, yaw, wz, vx, vy);
    }

    /// Reset speed display (for ZUPT)
    pub fn reset_speed(&mut self) {
        self.mode_classifier.reset_speed();
    }
}

/// Manages telemetry publishing
pub struct TelemetryPublisher {
    tcp_stream: Option<TcpTelemetryStream>,
    mqtt_client: Option<MqttClient>,
    telemetry_count: u32,
    telemetry_fail_count: u32,
}

impl TelemetryPublisher {
    pub fn new(tcp_stream: Option<TcpTelemetryStream>, mqtt_client: Option<MqttClient>) -> Self {
        Self {
            tcp_stream,
            mqtt_client,
            telemetry_count: 0,
            telemetry_fail_count: 0,
        }
    }

    /// Get mutable reference to MQTT client
    pub fn mqtt_client_mut(&mut self) -> Option<&mut MqttClient> {
        self.mqtt_client.as_mut()
    }

    /// Publish binary telemetry packet via TCP
    pub fn publish_telemetry(
        &mut self,
        sensors: &SensorManager,
        estimator: &StateEstimator,
        now_ms: u32,
    ) -> Result<(), SystemError> {
        if self.tcp_stream.is_none() {
            return Err(SystemError::CommunicationError("TCP not connected"));
        }

        let (ax_corr, ay_corr, az_corr) = sensors.imu_parser.get_accel_corrected();
        let (ax_b, ay_b, _) = remove_gravity(
            ax_corr,
            ay_corr,
            sensors.imu_parser.data().az,
            sensors.imu_parser.data().roll,
            sensors.imu_parser.data().pitch,
        );
        let (_ax_e, _ay_e) = body_to_earth(
            ax_b,
            ay_b,
            0.0,
            sensors.imu_parser.data().roll,
            sensors.imu_parser.data().pitch,
            estimator.ekf.yaw(),
        );

        let mut packet = binary_telemetry::TelemetryPacket::new();
        packet.timestamp_ms = now_ms;

        packet.ax = ax_corr;
        packet.ay = ay_corr;
        packet.az = az_corr;
        packet.wz = sensors.imu_parser.data().wz;
        packet.roll = sensors.imu_parser.data().roll.to_radians();
        packet.pitch = sensors.imu_parser.data().pitch.to_radians();
        packet.yaw = estimator.ekf.yaw();

        let (ekf_x, ekf_y) = estimator.ekf.position();
        let (mut ekf_vx, mut ekf_vy) = estimator.ekf.velocity();
        let mut ekf_speed_kmh = estimator.mode_classifier.get_speed_kmh();

        if ekf_speed_kmh < 0.5 {
            ekf_speed_kmh = 0.0;
            ekf_vx = 0.0;
            ekf_vy = 0.0;
        }

        packet.x = ekf_x;
        packet.y = ekf_y;
        packet.vx = ekf_vx;
        packet.vy = ekf_vy;
        packet.speed_kmh = ekf_speed_kmh;
        packet.mode = binary_telemetry::mode_to_u8(estimator.mode_classifier.get_mode().as_str());

        if sensors.gps_parser.last_fix().valid {
            packet.lat = sensors.gps_parser.last_fix().lat as f32;
            packet.lon = sensors.gps_parser.last_fix().lon as f32;
            packet.gps_valid = 1;
        } else {
            packet.gps_valid = 0;
        }

        let bytes = packet.to_bytes();

        let tcp = self
            .tcp_stream
            .as_mut()
            .ok_or(SystemError::CommunicationError("TCP not connected"))?;

        match tcp.send(bytes) {
            Ok(_) => {
                self.telemetry_count += 1;
                Ok(())
            }
            Err(_) => {
                self.telemetry_fail_count += 1;
                if self.telemetry_fail_count.is_multiple_of(100)
                    && tcp.reconnect().is_ok() {
                        info!("TCP reconnected");
                        self.telemetry_fail_count = 0;
                    }
                Err(SystemError::CommunicationError("TCP send failed"))
            }
        }
    }

    /// Publish MQTT status message
    pub fn publish_status(&mut self, status: &str, value: Option<f32>) -> Result<(), SystemError> {
        if let Some(mqtt) = &mut self.mqtt_client {
            mqtt.publish_status(status, value)
                .map_err(|_| SystemError::CommunicationError("MQTT publish failed"))
        } else {
            Ok(())
        }
    }

    /// Get telemetry statistics
    pub fn get_stats(&self) -> (u32, u32) {
        (self.telemetry_count, self.telemetry_fail_count)
    }

    /// Reset telemetry counters
    pub fn reset_stats(&mut self) {
        self.telemetry_count = 0;
    }
}

/// Manages LED status display
pub struct StatusManager {
    led: RgbLed,
    gps_was_locked: bool,
    last_gps_status_ms: u32,
}

impl StatusManager {
    pub fn new(led: RgbLed) -> Self {
        Self {
            led,
            gps_was_locked: false,
            last_gps_status_ms: 0,
        }
    }

    /// Update LED based on GPS status
    pub fn update_gps_status(
        &mut self,
        gps_locked: bool,
        gps_warmed_up: bool,
        now_ms: u32,
        mut mqtt: Option<&mut MqttClient>,
    ) {
        // Detect GPS lock changes
        if gps_locked && !self.gps_was_locked {
            if let Some(ref mut mqtt_client) = mqtt {
                mqtt_client.publish_status("gps_lock", None).ok();
            }
        }

        if !gps_locked && self.gps_was_locked {
            if let Some(ref mut mqtt_client) = mqtt {
                mqtt_client.publish_status("gps_lost", None).ok();
            }
        }

        // Periodic status updates
        if !gps_locked && now_ms - self.last_gps_status_ms >= 5000 {
            if let Some(ref mut mqtt_client) = mqtt {
                if !gps_warmed_up {
                    mqtt_client.publish_status("waiting_gps", None).ok();
                } else {
                    mqtt_client.publish_status("gps_lost", None).ok();
                }
            }
            self.last_gps_status_ms = now_ms;
        }

        self.gps_was_locked = gps_locked;
    }

    /// Update LED display
    pub fn update_led(&mut self, gps_locked: bool, now_ms: u32) {
        if gps_locked {
            if now_ms % 2000 < 100 {
                self.led.cyan().ok();
            } else {
                self.led.set_low().ok();
            }
        } else if now_ms % 500 < 250 {
            self.led.yellow().ok();
        } else {
            self.led.set_low().ok();
        }
    }

    /// Get mutable reference to LED (for boot sequence)
    pub fn led_mut(&mut self) -> &mut RgbLed {
        &mut self.led
    }
}
