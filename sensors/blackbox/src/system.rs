#![allow(dead_code)] // System API for future use

use std::sync::Arc;

/// System-level architecture for telemetry system
/// Implements proper separation of concerns and dependency inversion
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::uart::UartDriver;
// Import from framework crate
use sensor_fusion::ekf::Ekf;
use sensor_fusion::velocity::select_best_velocity;

use crate::{
    binary_telemetry,
    gps::NmeaParser,
    imu::{ImuBias, ImuCalibrator, PacketType, Wt901Parser},
    mode::ModeClassifier,
    mqtt::MqttClient,
    rgb_led::RgbLed,
    udp_stream::UdpTelemetryStream,
    websocket_server::TelemetryServerState,
};

const CALIB_SAMPLES: usize = 150;

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

    /// Get best available velocity estimate as (vx, vy, speed) tuple
    ///
    /// Returns consistent values from the same source:
    /// - Prefers EKF velocity when magnitude > 0.1 m/s
    /// - Falls back to GPS velocity (from speed + course)
    /// - Returns zeros if no valid source available
    pub fn get_velocity(&self, ekf: Option<&Ekf>) -> (f32, f32, f32) {
        let ekf_velocity = ekf.map(|e| e.velocity());
        let gps_velocity = self.gps_parser.get_velocity_enu();
        select_best_velocity(ekf_velocity, gps_velocity)
    }

    /// Get best available speed estimate (m/s)
    ///
    /// Convenience wrapper around get_velocity() for consumers
    /// that only need scalar speed.
    pub fn get_speed(&self, ekf: Option<&Ekf>) -> f32 {
        self.get_velocity(ekf).2
    }

    /// Calibrate IMU (must be stationary)
    pub fn calibrate_imu(
        &mut self,
        led: &mut RgbLed,
        mut mqtt: Option<&mut MqttClient>,
    ) -> Result<ImuBias, SystemError> {
        use log::info;

        info!("=== IMU Calibration Starting ===");
        info!(
            "Collecting {} samples, keep device stationary!",
            CALIB_SAMPLES
        );

        // Initial yellow LED burst to indicate calibration starting
        for _ in 0..3 {
            led.yellow().ok();
            FreeRtos::delay_ms(100);
            led.set_low().ok();
            FreeRtos::delay_ms(100);
        }

        let mut calibrator = ImuCalibrator::new(CALIB_SAMPLES);
        let mut buf = [0u8; 1];
        let mut last_progress = 0;

        // Keep LED on during calibration (solid yellow)
        led.yellow().ok();

        while !calibrator.is_complete() {
            if self.imu_uart.read(&mut buf, 0).unwrap_or(0) > 0 {
                let timestamp_us = unsafe { esp_idf_svc::sys::esp_timer_get_time() as u64 };
                if let Some(packet_type) = self.imu_parser.feed_byte(buf[0], timestamp_us) {
                    if packet_type == PacketType::Accel {
                        let data = self.imu_parser.data();
                        calibrator.add_sample(data.ax, data.ay, data.az);

                        let progress = (calibrator.progress() * 100.0) as u32;
                        if progress > last_progress && progress.is_multiple_of(10) {
                            info!("Calibration progress: {}%", progress);
                            if let Some(ref mut mqtt_client) = mqtt {
                                mqtt_client
                                    .publish_status("calib_running", Some(calibrator.progress()))
                                    .ok();
                            }
                            last_progress = progress;
                            // Blink off briefly then back on to show progress
                            led.set_low().ok();
                            FreeRtos::delay_ms(50);
                            led.yellow().ok();
                        }
                    }
                }
            }
        }

        // Turn off LED when done
        led.set_low().ok();
        info!("=== IMU Calibration Complete ===");

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
    pub fn update_mode(&mut self, ax_e: f32, ay_e: f32, yaw: f32, wz: f32, speed: f32) {
        self.mode_classifier.update(ax_e, ay_e, yaw, wz, speed);
    }

    /// Reset speed display (for ZUPT)
    pub fn reset_speed(&mut self) {
        self.mode_classifier.reset_speed();
    }
}

/// Manages telemetry publishing (supports both UDP/Station and HTTP/AP modes)
pub struct TelemetryPublisher {
    udp_stream: Option<UdpTelemetryStream>,
    mqtt_client: Option<MqttClient>,
    http_state: Option<Arc<TelemetryServerState>>,
    telemetry_count: u32,
    telemetry_fail_count: u32,
}

impl TelemetryPublisher {
    pub fn new(
        udp_stream: Option<UdpTelemetryStream>,
        mqtt_client: Option<MqttClient>,
        http_state: Option<Arc<TelemetryServerState>>,
    ) -> Self {
        Self {
            udp_stream,
            mqtt_client,
            http_state,
            telemetry_count: 0,
            telemetry_fail_count: 0,
        }
    }

    /// Get mutable reference to MQTT client
    pub fn mqtt_client_mut(&mut self) -> Option<&mut MqttClient> {
        self.mqtt_client.as_mut()
    }

    /// Publish binary telemetry packet via UDP and/or HTTP server state
    pub fn publish_telemetry(
        &mut self,
        sensors: &SensorManager,
        estimator: &StateEstimator,
        now_ms: u32,
    ) -> Result<(), SystemError> {
        // Send bias-corrected accelerations (client can process further if needed)
        let (ax_corr, ay_corr, az_corr) = sensors.imu_parser.get_accel_corrected();

        let mut packet = binary_telemetry::TelemetryPacket::new();
        packet.timestamp_ms = now_ms;

        // Send bias-corrected accelerations in body frame
        // Client-side processing can remove gravity and transform to vehicle frame
        packet.ax = ax_corr;
        packet.ay = ay_corr;
        packet.az = az_corr;
        packet.wz = sensors.imu_parser.data().wz;
        packet.roll = sensors.imu_parser.data().roll.to_radians();
        packet.pitch = sensors.imu_parser.data().pitch.to_radians();
        packet.yaw = estimator.ekf.yaw();

        let (ekf_x, ekf_y) = estimator.ekf.position();

        // Get velocity from best available source (EKF preferred, GPS fallback)
        // Returns consistent (vx, vy, speed) tuple from same source
        let (mut vx, mut vy, speed_ms) = sensors.get_velocity(Some(&estimator.ekf));
        let mut display_speed_kmh = speed_ms * 3.6;

        // Zero out very low speeds to clean up display
        if display_speed_kmh < 1.0 {
            display_speed_kmh = 0.0;
            vx = 0.0;
            vy = 0.0;
        }

        packet.x = ekf_x;
        packet.y = ekf_y;
        packet.vx = vx;
        packet.vy = vy;
        packet.speed_kmh = display_speed_kmh;
        packet.mode = estimator.mode_classifier.get_mode_u8();

        if sensors.gps_parser.last_fix().valid {
            packet.lat = sensors.gps_parser.last_fix().lat as f32;
            packet.lon = sensors.gps_parser.last_fix().lon as f32;
            packet.gps_valid = 1;
        } else {
            packet.gps_valid = 0;
        }

        let bytes = packet.to_bytes();
        let mut sent = false;

        // Send via UDP if available (Station mode)
        if let Some(ref mut udp) = self.udp_stream {
            if udp.is_ready() {
                match udp.send(bytes) {
                    Ok(_) => sent = true,
                    Err(_) => self.telemetry_fail_count += 1,
                }
            }
        }

        // Update HTTP server state if available (AP mode)
        if let Some(ref http_state) = self.http_state {
            http_state.update_telemetry(bytes);
            sent = true;
        }

        if sent {
            self.telemetry_count += 1;
            Ok(())
        } else {
            self.telemetry_fail_count += 1;
            Err(SystemError::CommunicationError(
                "No telemetry output available",
            ))
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
