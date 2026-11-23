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
    gps_timing::GpsTimingQueue,
    imu::{ImuBias, PacketType, Wt901Parser},
    imu_bias_collection::{BiasCollection, CALIB_SAMPLES},
    mode::ModeClassifier,
    mqtt::MqttClient,
    rgb_led::RgbLed,
    udp_stream::UdpTelemetryStream,
    websocket_server::TelemetryServerState,
};

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
    last_gyro_us: u64,
    last_angle_us: u64,
    stationary: crate::imu_evidence::StationaryTracker,
    gps_timing: GpsTimingQueue,
}

impl SensorManager {
    pub fn new(
        imu_uart: UartDriver<'static>,
        gps_uart: UartDriver<'static>,
        warmup_fixes: u8,
    ) -> Self {
        Self {
            imu_parser: Wt901Parser::new(),
            gps_parser: NmeaParser::with_warmup_fixes(warmup_fixes),
            imu_uart,
            gps_uart,
            last_imu_us: unsafe { esp_idf_svc::sys::esp_timer_get_time() as u64 },
            last_gyro_us: 0,
            last_angle_us: 0,
            stationary: crate::imu_evidence::StationaryTracker::default(),
            gps_timing: GpsTimingQueue::new(),
        }
    }

    pub fn calibration_imu_age_ms(&self, now_ms: u32) -> u32 {
        let accel_age = now_ms.wrapping_sub((self.last_imu_us / 1000) as u32);
        let gyro_age = now_ms.wrapping_sub((self.last_gyro_us / 1000) as u32);
        let angle_age = now_ms.wrapping_sub((self.last_angle_us / 1000) as u32);
        if self.last_gyro_us == 0 || self.last_angle_us == 0 || gyro_age > 200 || angle_age > 200 {
            u32::MAX
        } else {
            accel_age
        }
    }

    pub fn angular_rates(&self, now_ms: u32) -> Option<sensor_fusion::transforms::AngularRates> {
        crate::imu_evidence::angular_rates(
            self.imu_parser.data(),
            now_ms,
            [self.last_gyro_us, self.last_angle_us],
        )
    }

    pub fn motion_evidence(&self, now_ms: u32) -> Option<crate::imu_evidence::MotionEvidence> {
        crate::imu_evidence::motion_evidence(
            self.imu_parser.data(),
            self.imu_parser.get_accel_corrected(),
            now_ms,
            [self.last_imu_us, self.last_gyro_us, self.last_angle_us],
        )
    }

    pub fn record_gps_fix(&mut self, now_ms: u32) {
        self.stationary.record_gps_fix(now_ms);
    }

    pub fn gps_age_ms(&self, now_ms: u32) -> Option<u32> {
        self.stationary.gps_age_ms(now_ms)
    }

    pub fn gps_is_fresh(&self, now_ms: u32) -> bool {
        self.gps_parser.last_fix().valid && self.stationary.gps_is_fresh(now_ms)
    }

    /// Get IMU data (convenience method)
    pub fn imu_data(&self) -> &wt901::ImuData {
        self.imu_parser.data()
    }

    /// Get GPS fix (convenience method)
    pub fn gps_fix(&self) -> &ublox_gps::GpsFix {
        self.gps_parser.last_fix()
    }

    /// Get best available velocity estimate as (vx, vy, speed) tuple
    ///
    /// Returns consistent values from the same source:
    /// - Prefers fresh, warmed, valid GPS velocity (including zero)
    /// - Falls back to bounded EKF velocity when GPS is unavailable
    /// - Returns zeros if no valid source available
    pub fn get_velocity(&self, ekf: Option<&Ekf>, now_ms: u32) -> (f32, f32, f32) {
        let ekf_velocity = ekf.map(|e| e.velocity());
        let gps_velocity = self
            .gps_parser
            .get_velocity_enu()
            .filter(|_| self.gps_parser.is_warmed_up() && self.gps_is_fresh(now_ms));
        select_best_velocity(ekf_velocity, gps_velocity)
    }

    /// Get best available speed estimate (m/s)
    ///
    /// Convenience wrapper around get_velocity() for consumers
    /// that only need scalar speed.
    pub fn get_speed(&self, ekf: Option<&Ekf>, now_ms: u32) -> f32 {
        self.get_velocity(ekf, now_ms).2
    }

    /// Calibrate stationary bias using fresh AHRS gravity projection.
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

        let mut buf = [0u8; 1];
        let mut last_progress = 0;

        // Keep LED on during calibration (solid yellow)
        led.yellow().ok();

        let started_us = unsafe { esp_idf_svc::sys::esp_timer_get_time() as u64 };
        let mut calibrator = BiasCollection::new(started_us);
        while !calibrator.is_complete() {
            if calibrator.expired(unsafe { esp_idf_svc::sys::esp_timer_get_time() as u64 }) {
                led.set_low().ok();
                return Err(SystemError::CalibrationFailed);
            }
            if self.imu_uart.read(&mut buf, 0).unwrap_or(0) > 0 {
                let timestamp_us = unsafe { esp_idf_svc::sys::esp_timer_get_time() as u64 };
                if let Some(packet_type) = self.imu_parser.feed_byte(buf[0], timestamp_us) {
                    if packet_type == PacketType::Gyro {
                        self.last_gyro_us = timestamp_us;
                    }
                    if packet_type == PacketType::Angle {
                        self.last_angle_us = timestamp_us;
                    }
                    if packet_type == PacketType::Accel {
                        match calibrator.observe_accel(
                            timestamp_us,
                            self.imu_parser.data(),
                            self.last_angle_us,
                            self.last_gyro_us,
                        ) {
                            None => continue,
                            Some(false) => {
                                last_progress = 0;
                                continue;
                            }
                            Some(true) => {}
                        }

                        let progress = (calibrator.progress() * 100.0) as u32;
                        if progress < last_progress {
                            last_progress = 0;
                        }
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
            } else {
                FreeRtos::delay_ms(1);
            }
        }

        // Turn off LED when done
        led.set_low().ok();
        info!("=== IMU Calibration Complete ===");

        // Reset last_imu_us to current time so first poll_imu() doesn't have huge dt
        self.last_imu_us = unsafe { esp_idf_svc::sys::esp_timer_get_time() as u64 };

        calibrator
            .compute_bias()
            .ok_or(SystemError::CalibrationFailed)
    }

    /// Poll IMU and return dt if new accel data available
    ///
    /// Reads ALL available bytes from UART buffer to prevent overflow.
    /// At 115200 baud / 200 Hz, IMU sends ~6600 bytes/sec.
    /// Reading multiple bytes per call keeps up with the data rate.
    ///
    /// Returns Option<(dt, timestamp_us, accel_packet_count)>
    /// - dt: time since last valid Accel packet
    /// - timestamp_us: current timestamp
    /// - accel_packet_count: number of Accel packets processed (for
    ///   diagnostics)
    pub fn poll_imu(&mut self) -> Option<(f32, u64, u32)> {
        let now_us = unsafe { esp_idf_svc::sys::esp_timer_get_time() as u64 };
        let mut buf = [0u8; 64]; // Read up to 64 bytes at once (~10 packets worth)
        let previous_us = self.last_imu_us;
        let mut accel_count: u32 = 0;

        // Read all available bytes to prevent buffer overflow
        while let Ok(n) = self.imu_uart.read(&mut buf, 0) {
            if n == 0 {
                break;
            }

            // Process each byte
            for &byte in buf.iter().take(n) {
                if let Some(packet_type) = self.imu_parser.feed_byte(byte, now_us) {
                    if packet_type == PacketType::Gyro {
                        self.last_gyro_us = now_us;
                    }
                    if packet_type == PacketType::Angle {
                        self.last_angle_us = now_us;
                    }
                    if packet_type == PacketType::Accel {
                        accel_count += 1;
                        self.last_imu_us = now_us;
                    }
                }
            }
        }

        // Neither GPS nor IMU gaps may reuse earlier parked fixes on recovery.
        // poll_imu runs even when no packets arrived, so stale gaps clear the count.
        let now_ms = (now_us / 1000) as u32;
        if !self.gps_is_fresh(now_ms)
            || !self
                .motion_evidence(now_ms)
                .is_some_and(|e| e.stationary_candidate(self.gps_parser.last_fix().speed))
        {
            self.stationary.invalidate();
        }

        // Return result with packet count if we processed any Accel packets
        crate::imu_interval::elapsed_seconds(previous_us, now_us, accel_count)
            .map(|dt| (dt, now_us, accel_count))
    }

    /// Poll GPS and return true if new fix available
    ///
    /// Reads all available bytes from UART buffer to prevent overflow.
    pub fn poll_gps(
        &mut self,
        server: Option<&TelemetryServerState>,
        diagnostics: &crate::diagnostics::DiagnosticsState,
    ) -> bool {
        let Self {
            gps_uart,
            gps_parser,
            gps_timing,
            stationary,
            ..
        } = self;
        let mut new_fix = false;
        let mut active = 0;
        let mut inactive = 0;
        let outcome = crate::gps_rx::drain(&mut crate::gps_setup::Receiver(gps_uart), |bytes| {
            for &byte in bytes {
                let before = gps_parser.rmc_timing().sequence;
                if crate::gps_ingest::feed_byte(gps_parser, byte, diagnostics) {
                    new_fix = true;
                    let timing = gps_parser.rmc_timing();
                    if timing.sequence != before {
                        if timing.status_active {
                            active += 1;
                        } else {
                            inactive += 1;
                        }
                        let host_us = unsafe { esp_idf_svc::sys::esp_timer_get_time() as u64 };
                        let fix = *gps_parser.last_fix();
                        gps_timing.push(fix, host_us);
                        if fix.valid {
                            stationary.record_gps_fix((host_us / 1000) as u32);
                        } else {
                            stationary.invalidate();
                        }
                        if let Some(server) = server {
                            server.record_gps_event(timing, gps_parser.last_fix(), host_us);
                        }
                    }
                }
            }
        });
        if outcome.loss {
            gps_parser.invalidate_stream();
            gps_timing.invalidate();
            stationary.invalidate();
        }
        let mut transport = outcome.counters;
        transport.active_rmc = active;
        transport.inactive_rmc = inactive;
        transport.checksum_rejections = gps_parser.checksum_rejections();
        diagnostics.record_gps_transport(transport);
        new_fix
    }

    pub fn replay_gps_timing(&mut self, timer: &mut sensor_fusion::LapTimer, now_us: u64) -> u8 {
        self.gps_timing.replay(timer, now_us)
    }

    /// Update once per new GPS fix; repeated telemetry polls are not new evidence.
    pub fn update_stationary(&mut self, now_ms: u32) -> bool {
        let stationary = self.gps_is_fresh(now_ms)
            && self.motion_evidence(now_ms).is_some_and(|evidence| {
                evidence.stationary_candidate(self.gps_parser.last_fix().speed)
            });
        self.stationary.observe_candidate(stationary);
        self.is_stationary(now_ms)
    }

    pub fn is_stationary(&self, now_ms: u32) -> bool {
        self.gps_is_fresh(now_ms)
            && self.stationary.confirmed()
            && self
                .motion_evidence(now_ms)
                .is_some_and(|e| e.stationary_candidate(self.gps_parser.last_fix().speed))
    }
}

/// Manages state estimation (sensor fusion)
pub struct StateEstimator {
    pub ekf: Ekf,
    pub mode_classifier: ModeClassifier,
    position_recovery: sensor_fusion::position_recovery::PositionRecovery,
}

impl StateEstimator {
    pub fn new() -> Self {
        Self {
            ekf: Ekf::new(),
            mode_classifier: ModeClassifier::new(),
            position_recovery: Default::default(),
        }
    }

    /// Update EKF prediction with IMU data
    pub fn predict(&mut self, ax_earth: f32, ay_earth: f32, wz: f32, dt: f32) {
        if dt <= 0.2 {
            self.ekf.predict(ax_earth, ay_earth, wz, dt);
        } else {
            self.ekf.coast(dt);
        }
    }

    /// Update EKF with GPS position
    pub fn invalidate_position_recovery(&mut self) {
        self.position_recovery.reset();
    }

    /// Update EKF with GPS position
    pub fn update_position(&mut self, x: f32, y: f32, vx: f32, vy: f32, at_us: u64) {
        if self.ekf.update_position(x, y) == sensor_fusion::ekf::UpdateResult::Accepted {
            self.position_recovery.reset();
        } else if self.position_recovery.observe([x, y], [vx, vy], at_us) {
            self.ekf.reacquire_position(x, y);
            self.ekf.update_position(x, y);
            self.position_recovery.reset();
        }
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
    ///
    /// # Arguments
    /// * `lap_timer_data` - Optional tuple of (lap_time_ms, lap_count,
    ///   lap_flags)
    pub fn publish_telemetry(
        &mut self,
        sensors: &SensorManager,
        estimator: &StateEstimator,
        sensor_fusion: &crate::fusion::SensorFusion,
        now_ms: u32,
        lap_timer_data: Option<(&sensor_fusion::LapTimer, u8, u32)>,
    ) -> Result<(), SystemError> {
        // Get accelerations for display
        // lon: GPS-derived when fresh (no vibration), otherwise blended with heavy
        // smoothing lat: centripetal (speed × yaw_rate) - instant, doesn't
        // stick after corners      Uses calibrated yaw rate to prevent drift on
        // highway
        let lon_display = sensor_fusion.get_lon_display();
        let lat_display = sensor_fusion.get_lat_centripetal();

        // Keep raw az for reference (no tilt correction needed for vertical)
        let (_, _, az_corr) = sensors.imu_parser.get_accel_corrected();

        let mut packet = binary_telemetry::TelemetryPacket::new();
        packet.timestamp_ms = now_ms;

        // Send corrected vehicle-frame accelerations
        // Dashboard expects: lng = -ax/9.81, latg = ay/9.81
        // So: ax = -lon (negated so dashboard shows positive for accel)
        //     ay = lat (positive = right turn on G-meter)
        packet.ax = -lon_display;
        packet.ay = -lat_display; // Negate: fusion uses left-positive, dashboard uses right-positive
        packet.az = az_corr;
        packet.wz = sensors.imu_parser.data().wz;
        packet.roll = sensors.imu_parser.data().roll.to_radians();
        packet.pitch = sensors.imu_parser.data().pitch.to_radians();
        // Compass basis (π/2 - mathematical EKF yaw) + learned GPS reference offset.
        // Clockwise from north after learning; before that only the angular basis
        // is converted, and the absolute reference remains unverified.
        packet.yaw = sensor_fusion.get_aligned_heading();

        let (ekf_x, ekf_y) = estimator.ekf.position();

        // Use fresh receiver velocity before an unvalidated estimator fallback
        // Returns consistent (vx, vy, speed) tuple from same source
        let (mut vx, mut vy, speed_ms) = sensors.get_velocity(Some(&estimator.ekf), now_ms);
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
        // EKF position uncertainty: uses effective sigma that accounts for
        // innovation statistics, GPS discrepancy, and minimum floor (~2-3m)
        // This prevents false confidence when EKF may have drifted
        packet.pos_sigma = estimator.ekf.get_effective_pos_sigma();
        packet.mode = estimator.mode_classifier.get_mode_u8();

        if sensors.gps_is_fresh(now_ms) {
            packet.lat = sensors.gps_parser.last_fix().lat as f32;
            packet.lon = sensors.gps_parser.last_fix().lon as f32;
            packet.gps_valid = 1;
            // GPS course is only valid when moving (speed > ~1 m/s)
            // Use NaN to signal invalid when stationary
            if sensors.gps_parser.last_fix().speed > 1.0 {
                packet.gps_course = sensors.gps_parser.last_fix().course;
            } else {
                packet.gps_course = f32::NAN;
            }
        } else {
            packet.gps_valid = 0;
            packet.gps_course = f32::NAN;
        }

        // Add lap timer data if available
        if let Some((timer, lap_flags, _)) = lap_timer_data {
            packet.lap_time_ms = timer.current_lap_ms();
            packet.lap_count = timer.lap_count();
            packet.lap_flags = lap_flags;
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
            let fix = sensors.gps_parser.last_fix();
            let status = crate::websocket_server::TimingTelemetry {
                revision: lap_timer_data.map_or(0, |(_, _, revision)| revision),
                state: lap_timer_data.map_or("idle", |(timer, _, _)| match timer.state() {
                    sensor_fusion::LapTimerState::Idle => "idle",
                    sensor_fusion::LapTimerState::Armed => "armed",
                    sensor_fusion::LapTimerState::Timing => "timing",
                }),
                last_lap_ms: lap_timer_data.and_then(|(timer, _, _)| timer.last_lap_ms()),
                best_lap_ms: lap_timer_data.and_then(|(timer, _, _)| timer.best_lap_ms()),
                lat: fix.lat,
                lon: fix.lon,
                gps_valid: sensors.gps_is_fresh(now_ms),
                gps_age_ms: sensors.gps_age_ms(now_ms),
                // Invalid IMU evidence hides forces without blocking healthy GPS/timing.
                imu_age_ms: sensors.motion_evidence(now_ms).map(|e| e.age_ms),
                gps_boot_id: None,
                gps_event: None,
            };
            http_state.update_telemetry(bytes, status);
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
    connectivity: (bool, bool),
}

impl StatusManager {
    pub fn new(led: RgbLed) -> Self {
        Self {
            led,
            gps_was_locked: false,
            last_gps_status_ms: 0,
            connectivity: (true, true),
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
        if !gps_locked && now_ms.wrapping_sub(self.last_gps_status_ms) >= 5000 {
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

    pub fn set_connectivity(&mut self, wifi: bool, mqtt: bool) {
        self.connectivity = (wifi, mqtt);
    }

    /// Update LED display without sleeping in the sensor loop.
    pub fn update_led(&mut self, gps_locked: bool, now_ms: u32) {
        if !self.connectivity.0 && now_ms % 1000 < 200 {
            self.led.orange().ok();
        } else if !self.connectivity.1 && now_ms % 1000 < 200 {
            self.led.red().ok();
        } else if gps_locked {
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
