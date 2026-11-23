mod binary_telemetry;
mod calibration;
mod config;
mod diagnostics;
#[cfg(test)]
mod filter; // Only compiled for tests - production uses GPS blend instead of Biquad
mod fusion;
mod gps;
mod gps_ingest;
mod gps_log;
mod gps_rx;
mod gps_setup;
mod gps_timing;
mod imu;
mod imu_bias_collection;
mod imu_evidence;
mod mode;
mod mqtt;
mod nvs_storage;
mod recording_metadata;
mod recording_metadata_device;
mod rgb_led;
mod system;
mod telemetry_http;
mod udp_stream;
mod websocket_server;
mod wifi;

use std::sync::Arc;

use config::{SystemConfig, WifiModeConfig};
use diagnostics::{DiagnosticsState, FusionDiagnostics};
use esp_idf_hal::{
    delay::FreeRtos,
    peripherals::Peripherals,
    uart::{config::Config, UartDriver},
};
use esp_idf_svc::{eventloop::EspSystemEventLoop, nvs::EspDefaultNvsPartition};
use fusion::{FusionConfig, SensorFusion};
use log::info;
use mqtt::MqttClient;
use rgb_led::RgbLed;
use sensor_fusion::transforms::{body_to_earth, remove_gravity}; // Used for EKF prediction
use sensor_fusion::{LapTimer, TimingLine};
use system::{SensorManager, StateEstimator, StatusManager, TelemetryPublisher};
use udp_stream::UdpTelemetryStream;
use websocket_server::{LapTimerConfig, TelemetryServer, TelemetryServerState};
use wifi::WifiManager;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let config = SystemConfig::from_env();

    let is_ap_mode = config.network.wifi_mode == WifiModeConfig::AccessPoint;

    info!("=== ESP32-C3 Telemetry System ===");
    info!(
        "Mode: {}, SSID: {}, Rate: {}Hz, GPS: {}",
        if is_ap_mode {
            "Access Point"
        } else {
            "Station"
        },
        config.network.wifi_ssid,
        1000 / config.telemetry.interval_ms,
        config.gps.model.name()
    );

    // Create diagnostics state
    let diagnostics = Arc::new(DiagnosticsState::new());
    diagnostics.init(
        if is_ap_mode { "AP" } else { "Station" },
        config.network.wifi_ssid,
        config.gps.model.name(),
        config.gps.update_rate_hz,
        config.gps.warmup_fixes,
        1000 / config.telemetry.interval_ms,
        200.0, // IMU expected Hz (WT901 configured)
        config.gps.update_rate_hz as f32,
    );

    // Initialize hardware
    let peripherals = Peripherals::take().unwrap();
    let sysloop = EspSystemEventLoop::take().unwrap();
    let nvs = EspDefaultNvsPartition::take().ok();
    let mut mount_storage = nvs
        .as_ref()
        .and_then(|p| nvs_storage::NvsStorage::new(p.clone()).ok());
    let saved_mount = mount_storage.as_ref().map(|s| s.load()).unwrap_or_default();
    let mut mount_calibrator =
        calibration::MountCalibrator::new(saved_mount, mount_storage.is_some());

    // Create LED for status indication
    let led = RgbLed::new(peripherals.rmt.channel0, peripherals.pins.gpio8)
        .expect("Failed to initialize RGB LED");
    let mut status_mgr = StatusManager::new(led);

    // Boot sequence - purple for AP mode, blue for station mode
    status_mgr.led_mut().set_low().unwrap();
    FreeRtos::delay_ms(1000);

    for i in 0..3 {
        if is_ap_mode {
            status_mgr.led_mut().magenta().unwrap();
        } else {
            status_mgr.led_mut().blue().unwrap();
        }
        FreeRtos::delay_ms(300);
        status_mgr.led_mut().set_low().unwrap();
        FreeRtos::delay_ms(300);
        info!("Boot blink {}/3", i + 1);
    }
    FreeRtos::delay_ms(1000);

    // Initialize WiFi
    info!("Initializing WiFi");
    let mut wifi =
        WifiManager::new(peripherals.modem, sysloop.clone(), nvs).expect("WiFi init failed");

    // Telemetry server state (for AP mode)
    let mut telemetry_server: Option<TelemetryServer> = None;
    let mut telemetry_state: Option<Arc<TelemetryServerState>> = None;

    // MQTT client (for Station mode)
    let mut mqtt_opt: Option<MqttClient> = None;

    // UDP stream (for Station mode)
    let mut udp_stream: Option<UdpTelemetryStream> = None;

    if is_ap_mode {
        // === ACCESS POINT MODE ===
        info!("Starting WiFi Access Point: {}", config.network.wifi_ssid);
        match wifi.start_ap(config.network.wifi_ssid, config.network.wifi_password) {
            Ok(_) => {
                info!("WiFi AP started");
                // Green blinks for AP ready
                for _ in 0..5 {
                    status_mgr.led_mut().green().unwrap();
                    FreeRtos::delay_ms(200);
                    status_mgr.led_mut().set_low().unwrap();
                    FreeRtos::delay_ms(200);
                }

                // Start HTTP telemetry server
                info!(
                    "Starting telemetry server on port {}",
                    config.network.ws_port
                );
                match TelemetryServer::new(config.network.ws_port, Some(diagnostics.clone())) {
                    Ok(server) => {
                        telemetry_state = Some(server.state());
                        telemetry_server = Some(server);
                        info!("Telemetry server ready!");
                        // Cyan blinks for server ready
                        for _ in 0..3 {
                            status_mgr.led_mut().cyan().unwrap();
                            FreeRtos::delay_ms(150);
                            status_mgr.led_mut().set_low().unwrap();
                            FreeRtos::delay_ms(150);
                        }
                    }
                    Err(e) => {
                        info!("Telemetry server failed: {:?}", e);
                    }
                }
            }
            Err(e) => {
                info!("WiFi AP failed: {:?}", e);
                loop {
                    status_mgr.led_mut().red().unwrap();
                    FreeRtos::delay_ms(500);
                    status_mgr.led_mut().set_low().unwrap();
                    FreeRtos::delay_ms(500);
                }
            }
        }
    } else {
        // === STATION MODE ===
        info!("Connecting to WiFi: {}", config.network.wifi_ssid);
        match wifi.connect(config.network.wifi_ssid, config.network.wifi_password) {
            Ok(_) => {
                info!("WiFi connected");
                for _ in 0..5 {
                    status_mgr.led_mut().green().unwrap();
                    FreeRtos::delay_ms(200);
                    status_mgr.led_mut().set_low().unwrap();
                    FreeRtos::delay_ms(200);
                }
            }
            Err(e) => {
                info!("WiFi failed: {:?}", e);
                loop {
                    status_mgr.led_mut().red().unwrap();
                    FreeRtos::delay_ms(500);
                    status_mgr.led_mut().set_low().unwrap();
                    FreeRtos::delay_ms(500);
                }
            }
        }

        // Connect MQTT for status messages
        info!("Initializing MQTT");
        mqtt_opt = match MqttClient::new(config.network.mqtt_broker) {
            Ok(m) => {
                info!("MQTT connected");
                for _ in 0..3 {
                    status_mgr.led_mut().magenta().unwrap();
                    FreeRtos::delay_ms(200);
                    status_mgr.led_mut().set_low().unwrap();
                    FreeRtos::delay_ms(200);
                }
                Some(m)
            }
            Err(e) => {
                info!("MQTT failed: {:?}", e);
                for _ in 0..5 {
                    status_mgr.led_mut().red().unwrap();
                    FreeRtos::delay_ms(100);
                    status_mgr.led_mut().set_low().unwrap();
                    FreeRtos::delay_ms(100);
                }
                None
            }
        };

        // Initialize UDP for telemetry
        info!(
            "Initializing UDP telemetry to {}",
            config.network.udp_server
        );
        let mut stream = UdpTelemetryStream::new(config.network.udp_server);
        match stream.init() {
            Ok(_) => {
                info!("UDP ready!");
                for _ in 0..3 {
                    status_mgr.led_mut().cyan().unwrap();
                    FreeRtos::delay_ms(150);
                    status_mgr.led_mut().set_low().unwrap();
                    FreeRtos::delay_ms(150);
                }
                udp_stream = Some(stream);
            }
            Err(e) => {
                info!("UDP init failed: {:?}", e);
            }
        }
    }

    // Keep server alive (prevent drop)
    let _server = telemetry_server;

    let mut recording_metadata = telemetry_state
        .as_ref()
        .and_then(|state| state.boot_id())
        .and_then(|boot_id| recording_metadata_device::initial(boot_id, &config.gps));
    if let Some(ref mut snapshot) = recording_metadata {
        // This is the saved startup state; pending mounting candidates never enter it.
        if mount_storage.is_some() {
            snapshot.calibration.mount_alignment = recording_metadata::MountAlignment::finite(
                saved_mount.is_calibrated,
                saved_mount.yaw_offset,
            );
        }
        if let Some(ref state) = telemetry_state {
            state.publish_recording_metadata(snapshot.clone());
        }
    }

    // Initialize UART for IMU with auto-detection of baud rate
    // WT901 could be at 9600 (factory) or 115200 (configured for 200Hz)
    // LED feedback: orange blink = trying baud, green = found, red = not found
    info!("Detecting IMU baud rate...");

    // Start with 115200 (preferred for 200Hz operation)
    let imu_uart = UartDriver::new(
        peripherals.uart1,
        peripherals.pins.gpio18,
        peripherals.pins.gpio19,
        Option::<esp_idf_hal::gpio::Gpio0>::None,
        Option::<esp_idf_hal::gpio::Gpio1>::None,
        &Config::new().baudrate(115200.into()),
    )
    .unwrap();

    // Try to detect IMU at different baud rates
    let baud_rates: [u32; 4] = [115200, 9600, 38400, 19200];
    let mut detected_baud: u32 = 0;
    let mut configured_imu_baud = Some(115200);

    for (idx, &baud) in baud_rates.iter().enumerate() {
        if baud != 115200 {
            configured_imu_baud = imu_uart
                .change_baudrate(esp_idf_hal::units::Hertz(baud))
                .map(|_| baud)
                .ok();
        }

        // Orange blink to show which baud we're trying (1-4 blinks)
        for _ in 0..=idx {
            status_mgr.led_mut().orange().ok();
            FreeRtos::delay_ms(100);
            status_mgr.led_mut().set_low().ok();
            FreeRtos::delay_ms(100);
        }

        info!("  Trying {} baud...", baud);

        // Read for ~300ms and look for WT901 header bytes (0x55)
        let mut buf = [0u8; 100];
        let mut header_count = 0;

        // Do 30 read attempts with 10ms timeout each = ~300ms total
        for _ in 0..30 {
            if let Ok(n) = imu_uart.read(&mut buf, 10) {
                for &byte in buf.iter().take(n) {
                    if byte == 0x55 {
                        header_count += 1;
                    }
                }
            }
        }

        if header_count >= 5 {
            let estimated_hz = header_count as f32 / 0.3 / 3.0; // 3 packets per cycle
            info!(
                "  FOUND IMU at {} baud (~{:.0} Hz, {} headers)",
                baud, estimated_hz, header_count
            );
            detected_baud = baud;

            // Green blinks to confirm detection
            for _ in 0..3 {
                status_mgr.led_mut().green().ok();
                FreeRtos::delay_ms(150);
                status_mgr.led_mut().set_low().ok();
                FreeRtos::delay_ms(150);
            }
            break;
        } else {
            info!("  No data at {} baud ({} headers)", baud, header_count);
        }
    }

    if detected_baud == 0 {
        info!("WARNING: Could not detect IMU! Check wiring.");
        info!("Defaulting to 9600 baud...");

        // Red blinks to show IMU not found
        for _ in 0..5 {
            status_mgr.led_mut().red().ok();
            FreeRtos::delay_ms(200);
            status_mgr.led_mut().set_low().ok();
            FreeRtos::delay_ms(200);
        }

        configured_imu_baud = imu_uart
            .change_baudrate(esp_idf_hal::units::Hertz(9600))
            .map(|_| 9600)
            .ok();
    }

    // Clear IMU UART buffer after detection (limited reads to avoid infinite loop)
    // IMU continuously sends data, so we just do a few reads to clear backlog
    {
        let mut flush_buf = [0u8; 256];
        for _ in 0..5 {
            let _ = imu_uart.read(&mut flush_buf, 0); // Non-blocking
        }
    }
    FreeRtos::delay_ms(10);

    // Larger RX buffering absorbs scheduler jitter; the runtime drains the event
    // queue and reports overruns/errors instead of silently treating all as empty.
    let target_baud = config.gps.effective_baud();
    let gps_uart = UartDriver::new(
        peripherals.uart0,
        peripherals.pins.gpio5,
        peripherals.pins.gpio4,
        Option::<esp_idf_hal::gpio::Gpio2>::None,
        Option::<esp_idf_hal::gpio::Gpio3>::None,
        &Config::new()
            .baudrate(target_baud.into())
            .rx_fifo_size(4096)
            .queue_size(64),
    )
    .expect("Failed to create GPS UART");
    let gps_setup = if config.gps.model.needs_ubx_init() {
        gps_setup::configure(&gps_uart, target_baud, config.gps.update_rate_hz)
    } else {
        ublox_gps::setup::Report {
            detected_baud: None,
            uart_baud: target_baud,
            verified: false,
            error: None,
        }
    };
    info!(
        "GPS startup: detected={:?}, UART={}, verified={}, error={:?}",
        gps_setup.detected_baud, gps_setup.uart_baud, gps_setup.verified, gps_setup.error
    );
    diagnostics.set_gps_transport(gps_setup);
    if let Some(ref mut snapshot) = recording_metadata {
        snapshot.gps.set_setup(gps_setup);
    }

    if let Some(ref mut snapshot) = recording_metadata {
        snapshot.imu = if detected_baud == 0 {
            recording_metadata::Imu::selected(
                configured_imu_baud,
                recording_metadata::BaudSelection::Fallback,
            )
        } else {
            recording_metadata::Imu::selected(
                configured_imu_baud,
                recording_metadata::BaudSelection::HeaderProbe,
            )
        };
        snapshot.captured_uptime_ms = recording_metadata_device::uptime_ms();
        if let Some(ref state) = telemetry_state {
            state.publish_recording_metadata(snapshot.clone());
        }
    }

    // Create sensor manager
    let mut sensors = SensorManager::new(imu_uart, gps_uart, config.gps.warmup_fixes);

    // Calibrate IMU
    if let Some(ref mut mqtt) = mqtt_opt {
        mqtt.publish_status("calib_start", Some(0.0)).ok();
    }

    let bias = sensors
        .calibrate_imu(status_mgr.led_mut(), mqtt_opt.as_mut())
        .expect("Calibration failed");
    sensors.imu_parser.set_bias(bias);
    if let Some(ref mut snapshot) = recording_metadata {
        snapshot.calibration.accelerometer_bias_mps2 =
            recording_metadata::Bias::finite(bias.ax, bias.ay, bias.az);
        snapshot.captured_uptime_ms = recording_metadata_device::uptime_ms();
        if let Some(ref state) = telemetry_state {
            state.publish_recording_metadata(snapshot.clone());
        }
    }

    if let Some(ref mut mqtt) = mqtt_opt {
        mqtt.publish_status("calib_done", Some(1.0)).ok();
        mqtt.publish_status("waiting_gps", None).ok();
    }

    // Create state estimator and telemetry publisher
    let mut estimator = StateEstimator::new();
    let mut publisher = TelemetryPublisher::new(udp_stream, mqtt_opt, telemetry_state.clone());

    // Create lap timer (configured via HTTP API)
    let mut lap_timer = LapTimer::new();
    let mut pending_lap_flags = 0u8;
    let mut lap_revision = 0u32;

    // Create sensor fusion for mode detection at the telemetry cadence (~30 Hz).
    // OrientationCorrector estimates pitch/roll errors from GPS velocity changes
    // before gravity removal. Confidence controls the longitudinal IMU/GPS blend;
    // neither source is ground truth. IMU acquisition can run faster than this path.
    // Lateral acceleration uses speed * corrected yaw rate, an approximation for
    // coordinated turns; banking, sideslip and mounting errors still matter.
    let fusion_config = FusionConfig {
        gps_high_rate: 20.0,
        gps_medium_rate: 10.0,
        // GPS staleness threshold - must be > GPS interval + processing delay
        // At 5Hz GPS (200ms) with ~20Hz telemetry, 400ms gives safe margin
        gps_max_age: 0.4,
        // GPS weights based on orientation correction confidence:
        gps_weight_high: 0.3,   // 30% GPS when correction is very confident
        gps_weight_medium: 0.6, // 60% GPS when moderately confident
        gps_weight_low: 1.0,    // 100% GPS when not confident (no correction yet)
        tilt_learn_time: 3.0,
        // Butterworth filter to remove engine vibration from IMU
        // Sample rate must match ACTUAL process_imu() call rate (telemetry rate)
        lon_filter_cutoff: 10.0,
        lon_sample_rate: 30.0, // 33ms telemetry interval = ~30Hz (from TelemetryConfig::default)
    };
    let mut sensor_fusion = SensorFusion::new(fusion_config);
    if saved_mount.is_calibrated {
        sensor_fusion.set_mount_yaw(saved_mount.yaw_offset);
    }

    // Main loop timing
    let mut last_telemetry_ms = 0u32;
    let mut last_heap_check_ms = 0u32;
    let mut last_mqtt_diag_ms = 0u32;
    let mut last_connectivity_check_ms = 0u32;
    let mut loop_count = 0u32;
    let mut last_metadata_ms = 0u64;

    // Track previous connectivity state to detect changes
    let mut was_wifi_connected = true;
    let mut was_mqtt_connected = true;

    info!("=== Entering main loop ===");

    loop {
        let now_ms = unsafe { (esp_idf_svc::sys::esp_timer_get_time() / 1000) as u32 };
        loop_count += 1;
        diagnostics.record_loop();

        mount_calibrator.tick(now_ms);
        mount_calibrator.status.parked = sensors.gps_is_fresh(now_ms)
            && sensors.gps_parser.last_fix().speed < 0.5
            && sensors.is_stationary(now_ms)
            && sensors.calibration_imu_age_ms(now_ms) <= 60;
        if let Some(ref state) = telemetry_state {
            match state.take_mount_command() {
                1 => mount_calibrator.command("start", now_ms),
                2 => mount_calibrator.command("cancel", now_ms),
                action @ (3 | 4) => {
                    let changed = mount_calibrator.save_with(action == 4, |bytes| {
                        mount_storage.as_mut().ok_or(())?.save(bytes)
                    });
                    if changed {
                        sensor_fusion.set_mount_yaw(mount_calibrator.status.saved.yaw_offset);
                        if let Some(ref mut snapshot) = recording_metadata {
                            let saved = mount_calibrator.status.saved;
                            snapshot.calibration.mount_alignment =
                                recording_metadata::MountAlignment::finite(
                                    saved.is_calibrated,
                                    saved.yaw_offset,
                                );
                        }
                    }
                }
                _ => {}
            }
        }

        // Check for calibration request from web UI
        if let Some(ref state) = telemetry_state {
            if state.take_calibration_request() {
                mount_calibrator.command("cancel", now_ms);
                info!(">>> Calibration requested via web UI - recalibrating IMU...");
                match sensors.calibrate_imu(status_mgr.led_mut(), None) {
                    Ok(bias) => {
                        sensors.imu_parser.set_bias(bias);
                        if let Some(ref mut snapshot) = recording_metadata {
                            snapshot.calibration.accelerometer_bias_mps2 =
                                recording_metadata::Bias::finite(bias.ax, bias.ay, bias.az);
                        }
                        info!(
                            ">>> Recalibration complete! Bias: ax={:.4}, ay={:.4}, az={:.4}",
                            bias.ax, bias.ay, bias.az
                        );
                    }
                    Err(e) => {
                        info!(">>> Recalibration FAILED: {:?}", e);
                    }
                }
            }

            // Check for settings changes from web UI
            if let Some(s) = state.take_settings_change() {
                let new_config = mode::ModeConfig {
                    min_speed: s.min_speed,
                    acc_thr: s.acc_thr,
                    acc_exit: s.acc_exit,
                    brake_thr: s.brake_thr,
                    brake_exit: s.brake_exit,
                    lat_thr: s.lat_thr,
                    lat_exit: s.lat_exit,
                    yaw_thr: s.yaw_thr,
                    alpha: mode::DEFAULT_MODE_ALPHA, // Single source of truth
                };
                estimator.mode_classifier.update_config(new_config);
                info!(
                    ">>> Mode config updated: acc={:.2}g/{:.2}g, brake={:.2}g/{:.2}g, lat={:.2}g/{:.2}g, yaw={:.3}rad/s, min_spd={:.1}m/s",
                    s.acc_thr, s.acc_exit, s.brake_thr, s.brake_exit, s.lat_thr, s.lat_exit, s.yaw_thr, s.min_speed
                );

                // The regular LED task provides feedback without blocking sensor reads.
            }

            // Check for lap timer config changes from web UI
            if let Some((revision, config)) = state.take_lap_timer_config() {
                lap_revision = revision;
                pending_lap_flags = 0;
                match config {
                    LapTimerConfig::None => {
                        lap_timer.clear();
                        info!(">>> Lap timer cleared");
                    }
                    LapTimerConfig::Loop(line) => {
                        lap_timer.configure_loop(TimingLine::new(
                            line.p1_lat,
                            line.p1_lon,
                            line.p2_lat,
                            line.p2_lon,
                            line.direction,
                        ));
                        info!(
                            ">>> Lap timer configured: loop track, line ({:.6},{:.6})-({:.6},{:.6})",
                            line.p1_lat, line.p1_lon, line.p2_lat, line.p2_lon
                        );
                    }
                    LapTimerConfig::PointToPoint { start, finish } => {
                        lap_timer.configure_point_to_point(
                            TimingLine::new(
                                start.p1_lat,
                                start.p1_lon,
                                start.p2_lat,
                                start.p2_lon,
                                start.direction,
                            ),
                            TimingLine::new(
                                finish.p1_lat,
                                finish.p1_lon,
                                finish.p2_lat,
                                finish.p2_lon,
                                finish.direction,
                            ),
                        );
                        info!(
                            ">>> Lap timer configured: point-to-point, start ({:.6},{:.6})-({:.6},{:.6}), finish ({:.6},{:.6})-({:.6},{:.6})",
                            start.p1_lat, start.p1_lon, start.p2_lat, start.p2_lon,
                            finish.p1_lat, finish.p1_lon, finish.p2_lat, finish.p2_lon
                        );
                    }
                }
            }
        }

        // Bounded refresh also publishes applied changes; use the full monotonic clock.
        let metadata_now_ms = recording_metadata_device::uptime_ms();
        if metadata_now_ms.saturating_sub(last_metadata_ms) >= 1000 {
            if let (Some(ref state), Some(ref mut snapshot)) =
                (&telemetry_state, &mut recording_metadata)
            {
                snapshot.captured_uptime_ms = metadata_now_ms;
                state.publish_recording_metadata(snapshot.clone());
            }
            last_metadata_ms = metadata_now_ms;
        }

        // Periodic diagnostics (serial log every 5s)
        if now_ms.wrapping_sub(last_heap_check_ms) >= 5000 {
            let free_heap = unsafe { esp_idf_svc::sys::esp_get_free_heap_size() };
            let (telem_count, fail_count) = publisher.get_stats();
            let telem_rate = telem_count / 5;
            let loop_rate = loop_count / 5;
            info!(
                "Heap: {}B | UDP: {}Hz | Loop: {}Hz | Fails: {}",
                free_heap, telem_rate, loop_rate, fail_count
            );

            // Update diagnostics state
            let uptime_s = now_ms / 1000;
            diagnostics.update_rates(now_ms);
            diagnostics.update_system(free_heap, uptime_s, telem_count, fail_count);

            // EKF health: extract sigma from covariance diagonal
            let pos_sigma = (estimator.ekf.p[0] + estimator.ekf.p[1]).sqrt();
            let vel_sigma = (estimator.ekf.p[3] + estimator.ekf.p[4]).sqrt();
            let yaw_sigma_deg = estimator.ekf.p[2].sqrt().to_degrees();
            diagnostics.update_ekf(
                pos_sigma,
                vel_sigma,
                yaw_sigma_deg,
                estimator.ekf.x[5], // bias_x
                estimator.ekf.x[6], // bias_y
            );

            // GPS health (includes GPS reference point for lap timer coordinate alignment)
            let gps_fix = sensors.gps_parser.last_fix();
            let gps_ref = sensors.gps_parser.reference();
            diagnostics.update_gps(
                gps_fix.valid,
                sensors.gps_parser.is_warmed_up(),
                gps_fix.satellites,
                gps_fix.hdop,
                gps_fix.pdop,
                gps_ref.lat,
                gps_ref.lon,
            );

            // Note: Fusion diagnostics now updated at telemetry rate (see telemetry block)
            // for synchronized CSV export data

            publisher.reset_stats();
            loop_count = 0;
            last_heap_check_ms = now_ms;
        }

        // Connectivity check every 5 seconds (Station mode only)
        // In AP mode, we ARE the access point so connectivity checks don't apply
        if !is_ap_mode && now_ms.wrapping_sub(last_connectivity_check_ms) >= 5000 {
            let wifi_connected = wifi.is_connected();
            let mqtt_connected = publisher
                .mqtt_client_mut()
                .map(|m| m.is_connected())
                .unwrap_or(false);

            // Connectivity feedback must never pause GPS/IMU processing.
            status_mgr.set_connectivity(wifi_connected, mqtt_connected);

            // Log connectivity status changes
            if was_wifi_connected != wifi_connected || was_mqtt_connected != mqtt_connected {
                info!(
                    "Connectivity changed: WiFi={}, MQTT={}",
                    wifi_connected, mqtt_connected
                );
            }

            was_wifi_connected = wifi_connected;
            was_mqtt_connected = mqtt_connected;
            last_connectivity_check_ms = now_ms;
        }

        // MQTT diagnostics (every 30s - low priority, LED handles connectivity
        // feedback)
        if now_ms.wrapping_sub(last_mqtt_diag_ms) >= 30000 {
            if let Some(mqtt) = publisher.mqtt_client_mut() {
                let (ekf_x, ekf_y) = estimator.ekf.position();
                let gps_warmed = sensors.gps_parser.is_warmed_up();
                let gps_valid = sensors.gps_parser.last_fix().valid;
                let ref_pt = sensors.gps_parser.reference();

                let diag = if gps_warmed {
                    format!(
                        "{{\"gps\":\"ok\",\"ref\":[{:.4},{:.4}],\"pos\":[{:.1},{:.1}],\"valid\":{}}}",
                        ref_pt.lat, ref_pt.lon, ekf_x, ekf_y, gps_valid
                    )
                } else {
                    let progress = sensors.gps_parser.warmup_progress() * 100.0;
                    format!("{{\"gps\":\"warmup\",\"prog\":{:.0}}}", progress)
                };

                mqtt.publish("car/diag", &diag, false).ok();
            }
            last_mqtt_diag_ms = now_ms;
        }

        // Reset EKF position once when GPS warmup completes (check every loop, not just
        // on GPS poll)
        static mut EKF_RESET_DONE: bool = false;
        if sensors.gps_parser.is_warmed_up() && unsafe { !EKF_RESET_DONE } {
            // Reset position to origin now that we have a valid reference
            estimator.ekf.x[0] = 0.0;
            estimator.ekf.x[1] = 0.0;
            estimator.ekf.x[3] = 0.0; // vx
            estimator.ekf.x[4] = 0.0; // vy
            let mut variance = estimator.ekf.p;
            variance[0] = 100.0;
            variance[1] = 100.0;
            estimator.ekf.reset_covariance(variance);
            unsafe { EKF_RESET_DONE = true };
        }

        // Poll IMU and update EKF prediction (only after GPS warmup AND reset)
        if let Some((dt, imu_at_us, accel_count)) = sensors.poll_imu() {
            // Record all processed Accel packets for accurate rate measurement
            for _ in 0..accel_count {
                diagnostics.record_imu_packet();
            }
            // Only run EKF prediction after reset is done
            let imu_now_ms = (imu_at_us / 1000) as u32;
            if let Some(evidence) = sensors
                .motion_evidence(imu_now_ms)
                .filter(|_| unsafe { EKF_RESET_DONE })
            {
                // Near Euler pitch singularity, skip prediction rather than inventing a rate.
                if let Some(heading_rate) = evidence.rates.heading {
                    let [ax_b, ay_b, az_b] = evidence.linear_body;
                    let (ax_e, ay_e) = body_to_earth(
                        ax_b,
                        ay_b,
                        az_b,
                        sensors.imu_parser.data().roll,
                        sensors.imu_parser.data().pitch,
                        estimator.ekf.yaw(),
                    );
                    estimator.predict(ax_e, ay_e, heading_rate, dt);

                    // Update yaw from magnetometer
                    let yaw_mag = sensors.imu_parser.data().yaw.to_radians();
                    estimator.update_yaw(yaw_mag);
                }
            }
        }

        // Poll GPS and update EKF correction
        let gps_updated = sensors.poll_gps(telemetry_state.as_deref(), &diagnostics);
        // UART parsing can advance the clock beyond the loop-start timestamp.
        let gps_now_us = unsafe { esp_idf_svc::sys::esp_timer_get_time() as u64 };
        let now_ms = (gps_now_us / 1000) as u32;
        pending_lap_flags |= sensors.replay_gps_timing(&mut lap_timer, gps_now_us);
        if gps_updated {
            // Rates count each valid RMC at ingest; estimation remains latest-only.
            let is_new_fix = gps_ingest::take_latest_fix(&mut sensors.gps_parser);

            if is_new_fix
                && sensors.gps_parser.is_warmed_up()
                && sensors.gps_parser.last_fix().valid
            {
                let fix = sensors.gps_parser.last_fix();
                let imu = sensors.imu_parser.data();
                let (ax, ay, az) = sensors.imu_parser.get_accel_corrected();
                let (x, y, z) = remove_gravity(ax, ay, az, imu.roll, imu.pitch);
                let (x, y) = body_to_earth(x, y, z, imu.roll, imu.pitch, 0.0);
                if let Some(evidence) = sensors.motion_evidence(now_ms) {
                    if let Some(sample) = calibration::Sample::from_fix(
                        fix,
                        now_ms,
                        (x, y),
                        evidence.rates.earth_vertical,
                        sensors.calibration_imu_age_ms(now_ms),
                        (imu.roll, imu.pitch),
                    ) {
                        mount_calibrator.sample(sample);
                    }
                } else {
                    mount_calibrator.invalidate();
                }
            } else if !sensors.gps_parser.last_fix().valid {
                mount_calibrator.invalidate();
            }

            // ALWAYS feed GPS speed to sensor fusion when we have a new fix
            // GPS acceleration only needs scalar speed - doesn't require full position
            // validity This ensures OrientationCorrector can learn even during
            // brief GPS hiccups
            if sensors.gps_parser.is_warmed_up() && is_new_fix {
                let gps_speed = sensors.gps_parser.last_fix().speed;
                let time_s = now_ms as f32 / 1000.0;
                sensor_fusion.process_gps(gps_speed, time_s);

                // Feed GPS heading to yaw rate calibrator (only valid at speed)
                // Course over ground is unreliable below ~5 m/s
                if gps_speed > 5.0 {
                    let course = sensors.gps_parser.last_fix().course;
                    sensor_fusion.process_gps_heading(course);
                }
            }

            // Check if warmup complete (has reference point) AND fix is valid for EKF
            // updates
            if is_new_fix
                && sensors.gps_parser.is_warmed_up()
                && sensors.gps_parser.last_fix().valid
            {
                let (ax_corr, ay_corr, az_corr) = sensors.imu_parser.get_accel_corrected();

                // Always update position from GPS first (even when stationary)
                // This prevents EKF from diverging during extended stops
                if let Some((x, y)) = sensors.gps_parser.to_local_coords() {
                    let (ekf_x, ekf_y) = estimator.ekf.position();
                    let jump_dist = ((x - ekf_x).powi(2) + (y - ekf_y).powi(2)).sqrt();
                    if jump_dist < 1000.0 {
                        estimator.update_position(x, y);
                    }
                }

                // Check stationary FIRST - if stopped, lock position to prevent GPS noise
                let stationary = sensors.update_stationary(now_ms);
                if stationary {
                    // Vehicle stopped - perform ZUPT and bias estimation
                    estimator.zupt();
                    estimator.ekf.lock_position(); // Reduce position covariance
                                                   // Issue 4: Lock yaw to prevent magnetometer noise from drifting heading
                                                   // Magnetometer updates still occur but with very low gain (~1%)
                    estimator.ekf.lock_yaw();
                    diagnostics.record_zupt();

                    let (ax_b, ay_b, az_b) = remove_gravity(
                        ax_corr,
                        ay_corr,
                        az_corr,
                        sensors.imu_parser.data().roll,
                        sensors.imu_parser.data().pitch,
                    );
                    let (ax_e, ay_e) = body_to_earth(
                        ax_b,
                        ay_b,
                        az_b,
                        sensors.imu_parser.data().roll,
                        sensors.imu_parser.data().pitch,
                        estimator.ekf.yaw(),
                    );
                    estimator.update_bias(ax_e, ay_e);
                    estimator.reset_speed();
                } else {
                    // Issue 4: Store current aligned heading while moving
                    // This preserves heading for when we stop
                    estimator
                        .ekf
                        .store_moving_yaw(sensor_fusion.get_aligned_heading());
                }

                // Update EKF with velocity components (requires full validity)
                if let Some((vx, vy)) = sensors
                    .gps_parser
                    .get_velocity_enu()
                    .filter(|_| !stationary)
                {
                    let speed = (vx * vx + vy * vy).sqrt();
                    // Sanity check: reject extreme velocities (> 100 m/s = 360 km/h)
                    // This catches NaN and corrupt GPS data
                    if speed < 100.0 && speed.is_finite() {
                        estimator.update_velocity(vx, vy);
                    }
                }
            }
        }

        // Update GPS rate estimate periodically (every second)
        if now_ms % 1000 < 50 {
            let fix_count = diagnostics.gps_fix_count();
            sensor_fusion.update_gps_rate(fix_count, now_ms as f32 / 1000.0);
        }

        // Update GPS status
        let gps_locked = sensors.gps_is_fresh(now_ms);
        status_mgr.update_gps_status(
            gps_locked,
            sensors.gps_parser.is_warmed_up(),
            now_ms,
            publisher.mqtt_client_mut(),
        );

        // Publish telemetry at configured rate
        if now_ms.wrapping_sub(last_telemetry_ms) >= config.telemetry.interval_ms {
            if let Some(ref state) = telemetry_state {
                state.publish_mount_status(&mount_calibrator.status);
            }
            // Get current speed and check if stationary
            let speed = sensors.get_speed(Some(&estimator.ekf), now_ms);
            let (ax_corr, ay_corr, az_corr) = sensors.imu_parser.get_accel_corrected();
            let is_stationary = sensors.is_stationary(now_ms);

            // Process through sensor fusion (GPS-corrected orientation + blending)
            // Pass raw body-frame IMU data - fusion handles transforms with corrected
            // orientation
            let dt = if last_telemetry_ms == 0 {
                config.telemetry.interval_ms
            } else {
                now_ms.wrapping_sub(last_telemetry_ms)
            } as f32
                / 1000.0;
            if let Some(evidence) = sensors.motion_evidence(now_ms) {
                let (lon_blended, lat_filtered) = sensor_fusion.process_imu(
                    ax_corr,                         // Raw body-frame X accel (bias-corrected)
                    ay_corr,                         // Raw body-frame Y accel (bias-corrected)
                    az_corr,                         // Raw body-frame Z accel (gravity)
                    sensors.imu_parser.data().roll,  // AHRS roll (degrees)
                    sensors.imu_parser.data().pitch, // AHRS pitch (degrees)
                    estimator.ekf.yaw(),             // EKF yaw (radians)
                    speed,
                    evidence.rates.earth_vertical,
                    dt,
                    is_stationary,
                );

                // Update mode classifier with hybrid (blended) acceleration
                estimator.mode_classifier.update_hybrid(
                    lon_blended,
                    lat_filtered,
                    evidence.rates.earth_vertical,
                    speed,
                );
            } else {
                // Preserve finite v4 values; HTTP motion age marks forces unavailable.
                sensor_fusion.pause_imu(dt);
            }

            if !sensors.gps_is_fresh(now_ms) {
                lap_timer.invalidate_fix();
            }
            lap_timer.tick(now_ms);
            publisher
                .publish_telemetry(
                    &sensors,
                    &estimator,
                    &sensor_fusion,
                    now_ms,
                    Some((&lap_timer, pending_lap_flags, lap_revision)),
                )
                .ok();
            pending_lap_flags = 0;

            // Update fusion diagnostics at telemetry rate for synchronized CSV export
            // This ensures fusion data (lon_imu, lon_gps, etc.) matches the telemetry
            // packet
            let (tilt_x, tilt_y, tilt_valid) = sensor_fusion.get_tilt_offsets();
            let (pitch_corr, roll_corr) = sensor_fusion.get_orientation_correction();
            let (pitch_conf, roll_conf) = sensor_fusion.get_orientation_confidence();
            diagnostics.update_fusion(FusionDiagnostics {
                lon_imu_raw: sensor_fusion.get_lon_imu_raw(),
                lon_imu_filtered: sensor_fusion.get_lon_imu_filtered(),
                lon_blended: sensor_fusion.get_lon_blended(),
                gps_weight: sensor_fusion.get_last_gps_weight(),
                gps_accel: sensor_fusion.get_gps_accel(),
                gps_rate: sensor_fusion.get_gps_rate(),
                gps_rejected: sensor_fusion.is_gps_rejected(),
                pitch_correction_deg: pitch_corr,
                roll_correction_deg: roll_corr,
                pitch_confidence: pitch_conf,
                roll_confidence: roll_conf,
                yaw_bias: sensor_fusion.yaw_rate_calibrator.get_bias(),
                yaw_calibrated: sensor_fusion.yaw_rate_calibrator.is_valid(),
                tilt_offset_x: tilt_x,
                tilt_offset_y: tilt_y,
                tilt_valid,
            });

            last_telemetry_ms = now_ms;
        }

        // Update LED status
        status_mgr.update_led(gps_locked, now_ms);
    }
}
