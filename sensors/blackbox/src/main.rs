mod binary_telemetry;
mod config;
mod diagnostics;
mod filter;
mod fusion;
mod gps;
mod imu;
mod mode;
mod mqtt;
mod rgb_led;
mod system;
mod udp_stream;
mod websocket_server;
mod wifi;

use std::sync::Arc;

use config::{SystemConfig, WifiModeConfig};
use diagnostics::DiagnosticsState;
use fusion::{FusionConfig, SensorFusion};
use esp_idf_hal::{
    delay::FreeRtos,
    peripherals::Peripherals,
    uart::{config::Config, UartDriver},
};
use esp_idf_svc::{eventloop::EspSystemEventLoop, nvs::EspDefaultNvsPartition};
use log::info;
use mqtt::MqttClient;
use rgb_led::RgbLed;
use sensor_fusion::transforms::{body_to_earth, remove_gravity};
use system::{SensorManager, StateEstimator, StatusManager, TelemetryPublisher};
use udp_stream::UdpTelemetryStream;
use websocket_server::{TelemetryServer, TelemetryServerState};
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

    for (idx, &baud) in baud_rates.iter().enumerate() {
        if baud != 115200 {
            imu_uart
                .change_baudrate(esp_idf_hal::units::Hertz(baud))
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

        imu_uart
            .change_baudrate(esp_idf_hal::units::Hertz(9600))
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

    // Initialize UART for GPS
    // NEO-M9N two-step configuration:
    // 1. Connect at factory baud (38400) to send baud rate change command
    // 2. Reconnect at target baud (115200) to send rate/model config
    // Configuration is saved to GPS flash for persistence across power cycles.
    let target_baud = config.gps.effective_baud();
    let factory_baud = config.gps.model.factory_baud();

    info!(
        "Initializing GPS ({} @ {} baud, {}Hz)",
        config.gps.model.name(),
        target_baud,
        config.gps.update_rate_hz
    );

    // Generate UBX commands
    let mut ubx_buffer: [u8; 128] = [0; 128];
    let commands = if config.gps.model.needs_ubx_init() {
        Some(gps::generate_init_sequence(
            config.gps.update_rate_hz,
            target_baud,
            &mut ubx_buffer,
        ))
    } else {
        None
    };

    // Create GPS UART - start at factory baud if we need to configure
    let initial_baud = if config.gps.model.needs_ubx_init() && target_baud != factory_baud {
        factory_baud
    } else {
        target_baud
    };

    let gps_uart = UartDriver::new(
        peripherals.uart0,
        peripherals.pins.gpio5,
        peripherals.pins.gpio4,
        Option::<esp_idf_hal::gpio::Gpio2>::None,
        Option::<esp_idf_hal::gpio::Gpio3>::None,
        &Config::new().baudrate(initial_baud.into()),
    )
    .unwrap();

    // If we need to change baud rate, send command at factory baud then switch
    if config.gps.model.needs_ubx_init() && target_baud != factory_baud {
        info!("Configuring GPS at factory baud ({})...", factory_baud);

        // Send baud rate change command (first command in sequence)
        if let Some(cmds) = &commands {
            let (offset, len) = cmds[0];
            match gps_uart.write(&ubx_buffer[offset..offset + len]) {
                Ok(_) => info!("  CFG-VALSET (baud={}) sent ({} bytes)", target_baud, len),
                Err(e) => info!("  CFG-VALSET (baud) FAILED: {:?}", e),
            }
            FreeRtos::delay_ms(100); // Wait for GPS to process and switch baud
        }

        // Switch ESP32 UART to target baud rate
        info!("Switching to target baud ({})...", target_baud);
        gps_uart
            .change_baudrate(esp_idf_hal::units::Hertz(target_baud))
            .expect("Failed to change GPS UART baud rate");
    }

    // Send remaining UBX configuration commands (rate + model)
    if let Some(cmds) = commands {
        info!(
            "Sending UBX configuration (rate={}Hz, mode=Automotive)...",
            config.gps.update_rate_hz
        );

        // Skip first command if we already sent baud change
        let start_idx = if target_baud != factory_baud { 1 } else { 0 };

        for (i, (offset, len)) in cmds.iter().enumerate().skip(start_idx) {
            let cmd_name = match i {
                0 => "CFG-VALSET (baud)",
                1 => "CFG-VALSET (rate + automotive)",
                _ => "UBX",
            };

            match gps_uart.write(&ubx_buffer[*offset..*offset + *len]) {
                Ok(_) => info!("  {} sent ({} bytes)", cmd_name, len),
                Err(e) => info!("  {} FAILED: {:?}", cmd_name, e),
            }
            FreeRtos::delay_ms(100);
        }

        info!("GPS configuration saved to flash");
    }

    // Create sensor manager
    let mut sensors = SensorManager::new(imu_uart, gps_uart);

    // Calibrate IMU
    if let Some(ref mut mqtt) = mqtt_opt {
        mqtt.publish_status("calib_start", Some(0.0)).ok();
    }

    let bias = sensors
        .calibrate_imu(status_mgr.led_mut(), mqtt_opt.as_mut())
        .expect("Calibration failed");
    sensors.imu_parser.set_bias(bias);

    if let Some(ref mut mqtt) = mqtt_opt {
        mqtt.publish_status("calib_done", Some(1.0)).ok();
        mqtt.publish_status("waiting_gps", None).ok();
    }

    // Create state estimator and telemetry publisher
    let mut estimator = StateEstimator::new();
    let mut publisher = TelemetryPublisher::new(udp_stream, mqtt_opt, telemetry_state.clone());

    // Create sensor fusion for hybrid acceleration (GPS + IMU blending)
    //
    // This provides:
    // 1. Tilt correction - learns mounting offset when stopped (3s)
    // 2. Gravity correction - learns while driving at steady speed
    // 3. Vibration filtering - 2Hz Biquad removes engine/road noise
    // 4. GPS/IMU blending - 70% GPS at 25Hz for balanced response
    //
    // Latency: ~80-100ms (vs ~50ms IMU-only, ~150ms with 90% GPS)
    // Good for: city, highway, canyon. For track, increase GPS weights.
    //
    // NOTE: Filter runs at telemetry rate (~20Hz), not IMU rate (200Hz)
    let telemetry_rate = 1000.0 / config.telemetry.interval_ms as f32;
    let fusion_config = FusionConfig {
        imu_sample_rate: telemetry_rate,
        accel_filter_cutoff: 2.0,           // 2Hz - passes driving, blocks vibration
        gps_high_rate: 20.0,
        gps_medium_rate: 10.0,
        gps_max_age: 0.2,
        // Balanced blend for city/highway/canyon (faster than 90% GPS)
        gps_weight_high: 0.70,              // 70% GPS / 30% IMU at 25Hz
        gps_weight_medium: 0.50,            // 50% / 50% at 10-20Hz
        gps_weight_low: 0.30,               // 30% GPS / 70% IMU fallback
        tilt_learn_time: 3.0,
        gravity_learn_time: 2.0,
        steady_state_speed_tolerance: 0.5,
        steady_state_yaw_tolerance: 0.087,
        gravity_alpha: 0.02,
    };
    let mut sensor_fusion = SensorFusion::new(fusion_config);

    // Main loop timing
    let mut last_telemetry_ms = 0u32;
    let mut last_heap_check_ms = 0u32;
    let mut last_mqtt_diag_ms = 0u32;
    let mut last_connectivity_check_ms = 0u32;
    let mut loop_count = 0u32;

    // Track previous connectivity state to detect changes
    let mut was_wifi_connected = true;
    let mut was_mqtt_connected = true;

    info!("=== Entering main loop ===");

    loop {
        let now_ms = unsafe { (esp_idf_svc::sys::esp_timer_get_time() / 1000) as u32 };
        loop_count += 1;
        diagnostics.record_loop();

        // Check for calibration request from web UI
        if let Some(ref state) = telemetry_state {
            if state.take_calibration_request() {
                info!(">>> Calibration requested via web UI - recalibrating IMU...");
                match sensors.calibrate_imu(status_mgr.led_mut(), None) {
                    Ok(bias) => {
                        sensors.imu_parser.set_bias(bias);
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
                    alpha: 0.25, // Keep default smoothing
                };
                estimator.mode_classifier.update_config(new_config);
                info!(
                    ">>> Mode config updated: acc={:.2}g/{:.2}g, brake={:.2}g/{:.2}g, lat={:.2}g/{:.2}g, yaw={:.3}rad/s, min_spd={:.1}m/s",
                    s.acc_thr, s.acc_exit, s.brake_thr, s.brake_exit, s.lat_thr, s.lat_exit, s.yaw_thr, s.min_speed
                );

                // Visual confirmation: 3 green-white alternating flashes
                for _ in 0..3 {
                    status_mgr.led_mut().green().unwrap();
                    FreeRtos::delay_ms(80);
                    status_mgr.led_mut().white().unwrap();
                    FreeRtos::delay_ms(80);
                }
                status_mgr.led_mut().set_low().unwrap();
            }
        }

        // Periodic diagnostics (serial log every 5s)
        if now_ms - last_heap_check_ms >= 5000 {
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

            // GPS health
            let gps_fix = sensors.gps_parser.last_fix();
            diagnostics.update_gps(
                gps_fix.valid,
                sensors.gps_parser.is_warmed_up(),
                gps_fix.satellites,
                gps_fix.hdop,
                gps_fix.pdop,
            );

            publisher.reset_stats();
            loop_count = 0;
            last_heap_check_ms = now_ms;
        }

        // Connectivity check every 5 seconds (Station mode only)
        // In AP mode, we ARE the access point so connectivity checks don't apply
        if !is_ap_mode && now_ms - last_connectivity_check_ms >= 5000 {
            let wifi_connected = wifi.is_connected();
            let mqtt_connected = publisher
                .mqtt_client_mut()
                .map(|m| m.is_connected())
                .unwrap_or(false);

            // Blink while WiFi is disconnected (more critical - check first)
            if !wifi_connected {
                if was_wifi_connected {
                    info!("WiFi connection lost!");
                }
                // 3 orange blinks for WiFi down
                for _ in 0..3 {
                    status_mgr.led_mut().orange().unwrap();
                    FreeRtos::delay_ms(150);
                    status_mgr.led_mut().set_low().unwrap();
                    FreeRtos::delay_ms(150);
                }
            }
            // Blink while MQTT is disconnected (only if WiFi is up)
            else if !mqtt_connected {
                if was_mqtt_connected {
                    info!("MQTT connection lost!");
                }
                // 2 red blinks for MQTT down
                for _ in 0..2 {
                    status_mgr.led_mut().red().unwrap();
                    FreeRtos::delay_ms(150);
                    status_mgr.led_mut().set_low().unwrap();
                    FreeRtos::delay_ms(150);
                }
            }

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
        if now_ms - last_mqtt_diag_ms >= 30000 {
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
            estimator.ekf.p[0] = 100.0;
            estimator.ekf.p[1] = 100.0;
            unsafe { EKF_RESET_DONE = true };
        }

        // Poll IMU and update EKF prediction (only after GPS warmup AND reset)
        if let Some((dt, _, accel_count)) = sensors.poll_imu() {
            // Record all processed Accel packets for accurate rate measurement
            for _ in 0..accel_count {
                diagnostics.record_imu_packet();
            }
            // Only run EKF prediction after reset is done
            if unsafe { EKF_RESET_DONE } {
                let (ax_corr, ay_corr, az_corr) = sensors.imu_parser.get_accel_corrected();
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
                estimator.predict(ax_e, ay_e, sensors.imu_parser.data().wz, dt);

                // Update yaw from magnetometer
                let yaw_mag = sensors.imu_parser.data().yaw.to_radians();
                estimator.update_yaw(yaw_mag);
            }
        }

        // Poll GPS and update EKF correction
        if sensors.poll_gps() {
            // Count GPS rate only when a NEW valid RMC fix is received
            // (not on every GGA/GSA sentence while last_fix.valid is still true)
            if sensors.gps_parser.take_new_fix() {
                diagnostics.record_gps_fix();
            }

            // Check if warmup complete (has reference point) AND fix is valid
            if sensors.gps_parser.is_warmed_up() && sensors.gps_parser.last_fix().valid {
                let (ax_corr, ay_corr, _) = sensors.imu_parser.get_accel_corrected();

                // Update position from GPS (only when fix is valid)
                if let Some((x, y)) = sensors.gps_parser.to_local_coords() {
                    estimator.update_position(x, y);
                }

                if sensors.is_stationary(ax_corr, ay_corr, sensors.imu_parser.data().wz) {
                    // Vehicle stopped - perform ZUPT and bias estimation
                    estimator.zupt();
                    diagnostics.record_zupt();

                    let (ax_b, ay_b, _) = remove_gravity(
                        ax_corr,
                        ay_corr,
                        sensors.imu_parser.data().az,
                        sensors.imu_parser.data().roll,
                        sensors.imu_parser.data().pitch,
                    );
                    let (ax_e, ay_e) = body_to_earth(
                        ax_b,
                        ay_b,
                        0.0,
                        sensors.imu_parser.data().roll,
                        sensors.imu_parser.data().pitch,
                        estimator.ekf.yaw(),
                    );
                    estimator.update_bias(ax_e, ay_e);
                    estimator.reset_speed();
                }

                if let Some((vx, vy)) = sensors.gps_parser.get_velocity_enu() {
                    let speed = (vx * vx + vy * vy).sqrt();
                    estimator.update_velocity(vx, vy);
                    estimator.update_speed(speed);

                    // Feed GPS speed to sensor fusion for acceleration calculation
                    let time_s = now_ms as f32 / 1000.0;
                    sensor_fusion.process_gps(speed, time_s);
                }
            }
        }

        // Update GPS rate estimate periodically (every second)
        if now_ms % 1000 < 50 {
            let fix_count = diagnostics.gps_fix_count();
            sensor_fusion.update_gps_rate(fix_count, now_ms as f32 / 1000.0);
        }

        // Update GPS status
        let gps_locked = sensors.gps_parser.is_warmed_up(); // Remove the .valid check
        status_mgr.update_gps_status(
            gps_locked,
            sensors.gps_parser.is_warmed_up(),
            now_ms,
            publisher.mqtt_client_mut(),
        );

        // Publish telemetry at configured rate
        if now_ms - last_telemetry_ms >= config.telemetry.interval_ms {
            // Get current speed and check if stationary
            let speed = sensors.get_speed(Some(&estimator.ekf));
            let (ax_corr, ay_corr, _) = sensors.imu_parser.get_accel_corrected();
            let is_stationary =
                sensors.is_stationary(ax_corr, ay_corr, sensors.imu_parser.data().wz);

            // Transform IMU to earth frame for sensor fusion
            let (ax_b, ay_b, _) = remove_gravity(
                ax_corr,
                ay_corr,
                sensors.imu_parser.data().az,
                sensors.imu_parser.data().roll,
                sensors.imu_parser.data().pitch,
            );
            let (ax_e, ay_e) = body_to_earth(
                ax_b,
                ay_b,
                0.0,
                sensors.imu_parser.data().roll,
                sensors.imu_parser.data().pitch,
                estimator.ekf.yaw(),
            );

            // Process through sensor fusion (GPS/IMU blending, filtering, tilt correction)
            let dt = config.telemetry.interval_ms as f32 / 1000.0;
            let (lon_blended, lat_filtered) = sensor_fusion.process_imu(
                ax_e,
                ay_e,
                estimator.ekf.yaw(),
                speed,
                sensors.imu_parser.data().wz,
                dt,
                is_stationary,
            );

            // Update mode classifier with hybrid (blended) acceleration
            estimator.mode_classifier.update_hybrid(
                lon_blended,
                lat_filtered,
                sensors.imu_parser.data().wz,
                speed,
            );

            // Publish telemetry with tilt-corrected accelerations
            publisher
                .publish_telemetry(&sensors, &estimator, &sensor_fusion, now_ms)
                .ok();
            last_telemetry_ms = now_ms;
        }

        // Update LED status
        status_mgr.update_led(gps_locked, now_ms);
    }
}
