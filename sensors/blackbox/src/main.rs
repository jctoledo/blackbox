mod binary_telemetry;
mod config;
mod diagnostics;
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
        "Mode: {}, SSID: {}, Rate: {}Hz",
        if is_ap_mode {
            "Access Point"
        } else {
            "Station"
        },
        config.network.wifi_ssid,
        1000 / config.telemetry.interval_ms
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

    // Diagnostics state (shared between server and main loop, AP mode only)
    let diagnostics_state = DiagnosticsState::new();

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

                // Start HTTP telemetry server with diagnostics support
                info!(
                    "Starting telemetry server on port {}",
                    config.network.ws_port
                );
                match TelemetryServer::new(config.network.ws_port, Some(diagnostics_state.clone())) {
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

    // Initialize UART for IMU
    let imu_config = Config::new().baudrate(9600.into());
    let imu_uart = UartDriver::new(
        peripherals.uart1,
        peripherals.pins.gpio18,
        peripherals.pins.gpio19,
        Option::<esp_idf_hal::gpio::Gpio0>::None,
        Option::<esp_idf_hal::gpio::Gpio1>::None,
        &imu_config,
    )
    .unwrap();

    // Initialize UART for GPS
    info!(
        "Initializing GPS: {} @ {} Hz",
        config.gps.model.name(),
        config.gps.model.max_rate_hz()
    );
    let gps_uart = UartDriver::new(
        peripherals.uart0,
        peripherals.pins.gpio5,
        peripherals.pins.gpio4,
        Option::<esp_idf_hal::gpio::Gpio2>::None,
        Option::<esp_idf_hal::gpio::Gpio3>::None,
        &Config::new().baudrate(9600.into()),
    )
    .unwrap();

    // Configure GPS update rate via UBX command
    let ubx_cmd = ublox_neo::ubx::cfg_rate_command(config.gps.model.measurement_period_ms());
    gps_uart.write(&ubx_cmd).ok();
    info!(
        "GPS rate configured: {} ms period",
        config.gps.model.measurement_period_ms()
    );

    // Create sensor manager with GPS warmup from config
    let mut sensors = SensorManager::new(imu_uart, gps_uart, config.gps.warmup_fixes);

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

    // Initialize diagnostics with static config (AP mode only)
    if is_ap_mode {
        diagnostics_state.update(|d| {
            d.wifi_status.mode = "Access Point";
            d.wifi_status.ssid = config.network.wifi_ssid;
            d.wifi_status.ip_address = "192.168.71.1";
            d.config.telemetry_rate_hz = 1000 / config.telemetry.interval_ms;
            d.config.gps_model = config.gps.model.name();
            d.config.gps_warmup_fixes = config.gps.warmup_fixes;
            d.gps_health.model_name = config.gps.model.name();
            d.gps_health.configured_rate_hz = config.gps.model.max_rate_hz();
            d.sensor_rates.imu_expected_hz = 200.0;
            d.sensor_rates.gps_expected_hz = config.gps.model.max_rate_hz() as f32;
        });
    }

    // Main loop timing
    let mut last_telemetry_ms = 0u32;
    let mut last_heap_check_ms = 0u32;
    let mut last_mqtt_diag_ms = 0u32;
    let mut last_connectivity_check_ms = 0u32;
    let mut last_diagnostics_ms = 0u32;
    let mut loop_count = 0u32;

    // Track previous connectivity state to detect changes
    let mut was_wifi_connected = true;
    let mut was_mqtt_connected = true;

    info!("=== Entering main loop ===");

    loop {
        let now_ms = unsafe { (esp_idf_svc::sys::esp_timer_get_time() / 1000) as u32 };
        loop_count += 1;

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

            publisher.reset_stats();
            loop_count = 0;
            last_heap_check_ms = now_ms;
        }

        // Update diagnostics state (every second, AP mode only)
        if is_ap_mode && now_ms - last_diagnostics_ms >= 1000 {
            sensors.update_rates(now_ms);
            let (tx_ok, tx_fail) = publisher.get_stats();

            diagnostics_state.update(|d| {
                // Sensor rates (updated by sensors.update_rates())
                d.sensor_rates.imu_hz = sensors.imu_rate_hz;
                d.sensor_rates.gps_hz = sensors.gps_rate_hz;

                // EKF health (access public state x and covariance p directly)
                let p = &estimator.ekf.p;
                d.ekf_health.position_sigma = (p[0] + p[1]).sqrt(); // sqrt(px² + py²)
                d.ekf_health.velocity_sigma = (p[3] + p[4]).sqrt(); // sqrt(vx² + vy²)
                d.ekf_health.yaw_sigma_deg = p[2].sqrt().to_degrees();
                let (bias_x, bias_y) = estimator.ekf.biases();
                d.ekf_health.bias_x = bias_x;
                d.ekf_health.bias_y = bias_y;

                // System health
                d.system_health.free_heap_bytes =
                    unsafe { esp_idf_svc::sys::esp_get_free_heap_size() };
                d.system_health.uptime_seconds = now_ms / 1000;
                d.system_health.telemetry_sent = tx_ok;
                d.system_health.telemetry_failed = tx_fail;

                // GPS health
                d.gps_health.fix_valid = sensors.gps_parser.last_fix().valid;
                d.gps_health.warmup_complete = sensors.gps_parser.is_warmed_up();

                // IMU temperature
                d.imu_temp_celsius = sensors.imu_parser.data().temp;
            });

            last_diagnostics_ms = now_ms;
        }

        // Connectivity check every 5 seconds
        if now_ms - last_connectivity_check_ms >= 5000 {
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
            // Blink while MQTT is disconnected (only in Station mode, and only if WiFi is up)
            else if !is_ap_mode && !mqtt_connected {
                if was_mqtt_connected {
                    info!("MQTT connection lost!");
                }
                // 2 red blinks for MQTT down (Station mode only)
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
        if let Some((dt, _)) = sensors.poll_imu() {
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
                }
            }
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
            // Update mode classifier
            let speed = sensors.get_speed(Some(&estimator.ekf));
            let (ax_corr, ay_corr, _) = sensors.imu_parser.get_accel_corrected();
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
            estimator.update_mode(
                ax_e,
                ay_e,
                estimator.ekf.yaw(),
                sensors.imu_parser.data().wz,
                speed,
            );

            // Publish telemetry
            publisher
                .publish_telemetry(&sensors, &estimator, now_ms)
                .ok();
            last_telemetry_ms = now_ms;
        }

        // Update LED status
        status_mgr.update_led(gps_locked, now_ms);
    }
}
