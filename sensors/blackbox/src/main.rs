mod binary_telemetry;
mod config;
mod gps;
mod imu;
mod mode;
mod mqtt;
mod rgb_led;
mod system;
mod tcp_stream;
mod wifi;

use config::SystemConfig;
use esp_idf_hal::{
    delay::FreeRtos,
    peripherals::Peripherals,
    uart::{config::Config, UartDriver},
};
use esp_idf_svc::{eventloop::EspSystemEventLoop, nvs::EspDefaultNvsPartition};
use log::info;
use motorsport_telemetry::transforms::{body_to_earth, remove_gravity};
use mqtt::MqttClient;
use rgb_led::RgbLed;
use system::{SensorManager, StateEstimator, StatusManager, TelemetryPublisher};
use tcp_stream::TcpTelemetryStream;
use wifi::WifiManager;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let config = SystemConfig::from_env();

    info!("=== ESP32-C3 Telemetry System ===");
    info!(
        "Config: SSID={}, Rate={}Hz",
        config.network.wifi_ssid,
        1000 / config.telemetry.interval_ms
    );
    info!("MQTT: Status messages");
    info!(
        "TCP:  Binary telemetry @ {} Hz",
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

    // Boot sequence
    status_mgr.led_mut().set_low().unwrap();
    FreeRtos::delay_ms(1000);

    for i in 0..3 {
        status_mgr.led_mut().blue().unwrap();
        FreeRtos::delay_ms(300);
        status_mgr.led_mut().set_low().unwrap();
        FreeRtos::delay_ms(300);
        info!("Boot blink {}/3", i + 1);
    }
    FreeRtos::delay_ms(1000);

    // Connect WiFi
    info!("Initializing WiFi");
    let mut wifi =
        WifiManager::new(peripherals.modem, sysloop.clone(), nvs).expect("WiFi init failed");

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
    let mut mqtt_opt = match MqttClient::new(config.network.mqtt_broker) {
        Ok(m) => {
            info!("MQTT connected");
            Some(m)
        }
        Err(e) => {
            info!("MQTT failed: {:?}", e);
            None
        }
    };

    // Connect TCP for telemetry
    info!("Connecting TCP stream to {}", config.network.tcp_server);
    let mut tcp_stream = TcpTelemetryStream::new(config.network.tcp_server);
    let tcp_opt = match tcp_stream.connect() {
        Ok(_) => {
            info!("TCP connected!");
            for _ in 0..3 {
                status_mgr.led_mut().cyan().unwrap();
                FreeRtos::delay_ms(150);
                status_mgr.led_mut().set_low().unwrap();
                FreeRtos::delay_ms(150);
            }
            Some(tcp_stream)
        }
        Err(e) => {
            info!("TCP failed: {:?}", e);
            None
        }
    };

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
    info!("Initializing GPS");
    let gps_uart = UartDriver::new(
        peripherals.uart0,
        peripherals.pins.gpio5,
        peripherals.pins.gpio4,
        Option::<esp_idf_hal::gpio::Gpio2>::None,
        Option::<esp_idf_hal::gpio::Gpio3>::None,
        &Config::new().baudrate(9600.into()),
    )
    .unwrap();

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
    let mut publisher = TelemetryPublisher::new(tcp_opt, mqtt_opt);

    // Main loop timing
    let mut last_telemetry_ms = 0u32;
    let mut last_heap_check_ms = 0u32;
    let mut loop_count = 0u32;

    info!("=== Entering main loop ===");

    loop {
        let now_ms = unsafe { (esp_idf_svc::sys::esp_timer_get_time() / 1000) as u32 };
        loop_count += 1;

        // Periodic diagnostics
        if now_ms - last_heap_check_ms >= 5000 {
            let free_heap = unsafe { esp_idf_svc::sys::esp_get_free_heap_size() };
            let (telem_count, fail_count) = publisher.get_stats();
            let telem_rate = telem_count / 5;
            let loop_rate = loop_count / 5;
            info!(
                "Heap: {}B | TCP: {}Hz | Loop: {}Hz | Fails: {}",
                free_heap, telem_rate, loop_rate, fail_count
            );

            publisher.reset_stats();
            loop_count = 0;
            last_heap_check_ms = now_ms;
        }

        // Poll IMU and update EKF prediction
        if let Some((dt, _)) = sensors.poll_imu() {
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

        // Poll GPS and update EKF correction
        // if sensors.poll_gps() && sensors.gps_parser.last_fix().valid {
        //     let (ax_corr, ay_corr, _) = sensors.imu_parser.get_accel_corrected();

        //     if sensors.is_stationary(ax_corr, ay_corr, sensors.imu_parser.data().wz)
        // {         // Vehicle stopped - perform ZUPT and bias estimation
        //         estimator.zupt();

        //         let (ax_b, ay_b, _) = remove_gravity(
        //             ax_corr,
        //             ay_corr,
        //             sensors.imu_parser.data().az,
        //             sensors.imu_parser.data().roll,
        //             sensors.imu_parser.data().pitch,
        //         );
        //         let (ax_e, ay_e) = body_to_earth(
        //             ax_b,
        //             ay_b,
        //             0.0,
        //             sensors.imu_parser.data().roll,
        //             sensors.imu_parser.data().pitch,
        //             estimator.ekf.yaw(),
        //         );
        //         estimator.update_bias(ax_e, ay_e);
        //         estimator.reset_speed();
        //     } else {
        //         // Vehicle moving - normal GPS updates
        //         if let Some((x, y)) = sensors.gps_parser.to_local_coords() {
        //             estimator.update_position(x, y);
        //         }
        //     }

        //     if let Some((vx, vy)) = sensors.gps_parser.get_velocity_enu() {
        //         let speed = (vx * vx + vy * vy).sqrt();
        //         estimator.update_velocity(vx, vy);
        //         estimator.update_speed(speed);
        //     }
        // }
        sensors.poll_gps();

        // Update GPS status
        let gps_locked = sensors.gps_parser.is_warmed_up() && sensors.gps_parser.last_fix().valid;
        status_mgr.update_gps_status(
            gps_locked,
            sensors.gps_parser.is_warmed_up(),
            now_ms,
            publisher.mqtt_client_mut(),
        );

        // Publish telemetry at configured rate
        if now_ms - last_telemetry_ms >= config.telemetry.interval_ms {
            // Update mode classifier
            let (vx, vy) = estimator.ekf.velocity();
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
                vx,
                vy,
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
