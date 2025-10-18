mod imu;
mod gps;
mod wifi;
mod mqtt;
mod rgb_led;
mod ekf;
mod transforms;
mod mode;
mod binary_telemetry;

use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::uart::{UartDriver, config::Config};
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use log::info;

use imu::{Wt901Parser, ImuCalibrator, ImuBias};
use gps::NmeaParser;
use wifi::WifiManager;
use mqtt::MqttClient;
use rgb_led::RgbLed;
use ekf::Ekf;
use transforms::{body_to_earth, remove_gravity};
use mode::ModeClassifier;

const CALIB_SAMPLES: usize = 500;
const WIFI_SSID: &str = "GiraffeWireless";
const WIFI_PASSWORD: &str = "basicchair411";
const MQTT_BROKER: &str = "mqtt://192.168.50.46:1883";

// Binary protocol allows 20 Hz (50ms) sustainable rate
const MQTT_PUBLISH_INTERVAL_MS: u32 = 200;  // 20 Hz with binary encoding

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    
    info!("=== ESP32-C3 Telemetry System (Binary Protocol @ 20 Hz) ===");
    info!("Free heap at start: {} bytes", unsafe { 
        esp_idf_svc::sys::esp_get_free_heap_size() 
    });
    
    let peripherals = Peripherals::take().unwrap();
    let sysloop = EspSystemEventLoop::take().unwrap();
    let nvs = EspDefaultNvsPartition::take().ok();
    
    // Initialize RGB LED
    info!("Initializing RGB LED");
    let mut led = RgbLed::new(peripherals.rmt.channel0, peripherals.pins.gpio8)
        .expect("Failed to initialize RGB LED");
    
    led.set_low().unwrap();
    FreeRtos::delay_ms(1000);
    
    // Boot blinks
    info!("Boot sequence - 3 BLUE blinks");
    for i in 0..3 {
        led.blue().unwrap();
        FreeRtos::delay_ms(300);
        led.set_low().unwrap();
        FreeRtos::delay_ms(300);
        info!("Boot blink {}/3", i + 1);
    }
    FreeRtos::delay_ms(1000);
    
    // WiFi setup
    info!("Initializing WiFi");
    let mut wifi = WifiManager::new(peripherals.modem, sysloop.clone(), nvs)
        .expect("WiFi init failed");
    
    info!("Connecting to WiFi: {}", WIFI_SSID);
    match wifi.connect(WIFI_SSID, WIFI_PASSWORD) {
        Ok(_) => {
            info!("WiFi connected successfully");
            for i in 0..5 {
                led.green().unwrap();
                FreeRtos::delay_ms(200);
                led.set_low().unwrap();
                FreeRtos::delay_ms(200);
                info!("WiFi success blink {}/5", i + 1);
            }
            FreeRtos::delay_ms(1500);
        }
        Err(e) => {
            info!("WiFi connection failed: {:?}", e);
            loop {
                led.red().unwrap();
                FreeRtos::delay_ms(500);
                led.set_low().unwrap();
                FreeRtos::delay_ms(500);
            }
        }
    }
    
    // MQTT setup with better error handling
    info!("About to initialize MQTT");
    led.set_high().unwrap();
    FreeRtos::delay_ms(300);
    led.set_low().unwrap();
    FreeRtos::delay_ms(300);
    
    info!("Initializing MQTT");
    let mut mqtt_opt = match MqttClient::new(MQTT_BROKER) {
        Ok(m) => {
            info!("MQTT client created successfully");
            for _ in 0..2 {
                led.set_high().unwrap();
                FreeRtos::delay_ms(200);
                led.set_low().unwrap();
                FreeRtos::delay_ms(200);
            }
            Some(m)
        }
        Err(e) => {
            info!("MQTT init failed: {:?}", e);
            for _ in 0..3 {
                led.red().unwrap();
                FreeRtos::delay_ms(300);
                led.set_low().unwrap();
                FreeRtos::delay_ms(300);
            }
            None
        }
    };
    
    // Test MQTT publish with timeout
    if let Some(ref mut mqtt) = mqtt_opt {
        info!("Testing MQTT publish");
        led.blue().unwrap();
        FreeRtos::delay_ms(200);
        led.set_low().unwrap();
        FreeRtos::delay_ms(200);
        
        let mut publish_ok = false;
        for attempt in 0..50 {
            led.set_high().unwrap();
            FreeRtos::delay_ms(50);
            led.set_low().unwrap();
            FreeRtos::delay_ms(50);
            
            match mqtt.publish("car/status", "boot_binary_mode", false) {
                Ok(_) => {
                    info!("MQTT publish succeeded on attempt {}", attempt);
                    publish_ok = true;
                    break;
                }
                Err(e) => {
                    if attempt % 10 == 0 {
                        info!("Publish attempt {} failed: {:?}", attempt, e);
                    }
                    continue;
                }
            }
        }
        
        if publish_ok {
            for _ in 0..5 {
                led.magenta().unwrap();
                FreeRtos::delay_ms(100);
                led.set_low().unwrap();
                FreeRtos::delay_ms(100);
            }
        } else {
            info!("MQTT publish timed out after 5 seconds");
            for _ in 0..3 {
                led.yellow().unwrap();
                FreeRtos::delay_ms(300);
                led.set_low().unwrap();
                FreeRtos::delay_ms(300);
            }
            mqtt_opt = None;
        }
    }
    
    if mqtt_opt.is_none() {
        info!("Continuing without MQTT");
        for _ in 0..3 {
            led.set_color(255, 128, 0).unwrap();
            FreeRtos::delay_ms(200);
            led.set_low().unwrap();
            FreeRtos::delay_ms(200);
        }
    }
    
    // Initialize UARTs
    info!("Initializing UART1 for IMU (GPIO18/19)");
    let imu_config = Config::new().baudrate(9600.into());
    let mut imu_uart = UartDriver::new(
        peripherals.uart1,
        peripherals.pins.gpio18,
        peripherals.pins.gpio19,
        Option::<esp_idf_hal::gpio::Gpio0>::None,
        Option::<esp_idf_hal::gpio::Gpio1>::None,
        &imu_config,
    ).unwrap();
    
    info!("Initializing UART0 for GPS (GPIO5/4)");
    info!("NOTE: Serial console will stop after this point");
    let gps_uart = UartDriver::new(
        peripherals.uart0,
        peripherals.pins.gpio5,
        peripherals.pins.gpio4,
        Option::<esp_idf_hal::gpio::Gpio2>::None,
        Option::<esp_idf_hal::gpio::Gpio3>::None,
        &Config::new().baudrate(9600.into()),
    ).unwrap();
    
    // Configure GPS to 5 Hz
    let ubx_rate_5hz: [u8; 14] = [
        0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
        0xC8, 0x00, 0x01, 0x00, 0x01, 0x00,
        0xDE, 0x6A
    ];
    gps_uart.write(&ubx_rate_5hz).ok();
    FreeRtos::delay_ms(100);
    
    // Calibrate IMU
    led.yellow().unwrap();
    FreeRtos::delay_ms(500);
    led.set_low().unwrap();
    FreeRtos::delay_ms(500);
    
    if let Some(ref mut mqtt) = mqtt_opt {
        mqtt.publish_status("calib_start", Some(0.0)).ok();
    }
    
    let bias = calibrate_imu_optional(&mut imu_uart, &mut led, mqtt_opt.as_mut())
        .expect("Calibration failed");
    
    if let Some(ref mut mqtt) = mqtt_opt {
        mqtt.publish_status("calib_done", Some(1.0)).ok();
    }
    
    // Initialize parsers and filters
    let mut imu_parser = Wt901Parser::new();
    imu_parser.set_bias(bias);
    let mut gps_parser = NmeaParser::new();
    let mut ekf = Ekf::new();
    let mut mode_classifier = ModeClassifier::new();
    
    // GPS state tracking
    let mut gps_was_locked = false;
    let mut last_gps_status_ms = 0u32;
    let mut last_warmup_progress = 0.0f32;
    
    // ZUPT state tracking
    let mut stationary_count = 0u32;
    
    if let Some(ref mut mqtt) = mqtt_opt {
        mqtt.publish_status("waiting_gps", None).ok();
    }
    
    // Main loop timing
    let mut last_mqtt_ms = 0u32;
    let mut last_imu_us = unsafe { esp_idf_svc::sys::esp_timer_get_time() as u64 };
    let mut last_heap_check_ms = 0u32;
    let mut buf = [0u8; 1];
    
    // Publish failure tracking
    let mut consecutive_publish_failures = 0u32;
    const MAX_PUBLISH_FAILURES: u32 = 10;
    
    // Publish rate monitoring
    let mut publish_count = 0u32;
    let mut last_rate_check_ms = 0u32;
    let mut publish_attempt_count = 0u32;  // How many times we tried to publish
    let mut loop_count = 0u32;  // Total loop iterations
    
    info!("=== Entering main loop (Binary telemetry @ 20 Hz) ===");
    
    loop {
        let now_ms = unsafe { (esp_idf_svc::sys::esp_timer_get_time() / 1000) as u32 };
        let now_us = unsafe { esp_idf_svc::sys::esp_timer_get_time() as u64 };
        
        loop_count += 1;
        
        // Periodic heap monitoring (every 5 seconds)
        if now_ms - last_heap_check_ms >= 5000 {
            let free_heap = unsafe { esp_idf_svc::sys::esp_get_free_heap_size() };
            let publish_rate = publish_count / 5;
            let attempt_rate = publish_attempt_count / 5;
            let loop_rate = loop_count / 5;
            info!("Heap: {}B | Pub: {}Hz | Attempt: {}Hz | Loop: {}Hz", 
                  free_heap, publish_rate, attempt_rate, loop_rate);
            publish_count = 0;
            publish_attempt_count = 0;
            loop_count = 0;
            
            if free_heap < 10000 {
                info!("WARNING: Low heap memory! Rebooting...");
                unsafe { esp_idf_svc::sys::esp_restart(); }
            }
            
            last_heap_check_ms = now_ms;
        }
        
        // Read IMU and run EKF prediction
        if imu_uart.read(&mut buf, 0).unwrap_or(0) > 0 {
            if let Some(packet_type) = imu_parser.feed_byte(buf[0], now_us) {
                if packet_type == 0x51 {
                    let dt = (now_us - last_imu_us) as f32 * 1e-6;
                    last_imu_us = now_us;
                    
                    if dt > 5e-4 && dt < 0.05 {
                        let (ax_corr, ay_corr, az_corr) = imu_parser.get_accel_corrected();
                        
                        let (ax_b, ay_b, az_b) = remove_gravity(
                            ax_corr, ay_corr, az_corr,
                            imu_parser.data.roll, imu_parser.data.pitch,
                        );
                        
                        let (ax_e, ay_e) = body_to_earth(
                            ax_b, ay_b, az_b,
                            imu_parser.data.roll, imu_parser.data.pitch, ekf.yaw(),
                        );
                        
                        ekf.predict(ax_e, ay_e, imu_parser.data.wz, dt);
                    }
                }
                
                if packet_type == 0x53 {
                    let yaw_mag = imu_parser.data.yaw.to_radians();
                    ekf.update_yaw(yaw_mag);
                }
            }
        }
        
        // Read GPS and update EKF
        if gps_uart.read(&mut buf, 0).unwrap_or(0) > 0 {
            if gps_parser.feed_byte(buf[0]) {
                if gps_parser.last_fix.valid {
                    let (ax_corr, ay_corr, _) = imu_parser.get_accel_corrected();
                    
                    if is_stationary(
                        ax_corr, ay_corr, imu_parser.data.wz,
                        gps_parser.last_fix.speed, gps_parser.position_based_speed
                    ) {
                        stationary_count += 1;
                        
                        if stationary_count >= 5 {
                            ekf.zupt();
                            
                            let (ax_b, ay_b, _) = remove_gravity(
                                ax_corr, ay_corr, imu_parser.data.az,
                                imu_parser.data.roll, imu_parser.data.pitch,
                            );
                            let (ax_e, ay_e) = body_to_earth(
                                ax_b, ay_b, 0.0,
                                imu_parser.data.roll, imu_parser.data.pitch, ekf.yaw(),
                            );
                            ekf.update_bias(ax_e, ay_e);
                            mode_classifier.reset_speed();
                        } else {
                            if let Some((x, y)) = gps_parser.to_local_coords() {
                                ekf.update_position(x, y);
                            }
                        }
                    } else {
                        stationary_count = 0;
                        
                        if let Some((x, y)) = gps_parser.to_local_coords() {
                            ekf.update_position(x, y);
                        }
                    }
                    
                    if let Some((vx, vy)) = gps_parser.get_velocity_enu() {
                        let speed = (vx * vx + vy * vy).sqrt();
                        ekf.update_velocity(vx, vy);
                        ekf.update_speed(speed);
                    }
                }
            }
        }
        
        // GPS state machine
        let gps_locked_now = gps_parser.is_warmed_up() && gps_parser.last_fix.valid;
        
        if gps_locked_now && !gps_was_locked {
            if let Some(ref mut mqtt) = mqtt_opt {
                mqtt.publish_status("gps_lock", None).ok();
            }
            for _ in 0..3 {
                led.green().unwrap();
                FreeRtos::delay_ms(100);
                led.set_low().unwrap();
                FreeRtos::delay_ms(100);
            }
        }
        
        if !gps_locked_now && gps_was_locked {
            if let Some(ref mut mqtt) = mqtt_opt {
                mqtt.publish_status("gps_lost", None).ok();
            }
        }
        
        if !gps_locked_now && now_ms - last_gps_status_ms >= 5000 {
            if !gps_parser.is_warmed_up() {
                if let Some(ref mut mqtt) = mqtt_opt {
                    mqtt.publish_status("waiting_gps", None).ok();
                }
            } else {
                if let Some(ref mut mqtt) = mqtt_opt {
                    mqtt.publish_status("gps_lost", None).ok();
                }
            }
            last_gps_status_ms = now_ms;
        }
        
        if gps_parser.is_warmed_up() && !gps_locked_now {
            let progress = gps_parser.warmup_progress();
            if (progress - last_warmup_progress).abs() > 0.1 {
                if let Some(ref mut mqtt) = mqtt_opt {
                    mqtt.publish_status("gps_warmup", Some(progress)).ok();
                }
                last_warmup_progress = progress;
            }
        }
        
        gps_was_locked = gps_locked_now;
        
        // *** BINARY TELEMETRY @ 20 Hz (50ms intervals) ***
        if mqtt_opt.is_some() && now_ms - last_mqtt_ms >= MQTT_PUBLISH_INTERVAL_MS {
            publish_attempt_count += 1;
            
            let (vx, vy) = ekf.velocity();
            let (ax_corr, ay_corr, _) = imu_parser.get_accel_corrected();
            let (ax_b, ay_b, _) = remove_gravity(
                ax_corr, ay_corr, imu_parser.data.az,
                imu_parser.data.roll, imu_parser.data.pitch,
            );
            let (ax_e, ay_e) = body_to_earth(
                ax_b, ay_b, 0.0,
                imu_parser.data.roll, imu_parser.data.pitch, ekf.yaw(),
            );
            mode_classifier.update(ax_e, ay_e, ekf.yaw(), imu_parser.data.wz, vx, vy);
            
            // *** PUBLISH USING BINARY PROTOCOL ***
            match binary_telemetry::publish_telemetry_binary(
                mqtt_opt.as_mut().unwrap(), 
                &imu_parser, 
                &gps_parser, 
                &ekf, 
                &mode_classifier
            ) {
                Ok(_) => {
                    consecutive_publish_failures = 0;
                    publish_count += 1;
                    last_mqtt_ms = now_ms;
                }
                Err(e) => {
                    consecutive_publish_failures += 1;
                    if consecutive_publish_failures % 10 == 1 {
                        info!("Binary publish failed ({}): {:?}", consecutive_publish_failures, e);
                    }
                    
                    if consecutive_publish_failures >= MAX_PUBLISH_FAILURES {
                        info!("Too many publish failures, disabling MQTT");
                        mqtt_opt = None;
                    }
                    // DON'T update last_mqtt_ms on failure - try again next loop
                }
            }
        }
        
        // LED heartbeat
        if gps_locked_now {
            if now_ms % 2000 < 100 {
                led.cyan().ok();
            } else {
                led.set_low().ok();
            }
        } else {
            if now_ms % 500 < 250 {
                led.yellow().ok();
            } else {
                led.set_low().ok();
            }
        }
        
        // NO DELAY - run as fast as possible for 20 Hz telemetry
        // The UART reads are non-blocking (timeout=0) so this won't burn CPU
    }
}

fn calibrate_imu_optional(
    uart: &mut UartDriver,
    led: &mut RgbLed,
    mut mqtt: Option<&mut MqttClient>,
) -> Result<ImuBias, &'static str> {
    let mut parser = Wt901Parser::new();
    let mut calibrator = ImuCalibrator::new(CALIB_SAMPLES);
    let mut buf = [0u8; 1];
    let mut last_progress = 0;
    
    while !calibrator.is_complete() {
        if uart.read(&mut buf, 0).unwrap_or(0) > 0 {
            let timestamp_us = unsafe { esp_idf_svc::sys::esp_timer_get_time() as u64 };
            if let Some(packet_type) = parser.feed_byte(buf[0], timestamp_us) {
                if packet_type == 0x51 {
                    calibrator.add_sample(parser.data.ax, parser.data.ay, parser.data.az);
                    
                    let progress = (calibrator.progress() * 100.0) as u32;
                    if progress > last_progress && progress % 10 == 0 {
                        if let Some(mqtt) = mqtt.as_mut() {
                            mqtt.publish_status("calib_running", Some(calibrator.progress())).ok();
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
    
    calibrator.compute_bias().ok_or("Failed to compute bias")
}

fn is_stationary(ax: f32, ay: f32, wz: f32, gps_speed: f32, position_speed: f32) -> bool {
    const ACC_THR: f32 = 0.18 * 9.80665;
    const WZ_THR: f32 = 12.0 * 0.017453293;
    const GPS_SPEED_THR: f32 = 3.5;
    const POS_SPEED_THR: f32 = 5.0;
    
    let low_inertial = ax.abs() < ACC_THR && ay.abs() < ACC_THR && wz.abs() < WZ_THR;
    let low_speed = gps_speed < GPS_SPEED_THR && position_speed < POS_SPEED_THR;
    
    low_inertial && low_speed
}
