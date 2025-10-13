mod imu;
mod gps;
mod wifi;
mod mqtt;
mod rgb_led;
mod ekf;
mod transforms;

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

const CALIB_SAMPLES: usize = 500;
const WIFI_SSID: &str = "GiraffeWireless";
const WIFI_PASSWORD: &str = "basicchair411";
const MQTT_BROKER: &str = "mqtt://192.168.50.46:1883";

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    
    info!("Starting ESP32-C3 telemetry system");
    
    let peripherals = Peripherals::take().unwrap();
    let sysloop = EspSystemEventLoop::take().unwrap();
    let nvs = EspDefaultNvsPartition::take().ok();
    
    // Initialize RGB LED on GPIO8 using RMT channel 0
    info!("Initializing RGB LED");
    let mut led = RgbLed::new(peripherals.rmt.channel0, peripherals.pins.gpio8)
        .expect("Failed to initialize RGB LED");
    
    // Make absolutely sure LED is off first
    led.set_low().unwrap();
    FreeRtos::delay_ms(1000);  // 1 second pause at startup
    
    // Boot blink: EXACTLY 3 slow blue pulses
    info!("Boot sequence - 3 BLUE blinks");
    for i in 0..3 {
        led.blue().unwrap();
        FreeRtos::delay_ms(300);
        led.set_low().unwrap();
        FreeRtos::delay_ms(300);
        info!("Boot blink {}/3", i + 1);
    }
    FreeRtos::delay_ms(1000);
    
    // Initialize WiFi
    info!("Initializing WiFi");
    let mut wifi = WifiManager::new(peripherals.modem, sysloop.clone(), nvs)
        .expect("WiFi init failed");
    
    info!("Connecting to WiFi: {}", WIFI_SSID);
    match wifi.connect(WIFI_SSID, WIFI_PASSWORD) {
        Ok(_) => {
            info!("WiFi connected successfully");
            // WiFi OK: EXACTLY 5 slow green blinks
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
    
    // Single WHITE blink = about to init MQTT
    info!("About to initialize MQTT");
    led.set_high().unwrap();
    FreeRtos::delay_ms(300);
    led.set_low().unwrap();
    FreeRtos::delay_ms(300);
    
    // Try to initialize MQTT
    info!("Initializing MQTT");
    let mut mqtt_opt = match MqttClient::new(MQTT_BROKER) {
        Ok(m) => {
            info!("MQTT client created successfully");
            // 2 WHITE blinks = MQTT object created
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
            // 3 RED blinks = MQTT failed to create
            for _ in 0..3 {
                led.red().unwrap();
                FreeRtos::delay_ms(300);
                led.set_low().unwrap();
                FreeRtos::delay_ms(300);
            }
            None
        }
    };
    
    // If MQTT succeeded, try to publish with timeout
    if let Some(ref mut mqtt) = mqtt_opt {
        info!("Testing MQTT publish");
        
        // BLUE blink = about to publish
        led.blue().unwrap();
        FreeRtos::delay_ms(200);
        led.set_low().unwrap();
        FreeRtos::delay_ms(200);
        
        // Try publish with a simple timeout approach
        // Flash WHITE rapidly while trying to publish (max 5 seconds)
        let mut publish_ok = false;
        for _ in 0..50 {  // 50 * 100ms = 5 seconds max
            led.set_high().unwrap();
            FreeRtos::delay_ms(50);
            led.set_low().unwrap();
            FreeRtos::delay_ms(50);
            
            // Try non-blocking enqueue
            match mqtt.publish("car/status", "boot", false) {
                Ok(_) => {
                    info!("MQTT publish succeeded!");
                    publish_ok = true;
                    break;
                }
                Err(_) => {
                    // Keep trying
                    continue;
                }
            }
        }
        
        if publish_ok {
            // 5 MAGENTA blinks = MQTT working!
            for _ in 0..5 {
                led.magenta().unwrap();
                FreeRtos::delay_ms(100);
                led.set_low().unwrap();
                FreeRtos::delay_ms(100);
            }
        } else {
            info!("MQTT publish timed out after 5 seconds");
            // 3 YELLOW blinks = Timeout
            for _ in 0..3 {
                led.yellow().unwrap();
                FreeRtos::delay_ms(300);
                led.set_low().unwrap();
                FreeRtos::delay_ms(300);
            }
            mqtt_opt = None;  // Disable MQTT
        }
    }
    
    // 3 ORANGE blinks = continuing without MQTT (if it failed)
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
    
    // Console is now gone - LED only from here
    
    // Configure GPS to 5 Hz
    let ubx_rate_5hz: [u8; 14] = [
        0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
        0xC8, 0x00, 0x01, 0x00, 0x01, 0x00,
        0xDE, 0x6A
    ];
    gps_uart.write(&ubx_rate_5hz).ok();
    FreeRtos::delay_ms(100);
    
    // Yellow flash = starting calibration
    led.yellow().unwrap();
    FreeRtos::delay_ms(500);
    led.set_low().unwrap();
    FreeRtos::delay_ms(500);
    
    // Calibrate IMU
    if let Some(ref mut mqtt) = mqtt_opt {
        mqtt.publish_status("calib_start", Some(0.0)).ok();
    }
    
    let bias = calibrate_imu_optional(&mut imu_uart, &mut led, mqtt_opt.as_mut())
        .expect("Calibration failed");
    
    if let Some(ref mut mqtt) = mqtt_opt {
        mqtt.publish_status("calib_done", Some(1.0)).ok();
    }
    
    let mut imu_parser = Wt901Parser::new();
    imu_parser.set_bias(bias);
    let mut gps_parser = NmeaParser::new();
    let mut ekf = Ekf::new();  // Initialize EKF
    
    if let Some(ref mut mqtt) = mqtt_opt {
        mqtt.publish_status("waiting_gps", None).ok();
    }
    
    // Main loop - cyan heartbeat every 2s
    let mut last_mqtt_ms = 0u32;
    let mut last_imu_us = unsafe { esp_idf_svc::sys::esp_timer_get_time() as u64 };
    let mut gps_warmup_announced = false;
    let mut gps_fix_announced = false;
    let mut buf = [0u8; 1];
    
    loop {
        let now_ms = unsafe { (esp_idf_svc::sys::esp_timer_get_time() / 1000) as u32 };
        let now_us = unsafe { esp_idf_svc::sys::esp_timer_get_time() as u64 };
        
        // Read IMU and run EKF prediction
        if imu_uart.read(&mut buf, 0).unwrap_or(0) > 0 {
            if let Some(packet_type) = imu_parser.feed_byte(buf[0], now_us) {
                // On accelerometer packet, run EKF prediction
                if packet_type == 0x51 {
                    let dt = (now_us - last_imu_us) as f32 * 1e-6;
                    last_imu_us = now_us;
                    
                    if dt > 5e-4 && dt < 0.05 {  // Sanity check dt
                        let (ax_corr, ay_corr, az_corr) = imu_parser.get_accel_corrected();
                        
                        // Remove gravity from body-frame acceleration
                        let (ax_b, ay_b, az_b) = remove_gravity(
                            ax_corr,
                            ay_corr,
                            az_corr,
                            imu_parser.data.roll,
                            imu_parser.data.pitch,
                        );
                        
                        // Transform to earth frame
                        let (ax_e, ay_e) = body_to_earth(
                            ax_b,
                            ay_b,
                            az_b,
                            imu_parser.data.roll,
                            imu_parser.data.pitch,
                            ekf.yaw(),  // Use EKF yaw
                        );
                        
                        // EKF prediction step
                        ekf.predict(ax_e, ay_e, imu_parser.data.wz, dt);
                    }
                }
                
                // On angle packet, update EKF yaw
                if packet_type == 0x53 {
                    let yaw_mag = imu_parser.data.yaw.to_radians();
                    ekf.update_yaw(yaw_mag);
                }
            }
        }
        
        // Read GPS and update EKF
        if gps_uart.read(&mut buf, 0).unwrap_or(0) > 0 {
            if gps_parser.feed_byte(buf[0]) {
                // GPS warmup phase - collecting initial fixes
                if !gps_parser.is_warmed_up() && !gps_warmup_announced {
                    if let Some(ref mut mqtt) = mqtt_opt {
                        let progress = gps_parser.warmup_progress();
                        mqtt.publish_status("gps_warmup", Some(progress)).ok();
                    }
                    // Yellow LED pulses during warmup
                    if now_ms % 500 < 250 {
                        led.yellow().ok();
                    } else {
                        led.set_low().ok();
                    }
                }
                
                // GPS warmup complete - announce lock
                if gps_parser.is_warmed_up() && !gps_fix_announced {
                    if let Some(ref mut mqtt) = mqtt_opt {
                        mqtt.publish_status("gps_lock", None).ok();
                    }
                    // Green flash = GPS locked
                    for _ in 0..3 {
                        led.green().unwrap();
                        FreeRtos::delay_ms(100);
                        led.set_low().unwrap();
                        FreeRtos::delay_ms(100);
                    }
                    gps_warmup_announced = true;
                    gps_fix_announced = true;
                }
                
                // Update EKF with GPS measurements (only after warmup)
                if gps_parser.last_fix.valid {
                    if let Some((x, y)) = gps_parser.to_local_coords() {
                        ekf.update_position(x, y);
                    }
                    
                    if let Some((vx, vy)) = gps_parser.get_velocity_enu() {
                        let speed = (vx * vx + vy * vy).sqrt();
                        ekf.update_velocity(vx, vy);
                        ekf.update_speed(speed);
                        
                        // Zero velocity update if stationary
                        let (ax_corr, ay_corr, _) = imu_parser.get_accel_corrected();
                        if is_stationary(ax_corr, ay_corr, imu_parser.data.wz, speed) {
                            ekf.zupt();
                            // Also update biases when stationary
                            let (ax_b, ay_b, _) = remove_gravity(
                                ax_corr, ay_corr, imu_parser.data.az,
                                imu_parser.data.roll, imu_parser.data.pitch,
                            );
                            let (ax_e, ay_e) = body_to_earth(
                                ax_b, ay_b, 0.0,
                                imu_parser.data.roll, imu_parser.data.pitch, ekf.yaw(),
                            );
                            ekf.update_bias(ax_e, ay_e);
                        }
                    }
                }
            }
        }
        
        // Publish telemetry at 5 Hz if MQTT available
        if mqtt_opt.is_some() && now_ms - last_mqtt_ms >= 200 {
            publish_telemetry_ekf(mqtt_opt.as_mut().unwrap(), &imu_parser, &gps_parser, &ekf);
            last_mqtt_ms = now_ms;
        }
        
        // Heartbeat LED - CYAN pulse (only after GPS locked)
        if gps_fix_announced {
            if now_ms % 2000 < 100 {
                led.cyan().ok();
            } else {
                led.set_low().ok();
            }
        }
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

/// Check if vehicle is stationary
fn is_stationary(ax: f32, ay: f32, wz: f32, gps_speed: f32) -> bool {
    const ACC_THR: f32 = 0.08 * 9.80665; // 0.08 g
    const WZ_THR: f32 = 5.0 * 0.017453293; // 5 degrees/s in rad/s
    
    ax.abs() < ACC_THR && ay.abs() < ACC_THR && wz.abs() < WZ_THR && gps_speed < 0.15
}

fn publish_telemetry_ekf(mqtt: &mut MqttClient, imu: &Wt901Parser, gps: &NmeaParser, ekf: &Ekf) {
    use serde_json::json;
    
    let (ax, ay, az) = imu.get_accel_corrected();
    let (ekf_x, ekf_y) = ekf.position();
    let (ekf_vx, ekf_vy) = ekf.velocity();
    let ekf_speed_kmh = ekf.speed() * 3.6;
    
    let payload = json!({
        "t": unsafe { esp_idf_svc::sys::esp_timer_get_time() / 1000 },
        "ax": ax, "ay": ay, "az": az,
        "wz": imu.data.wz,
        "roll": imu.data.roll.to_radians(),
        "pitch": imu.data.pitch.to_radians(),
        "yaw": ekf.yaw(),  // Use EKF yaw
        "x": ekf_x,        // Use EKF position
        "y": ekf_y,
        "vx": ekf_vx,      // Use EKF velocity
        "vy": ekf_vy,
        "kmh": ekf_speed_kmh,  // Use EKF speed
        "lat": if gps.last_fix.valid { gps.last_fix.lat } else { f64::NAN },
        "lon": if gps.last_fix.valid { gps.last_fix.lon } else { f64::NAN },
        "gps_warmup": gps.is_warmed_up(),  // Add warmup status
        "mode": "IDLE"
    }).to_string();
    
    mqtt.publish("car/telemetry", &payload, false).ok();
}

// Old telemetry function - keeping for reference but not used
#[allow(dead_code)]
fn publish_telemetry(mqtt: &mut MqttClient, imu: &Wt901Parser, gps: &NmeaParser) {
    use serde_json::json;
    
    let (ax, ay, az) = imu.get_accel_corrected();
    let (x, y) = gps.to_local_coords().unwrap_or((0.0, 0.0));
    let (vx, vy) = gps.get_velocity_enu().unwrap_or((0.0, 0.0));
    let speed_kmh = (vx * vx + vy * vy).sqrt() * 3.6;
    
    let payload = json!({
        "t": unsafe { esp_idf_svc::sys::esp_timer_get_time() / 1000 },
        "ax": ax, "ay": ay, "az": az,
        "wz": imu.data.wz,
        "roll": imu.data.roll.to_radians(),
        "pitch": imu.data.pitch.to_radians(),
        "yaw": imu.data.yaw.to_radians(),
        "x": x, "y": y,
        "vx": vx, "vy": vy,
        "kmh": speed_kmh,
        "lat": if gps.last_fix.valid { gps.last_fix.lat } else { f64::NAN },
        "lon": if gps.last_fix.valid { gps.last_fix.lon } else { f64::NAN },
        "mode": "IDLE"
    }).to_string();
    
    mqtt.publish("car/telemetry", &payload, false).ok();
}
