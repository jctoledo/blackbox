//! WT901 IMU Configuration Tool
//!
//! This firmware configures a WT901 IMU for 200Hz output at 115200 baud.
//! Flash this to your ESP32-C3, watch the serial output, then flash your
//! normal blackbox firmware back.
//!
//! The configuration is saved to the WT901's EEPROM and persists across
//! power cycles.

use esp_idf_hal::{
    delay::FreeRtos,
    gpio::{Gpio8, Output, PinDriver},
    peripherals::Peripherals,
    uart::{config::Config, UartDriver},
};

// WT901 Configuration Commands (from datasheet)
// Format: 0xFF 0xAA [register] [value_low] [value_high]

const CMD_UNLOCK: [u8; 5] = [0xFF, 0xAA, 0x69, 0x88, 0xB5]; // Unlock for configuration
const CMD_SAVE: [u8; 5] = [0xFF, 0xAA, 0x00, 0x00, 0x00]; // Save to EEPROM

// Rate register (0x03): 0x0B = 200Hz, 0x09 = 100Hz, 0x08 = 50Hz
const CMD_RATE_200HZ: [u8; 5] = [0xFF, 0xAA, 0x03, 0x0B, 0x00];

// Baud rate register (0x04): 0x06 = 115200, 0x02 = 9600
const CMD_BAUD_115200: [u8; 5] = [0xFF, 0xAA, 0x04, 0x06, 0x00];

// WT901 packet header
const WT901_HEADER: u8 = 0x55;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    println!();
    println!("============================================================");
    println!("         WT901 IMU Configuration Tool");
    println!("         Target: 200Hz @ 115200 baud");
    println!("============================================================");
    println!();

    let peripherals = Peripherals::take().unwrap();

    // Set up LED for status feedback (GPIO8 on ESP32-C3 DevKit)
    let mut led = PinDriver::output(peripherals.pins.gpio8).ok();

    // Blink LED to show we're starting
    blink_led(&mut led, 3, 200);

    println!("Pin connections expected:");
    println!("  ESP32 GPIO18 (TX) -> WT901 RX");
    println!("  ESP32 GPIO19 (RX) <- WT901 TX");
    println!("  ESP32 GND         <> WT901 GND");
    println!("  ESP32 3V3         -> WT901 VCC");
    println!();

    // First try at 9600 baud (factory default)
    println!("------------------------------------------------------------");
    println!("Trying to connect at 9600 baud (factory default)...");

    let uart_config_9600 = Config::new().baudrate(9600.into());
    let uart = UartDriver::new(
        peripherals.uart1,
        peripherals.pins.gpio18,
        peripherals.pins.gpio19,
        Option::<esp_idf_hal::gpio::Gpio0>::None,
        Option::<esp_idf_hal::gpio::Gpio1>::None,
        &uart_config_9600,
    )
    .expect("Failed to initialize UART");

    // Clear any pending data
    FreeRtos::delay_ms(100);
    let mut discard = [0u8; 256];
    let _ = uart.read(&mut discard, 0);

    // Check if we're receiving valid WT901 data at 9600
    print!("  Checking for IMU data... ");
    let (found_at_9600, bytes_at_9600) = check_for_imu_data(&uart);

    if found_at_9600 {
        println!("found! ({} bytes)", bytes_at_9600);
        println!();
        println!("  Configuring IMU:");

        // Unlock
        send_command(&uart, &CMD_UNLOCK, "Unlock register access");

        // Set baud rate to 115200
        send_command(&uart, &CMD_BAUD_115200, "Set baud rate to 115200");

        // The IMU is now at 115200, but our UART is still at 9600
        // We need to reconfigure our UART. Since we can't re-take peripherals,
        // we'll use the esp-idf API directly to change baud rate.
        println!("    Switching UART to 115200 baud...");
        drop(uart);
        FreeRtos::delay_ms(300);

        // Reconfigure UART at 115200 using esp-idf-sys directly
        unsafe {
            esp_idf_svc::sys::uart_set_baudrate(1, 115200);
        }

        FreeRtos::delay_ms(200);

        // Now send commands at 115200
        // Create a simple write function using esp-idf-sys
        send_command_raw(&CMD_UNLOCK, "Unlock (after baud change)");
        send_command_raw(&CMD_RATE_200HZ, "Set output rate to 200Hz");
        send_command_raw(&CMD_SAVE, "Save configuration to EEPROM");

        // Verify
        verify_configuration_raw(&mut led);
        return;
    }

    println!("no valid data ({} bytes)", bytes_at_9600);
    drop(uart);

    // Try at 115200 (may already be configured)
    println!();
    println!("------------------------------------------------------------");
    println!("Trying to connect at 115200 baud...");

    FreeRtos::delay_ms(200);

    // Reconfigure UART at 115200
    unsafe {
        esp_idf_svc::sys::uart_set_baudrate(1, 115200);
    }

    FreeRtos::delay_ms(100);

    print!("  Checking for IMU data... ");
    let (found_at_115200, bytes_at_115200) = check_for_imu_data_raw();

    if found_at_115200 {
        println!("found! ({} bytes)", bytes_at_115200);
        println!();
        println!("  IMU already at 115200 baud. Configuring rate:");

        send_command_raw(&CMD_UNLOCK, "Unlock register access");
        send_command_raw(&CMD_RATE_200HZ, "Set output rate to 200Hz");
        send_command_raw(&CMD_SAVE, "Save configuration to EEPROM");

        verify_configuration_raw(&mut led);
        return;
    }

    println!("no valid data ({} bytes)", bytes_at_115200);

    // Failed to find IMU at any baud rate
    println!();
    println!("============================================================");
    println!("  ERROR: Could not communicate with WT901");
    println!("------------------------------------------------------------");
    println!("  Troubleshooting:");
    println!("  1. Check wiring (TX<->RX crossed correctly)");
    println!("  2. Ensure WT901 has power (LED should be on)");
    println!("  3. Try unplugging and replugging the WT901");
    println!("  4. Make sure you're using GPIO18/19");
    println!("============================================================");

    // Error blink pattern
    loop {
        blink_led(&mut led, 5, 100);
        FreeRtos::delay_ms(1000);
    }
}

fn check_for_imu_data(uart: &UartDriver) -> (bool, usize) {
    let mut buf = [0u8; 128];
    let mut found_header = false;
    let mut total_bytes = 0;

    // Read for 500ms
    for _ in 0..50 {
        if let Ok(len) = uart.read(&mut buf, 0) {
            total_bytes += len;
            if buf[..len].contains(&WT901_HEADER) {
                found_header = true;
            }
        }
        FreeRtos::delay_ms(10);
    }

    (found_header, total_bytes)
}

fn check_for_imu_data_raw() -> (bool, usize) {
    let mut buf = [0u8; 128];
    let mut found_header = false;
    let mut total_bytes = 0usize;

    // Read for 500ms
    for _ in 0..50 {
        let len = unsafe {
            esp_idf_svc::sys::uart_read_bytes(
                1,
                buf.as_mut_ptr() as *mut _,
                buf.len() as u32,
                0,
            )
        };
        if len > 0 {
            total_bytes += len as usize;
            if buf[..len as usize].contains(&WT901_HEADER) {
                found_header = true;
            }
        }
        FreeRtos::delay_ms(10);
    }

    (found_header, total_bytes)
}

fn send_command(uart: &UartDriver, cmd: &[u8], description: &str) {
    print!("    {} ... ", description);
    uart.write(cmd).ok();
    uart.wait_tx_done(100).ok();
    FreeRtos::delay_ms(200);
    println!("OK");
}

fn send_command_raw(cmd: &[u8], description: &str) {
    print!("    {} ... ", description);
    unsafe {
        esp_idf_svc::sys::uart_write_bytes(1, cmd.as_ptr() as *const _, cmd.len());
        esp_idf_svc::sys::uart_wait_tx_done(1, 100);
    }
    FreeRtos::delay_ms(200);
    println!("OK");
}

fn verify_configuration_raw(led: &mut Option<PinDriver<Gpio8, Output>>) {
    println!();
    println!("  Verifying configuration (counting packets for 2 seconds)...");

    // Clear buffer
    let mut discard = [0u8; 512];
    unsafe {
        esp_idf_svc::sys::uart_read_bytes(1, discard.as_mut_ptr() as *mut _, discard.len() as u32, 0);
    }
    FreeRtos::delay_ms(100);

    // Count packets for 2 seconds
    let mut buf = [0u8; 128];
    let mut packet_count = 0u32;
    let mut byte_count = 0u32;

    for _ in 0..200 {
        // 200 * 10ms = 2 seconds
        let len = unsafe {
            esp_idf_svc::sys::uart_read_bytes(
                1,
                buf.as_mut_ptr() as *mut _,
                buf.len() as u32,
                0,
            )
        };
        if len > 0 {
            byte_count += len as u32;
            packet_count += buf[..len as usize]
                .iter()
                .filter(|&&b| b == WT901_HEADER)
                .count() as u32;
        }
        FreeRtos::delay_ms(10);
    }

    // WT901 sends 3 packet types per cycle (accel, gyro, angle)
    let cycles = packet_count / 3;
    let hz = cycles / 2; // 2 seconds of data

    println!();
    println!("  Results:");
    println!("    Bytes received: {}", byte_count);
    println!("    Packets counted: {}", packet_count);
    println!("    Estimated rate: ~{} Hz", hz);

    println!();
    if hz >= 180 && hz <= 220 {
        println!("============================================================");
        println!("  SUCCESS! WT901 configured for 200Hz @ 115200 baud");
        println!("------------------------------------------------------------");
        println!("  Settings saved to EEPROM - they persist across power");
        println!("  cycles. You can now flash your normal blackbox firmware.");
        println!();
        println!("  IMPORTANT: Update your blackbox firmware to use 115200");
        println!("  baud for the IMU UART (currently may be set to 9600).");
        println!("============================================================");

        // Success blink pattern (slow)
        loop {
            blink_led(led, 1, 500);
            FreeRtos::delay_ms(1000);
        }
    } else if hz >= 90 && hz < 180 {
        println!("============================================================");
        println!("  PARTIAL SUCCESS: Got ~{}Hz instead of 200Hz", hz);
        println!("------------------------------------------------------------");
        println!("  The IMU is responding but may not have saved settings.");
        println!("  Try power cycling the WT901 and running this again.");
        println!("============================================================");

        // Warning blink pattern
        loop {
            blink_led(led, 2, 300);
            FreeRtos::delay_ms(1000);
        }
    } else if hz > 0 {
        println!("============================================================");
        println!("  LOW RATE: Only ~{}Hz detected", hz);
        println!("------------------------------------------------------------");
        println!("  Configuration may have failed. Power cycle the WT901");
        println!("  and try again. If this persists, the IMU may need");
        println!("  manual configuration via its Windows software.");
        println!("============================================================");

        loop {
            blink_led(led, 3, 200);
            FreeRtos::delay_ms(1000);
        }
    } else {
        println!("============================================================");
        println!("  VERIFICATION FAILED: No data after configuration");
        println!("------------------------------------------------------------");
        println!("  The IMU stopped responding. This can happen if baud");
        println!("  rate changed but we couldn't reconnect. Power cycle");
        println!("  and try again.");
        println!("============================================================");

        loop {
            blink_led(led, 5, 100);
            FreeRtos::delay_ms(1000);
        }
    }
}

fn blink_led(led: &mut Option<PinDriver<Gpio8, Output>>, times: u32, delay_ms: u32) {
    if let Some(ref mut pin) = led {
        for _ in 0..times {
            pin.set_high().ok();
            FreeRtos::delay_ms(delay_ms);
            pin.set_low().ok();
            FreeRtos::delay_ms(delay_ms);
        }
    }
}
