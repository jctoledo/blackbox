//! UBX Protocol support for u-blox GPS configuration
//!
//! Provides command generation for configuring u-blox GPS modules.
//! - NEO-6M: Uses legacy CFG-PRT, CFG-RATE, CFG-NAV5 messages
//! - NEO-M9N: Uses new CFG-VALSET interface (0x06 0x8A)
//!
//! Commands are generated as byte arrays that can be sent over UART.

/// UBX message header
const UBX_SYNC_1: u8 = 0xB5;
const UBX_SYNC_2: u8 = 0x62;

/// UBX message classes
const UBX_CLASS_CFG: u8 = 0x06;

/// UBX CFG message IDs
const UBX_CFG_VALSET: u8 = 0x8A; // Configuration value set (M9+)

/// CFG-VALSET layers (bitfield)
const LAYER_RAM: u8 = 0x01;
const LAYER_BBR: u8 = 0x02; // Battery-backed RAM
const LAYER_FLASH: u8 = 0x04; // Flash memory (persistent)

/// Configuration keys for NEO-M9N (from u-blox interface description)
/// Keys are 32-bit: [size:3][group:13][reserved:4][item:12]
const CFG_UART1_BAUDRATE: u32 = 0x40520001; // U4: UART1 baud rate
const CFG_RATE_MEAS: u32 = 0x30210001; // U2: Measurement period (ms)
const CFG_RATE_NAV: u32 = 0x30210002; // U2: Navigation rate (cycles)
const CFG_NAVSPG_DYNMODEL: u32 = 0x20110021; // E1: Dynamic platform model

/// Dynamic platform model
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[repr(u8)]
pub enum DynamicModel {
    Portable = 0,
    Stationary = 2,
    Pedestrian = 3,
    #[default]
    Automotive = 4,
    Sea = 5,
    Airborne1g = 6,
    Airborne2g = 7,
    Airborne4g = 8,
    Wrist = 9, // M9N specific
    Bike = 10, // M9N specific
}

/// Calculate UBX checksum (Fletcher checksum)
fn ubx_checksum(data: &[u8]) -> (u8, u8) {
    let mut ck_a: u8 = 0;
    let mut ck_b: u8 = 0;

    for &byte in data {
        ck_a = ck_a.wrapping_add(byte);
        ck_b = ck_b.wrapping_add(ck_a);
    }

    (ck_a, ck_b)
}

/// Build a complete UBX message with header and checksum
fn build_ubx_message(class: u8, id: u8, payload: &[u8], buffer: &mut [u8]) -> usize {
    let payload_len = payload.len() as u16;

    // Header
    buffer[0] = UBX_SYNC_1;
    buffer[1] = UBX_SYNC_2;
    buffer[2] = class;
    buffer[3] = id;
    buffer[4] = (payload_len & 0xFF) as u8; // Length LSB
    buffer[5] = ((payload_len >> 8) & 0xFF) as u8; // Length MSB

    // Payload
    buffer[6..6 + payload.len()].copy_from_slice(payload);

    // Checksum (over class, id, length, payload)
    let checksum_data = &buffer[2..6 + payload.len()];
    let (ck_a, ck_b) = ubx_checksum(checksum_data);
    buffer[6 + payload.len()] = ck_a;
    buffer[7 + payload.len()] = ck_b;

    8 + payload.len() // Total message length
}

/// Write a 32-bit key to buffer (little-endian)
fn write_key(buffer: &mut [u8], offset: usize, key: u32) {
    buffer[offset] = (key & 0xFF) as u8;
    buffer[offset + 1] = ((key >> 8) & 0xFF) as u8;
    buffer[offset + 2] = ((key >> 16) & 0xFF) as u8;
    buffer[offset + 3] = ((key >> 24) & 0xFF) as u8;
}

/// Write a 32-bit value to buffer (little-endian)
fn write_u32(buffer: &mut [u8], offset: usize, value: u32) {
    buffer[offset] = (value & 0xFF) as u8;
    buffer[offset + 1] = ((value >> 8) & 0xFF) as u8;
    buffer[offset + 2] = ((value >> 16) & 0xFF) as u8;
    buffer[offset + 3] = ((value >> 24) & 0xFF) as u8;
}

/// Write a 16-bit value to buffer (little-endian)
fn write_u16(buffer: &mut [u8], offset: usize, value: u16) {
    buffer[offset] = (value & 0xFF) as u8;
    buffer[offset + 1] = ((value >> 8) & 0xFF) as u8;
}

/// UBX command generator for GPS configuration
pub struct UbxCommands;

impl UbxCommands {
    /// Generate UBX-CFG-VALSET to set measurement rate (M9N)
    ///
    /// # Arguments
    /// * `rate_hz` - Desired update rate in Hz (1-25 for NEO-M9N)
    /// * `buffer` - Output buffer (must be at least 24 bytes)
    ///
    /// # Returns
    /// Number of bytes written to buffer
    pub fn cfg_valset_rate(rate_hz: u8, buffer: &mut [u8]) -> usize {
        // Measurement period in ms
        let meas_rate_ms: u16 = if rate_hz > 0 {
            1000 / rate_hz as u16
        } else {
            1000
        };

        // CFG-VALSET payload:
        // [0]: version = 0
        // [1]: layers (RAM + BBR + Flash = 0x07)
        // [2-3]: reserved
        // [4-7]: key (CFG-RATE-MEAS)
        // [8-9]: value (U2)
        // [10-13]: key (CFG-RATE-NAV)
        // [14-15]: value (U2)
        let mut payload = [0u8; 16];
        payload[0] = 0x00; // version
        payload[1] = LAYER_RAM | LAYER_BBR | LAYER_FLASH;
        // payload[2-3] reserved, already 0

        // CFG-RATE-MEAS (measurement period)
        write_key(&mut payload, 4, CFG_RATE_MEAS);
        write_u16(&mut payload, 8, meas_rate_ms);

        // CFG-RATE-NAV (nav rate = 1, every measurement)
        write_key(&mut payload, 10, CFG_RATE_NAV);
        write_u16(&mut payload, 14, 1);

        build_ubx_message(UBX_CLASS_CFG, UBX_CFG_VALSET, &payload, buffer)
    }

    /// Generate UBX-CFG-VALSET to set dynamic platform model (M9N)
    ///
    /// # Arguments
    /// * `model` - Dynamic platform model (e.g., Automotive)
    /// * `buffer` - Output buffer (must be at least 17 bytes)
    ///
    /// # Returns
    /// Number of bytes written to buffer
    pub fn cfg_valset_dynmodel(model: DynamicModel, buffer: &mut [u8]) -> usize {
        // CFG-VALSET payload:
        // [0]: version = 0
        // [1]: layers
        // [2-3]: reserved
        // [4-7]: key (CFG-NAVSPG-DYNMODEL)
        // [8]: value (E1 = 1 byte)
        let mut payload = [0u8; 9];
        payload[0] = 0x00; // version
        payload[1] = LAYER_RAM | LAYER_BBR | LAYER_FLASH;

        write_key(&mut payload, 4, CFG_NAVSPG_DYNMODEL);
        payload[8] = model as u8;

        build_ubx_message(UBX_CLASS_CFG, UBX_CFG_VALSET, &payload, buffer)
    }

    /// Generate UBX-CFG-VALSET to set UART1 baud rate (M9N)
    ///
    /// # Arguments
    /// * `baud_rate` - Desired baud rate (9600, 38400, 115200, etc.)
    /// * `buffer` - Output buffer (must be at least 20 bytes)
    ///
    /// # Returns
    /// Number of bytes written to buffer
    pub fn cfg_valset_baudrate(baud_rate: u32, buffer: &mut [u8]) -> usize {
        // CFG-VALSET payload:
        // [0]: version = 0
        // [1]: layers
        // [2-3]: reserved
        // [4-7]: key (CFG-UART1-BAUDRATE)
        // [8-11]: value (U4)
        let mut payload = [0u8; 12];
        payload[0] = 0x00; // version
        payload[1] = LAYER_RAM | LAYER_BBR | LAYER_FLASH;

        write_key(&mut payload, 4, CFG_UART1_BAUDRATE);
        write_u32(&mut payload, 8, baud_rate);

        build_ubx_message(UBX_CLASS_CFG, UBX_CFG_VALSET, &payload, buffer)
    }

    /// Generate combined UBX-CFG-VALSET for rate + dynmodel (M9N)
    ///
    /// Sets measurement rate and dynamic model in a single message.
    /// More efficient than separate messages.
    ///
    /// # Arguments
    /// * `rate_hz` - Desired update rate in Hz
    /// * `model` - Dynamic platform model
    /// * `buffer` - Output buffer (must be at least 30 bytes)
    ///
    /// # Returns
    /// Number of bytes written to buffer
    pub fn cfg_valset_rate_and_model(rate_hz: u8, model: DynamicModel, buffer: &mut [u8]) -> usize {
        let meas_rate_ms: u16 = if rate_hz > 0 {
            1000 / rate_hz as u16
        } else {
            1000
        };

        // Combined payload with 3 key-value pairs:
        // - CFG-RATE-MEAS (U2)
        // - CFG-RATE-NAV (U2)
        // - CFG-NAVSPG-DYNMODEL (E1)
        let mut payload = [0u8; 21];
        payload[0] = 0x00; // version
        payload[1] = LAYER_RAM | LAYER_BBR | LAYER_FLASH;

        let mut offset = 4;

        // CFG-RATE-MEAS
        write_key(&mut payload, offset, CFG_RATE_MEAS);
        offset += 4;
        write_u16(&mut payload, offset, meas_rate_ms);
        offset += 2;

        // CFG-RATE-NAV
        write_key(&mut payload, offset, CFG_RATE_NAV);
        offset += 4;
        write_u16(&mut payload, offset, 1);
        offset += 2;

        // CFG-NAVSPG-DYNMODEL
        write_key(&mut payload, offset, CFG_NAVSPG_DYNMODEL);
        offset += 4;
        payload[offset] = model as u8;

        build_ubx_message(UBX_CLASS_CFG, UBX_CFG_VALSET, &payload, buffer)
    }
}

/// Complete GPS initialization sequence for NEO-M9N in automotive mode
///
/// Generates UBX-CFG-VALSET commands for optimal automotive telemetry.
/// Commands should be sent with ~100ms delay between each.
///
/// **IMPORTANT**: The baud rate command must be sent at the GPS's CURRENT baud rate
/// (factory default: 38400). After sending, switch the ESP32 UART to the new rate.
///
/// # Arguments
/// * `rate_hz` - Desired update rate (recommended: 10 for automotive, max 25)
/// * `target_baud` - Target baud rate (115200 recommended for 10Hz+)
/// * `buffer` - Output buffer (must be at least 128 bytes)
///
/// # Returns
/// Array of (offset, length) pairs for each command in the buffer
///
/// # Initialization Order
/// 1. Set baud rate (sent at current/factory baud)
/// 2. Set rate + dynamic model (sent at new baud rate)
pub fn generate_init_sequence(
    rate_hz: u8,
    target_baud: u32,
    buffer: &mut [u8; 128],
) -> [(usize, usize); 2] {
    let mut offset = 0;

    // 1. Set baud rate (must send at current GPS baud rate first)
    let baud_len = UbxCommands::cfg_valset_baudrate(target_baud, &mut buffer[offset..]);
    let baud_cmd = (offset, baud_len);
    offset += baud_len;

    // 2. Set rate + automotive dynamic model (combined for efficiency)
    let config_len = UbxCommands::cfg_valset_rate_and_model(
        rate_hz,
        DynamicModel::Automotive,
        &mut buffer[offset..],
    );
    let config_cmd = (offset, config_len);

    [baud_cmd, config_cmd]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ubx_checksum() {
        // Test vector from u-blox documentation
        let data = [0x06, 0x01, 0x02, 0x00, 0x01, 0x01];
        let (ck_a, ck_b) = ubx_checksum(&data);
        assert_eq!(ck_a, 0x0B);
        assert_eq!(ck_b, 0x34);
    }

    #[test]
    fn test_cfg_valset_rate_10hz() {
        let mut buffer = [0u8; 32];
        let len = UbxCommands::cfg_valset_rate(10, &mut buffer);

        // Header check
        assert_eq!(buffer[0], 0xB5);
        assert_eq!(buffer[1], 0x62);
        assert_eq!(buffer[2], 0x06); // CFG class
        assert_eq!(buffer[3], 0x8A); // VALSET id

        // Payload starts at offset 6
        assert_eq!(buffer[6], 0x00); // version
        assert_eq!(buffer[7], 0x07); // layers (RAM | BBR | Flash)

        // CFG-RATE-MEAS key at offset 10
        assert_eq!(buffer[10], 0x01); // key LSB
        assert_eq!(buffer[11], 0x00);
        assert_eq!(buffer[12], 0x21);
        assert_eq!(buffer[13], 0x30); // key MSB

        // Value (100ms = 10Hz) at offset 14
        assert_eq!(buffer[14], 100); // 100 LSB
        assert_eq!(buffer[15], 0); // 100 MSB

        assert!(len > 0);
    }

    #[test]
    fn test_cfg_valset_dynmodel_automotive() {
        let mut buffer = [0u8; 20];
        let len = UbxCommands::cfg_valset_dynmodel(DynamicModel::Automotive, &mut buffer);

        // Header check
        assert_eq!(buffer[0], 0xB5);
        assert_eq!(buffer[1], 0x62);
        assert_eq!(buffer[2], 0x06); // CFG class
        assert_eq!(buffer[3], 0x8A); // VALSET id

        // CFG-NAVSPG-DYNMODEL key at offset 10
        assert_eq!(buffer[10], 0x21); // key LSB
        assert_eq!(buffer[11], 0x00);
        assert_eq!(buffer[12], 0x11);
        assert_eq!(buffer[13], 0x20); // key MSB

        // Value (Automotive = 4) at offset 14
        assert_eq!(buffer[14], 4);

        assert!(len > 0);
    }

    #[test]
    fn test_cfg_valset_baudrate() {
        let mut buffer = [0u8; 24];
        let len = UbxCommands::cfg_valset_baudrate(115200, &mut buffer);

        // Header check
        assert_eq!(buffer[0], 0xB5);
        assert_eq!(buffer[1], 0x62);
        assert_eq!(buffer[2], 0x06); // CFG class
        assert_eq!(buffer[3], 0x8A); // VALSET id

        // CFG-UART1-BAUDRATE key at offset 10
        assert_eq!(buffer[10], 0x01); // key LSB
        assert_eq!(buffer[11], 0x00);
        assert_eq!(buffer[12], 0x52);
        assert_eq!(buffer[13], 0x40); // key MSB

        // Value (115200) at offset 14, little-endian
        let baud_bytes = 115200u32.to_le_bytes();
        assert_eq!(buffer[14], baud_bytes[0]);
        assert_eq!(buffer[15], baud_bytes[1]);
        assert_eq!(buffer[16], baud_bytes[2]);
        assert_eq!(buffer[17], baud_bytes[3]);

        assert!(len > 0);
    }

    #[test]
    fn test_generate_init_sequence() {
        let mut buffer = [0u8; 128];
        let commands = generate_init_sequence(10, 115200, &mut buffer);

        // Should have 2 commands
        assert_eq!(commands.len(), 2);

        // First command is baud rate
        let (baud_off, baud_len) = commands[0];
        assert_eq!(baud_off, 0);
        assert!(baud_len > 0);

        // Second command is rate + dynmodel
        let (config_off, config_len) = commands[1];
        assert!(config_off > 0);
        assert!(config_len > 0);
    }
}
