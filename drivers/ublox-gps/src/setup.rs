//! Bounded M9N startup with RAM-only configuration and receiver readback.
//! Protocol keys/frames: u-blox M9 SPG 4.04 interface description UBX-21022436.
//! No GPIOs, clocks, allocations or persistent receiver writes live here.

const UART_BAUD: u32 = 0x40520001;
const MAX_FRAME: usize = 256;
const RESPONSE_MS: u64 = 1200;
const SUPPORTED_BAUDS: [u32; 8] = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600];

/// A blocking startup transport. Reads must return promptly with zero if empty.
pub trait Transport {
    fn set_baud(&mut self, baud: u32) -> Result<(), Error>;
    fn write(&mut self, data: &[u8]) -> Result<(), Error>;
    fn read(&mut self, data: &mut [u8]) -> Result<usize, Error>;
    fn now_ms(&self) -> u64;
    fn sleep_ms(&mut self, ms: u32);
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    Io,
    NoReceiver,
    Timeout,
    Rejected,
    Readback,
    InvalidRate,
    InvalidBaud,
    Bandwidth,
}
impl Error {
    pub fn label(self) -> &'static str {
        match self {
            Self::Io => "uart_io",
            Self::NoReceiver => "no_receiver",
            Self::Timeout => "timeout",
            Self::Rejected => "receiver_rejected",
            Self::Readback => "readback_mismatch",
            Self::InvalidRate => "invalid_rate",
            Self::InvalidBaud => "invalid_baud",
            Self::Bandwidth => "uart_bandwidth",
        }
    }
}

/// Actual controller baud and setup outcome, retained even on setup failure.
#[derive(Clone, Copy, Debug)]
pub struct Report {
    pub detected_baud: Option<u32>,
    pub uart_baud: u32,
    pub verified: bool,
    pub error: Option<Error>,
}

/// A single configuration item; value width follows the UBX key's size field.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Item(u32, u32);
fn width(key: u32) -> Option<usize> {
    match key >> 28 {
        1 | 2 => Some(1),
        3 => Some(2),
        4 => Some(4),
        _ => None,
    }
}

/// RMC every navigation epoch; GGA/GSA approximately once per second.
/// Disable default GSV/GLL/VTG traffic, and explicitly select continuous operation.
fn profile(rate: u8, baud: u32) -> Result<[Item; 13], Error> {
    if !(1..=25).contains(&rate) {
        return Err(Error::InvalidRate);
    }
    if !SUPPORTED_BAUDS.contains(&baud) {
        return Err(Error::InvalidBaud);
    }
    // Conservative 120-byte RMC/GGA and four GSA sentences, plus 20% headroom.
    // This budgets our profile, not unknown messages enabled by other tools.
    if (rate as u32 * 120 + 600) * 10 > baud * 8 / 10 {
        return Err(Error::Bandwidth);
    }
    Ok([
        Item(UART_BAUD, baud),
        Item(0x30210001, 1000 / rate as u32), // RATE-MEAS
        Item(0x30210002, 1),                  // RATE-NAV
        Item(0x20110021, 4),                  // NAVSPG-DYNMODEL: automotive
        Item(0x20d00001, 0),                  // PM-OPERATEMODE: full power
        Item(0x10740001, 1),                  // UART1OUTPROT-UBX: replies
        Item(0x10740002, 1),                  // UART1OUTPROT-NMEA: navigation
        Item(0x209100ac, 1),                  // RMC_UART1
        Item(0x209100bb, rate as u32),        // GGA_UART1
        Item(0x209100c0, rate as u32),        // GSA_UART1
        Item(0x209100c5, 0),                  // GSV_UART1
        Item(0x209100ca, 0),                  // GLL_UART1
        Item(0x209100b1, 0),                  // VTG_UART1
    ])
}

fn frame(class: u8, id: u8, payload: &[u8], out: &mut [u8; MAX_FRAME]) -> usize {
    let len = payload.len();
    out[..6].copy_from_slice(&[0xb5, 0x62, class, id, len as u8, (len >> 8) as u8]);
    out[6..6 + len].copy_from_slice(payload);
    let (a, b) = checksum(&out[2..6 + len]);
    out[6 + len] = a;
    out[7 + len] = b;
    len + 8
}
fn checksum(bytes: &[u8]) -> (u8, u8) {
    bytes.iter().fold((0u8, 0u8), |(a, b), byte| {
        let a = a.wrapping_add(*byte);
        (a, b.wrapping_add(a))
    })
}
fn valset(items: &[Item], out: &mut [u8; MAX_FRAME]) -> usize {
    let mut payload = [0u8; 192];
    payload[1] = 1; // RAM only: never write GPS flash on each boot.
    let mut len = 4;
    for Item(key, value) in items {
        payload[len..len + 4].copy_from_slice(&key.to_le_bytes());
        len += 4;
        let width = width(*key).expect("known configuration key");
        payload[len..len + width].copy_from_slice(&value.to_le_bytes()[..width]);
        len += width;
    }
    frame(6, 0x8a, &payload[..len], out)
}
fn valget(items: &[Item], out: &mut [u8; MAX_FRAME]) -> usize {
    let mut payload = [0u8; 128]; // version 0, RAM layer 0, position 0
    let mut len = 4;
    for Item(key, _) in items {
        payload[len..len + 4].copy_from_slice(&key.to_le_bytes());
        len += 4;
    }
    frame(6, 0x8b, &payload[..len], out)
}

/// Resynchronizing UBX parser; ignores NMEA and rejects oversized/corrupt frames.
struct Decoder {
    data: [u8; 1024], // MON-VER includes variable-length extension strings.
    len: usize,
}
impl Default for Decoder {
    fn default() -> Self {
        Self {
            data: [0; 1024],
            len: 0,
        }
    }
}
impl Decoder {
    fn push(&mut self, byte: u8) -> Option<(u8, u8, &[u8])> {
        if self.len == 0 && byte != 0xb5 {
            return None;
        }
        if self.len == 1 && byte != 0x62 {
            self.len = usize::from(byte == 0xb5);
            return None;
        }
        self.data[self.len] = byte;
        self.len += 1;
        if self.len < 6 {
            return None;
        }
        let payload = u16::from_le_bytes([self.data[4], self.data[5]]) as usize;
        let total = payload + 8;
        if total > self.data.len() {
            self.len = 0;
            return None;
        }
        if self.len != total {
            return None;
        }
        self.len = 0;
        if checksum(&self.data[2..total - 2]) != (self.data[total - 2], self.data[total - 1]) {
            return None;
        }
        Some((self.data[2], self.data[3], &self.data[6..6 + payload]))
    }
}

fn drain(io: &mut impl Transport) -> Result<(), Error> {
    let mut data = [0u8; 128];
    // Bounded even if the receiver continuously fills the UART.
    for _ in 0..64 {
        if io.read(&mut data)? == 0 {
            break;
        }
    }
    Ok(())
}
fn wait(
    io: &mut impl Transport,
    timeout_ms: u64,
    allow_nmea: bool,
    mut accept: impl FnMut(u8, u8, &[u8]) -> Option<Result<(), Error>>,
) -> Result<(), Error> {
    let start = io.now_ms();
    let mut decoder = Decoder::default();
    let mut data = [0u8; 128];
    let mut line = [0u8; 120];
    let mut line_len = 0;
    while io.now_ms().saturating_sub(start) < timeout_ms {
        let n = io.read(&mut data)?;
        for byte in &data[..n] {
            if allow_nmea {
                if *byte == b'$' {
                    line_len = 0;
                }
                if *byte == b'\n' {
                    if let Ok(text) = core::str::from_utf8(&line[..line_len]) {
                        if (text.starts_with("$GNRMC,")
                            || text.starts_with("$GPRMC,")
                            || text.starts_with("$GNGGA,")
                            || text.starts_with("$GPGGA,"))
                            && super::validate_nmea_checksum(text)
                        {
                            return Ok(());
                        }
                    }
                    line_len = 0;
                } else if line_len < line.len() {
                    line[line_len] = *byte;
                    line_len += 1;
                }
            }
            if let Some((class, id, payload)) = decoder.push(*byte) {
                if let Some(result) = accept(class, id, payload) {
                    return result;
                }
            }
        }
        if n == 0 {
            io.sleep_ms(2);
        }
    }
    Err(Error::Timeout)
}
fn send(io: &mut impl Transport, packet: &[u8]) -> Result<(), Error> {
    drain(io)?;
    io.write(packet)
}

fn probe(io: &mut impl Transport, target: u32) -> Result<u32, Error> {
    // Manufacturer UART table: UBX-19014286 R10, section 3.7.1. Prioritize the
    // requested/saved baud, factory baud, then historical local 9600 settings.
    let bauds = [
        target, 115200, 38400, 9600, 57600, 19200, 230400, 460800, 921600,
    ];
    for (i, baud) in bauds.iter().enumerate() {
        if bauds[..i].contains(baud) {
            continue;
        }
        io.set_baud(*baud)?;
        io.sleep_ms(20);
        let mut packet = [0u8; MAX_FRAME];
        let len = frame(0x0a, 4, &[], &mut packet); // MON-VER, works without a fix.
        send(io, &packet[..len])?;
        // Budget a full 1 KiB MON-VER frame (8N1) plus receiver/scheduler margin.
        // Fast ports get short windows; slow legacy ports can finish their reply.
        let timeout = (10_240_000u64.div_ceil(*baud as u64) + 150).max(250);
        if wait(io, timeout, true, |class, id, payload| {
            (class == 0x0a && id == 4 && payload.len() >= 40).then_some(Ok(()))
        })
        .is_ok()
        {
            return Ok(*baud);
        }
    }
    Err(Error::NoReceiver)
}

fn readback(io: &mut impl Transport, items: &[Item]) -> Result<(), Error> {
    let mut packet = [0u8; MAX_FRAME];
    let len = valget(items, &mut packet);
    send(io, &packet[..len])?;
    wait(io, RESPONSE_MS, false, |class, id, payload| {
        if class == 5 && id == 0 && payload == [6, 0x8b] {
            return Some(Err(Error::Rejected));
        }
        if class != 6 || id != 0x8b {
            return None;
        }
        Some(verify_items(payload, items))
    })
}
fn verify_items(payload: &[u8], items: &[Item]) -> Result<(), Error> {
    if payload.len() < 4 || payload[..4] != [1, 0, 0, 0] {
        return Err(Error::Readback);
    }
    let mut seen = 0u32;
    let mut pos = 4;
    while pos < payload.len() {
        let key = u32::from_le_bytes(
            payload
                .get(pos..pos + 4)
                .ok_or(Error::Readback)?
                .try_into()
                .unwrap(),
        );
        pos += 4;
        let width = width(key).ok_or(Error::Readback)?;
        let bytes = payload.get(pos..pos + width).ok_or(Error::Readback)?;
        pos += width;
        let index = items
            .iter()
            .position(|item| item.0 == key)
            .ok_or(Error::Readback)?;
        if seen & (1 << index) != 0 || bytes != &items[index].1.to_le_bytes()[..width] {
            return Err(Error::Readback);
        }
        seen |= 1 << index;
    }
    if seen == (1 << items.len()) - 1 {
        Ok(())
    } else {
        Err(Error::Readback)
    }
}

/// Detect, migrate baud if necessary, then ACK and read back the complete profile.
/// On failure re-probe to leave reception at a responsive baud; never claim success.
pub fn configure(io: &mut impl Transport, target: u32, rate: u8) -> Report {
    let mut report = Report {
        detected_baud: None,
        uart_baud: target,
        verified: false,
        error: None,
    };
    let result = (|| {
        let items = profile(rate, target)?;
        let detected = probe(io, target)?;
        report.detected_baud = Some(detected);
        report.uart_baud = detected;
        let mut packet = [0u8; MAX_FRAME];
        if detected != target {
            let len = valset(&[Item(UART_BAUD, target)], &mut packet);
            send(io, &packet[..len])?;
            // ACK can leave at the old baud. Confirm the migration by readback.
            io.sleep_ms(100);
            io.set_baud(target)?;
            report.uart_baud = target;
            readback(io, &[Item(UART_BAUD, target)])?;
        }
        let len = valset(&items, &mut packet);
        send(io, &packet[..len])?;
        wait(io, RESPONSE_MS, false, |class, id, payload| {
            (class == 5 && (id == 0 || id == 1) && payload == [6, 0x8a]).then_some(if id == 1 {
                Ok(())
            } else {
                Err(Error::Rejected)
            })
        })?;
        readback(io, &items)?;
        Ok(())
    })();
    match result {
        Ok(()) => report.verified = true,
        Err(error) => {
            report.error = Some(error);
            if report.detected_baud.is_some() {
                if let Ok(baud) = probe(io, target) {
                    report.uart_baud = baud;
                }
            }
            // Keep controller state truthful even if receiver disappears.
            let _ = io.set_baud(report.uart_baud);
        }
    }
    report
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::{BTreeMap, VecDeque};

    struct Receiver {
        host_baud: u32,
        baud: u32,
        clock: u64,
        pending: VecDeque<u8>,
        values: BTreeMap<u32, u32>,
        writes: usize,
        persistent: bool,
        reject: bool,
        wrong_readback: bool,
        missing_key: bool,
        wrong_ack: bool,
        chunk: usize,
        offline: bool,
        corrupt_probe: bool,
        nmea_only_probe: bool,
    }
    impl Receiver {
        fn new(baud: u32) -> Self {
            Self {
                host_baud: 0,
                baud,
                clock: 0,
                pending: VecDeque::new(),
                values: BTreeMap::from([(UART_BAUD, baud)]),
                writes: 0,
                persistent: false,
                reject: false,
                wrong_readback: false,
                missing_key: false,
                wrong_ack: false,
                chunk: 128,
                offline: false,
                corrupt_probe: false,
                nmea_only_probe: false,
            }
        }
        fn reply(&mut self, class: u8, id: u8, payload: &[u8]) {
            let mut bytes = vec![
                0xb5,
                0x62,
                class,
                id,
                payload.len() as u8,
                (payload.len() >> 8) as u8,
            ];
            bytes.extend_from_slice(payload);
            let mut a = 0u8;
            let mut b = 0u8;
            for byte in &bytes[2..] {
                a = a.wrapping_add(*byte);
                b = b.wrapping_add(a);
            }
            bytes.extend_from_slice(&[a, b]);
            if self.corrupt_probe && class == 0x0a {
                *bytes.last_mut().unwrap() ^= 1;
            }
            self.pending.extend(bytes);
        }
    }
    impl Transport for Receiver {
        fn set_baud(&mut self, baud: u32) -> Result<(), Error> {
            self.host_baud = baud;
            Ok(())
        }
        fn now_ms(&self) -> u64 {
            self.clock
        }
        fn sleep_ms(&mut self, ms: u32) {
            self.clock += ms as u64;
        }
        fn read(&mut self, out: &mut [u8]) -> Result<usize, Error> {
            let len = out.len().min(self.pending.len()).min(self.chunk);
            for byte in &mut out[..len] {
                *byte = self.pending.pop_front().unwrap();
            }
            Ok(len)
        }
        fn write(&mut self, data: &[u8]) -> Result<(), Error> {
            self.writes += 1;
            assert_eq!(
                checksum(&data[2..data.len() - 2]),
                (data[data.len() - 2], data[data.len() - 1])
            );
            if self.offline || self.host_baud != self.baud {
                return Ok(());
            }
            let payload = &data[6..data.len() - 2];
            match (data[2], data[3]) {
                (0x0a, 4) if self.nmea_only_probe => {
                    let body = "GNRMC,123456.00,V,,,,,,,221126,,,N";
                    let crc = body.bytes().fold(0u8, |a, b| a ^ b);
                    self.pending
                        .extend(format!("${body}*{crc:02X}\r\n").bytes());
                }
                (0x0a, 4) => self.reply(0x0a, 4, &[0; 400]),
                (6, 0x8a) => {
                    self.persistent |= payload[1] != 1;
                    if self.reject {
                        self.reply(5, 0, &[6, 0x8a]);
                        return Ok(());
                    }
                    assert_eq!(&payload[..4], &[0, 1, 0, 0]);
                    let mut pos = 4;
                    while pos < payload.len() {
                        let key = u32::from_le_bytes(payload[pos..pos + 4].try_into().unwrap());
                        pos += 4;
                        let size = width(key).unwrap();
                        let mut value = [0u8; 4];
                        value[..size].copy_from_slice(&payload[pos..pos + size]);
                        pos += size;
                        let value = u32::from_le_bytes(value);
                        self.values.insert(key, value);
                        if key == UART_BAUD {
                            self.baud = value;
                        }
                    }
                    self.reply(5, 1, &[6, if self.wrong_ack { 0x01 } else { 0x8a }]);
                }
                (6, 0x8b) => {
                    assert_eq!(&payload[..4], &[0, 0, 0, 0]); // RAM, not defaults/BBR/flash
                    let mut response = vec![1, 0, 0, 0];
                    for (index, key) in payload[4..].chunks_exact(4).enumerate() {
                        if self.missing_key && index == 1 {
                            continue;
                        }
                        let key = u32::from_le_bytes(key.try_into().unwrap());
                        response.extend_from_slice(&key.to_le_bytes());
                        let value = self.values.get(&key).copied().unwrap_or(0)
                            + u32::from(self.wrong_readback);
                        response.extend_from_slice(&value.to_le_bytes()[..width(key).unwrap()]);
                    }
                    self.reply(6, 0x8b, &response);
                }
                _ => panic!("unexpected receiver command"),
            }
            Ok(())
        }
    }

    #[test]
    fn cold_saved_and_legacy_bauds_reach_verified_ram_profile() {
        for baud in [38400, 115200, 9600, 57600, 19200, 230400, 460800, 921600] {
            let mut receiver = Receiver::new(baud);
            let report = configure(&mut receiver, 115200, 25);
            assert!(report.verified, "{report:?}");
            assert_eq!(report.detected_baud, Some(baud));
            assert_eq!(receiver.baud, 115200);
            assert_eq!(receiver.host_baud, 115200);
            assert!(!receiver.persistent);
            assert_eq!(receiver.values[&0x209100ac], 1);
            assert_eq!(receiver.values[&0x209100bb], 25);
            assert_eq!(receiver.values[&0x209100c0], 25);
            assert_eq!(receiver.values[&0x209100c5], 0);
            assert_eq!(receiver.values[&0x30210001], 40);
            assert_eq!(receiver.values[&0x20d00001], 0);
        }
    }
    #[test]
    fn fragmented_extended_mon_ver_and_nmea_only_probes_work_without_a_fix() {
        for nmea in [false, true] {
            let mut receiver = Receiver::new(115200);
            receiver.chunk = 1;
            receiver.nmea_only_probe = nmea;
            assert!(configure(&mut receiver, 115200, 25).verified);
        }
    }
    #[test]
    fn nak_wrong_ack_missing_or_mismatched_readback_never_claim_success() {
        for mode in 0..4 {
            let mut receiver = Receiver::new(115200);
            receiver.reject = mode == 0;
            receiver.wrong_ack = mode == 1;
            receiver.missing_key = mode == 2;
            receiver.wrong_readback = mode == 3;
            let report = configure(&mut receiver, 115200, 25);
            assert!(!report.verified);
            assert!(report.error.is_some());
            assert_eq!(report.uart_baud, 115200);
            assert_eq!(receiver.host_baud, 115200);
        }
    }
    #[test]
    fn no_response_or_corrupt_probe_is_bounded_and_never_writes_configuration() {
        for corrupt in [false, true] {
            let mut receiver = Receiver::new(115200);
            receiver.offline = !corrupt;
            receiver.corrupt_probe = corrupt;
            let report = configure(&mut receiver, 115200, 25);
            assert_eq!(report.error, Some(Error::NoReceiver));
            assert!(!report.verified);
            assert!(receiver.values.len() == 1);
            assert!(receiver.clock < 4500);
            assert_eq!(receiver.host_baud, report.uart_baud);
        }
    }
    #[test]
    fn malformed_duplicate_and_wrong_layer_readback_are_rejected() {
        let items = [Item(UART_BAUD, 115200)];
        for payload in [vec![1, 0, 0, 0], vec![1, 1, 0, 0], vec![1, 0, 0, 0, 1]] {
            assert_eq!(verify_items(&payload, &items), Err(Error::Readback));
        }
        let mut payload = vec![1, 0, 0, 0];
        for _ in 0..2 {
            payload.extend_from_slice(&UART_BAUD.to_le_bytes());
            payload.extend_from_slice(&115200u32.to_le_bytes());
        }
        assert_eq!(verify_items(&payload, &items), Err(Error::Readback));
    }
    #[test]
    fn zero_or_excessive_rate_does_not_touch_receiver_configuration() {
        for rate in [0, 26, 255] {
            let mut io = Receiver::new(115200);
            assert_eq!(
                configure(&mut io, 115200, rate).error,
                Some(Error::InvalidRate)
            );
            assert_eq!(io.writes, 0);
        }
    }

    #[test]
    fn unsupported_target_baud_never_starts_a_probe_or_configuration() {
        for baud in [0, 1, 500000] {
            let mut receiver = Receiver::new(115200);
            assert_eq!(
                configure(&mut receiver, baud, 25).error,
                Some(Error::InvalidBaud)
            );
            assert_eq!(receiver.writes, 0);
        }
    }
    #[test]
    fn slow_baud_cannot_be_verified_for_a_profile_it_cannot_carry() {
        let mut receiver = Receiver::new(9600);
        assert_eq!(
            configure(&mut receiver, 9600, 25).error,
            Some(Error::Bandwidth)
        );
        assert_eq!(receiver.writes, 0);
    }
}
