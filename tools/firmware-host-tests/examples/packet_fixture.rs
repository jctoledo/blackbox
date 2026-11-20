use blackbox_host_tests::binary_telemetry::TelemetryPacket;
fn main() {
    let mut packet = TelemetryPacket::new();
    packet.timestamp_ms = 123456;
    packet.speed_kmh = 72.0;
    packet.pos_sigma = 2.5;
    packet.mode = 4;
    packet.lat = 37.0;
    packet.lon = -122.0;
    packet.gps_valid = 1;
    packet.gps_course = 1.25;
    packet.lap_time_ms = 60023;
    packet.lap_count = 2;
    packet.lap_flags = 4;
    for byte in packet.to_bytes() {
        print!("{byte:02x}");
    }
    println!();
}
