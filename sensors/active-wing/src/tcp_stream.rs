use std::io::Write;
/// High-speed TCP streaming for telemetry
/// Simple, fast, reliable - perfect for 20+ Hz data streaming
use std::net::TcpStream;

use log::{info, warn};

pub struct TcpTelemetryStream {
    stream: Option<TcpStream>,
    server_addr: String,
}

impl TcpTelemetryStream {
    pub fn new(server_addr: &str) -> Self {
        info!("TCP stream client created for: {}", server_addr);

        Self {
            stream: None,
            server_addr: server_addr.to_string(),
        }
    }

    pub fn connect(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        info!("Connecting to TCP server: {}", self.server_addr);

        let stream = TcpStream::connect(&self.server_addr)?;
        stream.set_nodelay(true)?; // Disable Nagle's algorithm for low latency
        stream.set_write_timeout(Some(std::time::Duration::from_millis(100)))?;

        self.stream = Some(stream);
        info!("TCP stream connected!");

        Ok(())
    }

    pub fn send(&mut self, data: &[u8]) -> Result<(), Box<dyn std::error::Error>> {
        if let Some(ref mut stream) = self.stream {
            stream.write_all(data)?;
            Ok(())
        } else {
            Err("TCP stream not connected".into())
        }
    }

    pub fn is_connected(&self) -> bool {
        self.stream.is_some()
    }

    pub fn reconnect(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        warn!("Reconnecting TCP stream...");
        self.stream = None;
        self.connect()
    }
}
