/// UDP telemetry streaming
/// Simple, fast, connectionless - perfect for real-time telemetry
use std::net::UdpSocket;

use log::info;

pub struct UdpTelemetryStream {
    socket: Option<UdpSocket>,
    server_addr: String,
}

impl UdpTelemetryStream {
    pub fn new(server_addr: &str) -> Self {
        info!("UDP telemetry client created for: {}", server_addr);

        Self {
            socket: None,
            server_addr: server_addr.to_string(),
        }
    }

    /// Initialize the UDP socket (bind to any available port)
    pub fn init(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        info!("Initializing UDP socket for {}", self.server_addr);

        // Bind to any available port
        let socket = UdpSocket::bind("0.0.0.0:0")?;
        socket.set_nonblocking(false)?;

        self.socket = Some(socket);
        info!("UDP socket ready");

        Ok(())
    }

    /// Send data to the server (fire and forget)
    pub fn send(&self, data: &[u8]) -> Result<(), Box<dyn std::error::Error>> {
        if let Some(ref socket) = self.socket {
            socket.send_to(data, &self.server_addr)?;
            Ok(())
        } else {
            Err("UDP socket not initialized".into())
        }
    }

    /// Check if socket is initialized
    pub fn is_ready(&self) -> bool {
        self.socket.is_some()
    }
}
