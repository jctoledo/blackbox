#![allow(dead_code)] // MQTT methods for future use

/// MQTT client wrapper with binary support
use esp_idf_hal::delay::FreeRtos;
use esp_idf_svc::mqtt::client::{EspMqttClient, MqttClientConfiguration, QoS};
use log::info;
use serde_json::json;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

pub struct MqttClient {
    client: EspMqttClient<'static>,
    connected: Arc<AtomicBool>,
}

impl MqttClient {
    pub fn new(broker_url: &str) -> Result<Self, Box<dyn std::error::Error>> {
        info!("Creating MQTT config with optimized buffers for 20 Hz");

        let mqtt_config = MqttClientConfiguration {
            client_id: None,
            keep_alive_interval: Some(core::time::Duration::from_secs(30)),
            network_timeout: core::time::Duration::from_secs(2), // Reduced timeout
            buffer_size: 2048,                                   /* Smaller buffer, faster
                                                                  * throughput */
            out_buffer_size: 4096, // Larger outgoing for high-freq publishing
            ..Default::default()
        };

        // Track connection state via callback
        let connected = Arc::new(AtomicBool::new(false));
        let connected_clone = connected.clone();

        info!("Creating MQTT client with connection tracking");
        let client = EspMqttClient::new_cb(broker_url, &mqtt_config, move |event| {
            match event.payload() {
                esp_idf_svc::mqtt::client::EventPayload::Connected(_) => {
                    info!("MQTT: Connected event received");
                    connected_clone.store(true, Ordering::SeqCst);
                }
                esp_idf_svc::mqtt::client::EventPayload::Disconnected => {
                    info!("MQTT: Disconnected event received");
                    connected_clone.store(false, Ordering::SeqCst);
                }
                esp_idf_svc::mqtt::client::EventPayload::Error(e) => {
                    info!("MQTT: Error event: {:?}", e);
                }
                _ => {}
            }
        })?;

        info!("MQTT client created, waiting for connection (up to 5s)");

        // Wait for connection with timeout
        let start = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
        let timeout_us = 5_000_000i64; // 5 seconds

        while !connected.load(Ordering::SeqCst) {
            let elapsed = unsafe { esp_idf_svc::sys::esp_timer_get_time() } - start;
            if elapsed > timeout_us {
                info!("MQTT connection timed out after 5s");
                return Err("MQTT connection timeout - broker unreachable".into());
            }
            FreeRtos::delay_ms(100);
        }

        info!("MQTT connected successfully");
        Ok(Self { client, connected })
    }

    pub fn publish(
        &mut self,
        topic: &str,
        payload: &str,
        retain: bool,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let qos = if retain {
            QoS::AtLeastOnce
        } else {
            QoS::AtMostOnce
        };
        self.client
            .enqueue(topic, qos, retain, payload.as_bytes())?;
        Ok(())
    }

    /// Publish binary data (for high-speed telemetry)
    pub fn publish_binary(
        &mut self,
        topic: &str,
        payload: &[u8],
        retain: bool,
    ) -> Result<(), Box<dyn std::error::Error>> {
        // Always use QoS 0 (AtMostOnce) for high-frequency telemetry
        // This is non-blocking and won't wait for ACKs
        self.client
            .enqueue(topic, QoS::AtMostOnce, retain, payload)?;
        Ok(())
    }

    pub fn publish_status(
        &mut self,
        msg: &str,
        progress: Option<f32>,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let ts = unsafe { esp_idf_svc::sys::esp_timer_get_time() / 1000 };

        let payload = if let Some(p) = progress {
            json!({
                "ts": ts,
                "msg": msg,
                "prog": p
            })
            .to_string()
        } else {
            json!({
                "ts": ts,
                "msg": msg
            })
            .to_string()
        };

        self.publish("car/status", &payload, false)?;
        Ok(())
    }

    #[allow(dead_code)]
    pub fn subscribe(&mut self, topic: &str) -> Result<(), Box<dyn std::error::Error>> {
        self.client.subscribe(topic, QoS::AtLeastOnce)?;
        Ok(())
    }

    /// Check if MQTT is currently connected
    pub fn is_connected(&self) -> bool {
        self.connected.load(Ordering::SeqCst)
    }
}
