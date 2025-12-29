#![allow(dead_code)] // MQTT methods for future use

/// MQTT client wrapper with binary support
use esp_idf_hal::delay::FreeRtos;
use esp_idf_svc::mqtt::client::{EspMqttClient, MqttClientConfiguration, QoS};
use log::info;
use serde_json::json;

pub struct MqttClient {
    client: EspMqttClient<'static>,
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

        info!("Creating MQTT client (non-blocking)");
        let (client, _connection) = EspMqttClient::new(broker_url, &mqtt_config)?;

        info!("MQTT client created, waiting for async connection");
        FreeRtos::delay_ms(3000); // Reduced wait time

        info!("Returning MQTT client");
        Ok(Self { client })
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
}
