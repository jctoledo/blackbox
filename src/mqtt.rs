/// MQTT client wrapper
use esp_idf_svc::mqtt::client::{EspMqttClient, MqttClientConfiguration, QoS};
use serde_json::json;
use esp_idf_hal::delay::FreeRtos;
use log::info;

pub struct MqttClient {
    client: EspMqttClient<'static>,
}

impl MqttClient {
    pub fn new(broker_url: &str) -> Result<Self, Box<dyn std::error::Error>> {
        info!("Creating MQTT config with large buffers");
        
        let mqtt_config = MqttClientConfiguration {
            client_id: None,
            keep_alive_interval: Some(core::time::Duration::from_secs(30)),
            network_timeout: core::time::Duration::from_secs(5),
            buffer_size: 4096,
            out_buffer_size: 4096,
            ..Default::default()
        };
        
        info!("Creating MQTT client (non-blocking)");
        // Don't keep the connection handle - let it be managed automatically
        let (client, _connection) = EspMqttClient::new(broker_url, &mqtt_config)?;
        
        info!("MQTT client created, waiting for async connection");
        FreeRtos::delay_ms(5000);
        
        info!("Returning MQTT client");
        Ok(Self { client })
    }
    
    pub fn publish(&mut self, topic: &str, payload: &str, retain: bool) -> Result<(), Box<dyn std::error::Error>> {
        let qos = if retain { QoS::AtLeastOnce } else { QoS::AtMostOnce };
        
        // Use enqueue which is non-blocking - perfect for high-frequency publishing
        self.client.enqueue(topic, qos, retain, payload.as_bytes())?;
        Ok(())
    }
    
    pub fn publish_status(&mut self, msg: &str, progress: Option<f32>) -> Result<(), Box<dyn std::error::Error>> {
        let ts = unsafe { esp_idf_svc::sys::esp_timer_get_time() / 1000 };
        
        let payload = if let Some(p) = progress {
            json!({
                "ts": ts,
                "msg": msg,
                "prog": p
            }).to_string()
        } else {
            json!({
                "ts": ts,
                "msg": msg
            }).to_string()
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
