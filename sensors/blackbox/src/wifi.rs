use esp_idf_hal::peripheral;
/// WiFi connection manager - supports both STA (client) and AP (access point)
/// modes
use esp_idf_svc::wifi::{
    AccessPointConfiguration, AuthMethod, BlockingWifi, ClientConfiguration, Configuration, EspWifi,
};
use esp_idf_svc::{eventloop::EspSystemEventLoop, nvs::EspDefaultNvsPartition};
use log::info;

/// WiFi operating mode
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WifiMode {
    /// Station mode - connect to existing network
    Station,
    /// Access Point mode - create own network
    AccessPoint,
}

pub struct WifiManager {
    wifi: BlockingWifi<EspWifi<'static>>,
    mode: WifiMode,
}

impl WifiManager {
    pub fn new(
        modem: impl peripheral::Peripheral<P = esp_idf_hal::modem::Modem> + 'static,
        sysloop: EspSystemEventLoop,
        nvs: Option<EspDefaultNvsPartition>,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let wifi = BlockingWifi::wrap(EspWifi::new(modem, sysloop.clone(), nvs)?, sysloop)?;

        Ok(Self {
            wifi,
            mode: WifiMode::Station,
        })
    }

    /// Connect to an existing WiFi network (STA mode)
    pub fn connect(
        &mut self,
        ssid: &str,
        password: &str,
    ) -> Result<(), Box<dyn std::error::Error>> {
        info!("Setting WiFi configuration (STA mode)");
        let wifi_config = Configuration::Client(ClientConfiguration {
            ssid: ssid.try_into().unwrap(),
            password: password.try_into().unwrap(),
            ..Default::default()
        });

        self.wifi.set_configuration(&wifi_config)?;

        info!("Starting WiFi");
        self.wifi.start()?;

        info!("Connecting to AP");
        self.wifi.connect()?;

        info!("Waiting for DHCP lease");
        self.wifi.wait_netif_up()?;

        let ip_info = self.wifi.wifi().sta_netif().get_ip_info()?;
        info!("WiFi connected! IP: {}", ip_info.ip);

        self.mode = WifiMode::Station;
        Ok(())
    }

    /// Start as Access Point (AP mode) - creates own WiFi network
    /// Clients connect to this network and ESP32 is always at 192.168.71.1
    pub fn start_ap(
        &mut self,
        ssid: &str,
        password: &str,
    ) -> Result<(), Box<dyn std::error::Error>> {
        info!("Setting WiFi configuration (AP mode)");
        info!(
            "SSID: {}, Password: {}",
            ssid,
            if password.is_empty() {
                "(open)"
            } else {
                "****"
            }
        );

        let auth_method = if password.is_empty() {
            AuthMethod::None
        } else {
            AuthMethod::WPA2Personal
        };

        let wifi_config = Configuration::AccessPoint(AccessPointConfiguration {
            ssid: ssid.try_into().unwrap(),
            password: password.try_into().unwrap(),
            auth_method,
            channel: 6,
            max_connections: 4,
            ..Default::default()
        });

        self.wifi.set_configuration(&wifi_config)?;

        info!("Starting WiFi AP");
        self.wifi.start()?;

        info!("Waiting for AP to be ready");
        self.wifi.wait_netif_up()?;

        let ip_info = self.wifi.wifi().ap_netif().get_ip_info()?;
        info!("WiFi AP started! IP: {}", ip_info.ip);
        info!(
            "Clients should connect to '{}' and access 192.168.71.1",
            ssid
        );

        self.mode = WifiMode::AccessPoint;
        Ok(())
    }

    /// Check if WiFi is connected (STA mode) or running (AP mode)
    pub fn is_connected(&self) -> bool {
        match self.mode {
            WifiMode::Station => self.wifi.is_connected().unwrap_or(false),
            WifiMode::AccessPoint => self.wifi.is_up().unwrap_or(false),
        }
    }

    /// Get current WiFi mode
    #[allow(dead_code)]
    pub fn mode(&self) -> WifiMode {
        self.mode
    }
}
