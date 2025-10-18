/// WiFi connection manager
use esp_idf_svc::wifi::{
    BlockingWifi, ClientConfiguration, Configuration, EspWifi,
};
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_hal::peripheral;
use log::info;

pub struct WifiManager {
    wifi: BlockingWifi<EspWifi<'static>>,
}

impl WifiManager {
    pub fn new(
        modem: impl peripheral::Peripheral<P = esp_idf_hal::modem::Modem> + 'static,
        sysloop: EspSystemEventLoop,
        nvs: Option<EspDefaultNvsPartition>,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        
        let wifi = BlockingWifi::wrap(
            EspWifi::new(modem, sysloop.clone(), nvs)?,
            sysloop,
        )?;
        
        Ok(Self { wifi })
    }
    
    pub fn connect(&mut self, ssid: &str, password: &str) -> Result<(), Box<dyn std::error::Error>> {
        
        info!("Setting WiFi configuration");
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
        
        Ok(())
    }
    
    #[allow(dead_code)]
    pub fn is_connected(&self) -> bool {
        self.wifi.is_connected().unwrap_or(false)
    }
}
