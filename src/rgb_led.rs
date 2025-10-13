/// RGB LED wrapper for WS2812 on ESP32-C3 DevKit using RMT
use esp_idf_hal::gpio::OutputPin;
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::rmt::config::TransmitConfig;
use esp_idf_hal::rmt::{FixedLengthSignal, PinState, Pulse, RmtChannel, TxRmtDriver};

pub struct RgbLed {
    tx: TxRmtDriver<'static>,
}

impl RgbLed {
    pub fn new<C: RmtChannel, P: OutputPin>(
        channel: impl Peripheral<P = C> + 'static,
        pin: impl Peripheral<P = P> + 'static,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let config = TransmitConfig::new().clock_divider(1);
        let tx = TxRmtDriver::new(channel, pin, &config)?;
        Ok(Self { tx })
    }

    /// Set LED to a specific color (GRB order for WS2812)
    pub fn set_color(&mut self, r: u8, g: u8, b: u8) -> Result<(), Box<dyn std::error::Error>> {
        let grb = ((g as u32) << 16) | ((r as u32) << 8) | (b as u32);
        
        let ticks_hz = self.tx.counter_clock()?;
        let t0h = Pulse::new_with_duration(ticks_hz, PinState::High, &ns(350))?;
        let t0l = Pulse::new_with_duration(ticks_hz, PinState::Low, &ns(800))?;
        let t1h = Pulse::new_with_duration(ticks_hz, PinState::High, &ns(700))?;
        let t1l = Pulse::new_with_duration(ticks_hz, PinState::Low, &ns(600))?;
        
        // Build signal as fixed-length array
        let mut signal = FixedLengthSignal::<24>::new();
        
        for i in (0..24).rev() {
            let bit = (grb >> i) & 1;
            if bit == 1 {
                signal.set(23 - i as usize, &(t1h, t1l))?;
            } else {
                signal.set(23 - i as usize, &(t0h, t0l))?;
            }
        }
        
        self.tx.start_blocking(&signal)?;
        Ok(())
    }

    /// Turn LED off (black)
    pub fn set_low(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        self.set_color(0, 0, 0)
    }

    /// Turn LED on (white - dimmed to 25% to avoid burning eyes)
    pub fn set_high(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        self.set_color(64, 64, 64)
    }

    /// Set to red
    pub fn red(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        self.set_color(64, 0, 0)
    }

    /// Set to green
    pub fn green(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        self.set_color(0, 64, 0)
    }

    /// Set to blue
    pub fn blue(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        self.set_color(0, 0, 64)
    }

    /// Set to yellow
    pub fn yellow(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        self.set_color(64, 64, 0)
    }

    /// Set to cyan
    pub fn cyan(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        self.set_color(0, 64, 64)
    }

    /// Set to magenta
    pub fn magenta(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        self.set_color(64, 0, 64)
    }
}

fn ns(nanos: u64) -> std::time::Duration {
    std::time::Duration::from_nanos(nanos)
}
