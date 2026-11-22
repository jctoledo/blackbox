//! Hardware adapter for the host-tested receiver startup procedure.
use esp_idf_hal::{
    delay::{FreeRtos, TickType},
    uart::UartDriver,
};
use ublox_gps::setup::{self, Error, Transport};

struct UartTransport<'a, 'd>(&'a UartDriver<'d>);
pub struct Receiver<'a, 'd>(pub &'a UartDriver<'d>);
impl crate::gps_rx::Receiver for Receiver<'_, '_> {
    fn read(&mut self, out: &mut [u8]) -> Result<usize, crate::gps_rx::ReadError> {
        UartTransport(self.0)
            .read(out)
            .map_err(|_| crate::gps_rx::ReadError)
    }
    fn discard(&mut self) -> Result<(), crate::gps_rx::ReadError> {
        self.0.clear_rx().map_err(|_| crate::gps_rx::ReadError)
    }
    fn observe_loss(&mut self, counters: &mut crate::diagnostics::GpsTransportHealth) -> bool {
        use esp_idf_hal::uart::UartEventPayload;
        let mut lost = false;
        if let Some(queue) = self.0.event_queue() {
            for _ in 0..64 {
                let Some((event, _)) = queue.recv_front(0) else {
                    break;
                };
                match event.payload() {
                    UartEventPayload::RxFifoOverflow => counters.fifo_overflows += 1,
                    UartEventPayload::RxBufferFull => counters.buffer_full += 1,
                    UartEventPayload::FrameError => counters.frame_errors += 1,
                    UartEventPayload::ParityError => counters.parity_errors += 1,
                    _ => continue,
                }
                lost = true;
            }
        }
        lost
    }
}
impl Transport for UartTransport<'_, '_> {
    fn set_baud(&mut self, baud: u32) -> Result<(), Error> {
        self.0
            .change_baudrate(esp_idf_hal::units::Hertz(baud))
            .map(|_| ())
            .map_err(|_| Error::Io)
    }
    fn write(&mut self, mut bytes: &[u8]) -> Result<(), Error> {
        while !bytes.is_empty() {
            let n = self.0.write(bytes).map_err(|_| Error::Io)?;
            if n == 0 {
                return Err(Error::Io);
            }
            bytes = &bytes[n..];
        }
        self.0
            .wait_tx_done(TickType::new_millis(500).ticks())
            .map_err(|_| Error::Io)
    }
    fn read(&mut self, bytes: &mut [u8]) -> Result<usize, Error> {
        match self.0.read(bytes, 0) {
            Ok(n) => Ok(n),
            Err(error) if error.code() == esp_idf_svc::sys::ESP_ERR_TIMEOUT => Ok(0),
            Err(_) => Err(Error::Io),
        }
    }
    fn now_ms(&self) -> u64 {
        unsafe { esp_idf_svc::sys::esp_timer_get_time() as u64 / 1000 }
    }
    fn sleep_ms(&mut self, ms: u32) {
        FreeRtos::delay_ms(ms);
    }
}
pub fn configure(uart: &UartDriver<'_>, baud: u32, rate: u8) -> setup::Report {
    let report = setup::configure(&mut UartTransport(uart), baud, rate);
    // Wrong-baud probes can intentionally generate UART errors; runtime counters
    // start after setup, so they do not report those as acquisition failures.
    if let Some(queue) = uart.event_queue() {
        for _ in 0..64 {
            if queue.recv_front(0).is_none() {
                break;
            }
        }
    }
    report
}
