// Auto-generated project configuration
// Generated from sensors/*/project.json
// DO NOT EDIT MANUALLY - your changes will be overwritten

const PROJECTS = {
        'blackbox': {
            name: "Blackbox",
            description: "Open-source ESP32-C3 vehicle telemetry with GPS and IMU fusion, browser dashboard, lap timing, and local recording.",
            hardware: ["ESP32-C3 Development Board", "WT901 9-axis IMU (configured for 115200 baud)", "NEO-6M or NEO-M9N GPS (choose matching firmware configuration)", "WS2812 RGB LED for status indication"],
            software: ["Chrome or Edge browser (for web flasher)", "Windows, macOS, or Linux", "Python 3.x (for telemetry receiver tools)"],
            firmwareUrl: "https://github.com/jctoledo/blackbox/releases/download/v0.0.3/blackbox.bin",
            chip: "esp32c3",
            target: "riscv32imc-esp-espidf"
        }
};

// Export for use in index.html
if (typeof module !== 'undefined' && module.exports) {
    module.exports = PROJECTS;
}
