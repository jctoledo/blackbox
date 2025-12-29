// Auto-generated project configuration
// Generated from sensors/*/project.json
// DO NOT EDIT MANUALLY - your changes will be overwritten

const PROJECTS = {
        'blackbox': {
            name: "Blackbox",
            description: "Vehicle telemetry system with real-time sensor fusion using Extended Kalman Filtering. Fuses GPS and IMU data for accurate position and motion tracking at 20Hz over UDP.",
            hardware: ["ESP32-C3 Development Board", "WT901 9-axis IMU (UART @ 9600 baud)", "NEO-6M GPS Module (UART @ 9600 baud)", "WS2812 RGB LED for status indication"],
            software: ["Chrome or Edge browser (for web flasher)", "Windows, macOS, or Linux", "Python 3.x (for telemetry receiver tools)"],
            firmwareUrl: "https://github.com/jctoledo/blackbox/releases/download/latest/blackbox.bin",
            chip: "esp32c3",
            target: "riscv32imc-esp-espidf"
        }
};

// Export for use in index.html
if (typeof module !== 'undefined' && module.exports) {
    module.exports = PROJECTS;
}
