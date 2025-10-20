// Auto-generated project configuration
// Generated from sensors/*/project.json
// DO NOT EDIT MANUALLY - your changes will be overwritten

const PROJECTS = {
        'active-wing': {
            name: "Active Wing",
            description: "The reference implementation demonstrating real-time sensor fusion using Extended Kalman Filtering. This system fuses GPS and IMU data to provide accurate position and motion tracking, eliminating IMU drift through GPS corrections while maintaining high-frequency updates.",
            hardware: ["ESP32-C3 Development Board", "WT901 9-axis IMU (UART @ 9600 baud)", "NEO-6M GPS Module (UART @ 9600 baud)", "WS2812 RGB LED for status indication"],
            software: ["Chrome or Edge browser (for web flasher)", "Windows, macOS, or Linux", "Python 3.x (for telemetry receiver tools)"],
            firmwareUrl: "https://github.com/jctoledo/active_wing/releases/download/latest/active-wing.bin",
            chip: "esp32c3",
            target: "riscv32imc-esp-espidf"
        }
};

// Export for use in index.html
if (typeof module !== 'undefined' && module.exports) {
    module.exports = PROJECTS;
}
