# Basic I2C Usage Example

This example demonstrates basic Notecard usage with ESP-IDF in a standard ESP-IDF application structure, showing how to:

- Initialize the Notecard with I2C communication using the new I2C master API
- Configure Notehub connection
- Use FreeRTOS tasks for sensor data collection
- Read built-in sensors from the Notecard (temperature and voltage)
- Send sensor data to Notehub using a dedicated task
- Handle JSON requests and responses with proper thread safety

## Hardware Requirements

- ESP32 HUZZAH32 V2
- Notecard
- Notecarrier F
- Proper I2C connections:
  - SDA: GPIO 22
  - SCL: GPIO 20
  - VCC: 3.3V or 5V (depending on your board)
  - GND: Ground

## Configuration

1. **Product UID**: Define your Notehub Product UID by either:
   - Setting it in the Notecard directly via Notehub
   - Defining `PRODUCT_UID` in the code

2. **I2C Pins**: The example uses GPIO 23 (SDA) and GPIO 22 (SCL) by default. These can be changed in the configuration.

## Building and Running

```bash
# Set up ESP-IDF environment
source ~/esp/v5.5.1/esp-idf/export.sh

# Build the project
idf.py build

# Flash to ESP32
idf.py flash

# Monitor serial output
idf.py monitor
```

For more information, visit [Blues Developer Documentation](https://dev.blues.io/).
