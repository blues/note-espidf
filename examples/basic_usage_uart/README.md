# Basic UART Usage Example

This example demonstrates basic Notecard usage with ESP-IDF using UART communication in a standard ESP-IDF application structure, showing how to:

- Initialize the Notecard with UART communication
- Configure Notehub connection
- Use FreeRTOS tasks for sensor data collection
- Read built-in sensors from the Notecard (temperature and voltage)
- Send sensor data to Notehub using a dedicated task
- Handle JSON requests and responses with proper thread safety

## Hardware Requirements

- ESP32 HUZZAH32 V2
- Notecard
- Notecarrier F
- Proper UART connections:
  - TX: GPIO 17 (ESP32 TX to Notecard RX)
  - RX: GPIO 16 (ESP32 RX to Notecard TX)
  - VCC: 3.3V or 5V (depending on your board)
  - GND: Ground

## Configuration

1. **Product UID**: Define your Notehub Product UID by either:
   - Setting it in the Notecard directly via Notehub
   - Defining `PRODUCT_UID` in the code

2. **UART Pins**: The example uses GPIO 17 (TX) and GPIO 16 (RX) by default. These can be changed in the configuration.

3. **Baudrate**: Uses 9600 baud by default, which is the standard Notecard UART speed.

## Building and Running

```bash
# Set up ESP-IDF environment
source ~/esp/v5.5.1/esp-idf/export.sh

# Navigate to the UART example directory
cd examples/uart_usage

# Build the project
idf.py build

# Flash to ESP32
idf.py flash

# Monitor serial output
idf.py monitor
```

For more information, visit [Blues Developer Documentation](https://dev.blues.io/).
