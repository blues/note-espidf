# Basic Usage Example

This example demonstrates basic Notecard usage with ESP-IDF in a standard ESP-IDF application structure, showing how to:

- Initialize the Notecard with I2C communication using the new I2C master API
- Configure Notehub connection
- Use FreeRTOS tasks for sensor data collection
- Read built-in sensors from the Notecard (temperature and voltage)
- Send sensor data to Notehub using a dedicated task
- Handle JSON requests and responses with proper thread safety

## Hardware Requirements

- ESP32 development board
- Blues Notecard (cellular or WiFi)
- Proper I2C connections:
  - SDA: GPIO 23
  - SCL: GPIO 22
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

## Features

### ESP-IDF Application Structure
This example follows standard ESP-IDF patterns:
- **Task-based architecture**: Uses FreeRTOS tasks for sensor data collection
- **Thread safety**: Mutex protection for Notecard access
- **Resource management**: Proper task creation, cleanup, and monitoring
- **Error handling**: Structured error handling with appropriate return codes
- **Modular design**: Separate functions for hardware init, device detection, and configuration

### I2C Master API Support
This example uses the modern ESP-IDF I2C master driver (`driver/i2c_master.h`) instead of the legacy I2C driver, providing:
- Better performance and reliability
- Improved error handling
- Cleaner API design
- Enhanced resource management

### Notecard Integration
- Full JSON API support via note-c library
- Automatic I2C device scanning and detection
- Robust error handling and recovery
- Simulation mode when hardware is not connected
- Thread-safe Notecard operations using mutex

### Data Collection Task
- Dedicated FreeRTOS task for sensor reading
- Reads temperature from Notecard's built-in sensor
- Reads voltage from Notecard's V+ pin
- Includes system health monitoring (heap usage)
- Sends data to Notehub every 15 seconds
- Automatic task completion after 25 readings

## Expected Output

When running with a connected Notecard, you should see:
```
I (123) notecard_basic: === ESP-IDF Notecard Basic Usage Example ===
I (124) notecard_basic: Component version: 1.0.0
I (125) notecard_basic: ESP32 chip: esp32 rev 3, 2 cores
I (126) notecard_basic: Free heap: 295168 bytes
I (127) notecard_basic: Initializing Notecard hardware...
I (128) notecard_basic: Notecard hardware initialized successfully
I (129) notecard_basic: Scanning I2C bus for Notecard...
I (130) notecard_basic: Found 1 I2C device: 0x17
I (131) notecard_basic: ✓ Notecard detected at address 0x17
I (132) notecard_basic: Configuring Notecard for Notehub connection...
I (133) notecard_basic: Product ID set: com.your-company.your-product
I (134) notecard_basic: Notecard configured for Notehub connection
I (135) notecard_basic: Notecard sensor task created successfully
I (136) notecard_basic: Notecard sensor task started
I (137) notecard_basic: Reading sensors (sample 1/25)...
I (138) notecard_basic: Sample 1 sent: temp=23.45°C, voltage=5.12V, heap=290123
I (139) notecard_basic: System status: heap=290000 bytes, task running=yes
```

## Troubleshooting

- **No Notecard detected**: Check I2C wiring and power connections
- **Build fails**: Ensure ESP-IDF v5.5+ is properly installed and sourced
- **Connection issues**: Verify Product UID is set correctly in Notehub
- **Memory issues**: Check heap size if running on memory-constrained devices

## Next Steps

After running this basic example:
1. Modify sensor reading intervals
2. Add custom sensor data
3. Implement power management features
4. Explore other Notecard APIs (GPS, connectivity status, etc.)

For more information, visit [Blues Developer Documentation](https://dev.blues.io/).