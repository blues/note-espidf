# UART Usage Example

This example demonstrates basic Notecard usage with ESP-IDF using UART communication in a standard ESP-IDF application structure, showing how to:

- Initialize the Notecard with UART communication
- Configure Notehub connection
- Use FreeRTOS tasks for sensor data collection
- Read built-in sensors from the Notecard (temperature and voltage)
- Send sensor data to Notehub using a dedicated task
- Handle JSON requests and responses with proper thread safety

## Hardware Requirements

- ESP32 development board
- Blues Notecard (cellular or WiFi)
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

## Features

### ESP-IDF Application Structure
This example follows standard ESP-IDF patterns:
- **Task-based architecture**: Uses FreeRTOS tasks for sensor data collection
- **Thread safety**: Mutex protection for Notecard access
- **Resource management**: Proper task creation, cleanup, and monitoring
- **Error handling**: Structured error handling with appropriate return codes
- **Modular design**: Separate functions for hardware init and configuration

### UART Communication
- Standard ESP-IDF UART driver (`driver/uart.h`)
- Configurable pins, baudrate, and buffer sizes
- Optional hardware flow control support (RTS/CTS)
- Proper UART initialization and cleanup

### Notecard Integration
- Full JSON API support via note-c library
- Robust error handling and recovery
- Thread-safe Notecard operations using mutex
- UART stabilization delay after initialization

### Data Collection Task
- Dedicated FreeRTOS task for sensor reading
- Reads temperature from Notecard's built-in sensor
- Reads voltage from Notecard's V+ pin
- Includes system health monitoring (heap usage)
- Sends data to Notehub every 15 seconds
- Automatic task completion after 25 readings
- Interface identifier in data payload ("UART")

## Expected Output

When running with a connected Notecard via UART, you should see:
```
I (123) notecard_uart: === ESP-IDF Notecard UART Usage Example ===
I (124) notecard_uart: Component version: 1.0.0
I (125) notecard_uart: ESP32 chip: esp32 rev 3, 2 cores
I (126) notecard_uart: Free heap: 295168 bytes
I (127) notecard_uart: Initializing Notecard hardware...
I (128) notecard_uart: Notecard hardware initialized successfully
I (1129) notecard_uart: Configuring Notecard for Notehub connection...
I (1200) notecard_uart: Product ID set: com.your-company.your-product
I (1250) notecard_uart: Notecard configured for Notehub connection
I (1260) notecard_uart: Notecard sensor task created successfully
I (1270) notecard_uart: Notecard sensor task started
I (1280) notecard_uart: Reading sensors (sample 1/25)...
I (1350) notecard_uart: Sample 1 sent: temp=23.45°C, voltage=5.12V, heap=290123
I (61290) notecard_uart: System status: heap=290000 bytes, task running=yes
```

## UART Configuration Details

The example uses these UART settings:
- **Port**: UART_NUM_1 (UART1)
- **Baudrate**: 9600 bps (Notecard standard)
- **Data bits**: 8
- **Parity**: None
- **Stop bits**: 1
- **Flow control**: Disabled (no RTS/CTS)
- **TX buffer**: 1024 bytes
- **RX buffer**: 2048 bytes (larger for JSON responses)

## Pin Mapping

| ESP32 Pin | Notecard Pin | Function |
|-----------|--------------|----------|
| GPIO 17   | RX           | UART TX  |
| GPIO 16   | TX           | UART RX  |
| 3.3V      | V+           | Power    |
| GND       | GND          | Ground   |

## Troubleshooting

- **No response from Notecard**: Check UART wiring (TX/RX crossed correctly)
- **Build fails**: Ensure ESP-IDF v5.5+ is properly installed and sourced
- **Connection timeouts**: Verify baudrate matches (9600 bps)
- **Buffer overruns**: Increase RX buffer size if handling large JSON responses
- **Communication errors**: Check power supply stability
- **Pin conflicts**: Ensure GPIO 16/17 are not used by other peripherals

## Customization

To modify the UART configuration:

```c
// Change pins
config.uart.tx_pin = 4;   // Use GPIO 4 for TX
config.uart.rx_pin = 5;   // Use GPIO 5 for RX

// Change baudrate (if Notecard supports it)
config.uart.baudrate = 115200;

// Enable hardware flow control
config.uart.rts_pin = 18;
config.uart.cts_pin = 19;

// Adjust buffer sizes
config.uart.tx_buffer_size = 2048;
config.uart.rx_buffer_size = 4096;
```

## Comparison with I2C Example

| Feature | UART | I2C |
|---------|------|-----|
| Wiring | 4 wires | 4 wires |
| Pins | Fixed UART pins | Any GPIO |
| Speed | 9600 bps | 100 kHz |
| Setup | Simple | Bus management |
| Multiple devices | Point-to-point | Bus (multiple devices) |
| Error detection | Built-in | Protocol-level |

## Next Steps

After running this UART example:
1. Compare performance with I2C example
2. Implement hardware flow control if needed
3. Add custom sensor data collection
4. Explore different baudrates (if supported)
5. Implement power management features

For more information, visit [Blues Developer Documentation](https://dev.blues.io/).
