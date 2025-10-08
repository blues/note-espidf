# ESP-IDF Notecard Component

An ESP-IDF component for integrating Espressif devices with the Blues [Notecard](https://blues.com/products/notecard/).
This component provides a thread-safe interface to the Notecard using the [note-c](https://github.com/blues/note-c) library.

## Installation

```bash
idf.py add-dependency "blues/notecard"
```

## Usage

The component can be used to communicate with the Notecard using I2C or UART.
The GPIO pins can either be set within application code or via `menuconfig`.

### Basic Example

```c
#include "notecard.h"

void app_main(void) {
    // Initialize with default I2C configuration
    notecard_config_t config = NOTECARD_I2C_CONFIG_DEFAULT();
    // or with UART
    // notecard_config_t config = NOTECARD_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(notecard_init(&config));

    // Send a request using note-c API
    J *req = NoteNewRequest("hub.set");
    JAddStringToObject(req, "product", "com.your-company:your-product");
    JAddStringToObject(req, "mode", "continuous");
    if (!NoteRequest(req)) {
        ESP_LOGE("app", "hub.set failed");
    }
}
```

## Configuration

The component can be further configured with `menuconfig`.

```bash
idf.py menuconfig
Component config  --->  Notecard Configuration
```

## Thread Safety

The component automatically provides thread-safe access to the Notecard in multi-threaded FreeRTOS applications.
The underlying [note-c](https://github.com/blues/note-c) library protects the Notecard from concurrent access using internal mutexes, so no additional locking is required for normal use.

### I2C Bus Sharing

If you have other I2C peripherals on the same bus as the Notecard, register your I2C mutex with `note-c` using `NoteSetFnI2CMutex()` to minimize the time spent under lock.

 For your convenience, we have provided a default implementation of I2C mutex APIs to coordinate access (example shown below):

```c
#include "notecard.h"

// Access your I2C peripherals
notecard_lock_i2c();
i2c_master_transmit(my_peripheral_handle, data, len, timeout);
notecard_unlock_i2c();
```

This ensures the `note-c` won't attempt I2C communication while you're accessing your other peripherals.

In order to enable/disable the provided I2C bus mutex, for example to use your own mutex, use `menuconfig`:

```
Component config  --->  Notecard Configuration  --->  Default I2C Configuration  --->  [ ] Enable I2C mutex
```

> Note: The I2C mutex is enabled by default.

## Examples

For examples, see the [examples](examples) directory.

Examples are designed to run on:

- [Adafruit ESP32 Feather V2](https://learn.adafruit.com/adafruit-esp32-feather-v2?view=all)
- [Notecard](https://blues.com/products/notecard/)
- [Notecarrier F](https://blues.com/products/notecarrier/notecarrier-f/)

If you wish to use a different ESP32 board (or different ESP32 chip), you will need to adjust the GPIO pins used for the Notecard.
See the [Kconfig](Kconfig) file for the default I2C and UART GPIO pins.

## Documentation

For more information, visit [Blues Developer Documentation](https://dev.blues.io/).
