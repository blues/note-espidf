# ESP-IDF Notecard Component

An ESP-IDF component for integrating Espressif devices with the Blues [Notecard](https://blues.com/products/notecard/).

## Installation

```bash
idf.py add-dependency "blues/note-espidf"
```

## Usage

### I2C Example

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

The component can be configured via menuconfig.

```bash
idf.py menuconfig
Component config  --->  Notecard Configuration
```

## Examples

For examples, see the [examples](examples) directory.

Examples are designed to run on:

- [Adafruit ESP32 Feather V2](https://learn.adafruit.com/adafruit-esp32-feather-v2?view=all)
- [Notecard](https://blues.com/products/notecard/)
- [Notecarrier F](https://blues.com/products/notecarrier/notecarrier-f/)

If you wish to use a different MCU board, you will need to adjust the GPIO pins used for the Notecard.
See the Kconfig file for the default GPIO pins.

## Documentation

For more information, visit [Blues Developer Documentation](https://dev.blues.io/).
