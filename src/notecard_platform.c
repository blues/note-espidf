// src/notecard_platform.c

#include "notecard_platform.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include <string.h>
#include <stdlib.h>
#include "sdkconfig.h"

static const char *TAG = "notecard_platform";

// Timeout configuration from Kconfig
#define I2C_TIMEOUT_MS CONFIG_NOTECARD_I2C_TIMEOUT_MS
#define UART_TIMEOUT_MS CONFIG_NOTECARD_UART_TIMEOUT_MS

// Static configuration storage
static notecard_i2c_config_t g_i2c_config;
static notecard_uart_config_t g_uart_config;
static bool g_i2c_initialized = false;
static bool g_uart_initialized = false;
static i2c_master_bus_handle_t g_i2c_bus_handle = NULL;

//=============================================================================
// I2C Platform Implementation
//=============================================================================

esp_err_t notecard_platform_i2c_init(const notecard_i2c_config_t *config)
{
    if (g_i2c_initialized) {
        ESP_LOGW(TAG, "I2C already initialized");
        return ESP_OK;
    }

    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }

    // Store configuration
    memcpy(&g_i2c_config, config, sizeof(notecard_i2c_config_t));

    // Configure I2C master bus for synchronous operation
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = config->port,
        .scl_io_num = config->scl_pin,
        .sda_io_num = config->sda_pin,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = config->internal_pullup,
        .trans_queue_depth = 0,  // 0 = synchronous mode (blocking operations)
        .intr_priority = 0,      // Default interrupt priority
    };

    esp_err_t ret = i2c_new_master_bus(&bus_config, &g_i2c_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C master bus creation failed: %s", esp_err_to_name(ret));
        return ret;
    }

    g_i2c_initialized = true;
    ESP_LOGI(TAG, "I2C initialized on port %d (SDA:%d, SCL:%d, freq:%u)",
             config->port, config->sda_pin, config->scl_pin, config->frequency);
    ESP_LOGI(TAG, "I2C mode: synchronous (blocking), pullups: %s (external assumed present)",
             config->internal_pullup ? "internal enabled" : "internal disabled");

    return ESP_OK;
}

esp_err_t notecard_platform_i2c_deinit(void)
{
    if (!g_i2c_initialized) {
        return ESP_OK;
    }

    esp_err_t ret = i2c_del_master_bus(g_i2c_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C master bus deletion failed: %s", esp_err_to_name(ret));
        return ret;
    }

    g_i2c_bus_handle = NULL;
    g_i2c_initialized = false;
    ESP_LOGI(TAG, "I2C deinitialized");
    return ESP_OK;
}

bool notecard_platform_i2c_reset(uint16_t device_address)
{
    if (!g_i2c_initialized || !g_i2c_bus_handle) {
        ESP_LOGE(TAG, "I2C not initialized");
        return false;
    }

    ESP_LOGD(TAG, "I2C reset for device 0x%02X", device_address);

    // Perform a simple probe to check if device is responding
    esp_err_t ret = i2c_master_probe(g_i2c_bus_handle, device_address, 100);

    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "I2C device 0x%02X is responding", device_address);
        return true;
    } else {
        ESP_LOGW(TAG, "I2C device 0x%02X not responding: %s", device_address, esp_err_to_name(ret));
        return false;
    }
}

const char *notecard_platform_i2c_transmit(uint16_t device_address, uint8_t *buffer, uint16_t size)
{
    ESP_LOGD(TAG, "I2C transmit: addr=0x%02X, size=%u", device_address, size);

    if (!g_i2c_initialized || !g_i2c_bus_handle) {
        ESP_LOGE(TAG, "I2C not initialized");
        return "i2c not initialized";
    }

    if (!buffer) {
        ESP_LOGE(TAG, "Buffer is NULL");
        return "null buffer";
    }

    // Handle zero-length writes (Data Query operations)
    if (size == 0) {
        ESP_LOGI(TAG, "I2C TX: Data Query (zero-length write) to addr=0x%02X", device_address);

        // Create device configuration for zero-length write
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = device_address,
            .scl_speed_hz = g_i2c_config.frequency,
        };

        i2c_master_dev_handle_t dev_handle;
        esp_err_t ret = i2c_master_bus_add_device(g_i2c_bus_handle, &dev_cfg, &dev_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add I2C device for data query: %s", esp_err_to_name(ret));
            return esp_err_to_name(ret);
        }

        // Send zero-length write (just the I2C address, no data bytes)
        ret = i2c_master_transmit(dev_handle, NULL, 0, I2C_TIMEOUT_MS);
        i2c_master_bus_rm_device(dev_handle);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2C zero-length write failed: %s", esp_err_to_name(ret));
            return esp_err_to_name(ret);
        }

        ESP_LOGI(TAG, "Zero-length write successful");
        vTaskDelay(pdMS_TO_TICKS(2));
        return NULL; // Success
    }

    // Log JSON content if debug enabled
    if (size > 0 && buffer[0] >= 0x20 && buffer[0] <= 0x7E) {
        ESP_LOGD(TAG, "JSON TX: %.*s", size, (char*)buffer);
    }

    // Create device configuration for this transmission
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_address,
        .scl_speed_hz = g_i2c_config.frequency,
    };

    i2c_master_dev_handle_t dev_handle;
    esp_err_t ret = i2c_master_bus_add_device(g_i2c_bus_handle, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        return esp_err_to_name(ret);
    }

    // Notecard Serial-over-I2C protocol: prepend size byte to data
    uint8_t send_buffer[256];  // Max size for Notecard chunks
    if (size > 255) {
        ESP_LOGE(TAG, "Data size %u exceeds maximum chunk size 255", size);
        i2c_master_bus_rm_device(dev_handle);
        return "chunk too large";
    }

    send_buffer[0] = (uint8_t)(size & 0xFF);  // First byte is data length
    memcpy(&send_buffer[1], buffer, size);    // Copy actual data


    ret = i2c_master_transmit(dev_handle, send_buffer, size + 1, I2C_TIMEOUT_MS);

    // Remove device handle
    i2c_master_bus_rm_device(dev_handle);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C transmit failed to 0x%02X: %s (0x%x)",
                 device_address, esp_err_to_name(ret), ret);
        return esp_err_to_name(ret);
    }

    ESP_LOGD(TAG, "I2C transmitted %u bytes successfully", size);

    // Small delay after transmission for Notecard processing
    vTaskDelay(pdMS_TO_TICKS(2));

    return NULL; // NULL indicates success
}

const char *notecard_platform_i2c_receive(uint16_t device_address, uint8_t *buffer,
                                         uint16_t size, uint32_t *available)
{
    if (!g_i2c_initialized || !g_i2c_bus_handle) {
        ESP_LOGE(TAG, "I2C not initialized for receive");
        return "I2C not initialized";
    }

    // Handle special cases where buffer is NULL or size is 0 (Data Query operation)
    if (!buffer || size == 0) {
        ESP_LOGD(TAG, "I2C RX: Data Query operation (addr=0x%02X)", device_address);

        // Create device configuration
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = device_address,
            .scl_speed_hz = g_i2c_config.frequency,
        };

        i2c_master_dev_handle_t dev_handle;
        esp_err_t ret = i2c_master_bus_add_device(g_i2c_bus_handle, &dev_cfg, &dev_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add I2C device for data query: %s", esp_err_to_name(ret));
            return esp_err_to_name(ret);
        }

        // CORRECT Protocol: Send [0, 0] to request available data count (like Zephyr)
        uint8_t query_request[2] = {0, 0};  // Request format: [0, requested_size]
        ret = i2c_master_transmit(dev_handle, query_request, 2, I2C_TIMEOUT_MS);
        if (ret != ESP_OK) {
            i2c_master_bus_rm_device(dev_handle);
            ESP_LOGE(TAG, "Failed to send data query request: %s", esp_err_to_name(ret));
            return "I2C query write failed";
        }

        // Now read the 2-byte response
        uint8_t query_buffer[2];
        ret = i2c_master_receive(dev_handle, query_buffer, 2, I2C_TIMEOUT_MS);
        i2c_master_bus_rm_device(dev_handle);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2C data query read failed: %s", esp_err_to_name(ret));
            return "I2C query read failed";
        }

        // Parse the response
        uint8_t available_bytes = query_buffer[0];
        ESP_LOGD(TAG, "Data Query result: %u bytes available", available_bytes);

        if (available) {
            *available = available_bytes;
        }

        vTaskDelay(pdMS_TO_TICKS(25));  // Protocol timing delay

        return NULL; // Success
    }

    // Create device configuration for this transmission
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_address,
        .scl_speed_hz = g_i2c_config.frequency,
    };

    i2c_master_dev_handle_t dev_handle;
    esp_err_t ret = i2c_master_bus_add_device(g_i2c_bus_handle, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        return esp_err_to_name(ret);
    }

    // CORRECT Protocol (like Zephyr): Send read request first, then read response
    ESP_LOGD(TAG, "I2C RX: Reading %u bytes from addr=0x%02X", size, device_address);

    // Step 1: Send read request [0, requested_size] (following Zephyr example)
    uint8_t read_request[2] = {0, (uint8_t)(size & 0xFF)};
    ret = i2c_master_transmit(dev_handle, read_request, 2, I2C_TIMEOUT_MS);
    if (ret != ESP_OK) {
        i2c_master_bus_rm_device(dev_handle);
        ESP_LOGE(TAG, "Failed to send read request: %s", esp_err_to_name(ret));
        return "I2C read request failed";
    }

    ESP_LOGD(TAG, "Read request sent: [0, %u]", size);

    // Step 2: Read response [available_bytes, good_bytes, data...]
    uint16_t response_size = size + 2;  // 2-byte header + requested data
    uint8_t receive_buffer[256];

    if (response_size > sizeof(receive_buffer)) {
        ESP_LOGE(TAG, "Response size %u exceeds buffer size", response_size);
        i2c_master_bus_rm_device(dev_handle);
        return "read buffer overflow";
    }

    vTaskDelay(pdMS_TO_TICKS(25));  // Protocol timing delay

    ret = i2c_master_receive(dev_handle, receive_buffer, response_size, I2C_TIMEOUT_MS);

    // Remove device handle
    i2c_master_bus_rm_device(dev_handle);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C receive failed: %s", esp_err_to_name(ret));
        return "I2C read error";
    }

    // Parse Notecard protocol response: [available_bytes, good_bytes, data...]
    uint8_t available_bytes = receive_buffer[0];
    uint8_t good_bytes = receive_buffer[1];

    ESP_LOGD(TAG, "I2C RX: addr=0x%02X, requested=%u, available=%u, good=%u",
             device_address, size, available_bytes, good_bytes);
    ESP_LOG_BUFFER_HEXDUMP(TAG, receive_buffer, response_size, ESP_LOG_DEBUG);

    // Calculate how much actual data we got (subtract 2-byte header)
    uint16_t actual_data_bytes = (response_size >= 2) ? response_size - 2 : 0;

    ESP_LOGD(TAG, "Protocol validation: good_bytes=%u, actual_data=%u, buffer_size=%u",
             good_bytes, actual_data_bytes, size);

    // Validate the response
    if (good_bytes != actual_data_bytes) {
        ESP_LOGW(TAG, "I2C receive: protocol mismatch - good_bytes=%u, expected=%u",
                 good_bytes, actual_data_bytes);
        // This might be OK in some cases, continue with what we got
    }

    // Copy the actual data (skip the 2-byte header)
    if (good_bytes > 0 && buffer && size > 0) {
        uint16_t copy_size = (good_bytes < size) ? good_bytes : size;
        memcpy(buffer, &receive_buffer[2], copy_size);
        ESP_LOGD(TAG, "Copied %u bytes of data to buffer", copy_size);

        // Log JSON response if it looks like text
        if (copy_size > 0 && receive_buffer[2] >= 0x20 && receive_buffer[2] <= 0x7E) {
            ESP_LOGD(TAG, "JSON RX: %.*s", copy_size, (char*)&receive_buffer[2]);
        }
    }

    // Return the number of additional bytes available for next read
    if (available) {
        *available = available_bytes;
    }

    ESP_LOGD(TAG, "I2C received %u bytes from 0x%02X successfully", good_bytes, device_address);

    // Small delay after receive for Notecard processing
    vTaskDelay(pdMS_TO_TICKS(2));

    return NULL; // Success
}

//=============================================================================
// UART Platform Implementation
//=============================================================================

esp_err_t notecard_platform_uart_init(const notecard_uart_config_t *config)
{
    if (g_uart_initialized) {
        ESP_LOGW(TAG, "UART already initialized");
        return ESP_OK;
    }

    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }

    // Store configuration
    memcpy(&g_uart_config, config, sizeof(notecard_uart_config_t));

    // Configure UART parameters
    uart_config_t uart_conf = {
        .baud_rate = config->baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = (config->rts_pin != UART_PIN_NO_CHANGE && config->cts_pin != UART_PIN_NO_CHANGE)
                     ? UART_HW_FLOWCTRL_CTS_RTS : UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_param_config(config->port, &uart_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set UART pins
    ret = uart_set_pin(config->port, config->tx_pin, config->rx_pin,
                       config->rts_pin, config->cts_pin);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Install UART driver with buffers
    ret = uart_driver_install(config->port, config->rx_buffer_size,
                             config->tx_buffer_size, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    g_uart_initialized = true;
    ESP_LOGI(TAG, "UART initialized on port %d (TX:%d, RX:%d, baud:%u)",
             config->port, config->tx_pin, config->rx_pin, config->baudrate);

    return ESP_OK;
}

esp_err_t notecard_platform_uart_deinit(void)
{
    if (!g_uart_initialized) {
        return ESP_OK;
    }

    esp_err_t ret = uart_driver_delete(g_uart_config.port);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver delete failed: %s", esp_err_to_name(ret));
        return ret;
    }

    g_uart_initialized = false;
    ESP_LOGI(TAG, "UART deinitialized");
    return ESP_OK;
}

bool notecard_platform_uart_reset(void)
{
    if (!g_uart_initialized) {
        ESP_LOGE(TAG, "UART not initialized");
        return false;
    }

    esp_err_t ret = uart_flush(g_uart_config.port);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART flush failed: %s", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGD(TAG, "UART reset completed");
    return true;
}

void notecard_platform_uart_transmit(uint8_t *buffer, size_t size, bool flush)
{
    if (!g_uart_initialized || !buffer || size == 0) {
        ESP_LOGE(TAG, "Invalid UART transmit parameters");
        return;
    }

    int bytes_written = uart_write_bytes(g_uart_config.port, buffer, size);
    if (bytes_written != (int)size) {
        ESP_LOGE(TAG, "UART transmit incomplete: %d/%zu bytes", bytes_written, size);
        return;
    }

    if (flush) {
        esp_err_t ret = uart_wait_tx_done(g_uart_config.port, pdMS_TO_TICKS(UART_TIMEOUT_MS));
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "UART flush timeout: %s", esp_err_to_name(ret));
            // Don't return error here, data was still written
        }
    }

    ESP_LOGD(TAG, "UART transmitted %zu bytes%s", size, flush ? " (flushed)" : "");
}

bool notecard_platform_uart_available(void)
{
    if (!g_uart_initialized) {
        return false;
    }

    size_t bytes_available = 0;
    esp_err_t ret = uart_get_buffered_data_len(g_uart_config.port, &bytes_available);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get UART buffer length: %s", esp_err_to_name(ret));
        return false;
    }

    return (bytes_available > 0);
}

char notecard_platform_uart_receive(void)
{
    if (!g_uart_initialized) {
        return 0;
    }

    uint8_t data = 0;
    int bytes_read = uart_read_bytes(g_uart_config.port, &data, 1, pdMS_TO_TICKS(10));

    if (bytes_read != 1) {
        return 0; // Timeout or error
    }

    return (char)data;
}

//=============================================================================
// System Platform Implementation
//=============================================================================

uint32_t notecard_platform_millis(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

void notecard_platform_delay(uint32_t ms)
{
    if (ms == 0) {
        taskYIELD(); // Allow other tasks to run
    } else {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
}

void *notecard_platform_malloc(size_t size)
{
    // Use configured heap capabilities from Kconfig
    void *ptr = heap_caps_malloc(size, CONFIG_NOTECARD_HEAP_CAPS);

    if (!ptr && size > 0) {
        ESP_LOGE(TAG, "Failed to allocate %zu bytes", size);
    } else if (ptr) {
        ESP_LOGD(TAG, "Allocated %zu bytes at %p", size, ptr);
    }

    return ptr;
}

void notecard_platform_free(void *ptr)
{
    if (ptr) {
        ESP_LOGD(TAG, "Freeing memory at %p", ptr);
        heap_caps_free(ptr);
    }
}
