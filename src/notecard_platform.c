#include "notecard_platform.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
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

// FreeRTOS mutex handles for thread-safe access
#ifdef CONFIG_NOTECARD_I2C_MUTEX
static SemaphoreHandle_t g_i2c_mutex = NULL;
#endif
static SemaphoreHandle_t g_notecard_mutex = NULL;

//=============================================================================
// Mutex Platform Implementation
//=============================================================================

#ifdef CONFIG_NOTECARD_I2C_MUTEX
static void notecard_platform_i2c_lock(void)
{
    if (g_i2c_mutex != NULL) {
        for (;xSemaphoreTake(g_i2c_mutex, portMAX_DELAY) != pdTRUE;);
    }
}

static void notecard_platform_i2c_unlock(void)
{
    if (g_i2c_mutex != NULL) {
        xSemaphoreGive(g_i2c_mutex);
    }
}
#endif

static void notecard_platform_note_lock(void)
{
    if (g_notecard_mutex != NULL) {
        for (;xSemaphoreTake(g_notecard_mutex, portMAX_DELAY) != pdTRUE;);
    }
}

static void notecard_platform_note_unlock(void)
{
    if (g_notecard_mutex != NULL) {
        xSemaphoreGive(g_notecard_mutex);
    }
}

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

    // Create mutexes
    if (g_notecard_mutex == NULL) {
        g_notecard_mutex = xSemaphoreCreateMutex();
        if (g_notecard_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create Notecard mutex");
            return ESP_ERR_NO_MEM;
        }
    }

#ifdef CONFIG_NOTECARD_I2C_MUTEX
    if (g_i2c_mutex == NULL) {
        g_i2c_mutex = xSemaphoreCreateMutex();
        if (g_i2c_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create I2C mutex");
            vSemaphoreDelete(g_notecard_mutex);
            g_notecard_mutex = NULL;
            return ESP_ERR_NO_MEM;
        }
    }
#endif

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
#ifdef CONFIG_NOTECARD_I2C_MUTEX
        vSemaphoreDelete(g_i2c_mutex);
        g_i2c_mutex = NULL;
#endif
        vSemaphoreDelete(g_notecard_mutex);
        g_notecard_mutex = NULL;
        return ret;
    }

    g_i2c_initialized = true;
    ESP_LOGI(TAG, "I2C initialized on port %d (SDA:%d, SCL:%d, %uHz)",
             config->port, config->sda_pin, config->scl_pin, config->frequency);

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

    // Clean up mutexes
#ifdef CONFIG_NOTECARD_I2C_MUTEX
    if (g_i2c_mutex != NULL) {
        vSemaphoreDelete(g_i2c_mutex);
        g_i2c_mutex = NULL;
    }
#endif

    if (g_notecard_mutex != NULL) {
        vSemaphoreDelete(g_notecard_mutex);
        g_notecard_mutex = NULL;
    }

    ESP_LOGI(TAG, "I2C deinitialized");
    return ESP_OK;
}

bool notecard_platform_i2c_reset(uint16_t device_address)
{
    if (!g_i2c_initialized || !g_i2c_bus_handle) {
        ESP_LOGE(TAG, "I2C not initialized");
        return false;
    }

    esp_err_t ret = i2c_master_probe(g_i2c_bus_handle, device_address, 100);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "I2C device 0x%02X not responding: %s", device_address, esp_err_to_name(ret));
        return false;
    }

    return true;
}

const char *notecard_platform_i2c_transmit(uint16_t device_address, uint8_t *buffer, uint16_t size)
{
    if (!g_i2c_initialized || !g_i2c_bus_handle) {
        ESP_LOGE(TAG, "I2C not initialized");
        return "i2c not initialized";
    }

    if (!buffer) {
        return "null buffer";
    }

    if (size > 255) {
        ESP_LOGE(TAG, "Data size %u exceeds maximum chunk size 255", size);
        return "chunk too large";
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
    uint8_t send_buffer[size + 1];
    send_buffer[0] = (uint8_t)(size & 0xFF);
    memcpy(&send_buffer[1], buffer, size);

    ret = i2c_master_transmit(dev_handle, send_buffer, size + 1, I2C_TIMEOUT_MS);
    i2c_master_bus_rm_device(dev_handle);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C transmit failed: %s", esp_err_to_name(ret));
        return esp_err_to_name(ret);
    }

    vTaskDelay(pdMS_TO_TICKS(2));
    return NULL;
}

const char *notecard_platform_i2c_receive(uint16_t device_address, uint8_t *buffer,
                                         uint16_t size, uint32_t *available)
{
    if (!g_i2c_initialized || !g_i2c_bus_handle) {
        ESP_LOGE(TAG, "I2C not initialized for receive");
        return "I2C not initialized";
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

    if (size > 255) {
        ESP_LOGE(TAG, "Requested size %u exceeds maximum 255", size);
        i2c_master_bus_rm_device(dev_handle);
        return "size too large";
    }

    // Send read request [0, requested_size]
    uint8_t read_request[2] = {0, (uint8_t)(size & 0xFF)};
    ret = i2c_master_transmit(dev_handle, read_request, 2, I2C_TIMEOUT_MS);
    if (ret != ESP_OK) {
        i2c_master_bus_rm_device(dev_handle);
        ESP_LOGE(TAG, "I2C read request failed: %s", esp_err_to_name(ret));
        return "I2C read request failed";
    }

    uint16_t response_size = size + 2;  // 2-byte header + requested data
    uint8_t receive_buffer[response_size];

    vTaskDelay(pdMS_TO_TICKS(25));

    ret = i2c_master_receive(dev_handle, receive_buffer, response_size, I2C_TIMEOUT_MS);
    i2c_master_bus_rm_device(dev_handle);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C receive failed: %s", esp_err_to_name(ret));
        return "I2C read error";
    }

    // Parse Notecard protocol response: [available_bytes, good_bytes, data...]
    uint8_t good_bytes = receive_buffer[1];

    // Copy the actual data (skip the 2-byte header)
    if (good_bytes > 0 && buffer && size > 0) {
        uint16_t copy_size = (good_bytes < size) ? good_bytes : size;
        memcpy(buffer, &receive_buffer[2], copy_size);
    }

    if (available) {
        *available = receive_buffer[0];
    }

    vTaskDelay(pdMS_TO_TICKS(2));
    return NULL;
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

    // Create Notecard mutex
    if (g_notecard_mutex == NULL) {
        g_notecard_mutex = xSemaphoreCreateMutex();
        if (g_notecard_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create Notecard mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    memcpy(&g_uart_config, config, sizeof(notecard_uart_config_t));

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

    ret = uart_set_pin(config->port, config->tx_pin, config->rx_pin,
                       config->rts_pin, config->cts_pin);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_driver_install(config->port, config->rx_buffer_size,
                             config->tx_buffer_size, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
        vSemaphoreDelete(g_notecard_mutex);
        g_notecard_mutex = NULL;
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

    // Clean up Notecard mutex
    if (g_notecard_mutex != NULL) {
        vSemaphoreDelete(g_notecard_mutex);
        g_notecard_mutex = NULL;
    }

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
        }
    }
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
    void *ptr = malloc(size);
    if (!ptr && size > 0) {
        ESP_LOGE(TAG, "Failed to allocate %zu bytes", size);
    }
    return ptr;
}

void notecard_platform_free(void *ptr)
{
    free(ptr);
}

//=============================================================================
// Mutex Hook Registration
//=============================================================================

void notecard_platform_register_mutex_hooks(void)
{
    // Register mutex hooks with note-c based on Kconfig settings
    // Enable Notecard mutexes (I2C mutex is optional)
    NoteSetFnNoteMutex(notecard_platform_note_lock,
                       notecard_platform_note_unlock);

#ifdef CONFIG_NOTECARD_I2C_MUTEX
    // Enable I2C mutexes
    NoteSetFnI2CMutex(notecard_platform_i2c_lock,
                   notecard_platform_i2c_unlock);
#endif
}

//=============================================================================
// Public I2C Mutex API
//=============================================================================

#ifdef CONFIG_NOTECARD_I2C_MUTEX
void notecard_i2c_lock(void)
{
    notecard_platform_i2c_lock();
}

void notecard_i2c_unlock(void)
{
    notecard_platform_i2c_unlock();
}
#endif
