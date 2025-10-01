// src/notecard_esp.c

#include "notecard_esp.h"
#include "notecard_platform.h"
#include "esp_log.h"
#include "esp_system.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "notecard_esp";

// Global state
static bool g_initialized = false;
static notecard_config_t g_config;
static bool g_trace_enabled = false;

// Forward declarations
static size_t notecard_trace_output(const char *message);
#ifndef NOTE_C_LOW_MEM
static void notecard_user_agent_register(void);
#endif

esp_err_t notecard_init(const notecard_config_t *config)
{
    if (g_initialized) {
        ESP_LOGW(TAG, "Notecard already initialized");
        return ESP_OK;
    }

    if (!config) {
        ESP_LOGE(TAG, "Configuration cannot be NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // Copy configuration
    memcpy(&g_config, config, sizeof(notecard_config_t));
    g_trace_enabled = config->enable_trace;

    esp_err_t ret = ESP_OK;

    // Initialize the platform-specific interface
    if (config->interface == NOTECARD_INTERFACE_I2C) {
        ret = notecard_platform_i2c_init(&config->i2c);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize I2C interface: %s", esp_err_to_name(ret));
            goto cleanup;
        }

        // Configure note-c for I2C
        NoteSetFnI2C(config->i2c.address, NOTE_I2C_MAX_DEFAULT,
                     notecard_platform_i2c_reset,
                     notecard_platform_i2c_transmit,
                     notecard_platform_i2c_receive);
    }
    else if (config->interface == NOTECARD_INTERFACE_UART) {
        ret = notecard_platform_uart_init(&config->uart);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize UART interface: %s", esp_err_to_name(ret));
            goto cleanup;
        }

        // Configure note-c for UART
        NoteSetFnSerial(notecard_platform_uart_reset,
                       notecard_platform_uart_transmit,
                       notecard_platform_uart_available,
                       notecard_platform_uart_receive);
    }
    else {
        ESP_LOGE(TAG, "Invalid interface type: %d", config->interface);
        ret = ESP_ERR_INVALID_ARG;
        goto cleanup;
    }

    // Set up note-c platform functions
    NoteSetFnDefault(notecard_platform_malloc,
                    notecard_platform_free,
                    notecard_platform_delay,
                    notecard_platform_millis);

    // Set up debug output
    if (g_trace_enabled) {
        NoteSetFnDebugOutput(notecard_trace_output);
    }

    // Register user agent (only available when not in low memory mode)
#ifndef NOTE_C_LOW_MEM
    notecard_user_agent_register();
#endif

    g_initialized = true;
    ESP_LOGI(TAG, "Notecard initialized successfully (%s interface)",
             config->interface == NOTECARD_INTERFACE_I2C ? "I2C" : "UART");

    return ESP_OK;

cleanup:
    return ret;
}

esp_err_t notecard_deinit(void)
{
    if (!g_initialized) {
        return ESP_OK;
    }

    // Deinitialize platform interface
    if (g_config.interface == NOTECARD_INTERFACE_I2C) {
        notecard_platform_i2c_deinit();
    } else if (g_config.interface == NOTECARD_INTERFACE_UART) {
        notecard_platform_uart_deinit();
    }

    g_initialized = false;
    ESP_LOGI(TAG, "Notecard deinitialized");

    return ESP_OK;
}

bool notecard_is_initialized(void)
{
    return g_initialized;
}

void notecard_set_trace(bool enable)
{
    g_trace_enabled = enable;

    if (g_initialized) {
        if (enable) {
            NoteSetFnDebugOutput(notecard_trace_output);
        } else {
            NoteSetFnDebugOutput(NULL);
        }
    }

    ESP_LOGI(TAG, "Debug trace %s", enable ? "enabled" : "disabled");
}

// Private functions

static size_t notecard_trace_output(const char *message)
{
    if (message && g_trace_enabled) {
        ESP_LOGI("notecard_trace", "%s", message);
        return strlen(message);
    }
    return 0;
}

#ifndef NOTE_C_LOW_MEM
static void notecard_user_agent_register(void)
{
    NoteSetUserAgent((char *)("note-espidf " NOTECARD_ESP_VERSION));
}
#endif
