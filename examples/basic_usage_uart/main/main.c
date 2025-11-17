// examples/uart_usage/main/main.c

// Copyright 2025 Blues Inc.  All rights reserved.
//
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.
//
// This example demonstrates how to use the Notecard with ESP-IDF via UART
// in a standard ESP-IDF application structure using FreeRTOS tasks.
//
// Using the Notecard library, you can easily construct JSON commands and
// also extract responses from the Notecard.
//
// This example shows how to:
// - Initialize the Notecard with UART communication
// - Configure Notehub connection
// - Read built-in sensors from the Notecard
// - Send sensor data to Notehub using a dedicated task
// - Handle JSON requests and responses properly

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "notecard.h"

static const char *TAG = "notecard_uart";

#define PRODUCT_UID "com.my-company.my-name:my-project"

#ifndef PRODUCT_UID
#define PRODUCT_UID "" // "com.my-company.my-name:my-project"
#pragma message "PRODUCT_UID is not defined in this example. Please ensure your Notecard has a product identifier set before running this example or define it in code here. More details at https://dev.blues.io/tools-and-sdks/samples/product-uid"
#endif

#define myProductID PRODUCT_UID

// Task handles
static TaskHandle_t notecard_task_handle = NULL;
static bool notecard_initialized = false;

// Task stack sizes
#define NOTECARD_TASK_STACK_SIZE    (4 * 1024)
#define NOTECARD_TASK_PRIORITY      (tskIDLE_PRIORITY + 5)

static esp_err_t notecard_hardware_init(void)
{
    ESP_LOGI(TAG, "Initializing Notecard hardware...");

    // Configure Notecard for UART communication using Kconfig defaults
    notecard_config_t config = NOTECARD_UART_CONFIG_DEFAULT();

    // Optional: Override specific settings if needed
    // config.uart.tx_pin = 17;
    // config.uart.rx_pin = 16;
    // config.uart.baudrate = 9600;
    // config.uart.tx_buffer_size = 1024;
    // config.uart.rx_buffer_size = 1024;

    // Initialize Notecard
    esp_err_t ret = notecard_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Notecard: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "Check wiring and power supply");
        return ret;
    }

    ESP_LOGI(TAG, "Notecard hardware initialized successfully");
    return ESP_OK;
}

static esp_err_t notecard_configure_hub(void)
{
    ESP_LOGI(TAG, "Configuring Notecard for Notehub connection...");

    // Create hub.set request
    J *req = NoteNewRequest("hub.set");

    // Set Notehub product ID
    JAddStringToObject(req, "product", CONFIG_NOTEHUB_PRODUCT_UID);

    // Set connection mode to continuous for this example
    JAddStringToObject(req, "mode", "continuous");

    // Set serial number
    JAddStringToObject(req, "sn", "espidf-i2c-basic");

    // Send the request with retry
    bool success = NoteRequestWithRetry(req, 5);
    if (success) {
        ESP_LOGI(TAG, "Notecard configured for Notehub connection");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to configure Notecard");
        return ESP_FAIL;
    }
}

static void notecard_sensor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Notecard sensor task started");

    unsigned eventCounter = 0;
    const unsigned max_readings = 25;
    const TickType_t reading_interval = pdMS_TO_TICKS(15000); // 15 seconds

    while (eventCounter < max_readings) {
        eventCounter++;

        ESP_LOGI(TAG, "Reading sensors (sample %d/%d)...", eventCounter, max_readings);

        // Read temperature from Notecard's built-in sensor
        double temperature = 0;
        J *rsp = NoteRequestResponse(NoteNewRequest("card.temp"));
        if (rsp != NULL) {
            temperature = JGetNumber(rsp, "value");
            NoteDeleteResponse(rsp);
        } else {
            ESP_LOGW(TAG, "Failed to read temperature");
        }

        // Read voltage from Notecard's V+ pin
        double voltage = 0;
        rsp = NoteRequestResponse(NoteNewRequest("card.voltage"));
        if (rsp != NULL) {
            voltage = JGetNumber(rsp, "value");
            NoteDeleteResponse(rsp);
        } else {
            ESP_LOGW(TAG, "Failed to read voltage");
        }

        // Send data to Notehub
        J *req = NoteNewRequest("note.add");
        JAddBoolToObject(req, "sync", true);

        J *body = JAddObjectToObject(req, "body");
        JAddNumberToObject(body, "temp", temperature);
        JAddNumberToObject(body, "voltage", voltage);
        JAddNumberToObject(body, "count", eventCounter);

        bool success = NoteRequest(req);
        if (success) {
            ESP_LOGI(TAG, "Sample %d sent: temp=%.2f°C, voltage=%.2fV",
                        eventCounter, temperature, voltage);
        } else {
            ESP_LOGE(TAG, "Failed to send sample %d", eventCounter);
        }

        // Wait before next reading
        vTaskDelay(reading_interval);
    }

    ESP_LOGI(TAG, "Sensor task completed %d readings. Task ending.", max_readings);

    // Clean up and delete task
    notecard_task_handle = NULL;
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== ESP-IDF Notecard Basic Usage UART Example ===");
    ESP_LOGI(TAG, "Component version: %s", NOTECARD_ESP_VERSION);

    // Print system information
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "ESP32 chip: %s rev %d, %d cores",
             CONFIG_IDF_TARGET, chip_info.revision, chip_info.cores);
    ESP_LOGI(TAG, "Free heap: %u bytes", esp_get_free_heap_size());

    // Initialize Notecard hardware and configure for Notehub
    esp_err_t ret = notecard_hardware_init();
    if (ret == ESP_OK) {
        ret = notecard_configure_hub();
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Notecard initialization failed");
        return;
    }

    notecard_initialized = true;
    ESP_LOGI(TAG, "Notecard initialization complete");

    // Create sensor reading task
    BaseType_t task_created = xTaskCreate(
        notecard_sensor_task,
        "notecard_sensor",
        NOTECARD_TASK_STACK_SIZE,
        NULL,
        NOTECARD_TASK_PRIORITY,
        &notecard_task_handle
    );

    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Notecard sensor task");
        return;
    }

    ESP_LOGI(TAG, "Notecard sensor task created successfully");

    // Main task monitoring loop
    while (1) {
        if (notecard_task_handle == NULL) {
            ESP_LOGI(TAG, "Sensor task completed. Application finished.");
            ESP_LOGI(TAG, "Final heap size: %u bytes", esp_get_free_heap_size());
            break;
        }

        // Monitor system health every 60 seconds
        vTaskDelay(pdMS_TO_TICKS(60000));
        ESP_LOGI(TAG, "System status: heap=%u bytes, task running=%s",
                 esp_get_free_heap_size(),
                 (notecard_task_handle != NULL) ? "yes" : "no");
    }

    ESP_LOGI(TAG, "Application ended gracefully");
}
