// test/main/test_main.c

#include <stdio.h>
#include "unity.h"
#include "notecard_esp.h"
#include "esp_log.h"

static const char *TAG = "test_main";

void setUp(void)
{
    // This function is called before each test
    ESP_LOGI(TAG, "Setting up test");

    // Ensure notecard is deinitialized before each test
    if (notecard_is_initialized()) {
        notecard_deinit();
    }
}

void tearDown(void)
{
    // This function is called after each test
    ESP_LOGI(TAG, "Tearing down test");

    // Ensure notecard is deinitialized after each test
    if (notecard_is_initialized()) {
        notecard_deinit();
    }
}

void test_i2c_config_default_values(void)
{
    ESP_LOGI(TAG, "Testing I2C config default values");

    notecard_config_t config = NOTECARD_I2C_CONFIG_DEFAULT();

    TEST_ASSERT_EQUAL(NOTECARD_INTERFACE_I2C, config.interface);
    TEST_ASSERT_EQUAL(CONFIG_NOTECARD_I2C_PORT, config.i2c.port);
    TEST_ASSERT_EQUAL(CONFIG_NOTECARD_I2C_SDA_PIN, config.i2c.sda_pin);
    TEST_ASSERT_EQUAL(CONFIG_NOTECARD_I2C_SCL_PIN, config.i2c.scl_pin);
    TEST_ASSERT_EQUAL(CONFIG_NOTECARD_I2C_FREQUENCY, config.i2c.frequency);
    TEST_ASSERT_EQUAL(CONFIG_NOTECARD_I2C_ADDRESS, config.i2c.address);
    TEST_ASSERT_EQUAL(CONFIG_NOTECARD_I2C_PULLUP, config.i2c.internal_pullup);
    TEST_ASSERT_FALSE(config.enable_trace);
}

void test_uart_config_default_values(void)
{
    ESP_LOGI(TAG, "Testing UART config default values");

    notecard_config_t config = NOTECARD_UART_CONFIG_DEFAULT();

    TEST_ASSERT_EQUAL(NOTECARD_INTERFACE_UART, config.interface);
    TEST_ASSERT_EQUAL(CONFIG_NOTECARD_UART_PORT, config.uart.port);
    TEST_ASSERT_EQUAL(CONFIG_NOTECARD_UART_TX_PIN, config.uart.tx_pin);
    TEST_ASSERT_EQUAL(CONFIG_NOTECARD_UART_RX_PIN, config.uart.rx_pin);
    TEST_ASSERT_EQUAL(UART_PIN_NO_CHANGE, config.uart.rts_pin);
    TEST_ASSERT_EQUAL(UART_PIN_NO_CHANGE, config.uart.cts_pin);
    TEST_ASSERT_EQUAL(CONFIG_NOTECARD_UART_BAUDRATE, config.uart.baudrate);
    TEST_ASSERT_EQUAL(CONFIG_NOTECARD_UART_TX_BUFFER, config.uart.tx_buffer_size);
    TEST_ASSERT_EQUAL(CONFIG_NOTECARD_UART_RX_BUFFER, config.uart.rx_buffer_size);
    TEST_ASSERT_FALSE(config.enable_trace);
}

void test_initialization_without_hardware(void)
{
    ESP_LOGI(TAG, "Testing initialization logic");

    // Test that component reports not initialized initially
    TEST_ASSERT_FALSE(notecard_is_initialized());

    // Test NULL configuration handling
    esp_err_t ret = notecard_init(NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);
    TEST_ASSERT_FALSE(notecard_is_initialized());
}

void test_deinit_functionality(void)
{
    ESP_LOGI(TAG, "Testing deinit functionality");

    // Ensure not initialized
    TEST_ASSERT_FALSE(notecard_is_initialized());

    // Test that deinit is safe when not initialized
    notecard_deinit();
    TEST_ASSERT_FALSE(notecard_is_initialized());
}

void test_trace_functionality(void)
{
    ESP_LOGI(TAG, "Testing trace functionality");

    // Test trace enable/disable (should not crash)
    notecard_set_trace(true);
    notecard_set_trace(false);

    // This test passes if no crash occurs
    TEST_ASSERT_TRUE(true);
}

void test_constants(void)
{
    ESP_LOGI(TAG, "Testing constants");

    TEST_ASSERT_EQUAL_STRING("1.0.0", NOTECARD_ESP_VERSION);
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== ESP-IDF Notecard Component Unit Tests ===");
    ESP_LOGI(TAG, "Component version: %s", NOTECARD_ESP_VERSION);

    // Wait a moment for system to stabilize
    vTaskDelay(pdMS_TO_TICKS(1000));

    UNITY_BEGIN();

    RUN_TEST(test_constants);
    RUN_TEST(test_i2c_config_default_values);
    RUN_TEST(test_uart_config_default_values);
    RUN_TEST(test_initialization_without_hardware);
    RUN_TEST(test_deinit_functionality);
    RUN_TEST(test_trace_functionality);

    UNITY_END();

    ESP_LOGI(TAG, "=== All Tests Completed ===");

    // Keep running to see results
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
