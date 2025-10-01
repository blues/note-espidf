// test/test_notecard_config.c

#include <stdio.h>
#include <string.h>
#include "unity.h"
#include "notecard_esp.h"
#include "esp_log.h"

static const char *TAG = "test_config";

void setUp(void)
{
    // This function is called before each test
    ESP_LOGI(TAG, "Setting up test");
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

void test_config_constants(void)
{
    // Verify Kconfig defaults
    TEST_ASSERT_EQUAL(0x17, CONFIG_NOTECARD_I2C_ADDRESS);
    TEST_ASSERT_EQUAL(100000, CONFIG_NOTECARD_I2C_FREQUENCY);
    TEST_ASSERT_EQUAL(9600, CONFIG_NOTECARD_UART_BAUDRATE);
    TEST_ASSERT_EQUAL_STRING("1.0.0", NOTECARD_ESP_VERSION);
}

void test_interface_enum_values(void)
{
    TEST_ASSERT_EQUAL(0, NOTECARD_INTERFACE_I2C);
    TEST_ASSERT_EQUAL(1, NOTECARD_INTERFACE_UART);
    TEST_ASSERT_EQUAL(2, NOTECARD_INTERFACE_MAX);
}

void app_main(void)
{
    printf("\n=== ESP-IDF Notecard Component Configuration Tests ===\n");

    UNITY_BEGIN();

    RUN_TEST(test_i2c_config_default_values);
    RUN_TEST(test_uart_config_default_values);
    RUN_TEST(test_config_constants);
    RUN_TEST(test_interface_enum_values);

    UNITY_END();
}
