#ifndef NOTECARD_PLATFORM_H
#define NOTECARD_PLATFORM_H

/**
 * @file notecard_platform.h
 * @brief Platform abstraction layer for Notecard ESP-IDF component
 *
 * This file provides platform-specific implementations required by note-c
 */

#include "notecard.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// I2C platform functions
esp_err_t notecard_platform_i2c_init(const notecard_i2c_config_t *config);
esp_err_t notecard_platform_i2c_deinit(void);
bool notecard_platform_i2c_reset(uint16_t device_address);
const char *notecard_platform_i2c_transmit(uint16_t device_address, uint8_t *buffer, uint16_t size);
const char *notecard_platform_i2c_receive(uint16_t device_address, uint8_t *buffer, uint16_t size, uint32_t *available);
esp_err_t notecard_platform_i2c_scan(uint8_t start_addr, uint8_t end_addr, uint8_t *found_devices, uint8_t max_devices, uint8_t *device_count);

// UART platform functions
esp_err_t notecard_platform_uart_init(const notecard_uart_config_t *config);
esp_err_t notecard_platform_uart_deinit(void);
bool notecard_platform_uart_reset(void);
void notecard_platform_uart_transmit(uint8_t *buffer, size_t size, bool flush);
bool notecard_platform_uart_available(void);
char notecard_platform_uart_receive(void);

// System platform functions
uint32_t notecard_platform_millis(void);
void notecard_platform_delay(uint32_t ms);
void *notecard_platform_malloc(size_t size);
void notecard_platform_free(void *ptr);

// Mutex registration function
void notecard_platform_register_mutex_hooks(void);

#ifdef CONFIG_NOTECARD_I2C_MUTEX
// Public I2C mutex functions
void notecard_i2c_lock(void);
void notecard_i2c_unlock(void);
#endif

#ifdef __cplusplus
}
#endif

#endif // NOTECARD_PLATFORM_H
