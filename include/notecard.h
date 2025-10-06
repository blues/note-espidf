#ifndef NOTECARD_H
#define NOTECARD_H

/**
 * @file notecard.h
 * @brief ESP-IDF Component for Blues Notecard
 *
 * This component provides ESP-IDF integration for the Blues Notecard,
 * supporting both I2C and UART communication interfaces.
 *
 * @author Your Name
 * @version 1.0.0
 */

#include "sdkconfig.h"

// Override note-c's auto-detection of NOTE_C_LOW_MEM
// By default, disable low memory mode to enable full functionality.
// If NOTECARD_LOW_MEMORY is enabled in Kconfig, re-enable NOTE_C_LOW_MEM.
#include "note.h"
#ifndef CONFIG_NOTECARD_LOW_MEMORY
#ifdef NOTE_C_LOW_MEM
#undef NOTE_C_LOW_MEM
#endif
#endif

#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Version is automatically generated from idf_component.yml
#ifndef NOTE_ESPIDF_VER
#define NOTE_ESPIDF_VER "unknown"
#endif

#define NOTECARD_ESP_VERSION NOTE_ESPIDF_VER

/** Communication interface types */
typedef enum {
    NOTECARD_INTERFACE_I2C = 0,     /**< I2C interface */
    NOTECARD_INTERFACE_UART,        /**< UART interface */
    NOTECARD_INTERFACE_MAX
} notecard_interface_t;

/** I2C configuration structure */
typedef struct {
    i2c_port_t port;                /**< I2C port number */
    int sda_pin;                    /**< SDA GPIO pin */
    int scl_pin;                    /**< SCL GPIO pin */
    uint32_t frequency;             /**< I2C clock frequency in Hz */
    uint8_t address;                /**< Notecard I2C address */
    bool internal_pullup;           /**< Enable internal pull-up resistors */
} notecard_i2c_config_t;

/** UART configuration structure */
typedef struct {
    uart_port_t port;               /**< UART port number */
    int tx_pin;                     /**< TX GPIO pin */
    int rx_pin;                     /**< RX GPIO pin */
    int rts_pin;                    /**< RTS GPIO pin (optional) */
    int cts_pin;                    /**< CTS GPIO pin (optional) */
    uint32_t baudrate;              /**< UART baudrate */
    size_t tx_buffer_size;          /**< TX buffer size */
    size_t rx_buffer_size;          /**< RX buffer size */
} notecard_uart_config_t;

/** Main Notecard configuration structure */
typedef struct {
    notecard_interface_t interface; /**< Communication interface type */
    union {
        notecard_i2c_config_t i2c;  /**< I2C configuration */
        notecard_uart_config_t uart; /**< UART configuration */
    };
    bool enable_logging;              /**< Enable debug logging */
} notecard_config_t;

/**
 * @brief Default I2C configuration initializer
 *
 * Creates a default I2C configuration using values from Kconfig.
 * Values can be customized via menuconfig under "Notecard Configuration".
 */
#ifdef CONFIG_NOTECARD_LOGGING
#define NOTECARD_LOGGING_DEFAULT true
#else
#define NOTECARD_LOGGING_DEFAULT false
#endif

#ifdef CONFIG_NOTECARD_I2C_PULLUP
#define NOTECARD_I2C_PULLUP_DEFAULT true
#else
#define NOTECARD_I2C_PULLUP_DEFAULT false
#endif

#define NOTECARD_I2C_CONFIG_DEFAULT() {                    \
    .interface = NOTECARD_INTERFACE_I2C,                   \
    .i2c = {                                               \
        .port = CONFIG_NOTECARD_I2C_PORT,                  \
        .sda_pin = CONFIG_NOTECARD_I2C_SDA_PIN,            \
        .scl_pin = CONFIG_NOTECARD_I2C_SCL_PIN,            \
        .frequency = CONFIG_NOTECARD_I2C_FREQUENCY,        \
        .address = CONFIG_NOTECARD_I2C_ADDRESS,            \
        .internal_pullup = NOTECARD_I2C_PULLUP_DEFAULT     \
    },                                                     \
    .enable_logging = NOTECARD_LOGGING_DEFAULT                 \
}

/**
 * @brief Default UART configuration initializer
 *
 * Creates a default UART configuration using values from Kconfig.
 * Values can be customized via menuconfig under "Notecard Configuration".
 */
#define NOTECARD_UART_CONFIG_DEFAULT() {                   \
    .interface = NOTECARD_INTERFACE_UART,                  \
    .uart = {                                              \
        .port = CONFIG_NOTECARD_UART_PORT,                 \
        .tx_pin = CONFIG_NOTECARD_UART_TX_PIN,             \
        .rx_pin = CONFIG_NOTECARD_UART_RX_PIN,             \
        .rts_pin = UART_PIN_NO_CHANGE,                     \
        .cts_pin = UART_PIN_NO_CHANGE,                     \
        .baudrate = CONFIG_NOTECARD_UART_BAUDRATE,         \
        .tx_buffer_size = CONFIG_NOTECARD_UART_TX_BUFFER,  \
        .rx_buffer_size = CONFIG_NOTECARD_UART_RX_BUFFER   \
    },                                                     \
    .enable_logging = NOTECARD_LOGGING_DEFAULT                 \
}

/**
 * @brief Initialize the Notecard component
 *
 * @param config Pointer to configuration structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t notecard_init(const notecard_config_t *config);

/**
 * @brief Deinitialize the Notecard component
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t notecard_deinit(void);

/**
 * @brief Check if Notecard is initialized
 *
 * @return true if initialized, false otherwise
 */
bool notecard_is_initialized(void);

/**
 * @brief Enable or disable debug logging output
 *
 * @param enable true to enable tracing, false to disable
 */
void notecard_set_logging(bool enable);

#ifdef CONFIG_NOTECARD_I2C_MUTEX
/**
 * @brief Lock the I2C bus mutex
 *
 * Use this when accessing other I2C peripherals on the same bus to prevent
 * conflicts with Notecard I2C operations. Always pair with notecard_unlock_i2c().
 *
 * @note Only available if CONFIG_NOTECARD_I2C_MUTEX is enabled in Kconfig
 *
 * Example:
 * @code
 * notecard_lock_i2c();
 * // Access your I2C sensor here
 * i2c_master_transmit(my_sensor_handle, data, len, timeout);
 * notecard_unlock_i2c();
 * @endcode
 */
void notecard_lock_i2c(void);

/**
 * @brief Unlock the I2C bus mutex
 *
 * Must be called after notecard_lock_i2c() to release the I2C bus.
 *
 * @note Only available if CONFIG_NOTECARD_I2C_MUTEX is enabled in Kconfig
 */
void notecard_unlock_i2c(void);
#endif

#ifdef __cplusplus
}
#endif

#endif // NOTECARD_H
