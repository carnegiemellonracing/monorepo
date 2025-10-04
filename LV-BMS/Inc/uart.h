/**
 * @file uart.h
 * @brief Board-specific UART interface for LV-BMS.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef UART_H
#define UART_H

#include <CMR/uart.h>    // CMR UART interface

/**
 * @brief UART interface for voltage and temperature monitoring.
 */
typedef enum {
    UART_VOLTAGE_TEMP = 0,  /**< @brief UART for voltage and temperature data */
    UART_LEN                /**< @brief Total UART interfaces. */
} uart_interface_t;

/**
 * @brief UART message types for communication with external sensor.
 */
typedef enum {
    UART_MSG_REQUEST_VOLTAGES = 0x01,    /**< @brief Request voltage data */
    UART_MSG_REQUEST_TEMPS = 0x02,       /**< @brief Request temperature data */
    UART_MSG_REQUEST_CURRENT = 0x03,     /**< @brief Request current data */
    UART_MSG_RESPONSE_VOLTAGES = 0x11,   /**< @brief Voltage data response */
    UART_MSG_RESPONSE_TEMPS = 0x12,      /**< @brief Temperature data response */
    UART_MSG_RESPONSE_CURRENT = 0x13,    /**< @brief Current data response */
    UART_MSG_ERROR = 0xFF                 /**< @brief Error response */
} uart_msg_type_t;

/**
 * @brief UART voltage data structure.
 */
typedef struct {
    uint16_t cell_voltages[6];    /**< @brief Cell voltages in millivolts */
    uint8_t overvoltage_flags;    /**< @brief Overvoltage flags (bit per cell) */
} uart_voltage_data_t;

/**
 * @brief UART temperature data structure.
 */
typedef struct {
    uint16_t temperatures[8];     /**< @brief Temperatures in 0.01°C */
    uint8_t overtemperature_flags; /**< @brief Overtemperature flags (bit per sensor) */
} uart_temp_data_t;

/**
 * @brief UART current data structure.
 */
typedef struct {
    int16_t current_ma;           /**< @brief Current in milliamps */
} uart_current_data_t;

/**
 * @brief UART message header.
 */
typedef struct {
    uint8_t msg_type;             /**< @brief Message type */
    uint8_t length;               /**< @brief Data length */
    uint8_t checksum;             /**< @brief Simple checksum */
} uart_msg_header_t;

/**
 * @brief UART message structure.
 */
typedef struct {
    uart_msg_header_t header;     /**< @brief Message header */
    uint8_t data[64];             /**< @brief Message data */
} uart_message_t;

/**
 * @brief Initializes the UART interface.
 */
void uart_init(void);

/**
 * @brief Requests voltage data from external sensor.
 * 
 * @return 0 on success, 1 on error
 */
uint8_t uart_request_voltages(void);

/**
 * @brief Requests temperature data from external sensor.
 * 
 * @return 0 on success, 1 on error
 */
uint8_t uart_request_temperatures(void);

/**
 * @brief Requests current data from external sensor.
 * 
 * @return 0 on success, 1 on error
 */
uint8_t uart_request_current(void);

/**
 * @brief Receives voltage data from UART.
 * 
 * @param voltage_data Pointer to store voltage data
 * @return 0 on success, 1 on error
 */
uint8_t uart_receive_voltages(uart_voltage_data_t *voltage_data);

/**
 * @brief Receives temperature data from UART.
 * 
 * @param temp_data Pointer to store temperature data
 * @return 0 on success, 1 on error
 */
uint8_t uart_receive_temperatures(uart_temp_data_t *temp_data);

/**
 * @brief Receives current data from UART.
 * 
 * @param current_data Pointer to store current data
 * @return 0 on success, 1 on error
 */
uint8_t uart_receive_current(uart_current_data_t *current_data);

/**
 * @brief Calculates simple checksum for UART message.
 * 
 * @param data Pointer to data
 * @param length Data length
 * @return Calculated checksum
 */
uint8_t uart_calculate_checksum(const uint8_t *data, uint8_t length);

#endif /* UART_H */
