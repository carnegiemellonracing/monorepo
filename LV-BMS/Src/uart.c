/**
 * @file uart.c
 * @brief Board-specific UART implementation for LV-BMS.
 *
 * @author Carnegie Mellon Racing
 */

#include "uart.h"
#include <string.h>
#include <stdint.h>

/** @brief UART interface for voltage and temperature monitoring. */
static cmr_uart_t uart_voltage_temp;

/** @brief UART message for communication. */
static cmr_uartMsg_t uart_msg;

/** @brief UART configuration. */
static const UART_InitTypeDef uart_config = {
    .BaudRate = 115200,
    .WordLength = UART_WORDLENGTH_8B,
    .StopBits = UART_STOPBITS_1,
    .Parity = UART_PARITY_NONE,
    .Mode = UART_MODE_TX_RX,
    .HwFlowCtl = UART_HWCONTROL_NONE,
    .OverSampling = UART_OVERSAMPLING_16
};

/**
 * @brief Initializes the UART interface.
 */
void uart_init(void) {
    // Initialize UART message
    cmr_uartMsgInit(&uart_msg);
    
    // Initialize UART interface (using USART1 for voltage/temperature monitoring)
    cmr_uart_polling_init(
        &uart_voltage_temp,
        USART1,
        &uart_config,
        GPIOA, GPIO_PIN_10,  // RX pin
        GPIOA, GPIO_PIN_9     // TX pin
    );
}

/**
 * @brief Requests voltage data from external sensor.
 * 
 * @return 0 on success, 1 on error
 */
uint8_t uart_request_voltages(void) {
    uart_message_t msg;
    msg.header.msg_type = UART_MSG_REQUEST_VOLTAGES;
    msg.header.length = 0;
    msg.header.checksum = uart_calculate_checksum((uint8_t*)&msg.header, sizeof(msg.header));
    
    cmr_uart_result_t result = cmr_uart_pollingTX(
        &uart_voltage_temp,
        (uint8_t*)&msg,
        sizeof(msg.header)
    );
    
    return (result == UART_SUCCESS) ? 0 : 1;
}

/**
 * @brief Requests temperature data from external sensor.
 * 
 * @return 0 on success, 1 on error
 */
uint8_t uart_request_temperatures(void) {
    uart_message_t msg;
    msg.header.msg_type = UART_MSG_REQUEST_TEMPS;
    msg.header.length = 0;
    msg.header.checksum = uart_calculate_checksum((uint8_t*)&msg.header, sizeof(msg.header));
    
    cmr_uart_result_t result = cmr_uart_pollingTX(
        &uart_voltage_temp,
        (uint8_t*)&msg,
        sizeof(msg.header)
    );
    
    return (result == UART_SUCCESS) ? 0 : 1;
}

/**
 * @brief Requests current data from external sensor.
 * 
 * @return 0 on success, 1 on error
 */
uint8_t uart_request_current(void) {
    uart_message_t msg;
    msg.header.msg_type = UART_MSG_REQUEST_CURRENT;
    msg.header.length = 0;
    msg.header.checksum = uart_calculate_checksum((uint8_t*)&msg.header, sizeof(msg.header));
    
    cmr_uart_result_t result = cmr_uart_pollingTX(
        &uart_voltage_temp,
        (uint8_t*)&msg,
        sizeof(msg.header)
    );
    
    return (result == UART_SUCCESS) ? 0 : 1;
}

/**
 * @brief Receives voltage data from UART.
 * 
 * @param voltage_data Pointer to store voltage data
 * @return 0 on success, 1 on error
 */
uint8_t uart_receive_voltages(uart_voltage_data_t *voltage_data) {
    uart_message_t msg;
    
    // Receive message header
    cmr_uart_result_t result = cmr_uart_pollingRX(
        &uart_voltage_temp,
        (uint8_t*)&msg.header,
        sizeof(msg.header)
    );
    
    if (result != UART_SUCCESS) {
        return 1;
    }
    
    // Verify message type
    if (msg.header.msg_type != UART_MSG_RESPONSE_VOLTAGES) {
        return 1;
    }
    
    // Verify checksum
    uint8_t calculated_checksum = uart_calculate_checksum((uint8_t*)&msg.header, sizeof(msg.header) - 1);
    if (calculated_checksum != msg.header.checksum) {
        return 1;
    }
    
    // Receive voltage data
    result = cmr_uart_pollingRX(
        &uart_voltage_temp,
        (uint8_t*)voltage_data,
        sizeof(uart_voltage_data_t)
    );
    
    return (result == UART_SUCCESS) ? 0 : 1;
}

/**
 * @brief Receives temperature data from UART.
 * 
 * @param temp_data Pointer to store temperature data
 * @return 0 on success, 1 on error
 */
uint8_t uart_receive_temperatures(uart_temp_data_t *temp_data) {
    uart_message_t msg;
    
    // Receive message header
    cmr_uart_result_t result = cmr_uart_pollingRX(
        &uart_voltage_temp,
        (uint8_t*)&msg.header,
        sizeof(msg.header)
    );
    
    if (result != UART_SUCCESS) {
        return 1;
    }
    
    // Verify message type
    if (msg.header.msg_type != UART_MSG_RESPONSE_TEMPS) {
        return 1;
    }
    
    // Verify checksum
    uint8_t calculated_checksum = uart_calculate_checksum((uint8_t*)&msg.header, sizeof(msg.header) - 1);
    if (calculated_checksum != msg.header.checksum) {
        return 1;
    }
    
    // Receive temperature data
    result = cmr_uart_pollingRX(
        &uart_voltage_temp,
        (uint8_t*)temp_data,
        sizeof(uart_temp_data_t)
    );
    
    return (result == UART_SUCCESS) ? 0 : 1;
}

/**
 * @brief Receives current data from UART.
 * 
 * @param current_data Pointer to store current data
 * @return 0 on success, 1 on error
 */
uint8_t uart_receive_current(uart_current_data_t *current_data) {
    uart_message_t msg;
    
    // Receive message header
    cmr_uart_result_t result = cmr_uart_pollingRX(
        &uart_voltage_temp,
        (uint8_t*)&msg.header,
        sizeof(msg.header)
    );
    
    if (result != UART_SUCCESS) {
        return 1;
    }
    
    // Verify message type
    if (msg.header.msg_type != UART_MSG_RESPONSE_CURRENT) {
        return 1;
    }
    
    // Verify checksum
    uint8_t calculated_checksum = uart_calculate_checksum((uint8_t*)&msg.header, sizeof(msg.header) - 1);
    if (calculated_checksum != msg.header.checksum) {
        return 1;
    }
    
    // Receive current data
    result = cmr_uart_pollingRX(
        &uart_voltage_temp,
        (uint8_t*)current_data,
        sizeof(uart_current_data_t)
    );
    
    return (result == UART_SUCCESS) ? 0 : 1;
}

/**
 * @brief Calculates simple checksum for UART message.
 * 
 * @param data Pointer to data
 * @param length Data length
 * @return Calculated checksum
 */
uint8_t uart_calculate_checksum(const uint8_t *data, uint8_t length) {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}
