/*
 * uart.h
 *
 *  Created on: Mar 29, 2023
 *      Author: sidsr
 */

#ifndef UART_H_
#define UART_H_

#include <CMR/uart.h>
#include <stdbool.h>

#define CRC_LEN 2
#define MAX_RESPONSE_LENGTH 128u

#define BQ_BAUD_RATE 1000000

#define UART_TIMEOUT 500


typedef enum {
	SINGLE_READ = 0x08,
	SINGLE_WRITE,
	STACK_READ,
	STACK_WRITE,
	BROADCAST_READ,
	BROADCAST_WRITE,
	BROADCAST_WRITE_REVERSE
} frame_t;

typedef struct {
	frame_t readWrite;
	uint8_t dataLen;
	uint8_t deviceAddress;
	uint16_t registerAddress;
	uint8_t data[MAX_RESPONSE_LENGTH];
	uint8_t crc[CRC_LEN];
} uart_command_t;

typedef struct {
	uint8_t len_bytes;
	uint8_t deviceAddress;
	uint16_t registerAddress;
	uint8_t data[MAX_RESPONSE_LENGTH];
	uint8_t crc[CRC_LEN];
} uart_response_t;

void uartInit(void);
cmr_uart_result_t uart_receiveResponse(uart_response_t *response, bool deviceResponse);
cmr_uart_result_t uart_sendCommand(const uart_command_t *command);
#endif /* UART_H_ */
