/*
 * uart.h
 *
 *  Created on: Mar 29, 2023
 *      Author: sidsr
 */

#ifndef UART_H_
#define UART_H_

#define CRC_FRAME 2
#define MAX_RESPONSE_LENGTH 128u
#define STACK_RESPONSE true
#define DEVICE_RESPONSE false

#define BQ_BAUD_RATE 1000000

typedef enum {

}reg_t;

typedef enum {

}frame_t;

typedef struct {
	frame_t readWrite;
	uint8_t dataLen;
	uint8_t deviceAddress;
	reg_t registerAddress;
	uint8_t data[MAX_RESPONSE_LENGTH];
	uint8_t crc[CRC_LEN];
} uart_command_t;

typedef struct {
	uint8_t len_bytes;
	uint8_t deviceAddress;
	reg_t registerAddress;
	uint8_t data[MAX_RESPONSE_LENGTH];
	uint8_t crc[CRC_LEN];
} uart_response_t;

void uartInit(void);

#endif /* UART_H_ */
