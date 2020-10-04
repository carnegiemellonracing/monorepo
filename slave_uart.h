/*
 * slave_uart.h
 * Header for protocol level delivery and receipt of UART messages
 * Originally Written by Homer Baker, Carnegie Mellon Racing
 *  Edited on: Jun 14, 2020
 * 
 */

#ifndef SLAVE_UART_H_
#define SLAVE_UART_H_

// Import library modules
// NONE

// Import custom modules
#include "uart.h"

//-----------------------------------------------------------------------------
// DEFINES                                                                    |
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// STATIC VARIABLE DEFINITIONS                                                |
//-----------------------------------------------------------------------------
typedef enum {
	TOP_SLAVE_BOARD = 0x0B,
	BOT_SLAVE_BOARD = 0x00,
} slave_board_t;

static const uint8_t NUM_BMS_SLAVE_BOARDS = TOP_SLAVE_BOARD - BOT_SLAVE_BOARD + 1;


//-----------------------------------------------------------------------------
// GLOBAL INTERFACE FUNCTION PROTOTYPES                                       |
//-----------------------------------------------------------------------------
uart_result_t slave_uart_autoAddress(void);
uart_result_t slave_uart_configureSampling(uint8_t boardNum);
uart_result_t slave_uart_configureChannels(void);
uart_result_t slave_uart_sampleAllChannels(uart_response_t response[NUM_BMS_SLAVE_BOARDS]);
uart_result_t slave_uart_sampleDeviceChannels(uint8_t deviceAddress, uart_response_t *response);
uart_result_t slave_uart_broadcast_sampleAndStore();
uart_result_t slave_uart_configureGPIODirection(uint8_t DDRVector, uint8_t deviceAddress);
uart_result_t slave_uart_setGPIO(uint8_t data, uint8_t deviceAddress);
uart_result_t slave_uart_broadcast_setBMBTimeout(void);
uart_result_t slave_uart_sendBalanceCmd(uint16_t cells, uint8_t deviceAddress);

#endif /* SLAVE_UART_H_ */
