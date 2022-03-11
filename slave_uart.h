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

#define NUM_BMBS 12
typedef enum {
	TOP_SLAVE_BOARD = (NUM_BMBS - 1),
	BOT_SLAVE_BOARD = 0x00,
} slave_board_t;


//-----------------------------------------------------------------------------
// GLOBAL INTERFACE FUNCTION PROTOTYPES                                       |
//-----------------------------------------------------------------------------
cmr_uart_result_t slave_uart_autoAddress(void);
cmr_uart_result_t slave_uart_configureSampling(uint8_t boardNum);
cmr_uart_result_t slave_uart_configureChannels(void);
cmr_uart_result_t slave_uart_sampleAllChannels(uart_response_t response[NUM_BMBS]);
cmr_uart_result_t slave_uart_sampleDeviceChannels(uint8_t deviceAddress, uart_response_t *response);
cmr_uart_result_t slave_uart_broadcast_sampleAndStore();
cmr_uart_result_t slave_uart_configureGPIODirection(uint8_t DDRVector, uint8_t deviceAddress);
cmr_uart_result_t slave_uart_setGPIO(uint8_t data, uint8_t deviceAddress);
cmr_uart_result_t slave_uart_broadcast_setBMBTimeout(void);
cmr_uart_result_t slave_uart_sendBalanceCmd(uint16_t cells, uint8_t deviceAddress);
cmr_uart_result_t slave_uart_sendEnableTempMuxCmd(uint8_t enable);

#endif /* SLAVE_UART_H_ */
