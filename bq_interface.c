/*
 * bq_interface.c
 *
 *  Created on: Apr 30, 2023
 *      Author: sidsr
 */
#include "bq_interface.h"
#include "uart.h"
#include <CMR/uart.h>
#include <stdbool.h>


/** Auto Address Function
 * This helper function will autoaddress a certain amount of BQ79616-Q1
 * chips. It runs the procedure exactly described in the datasheet and the TI
 * sample code.
 * @param num_boards Number of boards
 * @return True if all uart commands succeeded, false otherwise
 */
bool autoAddr(uint8_t num_boards) {
	if(num_boards > 64 || num_boards < 1) {
		return false;
	}

	//broadcast write to enable autoaddressing
	uart_command_t enableAutoaddress = {
			.readWrite = BROADCAST_WRITE,
			.dataLen = 1,
			.deviceAddress = 0x00, //not used!!!
			.registerAddress = CONTROL1,
			.data = {0x01},
			.crc = {0x00, 0x00}
	};

	cmr_uart_result_t res;


	res = uart_sendCommand(&enableAutoaddress);
	if(res != UART_SUCCESS) {
		return false;
	}

	//set all the addresses of the boards in DIR0_ADDR
	uart_command_t set_addr = {
			.readWrite = BROADCAST_WRITE,
			.dataLen = 1,
			.deviceAddress = 0x00, //not used!!!
			.registerAddress = DIR0_ADDR,
			.data = {0x00},
			.crc = {0x00, 0x00}
	};
	for(int i = 0; i < num_boards; i++) {
		res = uart_sendCommand(&set_addr);
		if(res != UART_SUCCESS) {
			return false;
		}
		set_addr.data[0]++;
	}

	//Set all devices as stack devices first
	uart_command_t set_stack_devices = {
		.readWrite = BROADCAST_WRITE,
		.dataLen = 1,
		.deviceAddress = 0x00, //not used!!!
		.registerAddress = COMM_CTRL,
		.data = {0x02},
		.crc = {0x00, 0x00}
	};
	res = uart_sendCommand(&set_stack_devices);
	if(res != UART_SUCCESS) {
		return false;
	}

	uart_command_t set_comm_ctrl = {
		.readWrite = BROADCAST_WRITE,
		.dataLen = 1,
		.deviceAddress = 0x00,
		.registerAddress = COMM_CTRL,
		.data = {0x01},
		.crc = {0x00, 0x00}
	};

	//if one board, set as both base and top of stack
	if(num_boards == 1) {
		res = uart_sendCommand(&set_comm_ctrl);
		if(res != UART_SUCCESS) {
			return false;
		}
	}

	//otherwise, set 0x00 as base and num_board-1 as top
	else {
		set_comm_ctrl.data[0] = 0x00;
		res = uart_sendCommand(&set_comm_ctrl);
		if(res != UART_SUCCESS) {
			return false;
		}
		set_comm_ctrl.deviceAddress = num_boards-1;
		set_comm_ctrl.data[0] = 0x03;
		res = uart_sendCommand(&set_comm_ctrl);
		if(res != UART_SUCCESS) {
			return false;
		}
	}

}

//start the main ADC in continuous mode
bool enableMainADC() {
	uart_command_t adcMsg = {
		.readWrite = BROADCAST_WRITE,
		.dataLen = 1,
		.deviceAddress = 0x00, //not used!
		.registerAddress = ADC_CTRL1,
		.data = {0x06},
		.crc = {0x00, 0x00}
	};
	cmr_uart_result_t res;
	res = uart_sendCommand(&adcMsg);
	if(res != UART_SUCCESS) {
		return false;
	}
	return true;
}

//enable however many cells are in series in our segment
bool enableNumCells(uint8_t num_cells) {
	if(num_cells < 1 || num_cells > 16) {
		return false;
	}
	uart_command_t active_cell = {
		.readWrite = BROADCAST_WRITE,
		.dataLen = 1,
		.deviceAddress = 0x00, //not used!
		.registerAddress = ACTIVE_CELL,
		.data = {num_cells-0x06}, //there is a min of 6 cells
		.crc = {0x00, 0x00}
	};
	cmr_uart_result_t res;
	res = uart_sendCommand(&active_cell);
	if(res != UART_SUCCESS) {
		return false;
	}
	return true;
}



