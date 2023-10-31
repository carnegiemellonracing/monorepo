/*
 * bq_interface.c
 *
 *  Created on: Apr 30, 2023
 *      Author: sidsr
 */
#include "bq_interface.h"
#include "state_task.h"
#include "uart.h"
#include "gpio.h"
#include <CMR/uart.h>
#include <stm32f4xx_hal.h>
#include <stdbool.h>

#define SINGLE_DEVICE true
#define MULTIPLE_DEVICE false

extern volatile int BMBTimeoutCount[BOARD_NUM];
extern volatile int BMBErrs[BOARD_NUM];

//Fill in data to this array
BMB_Data_t BMBData[BOARD_NUM];

static void setBMBErr(uint8_t BMBIndex, BMB_UART_ERRORS err) {
	BMBErrs[BMBIndex] = err;
}

void turnOn() {
	HAL_GPIO_WritePin(
		GPIOB, GPIO_PIN_13,
		GPIO_PIN_SET
	);
	//TickType_t xLastWakeTime = xTaskGetTickCount();
	HAL_Delay(1000);
	//vTaskDelayUntil(&xLastWakeTime, 2.5);
	HAL_GPIO_WritePin(
		GPIOB, GPIO_PIN_13,
		GPIO_PIN_RESET
	);
	HAL_Delay(3);
	HAL_GPIO_WritePin(
		GPIOB, GPIO_PIN_13,
		GPIO_PIN_SET
	);
	HAL_Delay(5);
	HAL_GPIO_WritePin(
			GPIOB, GPIO_PIN_13,
			GPIO_PIN_RESET
		);
		HAL_Delay(3);
		HAL_GPIO_WritePin(
				GPIOB, GPIO_PIN_13,
			GPIO_PIN_SET
		);
	//pinConfig.Alternate = GPIO_AF11_UART5;
	HAL_Delay(1000);
	uartInit();

	uart_command_t enableAutoaddress = {
			.readWrite = SINGLE_WRITE,
			.dataLen = 1,
			.deviceAddress = 0x00,
			.registerAddress = CONTROL1,
			.data = {0x20},
			.crc = {0x00, 0x00}
	};
	cmr_uart_result_t res = uart_sendCommand(&enableAutoaddress);
	if(res != UART_SUCCESS) {
		return false;
	}
}

/** Auto Address Function
 * This helper function will autoaddress a certain amount of BQ79616-Q1
 * chips. It runs the procedure exactly described in the datasheet and the TI
 * sample code.
 * @param num_boards Number of boards
 * @return True if all uart commands succeeded, false otherwise
 */
bool autoAddr() {
	if(BOARD_NUM > 64 || BOARD_NUM < 1) {
		return false;
	}

	uart_command_t otpSync = {
		.readWrite = STACK_WRITE,
		.dataLen = 1,
		.deviceAddress = 0xFF, //not used!!!
		.registerAddress = OTP_ECC_DATAIN1,
		.data = {0x00},
		.crc = {0x00, 0x00}
	};
	cmr_uart_result_t res;
	for(int i = 0; i < 8; i++) {
		otpSync.registerAddress = OTP_ECC_DATAIN1 + i;
		res = uart_sendCommand(&otpSync);
		if(res != UART_SUCCESS) {
			return false;
		}
	}

	//broadcast write to enable autoaddressing
	uart_command_t enableAutoaddress = {
			.readWrite = BROADCAST_WRITE,
			.dataLen = 1,
			.deviceAddress = 0xFF, //not used!!!
			.registerAddress = CONTROL1,
			.data = {0x01},
			.crc = {0x00, 0x00}
	};



	res = uart_sendCommand(&enableAutoaddress);
	if(res != UART_SUCCESS) {
		return false;
	}

	//set all the addresses of the boards in DIR0_ADDR
	uart_command_t set_addr = {
			.readWrite = BROADCAST_WRITE,
			.dataLen = 1,
			.deviceAddress = 0xFF, //not used!!!
			.registerAddress = DIR0_ADDR,
			.data = {0x00},
			.crc = {0x00, 0x00}
	};
	for(int i = 0; i < BOARD_NUM; i++) {
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
		.deviceAddress = 0xFF, //not used!!!
		.registerAddress = COMM_CTRL,
		.data = {0x02},
		.crc = {0x00, 0x00}
	};
	res = uart_sendCommand(&set_stack_devices);
	if(res != UART_SUCCESS) {
		return false;
	}

	uart_command_t set_comm_ctrl = {
		.readWrite = SINGLE_WRITE,
		.dataLen = 1,
		.deviceAddress = BOARD_NUM-1,
		.registerAddress = COMM_CTRL,
		.data = {0x03},
		.crc = {0x00, 0x00}
	};

	//set 0x00 as base and num_board-1 as top
	set_comm_ctrl.data[0] = 0x00;
	res = uart_sendCommand(&set_comm_ctrl);
	if(res != UART_SUCCESS) {
		return false;
	}
	set_comm_ctrl.deviceAddress = BOARD_NUM-1;
	set_comm_ctrl.data[0] = 0x03;
	res = uart_sendCommand(&set_comm_ctrl);
	if(res != UART_SUCCESS) {
		return false;
	}

	otpSync.readWrite = STACK_READ;
	otpSync.data[0] = 1;
	uart_response_t response;
	for(int i = 0; i < 8; i++) {
		otpSync.registerAddress = OTP_ECC_DATAIN1 + i;
		res = uart_sendCommand(&otpSync);
		if(res != UART_SUCCESS) {
			return false;
		}
	}
	uart_command_t readReg = {
		.readWrite = SINGLE_READ,
		.dataLen = 1,
		.deviceAddress = 0,
		.registerAddress = 0x2001,
		.data = {0x00},
		.crc = {0x00, 0x00}
	};
	res = uart_sendCommand(&readReg);
	if(uart_receiveResponse(&response, SINGLE_DEVICE) == UART_FAILURE) {
		return false;
	}
	return true;

}

/** Enable ADC function
 * This helper function enables the main ADC on the bq79616 for starting
 * the main ADC. It runs the procedure exactly described in the datasheet
 * and the TI sample code.
 * @return True if all uart commands succeeded, false otherwise
 */
bool enableMainADC() {
	uart_command_t adcMsg = {
		.readWrite = STACK_WRITE,
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
bool enableNumCells() {
	uart_command_t active_cell = {
		.readWrite = STACK_WRITE,
		.dataLen = 1,
		.deviceAddress = 0x00, //not used!
		.registerAddress = ACTIVE_CELL,
		.data = {VSENSE_CHANNELS-0x06},
		.crc = {0x00, 0x00}
	};
	cmr_uart_result_t res;
	res = uart_sendCommand(&active_cell);
	if(res != UART_SUCCESS) {
		return false;
	}
	return true;
}

bool enableGPIOPins() {
	uart_command_t enable_tsref = {
			.readWrite = STACK_WRITE,
			.dataLen = 1,
			.deviceAddress = 0x00, //not used!
			.registerAddress = CONTROL2,
			.data = {0x01}, //enable TSREF for NTC Thermistor Biasing
			.crc = {0x00, 0x00}
	};
	cmr_uart_result_t res = uart_sendCommand(&enable_tsref);
	if(res != UART_SUCCESS) {
		return false;
	}

	//enable GPIO inputs
	uart_command_t enable_gpio = {
			.readWrite = STACK_WRITE,
			.dataLen = 1,
			.deviceAddress = 0x00, //not used!
			.registerAddress = GPIO_CONF3,
			.data = {0x12},
			.crc = {0x00, 0x00}
	};
	res = uart_sendCommand(&enable_gpio);
	if(res != UART_SUCCESS) {
		return false;
	}
	enable_gpio.registerAddress = GPIO_CONF4;
	res = uart_sendCommand(&enable_gpio);
	if(res != UART_SUCCESS) {
		return false;
	}

	//enable MUX outputs as low initially
	enable_gpio.registerAddress = GPIO_CONF2;
	enable_gpio.data[0] = 0x2D;
	res = uart_sendCommand(&enable_gpio);
	if(res != UART_SUCCESS) {
		return false;
	}

	return true;

}

void BMBInit() {
	turnOn();
	autoAddr();
	enableMainADC();
	enableNumCells();
	enableGPIOPins();
	cellBalancingSetup();
}

static uint16_t calculateVoltage(uint8_t msb, uint8_t lsb) {
	//formula from TI's code
	//Bitwise OR high byte shifted by 8 and low byte, apply scaling factor
	//TODO explain this better
	return (uint16_t) ((190.73) * (((uint16_t)msb << 8) | lsb));
}

//return voltage data
void pollAllVoltageData() {
	uart_command_t read_voltage = {
		.readWrite = STACK_READ,
		.dataLen = 1,
		.deviceAddress = 0x00, //not used!
		.registerAddress = TOP_CELL,
		.data = {VSENSE_CHANNELS*2-1}, //reading high and low for cell 0-VSENSE_CHANNELS
		.crc = {0x00, 0x00}
	};
	cmr_uart_result_t res;
	res = uart_sendCommand(&read_voltage);
	//TODO add tx error handler

	//loop through boards and parse all response frames
	for(uint8_t i = 0; i < BOARD_NUM; i++) {
		uart_response_t response;
		if(uart_receiveResponse(&response, MULTIPLE_DEVICE) != UART_FAILURE) {
			if(response.len_bytes * 2 != VSENSE_CHANNELS) {
				setBMBErr(i, BMB_VOLTAGE_READ_ERROR);
				BMBTimeoutCount[i]+=1;
			}
			else {
				//loop through each channel
				for(uint8_t j = 0; j < VSENSE_CHANNELS; j++) {
					uint8_t high_byte_data = response.data[2*j];
					uint8_t low_byte_data = response.data[2*j+1];
					BMBData[i].cellVoltages[j] = calculateVoltage(high_byte_data, low_byte_data);
				}
			}
		}
		else {
			setBMBErr(i, BMB_VOLTAGE_READ_ERROR);
			BMBTimeoutCount[i]+=1;
		}
	}

}

static bool setMuxOutput(uint8_t channel) {
	uint8_t data;
	switch(channel) {
	case 0:
		data = 0x2D; //0 and 0
		break;
	case 1:
		data = 0x25; //0 and 1
		break;
	case 2:
		data = 0x2C; //1 and 0
		break;
	case 3:
		data = 0x24; //1 and 1
		break;
	default:
		return false;
	}
	uart_command_t enable_gpio = {
			.readWrite = STACK_WRITE,
			.dataLen = 1,
			.deviceAddress = 0x00, //not used!
			.registerAddress = GPIO_CONF3,
			.data = {data},
			.crc = {0x00, 0x00}
	};
	cmr_uart_result_t res = uart_sendCommand(&enable_gpio);
	if(res != UART_SUCCESS) {
		return false;
	}
	return true;
}

static int16_t calculateTemp(uint8_t msb, uint8_t lsb) {
	return (int16_t)((152.59) * (((int16_t)msb << 8) | lsb));
}

void pollAllTemperatureData() {
	uart_command_t read_therms = {
		.readWrite = STACK_READ,
		.dataLen = 1,
		.deviceAddress = 0x00, //not used!
		.registerAddress = GPIO5_HI,
		.data = {0x07},
		.crc = {0x00, 0x00}
	};
	cmr_uart_result_t res;
	res = uart_sendCommand(&read_therms);


	//loop through each muxed channel
	for(uint8_t j = 0; j < 4; j++) {
		setMuxOutput(j);
		for(uint8_t i = 0; i < BOARD_NUM; i++) {
			uart_response_t response;
			if(uart_receiveResponse(&response, MULTIPLE_DEVICE) != UART_FAILURE) {
				if(response.len_bytes * 2 != 8) {
					setBMBErr(i, BMB_TEMP_READ_ERROR);
					BMBTimeoutCount[i]+=1;
				}
				else {
					//loop through each GPIO channel
					for(uint8_t k = 0; k < NUM_GPIO_CHANNELS; j++) {
						uint8_t high_byte_data = response.data[2*k];
						uint8_t low_byte_data = response.data[2*k+1];

						//TODO: make sure this is matching the thermistor indices properly
						BMBData[i].cellTemperatures[(4*j)+k] = calculateTemp(high_byte_data, low_byte_data);
					}
				}
			}
			else {
				setBMBErr(i, BMB_TEMP_READ_ERROR);
				BMBTimeoutCount[i]+=1;
			}
		}
	}
}

void cellBalancingSetup() {
	//set up cell balancing timers
	//done in two sets because max register write is 8 :(

	uart_command_t balance_register = {
		.readWrite = STACK_WRITE,
		.dataLen = VSENSE_CHANNELS/2,
		.deviceAddress = 0x00, //not used!
		.registerAddress = CB_CELL14_CTRL,
		.data = {0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02},
		.crc = {0x00, 0x00}
	};
	cmr_uart_result_t res;
	res = uart_sendCommand(&balance_register);

	balance_register.registerAddress = CB_CELL7_CTRL;
	res = uart_sendCommand(&balance_register);

	//set duty cycle to switch between even and odd cells
	uart_command_t duty_cycle = {
		.readWrite = STACK_WRITE,
		.dataLen = 1,
		.deviceAddress = 0x00, //not used!
		.registerAddress = BAL_CTRL1,
		.data = {0x01}, //TODO what is this value supposed to be?
		.crc = {0x00, 0x00}
	};
	res = uart_sendCommand(&duty_cycle);

	//set UV stuff for stopping balancing
	uart_command_t UV = {
		.readWrite = STACK_WRITE,
		.dataLen = 1,
		.deviceAddress = 0x00, //not used!
		.registerAddress = VCB_DONE_THRESH,
		.data = {0x3F}, //TODO figure out correct low value
		.crc = {0x00, 0x00}
	};
	res = uart_sendCommand(&UV);

	UV.registerAddress = OVUV_CTRL;
	UV.data[0] = 0x05;

	res = uart_sendCommand(&UV);

}

void cellBalancing(bool set) {
	uart_command_t enable = {
		.readWrite = STACK_WRITE,
		.dataLen = 1,
		.deviceAddress = 0x00, //not used!
		.registerAddress = BAL_CTRL2,
		.data = {0x03},
		.crc = {0x00, 0x00}
	};
	if(!set) {
		enable.data[0] = 0x02;
	}
	cmr_uart_result_t res;
	res = uart_sendCommand(&enable);
}




