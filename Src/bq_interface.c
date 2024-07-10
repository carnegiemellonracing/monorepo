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

	HAL_Delay(100);
	HAL_GPIO_WritePin(
		GPIOB, GPIO_PIN_13,
		GPIO_PIN_SET
	);
	//TickType_t xLastWakeTime = xTaskGetTickCount();
	HAL_Delay(100);
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
	HAL_Delay(100);
	uartInit();

	uart_command_t sendWake = {
			.readWrite = SINGLE_WRITE,
			.dataLen = 1,
			.deviceAddress = 0x00,
			.registerAddress = CONTROL1,
			.data = {0x20},
			.crc = {0x00, 0x00}
	};
	cmr_uart_result_t res = uart_sendCommand(&sendWake);
	if(res != UART_SUCCESS) {
		return false;
	}

	HAL_Delay(100);

	uart_command_t sendShutdown = {
			.readWrite = BROADCAST_WRITE,
			.dataLen = 1,
			.deviceAddress = 0xFF,
			.registerAddress = CONTROL1,
			.data = {0x40},
			.crc = {0x00, 0x00}
	};

	res = uart_sendCommand(&sendShutdown);
	if(res != UART_SUCCESS) {
		return false;
	}

	HAL_Delay(100);
//

	res = uart_sendCommand(&sendWake);
	if(res != UART_SUCCESS) {
		return false;
	}

	HAL_Delay(100);

	uart_command_t softReset = {
			.readWrite = BROADCAST_WRITE,
			.dataLen = 1,
			.deviceAddress = 0xFF,
			.registerAddress = CONTROL1,
			.data = {0x02},
			.crc = {0x00, 0x00}
	};

	res = uart_sendCommand(&softReset);
	if(res != UART_SUCCESS) {
		return false;
	}

	uart_command_t hardReset = {
			.readWrite = BROADCAST_WRITE,
			.dataLen = 1,
			.deviceAddress = 0xFF,
			.registerAddress = CONTROL2,
			.data = {0x02},
			.crc = {0x00, 0x00}
	};
//	res = uart_sendCommand(&hardReset);
//	if(res != UART_SUCCESS) {
//		return false;
//	}
}

/** Auto Address Function
 * This helper function will autoaddress a certain amount of BQ79616-Q1
 * chips. It runs the procedure exactly described in the datasheet and the TI
 * sample code.
 * @param num_boards Number of boards
 * @return True if all uart commands succeeded, false otherwise
 */
bool autoAddr() {
	// Sanity check number of boards
	if(BOARD_NUM > 64 || BOARD_NUM < 1) {
		return false;
	}

	// Dummy write to sync OTP addresses
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
		HAL_Delay(10);
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
	HAL_Delay(10);

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
		HAL_Delay(10);
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
	HAL_Delay(10);

	uart_command_t set_comm_ctrl = {
		.readWrite = SINGLE_WRITE,
		.dataLen = 1,
		.deviceAddress = BOARD_NUM-1,
		.registerAddress = COMM_CTRL,
		.data = {0x03},
		.crc = {0x00, 0x00}
	};

	//set 0x00 as base and num_board-1 as top
//	set_comm_ctrl.data[0] = 0x00;
//	res = uart_sendCommand(&set_comm_ctrl);
//	if(res != UART_SUCCESS) {
//		return false;
//	}
	res = uart_sendCommand(&set_comm_ctrl);
	if(res != UART_SUCCESS) {
		return false;
	}
	HAL_Delay(10);

	// Resync OTP registers with dummy reads
	otpSync.readWrite = STACK_READ;
	otpSync.data[0] = 0;

	for(int i = 0; i < 8; i++) {
		otpSync.registerAddress = OTP_ECC_DATAIN1 + i;
		res = uart_sendCommand(&otpSync);
		if(res != UART_SUCCESS) {
			return false;
		}
		HAL_Delay(10);
	}

	// COMMENTED OUT CODE THAT IS USED FOR SANITY CHECKING AUTOADDRESSING

//	uart_response_t response;
//	uart_command_t readReg = {
//		.readWrite = SINGLE_READ,
//		.dataLen = 1,
//		.deviceAddress = 0x00,
//		.registerAddress = 0x306,
//		.data = {0x00},
//		.crc = {0x00, 0x00}
//	};
//	uart_sendCommand(&readReg);
//
//	if(uart_receiveResponse(&response) == UART_FAILURE) {
//		return false;
//	}
//
//	readReg.deviceAddress = 0x01;
//	uart_sendCommand(&readReg);
//
//	if(uart_receiveResponse(&response) == UART_FAILURE) {
//		return false;
//	}
//
//	readReg.deviceAddress = 0x02;
//	uart_sendCommand(&readReg);
//
//	if(uart_receiveResponse(&response) == UART_FAILURE) {
//		return false;
//	}

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
		.deviceAddress = 0xFF, //not used!
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

// Enable however many cells are in series in one segment
bool enableNumCells() {
	uart_command_t active_cell = {
		.readWrite = STACK_WRITE,
		.dataLen = 1,
		.deviceAddress = 0xFF, //not used!
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

// Enable all GPIO registers and TSREF for thermistor biasing
bool enableGPIOPins() {
	uart_command_t enable_tsref = {
			.readWrite = STACK_WRITE,
			.dataLen = 1,
			.deviceAddress = 0xFF, //not used!
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
			.deviceAddress = 0xFF, //not used!
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

// Enable command timeout so BQ sleeps turns off when car is off
void enableTimeout() {
	uart_command_t enable_timeout = {
			.readWrite = STACK_WRITE,
			.dataLen = 1,
			.deviceAddress = 0xFF, //not used!
			.registerAddress = COMM_TIMEOUT_CONF,
			.data = {0x0A},
			.crc = {0x00, 0x00}
	};
	uart_sendCommand(&enable_timeout);
}

// Init function for all BMBs
void BMBInit() {
	turnOn();
	HAL_Delay(1000);
	autoAddr();
	HAL_Delay(100);
	enableNumCells();
	HAL_Delay(100);
	enableGPIOPins();
	HAL_Delay(100);
	enableMainADC();
	HAL_Delay(100);
	enableTimeout();
	//disableTimeout();
	HAL_Delay(100);

	//No idea lol
	txToRxDelay();
	HAL_Delay(100);
	byteDelay(0x3F);
	HAL_Delay(100);

	HAL_Delay(100);
	cellBalancingSetup();
}

static uint16_t calculateVoltage(uint8_t msb, uint8_t lsb) {
	//formula from TI's code
	//Bitwise OR high byte shifted by 8 and low byte, apply scaling factor

	return (uint16_t) (0.19073*((((uint16_t)msb << 8) | lsb)));
}

//return voltage data
uint8_t pollAllVoltageData() {
	uart_command_t read_voltage = {
			.readWrite = STACK_READ,
			.dataLen = 1,
			.deviceAddress = 0xFF, //not used!
			.registerAddress = TOP_CELL,
			.data = {VSENSE_CHANNELS*2-1}, //reading high and low for cell 0-VSENSE_CHANNELS
			.crc = {0xFF, 0xFF}
		};

		uart_response_t response[BOARD_NUM-1] = {0};

		//TODO add tx error handler

		int x= 0;

		// Critical section used so UART RX is not preempted
		taskENTER_CRITICAL();
		uart_sendCommand(&read_voltage);
		//loop through each BMB and channel
		for(uint8_t i = BOARD_NUM-1; i >= 1; i--) {

			uint8_t status = uart_receiveResponse(&response[i-1], 27);
			if(status != 0) {
				setBMBErr(i-1, BMB_VOLTAGE_READ_ERROR);
				BMBTimeoutCount[i-1]+=1;
				taskEXIT_CRITICAL();
				return i;
			}
		}
		taskEXIT_CRITICAL();

		// Handle writing data separately from receive so you don't miss a byte
		for(uint8_t i = 0; i < BOARD_NUM-1; i++) {
			for(uint8_t j = 0; j < VSENSE_CHANNELS; j++) {
				uint8_t high_byte_data = response[i].data[2*j];
				uint8_t low_byte_data = response[i].data[2*j+1];
				BMBData[i].cellVoltages[VSENSE_CHANNELS-j-1] = calculateVoltage(high_byte_data, low_byte_data);
			}
		}

		return 0;
}

bool setMuxOutput(uint8_t channel) {
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
			.deviceAddress = 0xFF, //not used!
			.registerAddress = GPIO_CONF2,
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
	int16_t voltage_mv = (uint16_t)((0.15259) * (((int16_t) msb << 8) | lsb));
//    uint32_t resistance_centiOhm = (47 * voltage_mv) / (5000 - voltage_mv); // Calculate the resistance of thermistor
//    uint32_t temp = (resistance_centiOhm*resistance_centiOhm) * 46 - (resistance_centiOhm) * 11303 + 905666;
    return (voltage_mv);
}

void pollAllTemperatureData(int channel) {
	uart_command_t read_therms = {
		.readWrite = STACK_READ,
		.dataLen = 1,
		.deviceAddress = 0xFF, //not used!
		.registerAddress = GPIO5_HI,
		.data = {0x07},
		.crc = {0xFF, 0xFF}
	};

	taskENTER_CRITICAL();
	uart_sendCommand(&read_therms);

	uart_response_t response[BOARD_NUM-1] = {0};



	for(uint8_t i = BOARD_NUM-1; i >= 1; i--) {
		if(uart_receiveResponse(&response[i-1], 7) != UART_FAILURE) {
				//loop through each GPIO channel
			setBMBErr(i-1, BMB_TEMP_READ_ERROR);
			BMBTimeoutCount[i-1]+=1;
			return;
		}
	}
	taskEXIT_CRITICAL();


	for(uint8_t i = 0; i < BOARD_NUM-1; i++) {
		for(uint8_t k = 0; k < NUM_GPIO_CHANNELS; k++) {
			uint8_t high_byte_data = response[i].data[2*k];
			uint8_t low_byte_data = response[i].data[2*k+1];
			size_t index = (4*channel) + k;
			//TODO: make sure this is matching the thermistor indices properly

			if (index == 3 || index == 10) {
				continue;
			}

			if (index > 10) {
				index--;
			}

			if (index > 3) {
				index--;
			}

			BMBData[i].cellTemperatures[index] = calculateTemp(high_byte_data, low_byte_data);
		}
	}
	return;
}

void cellBalancingSetup() {
	//set up cell balancing timers
	//done in two sets because max register write is 8 :(

	uart_command_t balance_register = {
		.readWrite = STACK_WRITE,
		.dataLen = VSENSE_CHANNELS/2,
		.deviceAddress = 0xFF, //not used!
		.registerAddress = CB_CELL14_CTRL,
		.data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
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
		.deviceAddress = 0xFF, //not used!
		.registerAddress = BAL_CTRL1,
		.data = {0x01}, //TODO what is this value supposed to be?
		.crc = {0x00, 0x00}
	};
	res = uart_sendCommand(&duty_cycle);

	//set UV stuff for stopping balancing to default at 4.2 volts
	uart_command_t UV = {
		.readWrite = STACK_WRITE,
		.dataLen = 1,
		.deviceAddress = 0xFF, //not used!
		.registerAddress = VCB_DONE_THRESH,
		.data = {0x3F}, //TODO figure out correct low value
		.crc = {0x00, 0x00}
	};
	//res = uart_sendCommand(&UV);

	UV.registerAddress = OVUV_CTRL;
	UV.data[0] = 0x05;

	//res = uart_sendCommand(&UV);

}

bool getBalDone() {
	uart_command_t getBalDone = {
		.readWrite = STACK_READ,
		.dataLen = 1,
		.deviceAddress = 0xFF, //not used!
		.registerAddress = CB_COMPLETE1,
		.data = {0x02},
		.crc = {0x00, 0x00}
	};
	bool shitter = false;
	uart_sendCommand(&getBalDone);
	uart_response_t response[BOARD_NUM-1] = {0};
	for(uint8_t i = BOARD_NUM-1; i >= 1; i--) {
		uint8_t status = uart_receiveResponse(&response[i-1], 2);
		if(status != 0) {
			shitter = false;
		}
		else if(i != 7 && (response[i-1].data[0] != 0 || response[i-1].data[1] != 0)) {
			shitter = true;
		}
	}
	return shitter;

}

void cellBalancing(bool set, uint16_t thresh) {
	cmr_uart_result_t res;

	uart_command_t enable = {
		.readWrite = STACK_WRITE,
		.dataLen = 1,
		.deviceAddress = 0xFF, //not used!
		.registerAddress = BAL_CTRL2,
		.data = {0x03},
		.crc = {0x00, 0x00}
	};


	if(set) {
		if(thresh >= 4250 || thresh <= 2450) {
			thresh = 3700;
		}
		//board index by 0 but don't send to interface chip
		for(int i = 0; i < BOARD_NUM-1; i++) {
//			thresh = 0;
//			for(int j = 0; j < VSENSE_CHANNELS; j++) {
//				if(BMBData[i].cellVoltages[j] > thresh) {
//					thresh = BMBData[i].cellVoltages[j];
//				}
//			}
//			thresh = thresh - 10;
			uart_command_t balance_register = {
				.readWrite = SINGLE_WRITE,
				.dataLen = VSENSE_CHANNELS/2,
				.deviceAddress = i+1,
				.registerAddress = CB_CELL14_CTRL,
				.data = {0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04},
				.crc = {0x00, 0x00}
			};
			for(int j = 13; j > 6; j--) {
				if(BMBData[i].cellVoltages[j] < thresh) {
					balance_register.data[13-j] = 0x00;
				}
			}
			res = uart_sendCommand(&balance_register);
			balance_register.registerAddress = CB_CELL7_CTRL;
			for(int j = 0; j < 7; j++) {
				balance_register.data[j] = 0x04;
			}
			for(int j = 6; j >=0 ; j--) {
				if(BMBData[i].cellVoltages[j] < thresh) {
					balance_register.data[6-j] = 0x00;
				}
			}
			res = uart_sendCommand(&balance_register);
		}
		//set duty cycle to switch between even and odd cells
//		uart_command_t duty_cycle = {
//			.readWrite = STACK_WRITE,
//			.dataLen = 1,
//			.deviceAddress = 0xFF, //not used!
//			.registerAddress = BAL_CTRL1,
//			.data = {0x01}, //TODO what is this value supposed to be?
//			.crc = {0x00, 0x00}
//		};
//		res = uart_sendCommand(&duty_cycle);

		//see bq datasheet in register VCB_DONE_THRESH, maps threshold in 25 mv increments
		//between 245 mV and 4000 mV
		uint8_t threshIndex = (uint8_t)((thresh - 2450)/25.0) + 1;

		//set UV stuff for stopping balancing based on parameter
		uart_command_t UV = {
			.readWrite = STACK_WRITE,
			.dataLen = 1,
			.deviceAddress = 0xFF, //not used!
			.registerAddress = VCB_DONE_THRESH,
			.data = {0x1},
			.crc = {0x00, 0x00}
		};
		enable.data[0] = 0x03;
//		res = uart_sendCommand(&UV);
//		TickType_t lastTime = xTaskGetTickCount();
//		vTaskDelayUntil(&lastTime, 1);

		UV.registerAddress = OVUV_CTRL;
		UV.data[0] = 0x05;
//		res = uart_sendCommand(&UV);
//		lastTime = xTaskGetTickCount();
//		vTaskDelayUntil(&lastTime, 1);

	}
	else {
		uart_command_t balance_register = {
			.readWrite = STACK_WRITE,
			.dataLen = VSENSE_CHANNELS/2,
			.deviceAddress = 0xFF, //not used!
			.registerAddress = CB_CELL14_CTRL,
			.data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
			.crc = {0x00, 0x00}
		};
		cmr_uart_result_t res;
		res = uart_sendCommand(&balance_register);

		balance_register.registerAddress = CB_CELL7_CTRL;
		res = uart_sendCommand(&balance_register);
	}
	res = uart_sendCommand(&enable);
}

void writeLED(bool set) {
	uint8_t enableLed = set ? 0 : 1;
	uart_command_t write_led = {
			.readWrite = STACK_WRITE,
			.dataLen = 1,
			.deviceAddress = 0xFF, //not used!
			.registerAddress = GPIO_CONF1,
			.data = {0x04 + enableLed},
			.crc = {0x00, 0x00}
	};
	uart_sendCommand(&write_led);
}

void disableTimeout() {
	uart_command_t disable_timeout = {
			.readWrite = SINGLE_WRITE,
			.dataLen = 1,
			.deviceAddress = 0x00,
			.registerAddress = 0x2005,
			.data = 0x00,
			.crc = {0x00, 0x00}
	};
	uart_sendCommand(&disable_timeout);
}

void byteDelay(uint8_t delay) {
	if (delay > 0x3F) return;

	uart_command_t byte_delay = {
			.readWrite = STACK_WRITE,
			.dataLen = 1,
			.deviceAddress = 0xFF,
			.registerAddress = 0x29,
			.data = delay,
			.crc = {0x00, 0x00}
	};
	uart_sendCommand(&byte_delay);
}

void txToRxDelay(uint8_t delay) {
	uart_command_t tx_to_rx_delay = {
			.readWrite = SINGLE_WRITE,
			.dataLen = 1,
			.deviceAddress = 0x00,
			.registerAddress = 0x2003,
			.data = delay,
			.crc = {0x00, 0x00}
	};
	uart_sendCommand(&tx_to_rx_delay);
}

void twoStop() {
	uart_command_t two_stop_single = {
			.readWrite = SINGLE_WRITE,
			.dataLen = 1,
			.deviceAddress = 0x00,
			.registerAddress = 0x2001,
			.data = 0b00111000,
			.crc = {0x00, 0x00}
	};
	uart_sendCommand(&two_stop_single);

	uart_command_t two_stop_stack = {
			.readWrite = STACK_WRITE,
			.dataLen = 1,
			.deviceAddress = 0xFF,
			.registerAddress = 0x02,
			.data = 0b00111010,
			.crc = {0x00, 0x00}
	};
	uart_sendCommand(&two_stop_stack);
}



