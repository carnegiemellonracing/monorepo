/*
 * bq_interface.c
 *
 *  Created on: Apr 30, 2023
 *      Author: sidsr
 */
#include "bq_interface.h"
// #include "state_task.h"
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

// static void setBMBErr(uint8_t BMBIndex, BMB_UART_ERRORS err) {
// 	BMBErrs[BMBIndex] = err;
// }

__STATIC_INLINE void DWT_Delay_us(volatile uint32_t au32_microseconds)
{
  uint32_t au32_initial_ticks = DWT->CYCCNT;
  uint32_t au32_ticks = (HAL_RCC_GetHCLKFreq() / 1000000);
  au32_microseconds *= au32_ticks;
  while ((DWT->CYCCNT - au32_initial_ticks) < au32_microseconds-au32_ticks);
}

__STATIC_INLINE void DWT_Delay_ms(volatile uint32_t au32_milliseconds)
{
  uint32_t au32_initial_ticks = DWT->CYCCNT;
  uint32_t au32_ticks = (HAL_RCC_GetHCLKFreq() / 1000);
  au32_milliseconds *= au32_ticks;
  while ((DWT->CYCCNT - au32_initial_ticks) < au32_milliseconds);
}

void turnOn() {


	//Turn On Ping
	// HAL_Delay(100);
	DWT_Delay_ms(100);
	HAL_GPIO_WritePin(
		GPIOB, GPIO_PIN_13,
		GPIO_PIN_SET
	);
	// HAL_Delay(100);
	DWT_Delay_ms(100);
	HAL_GPIO_WritePin(
		GPIOB, GPIO_PIN_13,
		GPIO_PIN_RESET
	);
	// HAL_Delay(3);
	DWT_Delay_ms(3);
	HAL_GPIO_WritePin(
		GPIOB, GPIO_PIN_13,
		GPIO_PIN_SET
	);
	// HAL_Delay(5);
	DWT_Delay_ms(5);
	HAL_GPIO_WritePin(
		GPIOB, GPIO_PIN_13,
		GPIO_PIN_RESET
	);
	// HAL_Delay(3);
	DWT_Delay_ms(3);
	HAL_GPIO_WritePin(
			GPIOB, GPIO_PIN_13,
		GPIO_PIN_SET
	);

	// HAL_Delay(100);
	DWT_Delay_ms(100);
	uartInit();

	cmr_uart_result_t res;

	uart_command_t sendWake = {
			.readWrite = SINGLE_WRITE,
			.dataLen = 1,
			.deviceAddress = 0x00,
			.registerAddress = CONTROL1,
			.data = {0x20},
			.crc = {0x00, 0x00}
	};
	res = uart_sendCommand(&sendWake);
	if(res != UART_SUCCESS) {
		return false;
	}

	// HAL_Delay(1000);
	DWT_Delay_ms(1000);

	autoAddr();

	for (int i = BOARD_NUM - 1; i >= 0; i--) {
		uart_command_t hardReset = {
				.readWrite = SINGLE_WRITE,
				.dataLen = 1,
				.deviceAddress = i,
				.registerAddress = CONTROL2,
				.data = {0x02},
				.crc = {0x00, 0x00}
		};
		res = uart_sendCommand(&hardReset);
		if(res != UART_SUCCESS) {
			return false;
		}

		// HAL_Delay(200);
		DWT_Delay_ms(200);
	}

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

	// HAL_Delay(1000);
	DWT_Delay_ms(1000);


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
		.readWrite = BROADCAST_WRITE,
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
		// HAL_Delay(10);
		DWT_Delay_ms(10);
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
	// HAL_Delay(10);
	DWT_Delay_ms(10);

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
		// HAL_Delay(10);
		DWT_Delay_ms(10);
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
	// HAL_Delay(10);
	DWT_Delay_ms(10);

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
	// HAL_Delay(10);
	DWT_Delay_ms(10);

	// Resync OTP registers with dummy reads
	otpSync.readWrite = BROADCAST_READ;
	otpSync.data[0] = 0;

	for(int i = 0; i < 8; i++) {
		otpSync.registerAddress = OTP_ECC_DATAIN1 + i;
		res = uart_sendCommand(&otpSync);
		if(res != UART_SUCCESS) {
			return false;
		}
		// HAL_Delay(10);
		DWT_Delay_ms(10);
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
		.readWrite = BROADCAST_WRITE,
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
		.readWrite = BROADCAST_WRITE,
		.dataLen = 1,
		.deviceAddress = 0xFF, //not used!
		.registerAddress = ACTIVE_CELL,
		.data = {CELL_NUM-0x06},
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
			.readWrite = BROADCAST_WRITE,
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
			.readWrite = BROADCAST_WRITE,
			.dataLen = 1,
			.deviceAddress = 0xFF, //not used!
			.registerAddress = GPIO_CONF1,
			.data = {0x12},
			.crc = {0x00, 0x00}
	};
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
			.readWrite = BROADCAST_WRITE,
			.dataLen = 1,
			.deviceAddress = 0xFF, //not used!
			.registerAddress = COMM_TIMEOUT_CONF,
			.data = {0x0B},
			.crc = {0x00, 0x00}
	};
	uart_sendCommand(&enable_timeout);
}

// Init function for all BMBs
void BMBInit() {
	turnOn();
	// HAL_Delay(1000);
	DWT_Delay_ms(1000);
	autoAddr();
	// HAL_Delay(100);
	DWT_Delay_ms(100);
	enableNumCells();
	// HAL_Delay(100);
	DWT_Delay_ms(100);
	enableGPIOPins();
	// HAL_Delay(100);
	DWT_Delay_ms(100);
	enableMainADC();
	// HAL_Delay(100);
	DWT_Delay_ms(100);
	enableTimeout();
	//disableTimeout();
	// HAL_Delay(100);
	DWT_Delay_ms(100);

	//No idea lol
	txToRxDelay();
	// HAL_Delay(100);
	DWT_Delay_ms(100);
	byteDelay(0x3F);
	// HAL_Delay(100);
	DWT_Delay_ms(100);

	// HAL_Delay(100);
	DWT_Delay_ms(100);
	cellBalancingSetup();
}

static uint16_t calculateVoltage(uint8_t msb, uint8_t lsb) {
	//formula from TI's code
	//Bitwise OR high byte shifted by 8 and low byte, apply scaling factor

	return (uint16_t) (0.19073*((((uint16_t)msb << 8) | lsb)));
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
			.readWrite = BROADCAST_WRITE,
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

int16_t calculateTemp(uint8_t msb, uint8_t lsb) {
	int16_t voltage_mv = (uint16_t)((0.15259) * (((int16_t) msb << 8) | lsb));
//    uint32_t resistance_centiOhm = (47 * voltage_mv) / (5000 - voltage_mv); // Calculate the resistance of thermistor
//    uint32_t temp = (resistance_centiOhm*resistance_centiOhm) * 46 - (resistance_centiOhm) * 11303 + 905666;
    return (voltage_mv);
}




void cellBalancingSetup() {
	//set up cell balancing timers
	//done in two sets because max register write is 8 :(

	uart_command_t balance_register = {
		.readWrite = BROADCAST_WRITE,
		.dataLen = CELL_NUM/2,
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
		.readWrite = BROADCAST_WRITE,
		.dataLen = 1,
		.deviceAddress = 0xFF, //not used!
		.registerAddress = BAL_CTRL1,
		.data = {0x01}, //TODO what is this value supposed to be?
		.crc = {0x00, 0x00}
	};
	res = uart_sendCommand(&duty_cycle);

	//set UV stuff for stopping balancing to default at 4.2 volts
	uart_command_t UV = {
		.readWrite = BROADCAST_WRITE,
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
		.readWrite = BROADCAST_READ,
		.dataLen = 1,
		.deviceAddress = 0xFF, //not used!
		.registerAddress = CB_COMPLETE1,
		.data = {0x02},
		.crc = {0x00, 0x00}
	};
	bool shitter = false;
	uart_sendCommand(&getBalDone);
	uart_response_t response[BOARD_NUM-1];
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
		.readWrite = BROADCAST_WRITE,
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
			thresh = 0;
			for(int j = 0; j < CELL_NUM; j++) {
				if(BMBData[i].cellVoltages[j] > thresh) {
					thresh = BMBData[i].cellVoltages[j];
				}
			}
			thresh = thresh - 10;
			uart_command_t balance_register = {
				.readWrite = SINGLE_WRITE,
				.dataLen = CELL_NUM/2,
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

		//see bq datasheet in register VCB_DONE_THRESH, maps threshold in 25 mv increments
		//between 245 mV and 4000 mV
		uint8_t threshIndex = (uint8_t)((thresh - 2450)/25.0) + 1;

		//set UV stuff for stopping balancing based on parameter
		uart_command_t UV = {
			.readWrite = BROADCAST_WRITE,
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
			.readWrite = BROADCAST_WRITE,
			.dataLen = CELL_NUM/2,
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
			.readWrite = BROADCAST_WRITE,
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
			.readWrite = BROADCAST_WRITE,
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
			.readWrite = BROADCAST_WRITE,
			.dataLen = 1,
			.deviceAddress = 0xFF,
			.registerAddress = 0x02,
			.data = 0b00111010,
			.crc = {0x00, 0x00}
	};
	uart_sendCommand(&two_stop_stack);
}


