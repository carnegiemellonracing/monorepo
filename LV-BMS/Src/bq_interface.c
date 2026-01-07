/*
 * bq_interface.c
 *
 *  Created on: Jan 5, 2026
 *      Author: Ayush Garg and Yi-An Liao
 */

#include <stdbool.h>
#include <string.h>
#include <stm32f4xx_hal.h>

#include <CMR/gpio.h>    // cmr_rccGPIOClockEnable()
#include <CMR/uart.h>

#include "bq_interface.h"
#include "dwt.h"
#include "gpio.h"
#include "uart.h"

extern volatile int BMBTimeoutCount[BOARD_NUM];
extern volatile int BMBErrs[BOARD_NUM];

//Fill in data to this array
BMB_Data_t BMBData[BOARD_NUM];

// static void setBMBErr(uint8_t BMBIndex, BMB_UART_ERRORS err) {
// 	BMBErrs[BMBIndex] = err;
// }

// CHANNEL_GPIO_TO_CELL_MAP[i][j] yields the corresponding cell number for 
// ith mux setting and the jth GPIO channel. We choose to zero index the cell nums
uint8_t CHANNEL_GPIO_TO_CELL_MAP [4][NUM_GPIO_CHANNELS]  = {{0, 4},
                                                            {1, 5},
                                                            {2, 6},
                                                            {3, 255}};

//Forward Decelerations
void txToRxDelay(uint8_t delay);
void byteDelay(uint8_t delay);
static bool sendUartBroadcastWrite(
    uint16_t registerAddress, uint8_t* data, uint8_t dataLen);

void turnOn() {

	//Turn On Ping
	DWT_Delay_ms(100);
    cmr_gpioWrite(RX_TURNON, 1);

	DWT_Delay_ms(100);
    cmr_gpioWrite(RX_TURNON, 0);

	DWT_Delay_ms(3);
    cmr_gpioWrite(RX_TURNON, 1);

	DWT_Delay_ms(5);
    cmr_gpioWrite(RX_TURNON,0);

	DWT_Delay_ms(3);
    cmr_gpioWrite(RX_TURNON, 1);

	DWT_Delay_ms(100);
	uartInit();

	cmr_uart_result_t res;

	for (int i = BOARD_NUM - 1; i >= 0; i--) {
		uart_command_t hardReset = {
				.readWrite = SINGLE_WRITE,
				.dataLen = 1,
				.deviceAddress = i,
				.registerAddress = CONTROL2,
				.data = {0x02},
				.crc = {0x00, 0x00}
		};

		DWT_Delay_ms(200);
	}

	uart_command_t sendShutdown = {
			.readWrite = BROADCAST_WRITE,
			// .readWrite = SINGLE_WRITE,
			.dataLen = 1,
			.deviceAddress = 0x00,
			.registerAddress = CONTROL1,
			.data = {0x40},
			.crc = {0x00, 0x00}
	};

	res = uart_sendCommand(&sendShutdown);

	DWT_Delay_ms(1000);

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
		// .readWrite = SINGLE_WRITE,
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

// Enable however many cells are in series in one segment
bool enableNumCells() {

	uart_command_t active_cell = {
		.readWrite = BROADCAST_WRITE,
		// .readWrite = SINGLE_WRITE,
		.dataLen = 1,
		.deviceAddress = 0x00, //not used!
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
    bool res;

    //Enables TS-Ref
    uint8_t data = 0x01;
    res = sendUartBroadcastWrite(CONTROL2, &data, 1);
    if (!res) return false;

    // Enables GPIO Inputs (ADC)
    data = 0x12;
    res = sendUartBroadcastWrite(GPIO_CONF1, &data, 1);
    if (!res) return false;

    // Enables Mux Starting as 0s
    data = 0x2D;
    res = sendUartBroadcastWrite(GPIO_CONF2, &data, 1);
    if (!res) return false;

	return true;
}

// Enable command timeout so BQ sleeps turns off when car is off
void enableTimeout() {
	uart_command_t enable_timeout = {
			.readWrite = BROADCAST_WRITE,
			// .readWrite = SINGLE_WRITE,
			.dataLen = 1,
			.deviceAddress = 0x00, //not used!
			.registerAddress = COMM_TIMEOUT_CONF,
			.data = {0x0B},
			.crc = {0x00, 0x00}
	};
	uart_sendCommand(&enable_timeout);
}

// Init function for all BMBs
void BMBInit() {
	turnOn();
	DWT_Delay_ms(1000);
	autoAddr();

    // //Set all devices as stack devices first
	// uart_command_t set_stack_devices = {
	// 	.readWrite = SINGLE_WRITE,
	// 	.dataLen = 1,
	// 	.deviceAddress = 0x00,
	// 	.registerAddress = COMM_CTRL,
	// 	.data = {0x01},
	// 	.crc = {0x00, 0x00}
	// };

    // uart_sendCommand(&set_stack_devices);

    // //Set all devices as stack devices first
	// uart_command_t set_stack_devices2 = {
	// 	.readWrite = SINGLE_WRITE,
	// 	.dataLen = 1,
	// 	.deviceAddress = 0x00,
	// 	.registerAddress = DEBUG_COMM_CTRL1,
	// 	.data = {0x0E},
	// 	.crc = {0x00, 0x00}
	// };

    // uart_sendCommand(&set_stack_devices2);

	enableNumCells();
	DWT_Delay_ms(100);

	enableGPIOPins();
	DWT_Delay_ms(100);

	enableMainADC();
	DWT_Delay_ms(100);

	enableTimeout();
	DWT_Delay_ms(100);

	// txToRxDelay(10);
	DWT_Delay_ms(100);

	byteDelay(0x3F);
	DWT_Delay_ms(100);

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

    // Switches Mux
    bool res = sendUartBroadcastWrite(GPIO_CONF2, &data, 1);
    return res;
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
		// .readWrite = SINGLE_WRITE,
		.dataLen = CELL_NUM/2,
		.deviceAddress = 0x00, //not used!
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
		// .readWrite = SINGLE_WRITE,
		.dataLen = 1,
		.deviceAddress = 0x00, //not used!
		.registerAddress = BAL_CTRL1,
		.data = {0x01}, //TODO what is this value supposed to be?
		.crc = {0x00, 0x00}
	};
	res = uart_sendCommand(&duty_cycle);

	//set UV stuff for stopping balancing to default at 4.2 volts
	uart_command_t UV = {
		.readWrite = BROADCAST_WRITE,
		// .readWrite = SINGLE_WRITE,
		.dataLen = 1,
		.deviceAddress = 0x00, //not used!
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
		// .readWrite = BROADCAST_READ,
		.readWrite = SINGLE_READ,
		.dataLen = 1,
		.deviceAddress = 0x00, //not used!
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
		// .readWrite = SINGLE_WRITE,
		.dataLen = 1,
		.deviceAddress = 0x00, //not used!
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
			// .readWrite = SINGLE_WRITE,
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
			// .readWrite = SINGLE_WRITE,
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
    uint8_t data = 0x04 + enableLed;
    sendUartBroadcastWrite(GPIO_CONF3, &data, 1);
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
			// .readWrite = SINGLE_WRITE,
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
	uart_command_t two_stop_stack = {
			.readWrite = BROADCAST_WRITE,
			// .readWrite = SINGLE_WRITE,
			.dataLen = 1,
			.deviceAddress = 0xFF,
			.registerAddress = DEV_CONF,
			.data = 0b00111010,
			.crc = {0x00, 0x00}
	};
	uart_sendCommand(&two_stop_stack);
}

/**
 * @brief Send a UART "stack write" command to the device.
 *
 * Constructs a BROADCAST_WRITE uart_command_t with the provided register
 * address and data payload, then transmits it over UART.
 *
 * @note I (Ayush Garg) added this function during 2025 to clean up code
 * that I write although there is much legacy code that does not utilize
 * this helper function
 *
 * @param registerAddress 16-bit register address to write to.
 * @param data            Pointer to the data buffer to be written.
 * @param dataLen         Number of bytes in the data buffer.
 *
 * @return true if the UART command was sent successfully (UART_SUCCESS),
 *         false otherwise.
 */
static bool sendUartBroadcastWrite(  uint16_t registerAddress, 
										    uint8_t* data, 
										    uint8_t dataLen) {
    uart_command_t broadcastWriteCmd = {
        .readWrite = BROADCAST_WRITE,
		// .readWrite = SINGLE_WRITE,
        .dataLen = dataLen,
        .deviceAddress = 0xFF, //not used!
        .registerAddress = registerAddress,
        .crc = {0x00, 0x00}
    };

    memcpy(broadcastWriteCmd.data, data, dataLen);

    cmr_uart_result_t res = uart_sendCommand(&broadcastWriteCmd);
    return (res == UART_SUCCESS);
}


// For efficiency we choose to do as little computation as possible here and 
// just compute voltage. To convert from voltage to temperature we would need
// to do the Steinhart equation which is very expensive. However, since the
// Steinhart is strictly decreasing we are able to simply probe the voltage
// values for the hottest and coldest cells. The transfer function should be 
// on PCAN to convert to temperature for easy viewing
static int16_t calculateTempVoltageReading(uint8_t msb, uint8_t lsb) {
	int16_t voltage_mv = (uint16_t)((0.15259) * (((int16_t) msb << 8) | lsb));
    return (voltage_mv);
}

void pollAllTemperatureData(int channel) {
    uint8_t bytesToRead = 2 * NUM_GPIO_CHANNELS;
	uart_command_t read_therms = {
		// .readWrite = BROADCAST_READ,
		.readWrite = SINGLE_READ,
		.dataLen = 1,
		.deviceAddress = 0x00, //not used!
		.registerAddress = GPIO1_HI,
		.data = {bytesToRead-1},
		.crc = {0xFF, 0xFF}
	};

	taskENTER_CRITICAL();
	uart_sendCommand(&read_therms);

	uart_response_t response;
    if(uart_receiveResponse(&response, bytesToRead-1) != UART_FAILURE) {
        //loop through each GPIO channel
        // setBMBErr(i-1, BMB_TEMP_READ_ERROR);
        // BMBTimeoutCount[i-1]+=1;
        return;
    }
	
	taskEXIT_CRITICAL();

	for(uint8_t i = 0; i < BOARD_NUM; i++) {
		for(uint8_t k = 0; k < NUM_GPIO_CHANNELS; k++) {
            uint8_t cellNum = CHANNEL_GPIO_TO_CELL_MAP[channel][k];
            if (cellNum == 255)
                continue;

			uint8_t high_byte_data = response.data[2*k];
			uint8_t low_byte_data = response.data[2*k+1];
            int16_t cellTempVoltageReading = calculateTempVoltageReading(high_byte_data, low_byte_data);
            
			BMBData[i].cellTemperaturesVoltageReading[cellNum] = cellTempVoltageReading;

		}
	}
	return;
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
		.data = {0x01},
		.crc = {0x00, 0x00}
	};
	res = uart_sendCommand(&set_stack_devices);
	if(res != UART_SUCCESS) {
		return false;
	}
	// HAL_Delay(10);
	DWT_Delay_ms(10);

	// uart_command_t set_comm_ctrl = {
	// 	.readWrite = SINGLE_WRITE,
	// 	.dataLen = 1,
	// 	.deviceAddress = BOARD_NUM-1,
	// 	.registerAddress = COMM_CTRL,
	// 	.data = {0x03},
	// 	.crc = {0x00, 0x00}
	// };

	//set 0x00 as base and num_board-1 as top
//	set_comm_ctrl.data[0] = 0x00;
//	res = uart_sendCommand(&set_comm_ctrl);
//	if(res != UART_SUCCESS) {
//		return false;
//	}
	// res = uart_sendCommand(&set_comm_ctrl);
	// if(res != UART_SUCCESS) {
	// 	return false;
	// }
	// // HAL_Delay(10);
	// DWT_Delay_ms(10);

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