/*
 * bq_interface.c
 *
 *  Created on: Apr 30, 2023
 *      Author: sidsr
 */
#include "bq_interface.h"
#include "uart.h"
#include "gpio.h"
#include <CMR/uart.h>
#include <stm32f4xx_hal.h>
#include <stdbool.h>
#include <string.h> // memcpy


#define SINGLE_DEVICE true
#define MULTIPLE_DEVICE false

extern volatile int BMBTimeoutCount[BOARD_NUM-1];
extern volatile int BMBErrs[BOARD_NUM-1];


// Fill in data to this array
BMB_Data_t BMBData[BOARD_NUM-1];

// Balanced down to threshold after receiving bal command 
bool firstBalDone[BOARD_NUM-1][VSENSE_CHANNELS]; 

// CHANNEL_GPIO_TO_CELL_MAP[i][j] yields the corresponding cell number for 
// ith mux setting and the jth GPIO channel. We choose to zero index the cell nums
uint8_t CHANNEL_GPIO_TO_CELL_MAP[4][NUM_GPIO_CHANNELS]  = {{6, 3, 1, 255},
                                                            {255, 255, 0, 5},
                                                            {255, 4, 255, 255},
                                                            {8, 255, 2, 7}};

static void setBMBErr(uint8_t BMBIndex, BMB_UART_ERRORS err) {
	BMBErrs[BMBIndex] = err;
}

__STATIC_INLINE void DWT_Delay_ms(volatile uint32_t au32_milliseconds)
{
  uint32_t au32_initial_ticks = DWT->CYCCNT;
  uint32_t au32_ticks = (HAL_RCC_GetHCLKFreq() / 1000);
  au32_milliseconds *= au32_ticks;
  while ((DWT->CYCCNT - au32_initial_ticks) < au32_milliseconds);
}

//forward declarations
static inline bool sendUartStackWrite(uint16_t registerAddress, 
										uint8_t* data, 
										uint8_t dataLen);
void txToRxDelay(uint8_t delay);
void byteDelay(uint8_t delay);

/**
 * @return True if all UART commands succeeded, false otherwise
 */
bool turnOn() {
	// Turn On Ping
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

	return true;
}

/** Auto Address Function
 * This helper function will autoaddress a certain amount of BQ79616-Q1
 * chips. It runs the procedure exactly described in the datasheet and the TI
 * sample code.
 * @return True if all UART commands succeeded, false otherwise
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
		// HAL_Delay(10);
		DWT_Delay_ms(10);
	}

	// Broadcast write to enable autoaddressing
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

	// Set all the addresses of the boards in DIR0_ADDR
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

	// Set all devices as stack devices first
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
	// set 0x00 as base and num_board-1 as top
	// set_comm_ctrl.data[0] = 0x00;
	// res = uart_sendCommand(&set_comm_ctrl);
	// if (res != UART_SUCCESS) {
	// 	return false;
	// }
	res = uart_sendCommand(&set_comm_ctrl);
	if(res != UART_SUCCESS) {
		return false;
	}
	// HAL_Delay(10);
	DWT_Delay_ms(10);

	// Resync OTP registers with dummy reads
	otpSync.readWrite = STACK_READ;
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
	
	return true;

	/*
	COMMENTED OUT CODE THAT IS USED FOR SANITY CHECKING AUTOADDRESSING
	--- BEGIN ---
	uart_response_t response;
	uart_command_t readReg = {
		.readWrite = SINGLE_READ,
		.dataLen = 1,
		.deviceAddress = 0x00,
		.registerAddress = 0x306,
		.data = {0x00},
		.crc = {0x00, 0x00}
	};
	uart_sendCommand(&readReg);

	if(uart_receiveResponse(&response) == UART_FAILURE) {
		return false;
	}

	readReg.deviceAddress = 0x01;
	uart_sendCommand(&readReg);

	if(uart_receiveResponse(&response) == UART_FAILURE) {
		return false;
	}

	readReg.deviceAddress = 0x02;
	uart_sendCommand(&readReg);

	if(uart_receiveResponse(&response) == UART_FAILURE) {
		return false;
	}
	--- END ---
	*/
}

/** Enable ADC function
 * This helper function enables the main ADC on the bq79616 for starting
 * the main ADC. It runs the procedure exactly described in the datasheet
 * and the TI sample code.
 * @return True if all uart commands succeeded, false otherwise
 */
bool enableMainADC() {
	uint8_t dataToSend = 0x06;

	return sendUartStackWrite(ADC_CTRL1, &dataToSend, 1);
}

/**
 * Enable however many cells are in series in one segment
 * @return True if all UART commands succeeded, false otherwise
 */
bool enableNumCells() {
	uint8_t dataToSend = (VSENSE_CHANNELS - 0x06);

	return sendUartStackWrite(ACTIVE_CELL, &dataToSend, 1);
}

/**
 * Enable all GPIO registers and TSREF for thermistor biasing
 * @return True if all UART commands succeeded, false otherwise
 */
bool enableGPIOPins() {
	// enableTSref
	uint8_t dataToSend = 0x01;
	if (!sendUartStackWrite(CONTROL2, &dataToSend, 1))
			return false;

	// configures GPIO 5 and 6 as analog input
	dataToSend = 0x12;
	if (!sendUartStackWrite(GPIO_CONF3, &dataToSend, 1))
			return false;

	// configures GPIO 7 and 8 as analog input
	dataToSend = 0x12;
	if (!sendUartStackWrite(GPIO_CONF4, &dataToSend, 1))
			return false;

	//enable MUX outputs as low initially
	dataToSend = 0x2D;
	if (!sendUartStackWrite(GPIO_CONF2, &dataToSend, 1))
		return false;

	return true;
}

/* Enable command timeout so BQ sleeps turns off when car is off */
void enableTimeout() {
	uint8_t dataToSend = 0x0B;

	sendUartStackWrite(COMM_TIMEOUT_CONF, &dataToSend, 1);
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

	//Unsure about this value. I believe it should not matter much
	txToRxDelay(10);

	// HAL_Delay(100);
	DWT_Delay_ms(100);
	byteDelay(0x3F);
	// HAL_Delay(100);
	DWT_Delay_ms(100);

	// HAL_Delay(100);
	DWT_Delay_ms(100);
	cellBalancingSetup();
}

/* Calculates voltage following TI's code */
static uint16_t calculateVoltage(uint8_t msb, uint8_t lsb) {
	// Formula from TI's code
	// Bitwise OR high byte shifted by 8 and low byte, apply scaling factor

	return (uint16_t) (0.19073*((((uint16_t)msb << 8) | lsb)));
}

/**
 * Reads voltage data of all VSENSE channels into BMBData
 * @return 0 on success, ASIC number on failure
 */
uint8_t pollAllVoltageData() {
  uint8_t toReadLen = VSENSE_CHANNELS*2-1;
	uart_command_t read_voltage = {
		.readWrite = STACK_READ,
		.dataLen = 1,
		.deviceAddress = 0xFF, //not used!
		.registerAddress = TOP_CELL,
		.data = {toReadLen}, //reading high and low for cell 0-VSENSE_CHANNELS
		.crc = {0xFF, 0xFF}
	};

	uart_response_t response[BOARD_NUM-1];
	// Critical section used so UART RX is not preempted
	taskENTER_CRITICAL();
	uart_sendCommand(&read_voltage);
		//loop through each BMB and channel
		for(uint8_t i = BOARD_NUM-1; i >= 1; i--) {

		uint8_t status = uart_receiveResponse(&response[i-1], toReadLen);
			if(status != 0) {
			// setBMBErr(i-1, BMB_VOLTAGE_READ_ERROR);
			// BMBTimeoutCount[i-1]+=1;
			DWT_Delay_ms(10000);
			RXTurnOnInit();
			BMBInit();
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
			// if(i == 1 && (VSENSE_CHANNELS-j-1 == 12 || VSENSE_CHANNELS-j-1 == 13)) {
			// 	BMBData[i].cellVoltages[VSENSE_CHANNELS-j-1] = 3456;
			// } else if(i == 3 && (VSENSE_CHANNELS-j-1 == 13)) {
			// 	BMBData[i].cellVoltages[VSENSE_CHANNELS-j-1] = 3456;
			// } else if(i == 4 && (VSENSE_CHANNELS-j-1 == 12 || VSENSE_CHANNELS-j-1 == 13)) {
			// 	BMBData[i].cellVoltages[VSENSE_CHANNELS-j-1] = 3456;
			// } else if(i == 2 && (VSENSE_CHANNELS-j-1 == 12 || VSENSE_CHANNELS-j-1 == 13)) {
			// 	BMBData[i].cellVoltages[VSENSE_CHANNELS-j-1] = 3456;
			// }
			// if(i == 2 && (VSENSE_CHANNELS-j-1 == 12 || VSENSE_CHANNELS-j-1 == 13)) {
			// 					BMBData[i].cellVoltages[VSENSE_CHANNELS-j-1] = BMBData[i].cellVoltages[11];
			// 				}
			// if(i == 3 && (VSENSE_CHANNELS-j-1 == 12 || VSENSE_CHANNELS-j-1 == 13)) {
			// 	BMBData[i].cellVoltages[VSENSE_CHANNELS-j-1] = BMBData[i].cellVoltages[11];
			// }
		}
	}

	return 0;
}

/**
 * @return True on success, false otherwise
 */
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

	// set the Mux output
	if (!sendUartStackWrite(GPIO_CONF2, &data, 1)) {
		return false;
	}
    
	return true;
}

/**
 * For efficiency we choose to do as little computation as possible here and 
 * just compute voltage. To convert from voltage to temperature we would need
 * to do the Steinhart equation which is very expensive. However, since the
 * Steinhart is strictly decreasing we are able to simply probe the voltage
 * values for the hottest and coldest cells. The transfer function should be 
 * on PCAN to convert to temperature for easy viewing.
 */
static int16_t calculateTempVoltageReading(uint8_t msb, uint8_t lsb) {
	int16_t voltage_mv = (uint16_t)((0.15259) * (((int16_t) msb << 8) | lsb));
  return (voltage_mv);
}

/* Reads voltage data of all thermistor channels into BMBData */
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

	uart_response_t response[BOARD_NUM-1];

	for(uint8_t i = BOARD_NUM-1; i >= 1; i--) {
		if(uart_receiveResponse(&response[i-1], 7) != 0) {
				// Loop through each GPIO channel
			setBMBErr(i-1, BMB_TEMP_READ_ERROR);
			BMBTimeoutCount[i-1]+=1;
			taskEXIT_CRITICAL();
			return;
		}
	}
	taskEXIT_CRITICAL();

	for(uint8_t i = 0; i < BOARD_NUM-1; i++) {
		for(uint8_t k = 0; k < NUM_GPIO_CHANNELS; k++) {
			uint8_t cellNum = CHANNEL_GPIO_TO_CELL_MAP[channel][k];
			if (cellNum == 255)
					continue;

			uint8_t high_byte_data = response[i].data[2*k];
			uint8_t low_byte_data = response[i].data[2*k+1];
			int16_t cellTempVoltageReading = calculateTempVoltageReading(high_byte_data, low_byte_data);

			if (cellTempVoltageReading < 4990){
				BMBData[i].cellTemperaturesVoltageReading[cellNum] = cellTempVoltageReading;
			}
		}
	}
	return;
}

/**
 * Initializes cell balancing timers
 * @return True if all UART commands succeeded, false otherwise
 */
bool cellBalancingSetup() {
	// Set up cell balancing timers
	// Done in two sets because max register write is 8 :(

	uint8_t balanceData[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	if (!sendUartStackWrite(CB_CELL14_CTRL, balanceData, VSENSE_CHANNELS / 2)) {
		return false;
	}
	if (!sendUartStackWrite(CB_CELL7_CTRL, balanceData, VSENSE_CHANNELS / 2)) {
		return false;
	}

	// Set duty cycle to switch between even and odd cells
	uint8_t dataToSend = 0x00; // TODO: what is this value supposed to be?
	return sendUartStackWrite(BAL_CTRL1, &dataToSend, 1);
}

/**
 * @brief Checks the cell balancing status across all BMBs.
 *
 * Sends a UART read command to the BAL_STAT register to determine if 
 * any board is currently performing cell balancing (checking the CB_RUN bit).
 * This function enters a critical section to ensure UART RX integrity.
 *
 * @return int
 * 1 : Balancing is complete (all boards are idle).
 * 0 : Balancing is in progress (at least one board is active).
 * -1 : Error occurred (UART transmission or reception failure).
 */
int getBalDone() {

	uart_command_t getBalStatus = {
		.readWrite = STACK_READ,
		.dataLen = 1,
		.deviceAddress = 0xFF, //not used!
		.registerAddress = BAL_STAT,
		.data = {1}, 
		.crc = {0xFF, 0xFF}
	};

	uart_response_t response[BOARD_NUM-1] = {0};

	// Critical section used so UART RX is not preempted
	taskENTER_CRITICAL();

	cmr_uart_result_t res = uart_sendCommand(&getBalStatus);
	if (res != UART_SUCCESS)
		return -1; 

	// Loop through each BMB and channel
	for(uint8_t i = BOARD_NUM-1; i >= 1; i--) {
		uint8_t status = uart_receiveResponse(&response[i-1], 1);
		if (status == 1) {
			return -1; 
		} 
	}
	taskEXIT_CRITICAL();
	
	// Determines if we are done balancing
	bool doneBalancing = 1;
	for(uint8_t i = 0; i < BOARD_NUM-1; i++) {
		if ((response[i].data[0] & 8) == 8) //CB_RUN is 1 
			doneBalancing = 0;
	}
	
	return doneBalancing; 
}

/**
 * Performs cell balancing to a specific threshold
 * @param set True if balancing should be performed, false otherwise
 * @param thresh Voltage value to balance to
 */
void cellBalancing(bool set, uint16_t thresh) {
	if (thresh >= 4250 || thresh <= 2450) {
		thresh = 3700;
	}

	// board index by 0 but don't send to interface chip
	for(int i = 0; i < BOARD_NUM-1; i++) {
		// selections for cells--0x04 to balance for 5 minute intervals
		uint8_t top_len; 

		// Balance cells above 8 
		if (VSENSE_CHANNELS > 8) { 
			top_len = VSENSE_CHANNELS - 8; 
		} else {
			top_len = 0; 
		}

		// Decide which cells to balance before starting balancing. Inits to 10s (0x01)
		uint8_t cell_selects[] = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 
			0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01}; 

		// Disable selected cells below threshold 
		uint16_t cell_thresh = thresh; 
		for (int j = 0; j < VSENSE_CHANNELS; j ++) {
			// If we've already finished balancing once, threshold is mincell + 10 instead of mincell + 5
			if (firstBalDone[i][VSENSE_CHANNELS-1-j]) {
				cell_thresh += 5; 
			}
			// Check to see which ones we don't need to balance 
			if ((BMBData[i].cellVoltages[VSENSE_CHANNELS-1-j] < cell_thresh) || !set) {
				cell_selects[j] = 0x00;
			} else if (!firstBalDone[i][VSENSE_CHANNELS-1-j]){ //first time we've finished balancing this cell 
				firstBalDone[i][VSENSE_CHANNELS-1-j] = true; 
			}
		}

		// Balance top length
		uart_command_t balance_register = {
			.readWrite = SINGLE_WRITE,
			.dataLen = top_len,
			.deviceAddress = i+1,
			.registerAddress = TOP_CELL_CB_ADDR,
			.crc = {0x00, 0x00}
		};
		if (top_len > 0){
			memcpy(balance_register.data, cell_selects, top_len);
			uart_sendCommand(&balance_register);
		}

		// Balance bottom 
		uint8_t bottom_len = 8; 
		balance_register.registerAddress = CB_CELL8_CTRL; 

		if (VSENSE_CHANNELS < 8){
			bottom_len = VSENSE_CHANNELS; 
			balance_register.registerAddress = TOP_CELL_CB_ADDR; 
		}

		balance_register.dataLen = bottom_len; 
		balance_register.registerAddress = CB_CELL8_CTRL; 
		memcpy(balance_register.data, &(cell_selects[top_len]), bottom_len);
		uart_sendCommand(&balance_register); 
	}
		
	uint8_t toSend = 3;
	sendUartStackWrite(BAL_CTRL2, &toSend, 1);
}

void writeLED(bool set) {
	uint8_t enableLed = set ? 0 : 1;
	uint8_t dataToSend = (0x04 + enableLed);
	sendUartStackWrite(GPIO_CONF1, &dataToSend, 1);
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

	uint8_t dataToSend = delay;
	sendUartStackWrite(0x29, &dataToSend, 1);
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

	uint8_t dataToSend = 0b00111010;
	sendUartStackWrite(0x02, &dataToSend, 1);
}


/**
 * @brief Send a UART "stack write" command to the device.
 *
 * Constructs a STACK_WRITE uart_command_t with the provided register
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
static inline bool sendUartStackWrite(uint16_t registerAddress, 
										uint8_t* data, 
										uint8_t dataLen) {
	uart_command_t stackWriteCmd = {
		.readWrite = STACK_WRITE,
		.dataLen = dataLen,
		.deviceAddress = 0xFF, //not used!
		.registerAddress = registerAddress,
		.crc = {0x00, 0x00}
	};

  memcpy(stackWriteCmd.data, data, dataLen);

	cmr_uart_result_t res = uart_sendCommand(&stackWriteCmd);
  return (res == UART_SUCCESS);
}

