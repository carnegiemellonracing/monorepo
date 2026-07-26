/*
 * bq_interface.c
 *
 *  Created on: June 4, 2026
 *      Author: anvitaa, ashleyyu
 */

#include <stdbool.h>
#include <string.h>
#include <stm32f4xx_hal.h>

#include <CMR/gpio.h>    
#include <CMR/uart.h>

#include "bq_interface.h"
#include "dwt.h"
#include "gpio.h"
#include "uart.h"

extern volatile int BMBTimeoutCount[BMS_NUM];
extern volatile int BMBErrs[BMS_NUM];

BMB_Data_t BMSData[BMS_NUM]; // Data stored in this array
bool firstBalDone[BMS_NUM][CELL_NUM]; // Track balance status

// CHANNEL_GPIO_TO_CELL_MAP on board-side

// Only relevant for BMSM
static void setBMBErr(uint8_t BMBIndex, BMB_UART_ERRORS err) {
	BMBErrs[BMBIndex] = err;
}

/**
 * @return True if all UART commands succeeded, false otherwise
 */
bool turnOn() {
	// Turn On Ping
	DWT_Delay_ms(100);
	HAL_GPIO_WritePin(
		GPIOB, GPIO_PIN_13,
		GPIO_PIN_SET
	);

	DWT_Delay_ms(100);
	HAL_GPIO_WritePin(
		GPIOB, GPIO_PIN_13,
		GPIO_PIN_RESET
	);

	DWT_Delay_ms(3);
	HAL_GPIO_WritePin(
		GPIOB, GPIO_PIN_13,
		GPIO_PIN_SET
	);

	DWT_Delay_ms(5);
	HAL_GPIO_WritePin(
		GPIOB, GPIO_PIN_13,
		GPIO_PIN_RESET
	);

	DWT_Delay_ms(3);
	HAL_GPIO_WritePin(
			GPIOB, GPIO_PIN_13,
		GPIO_PIN_SET
	);

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

	DWT_Delay_ms(1000);

	return true;
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
		.readWrite = BMS_WRITE,
		.dataLen = 1,
		.deviceAddress = 0x00, // not used
		.registerAddress = OTP_ECC_DATAIN1,
		.data = {0x00},
		.crc = {0x00, 0x00}
	};
	cmr_uart_result_t res;
  res = uart_sendCommand(&otpSync);
	for(int i = 0; i < 8; i++) {
		otpSync.registerAddress = OTP_ECC_DATAIN1 + i;
		res = uart_sendCommand(&otpSync);
		if(res != UART_SUCCESS) {
			return false;
		}
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
	DWT_Delay_ms(10);

	uint8_t comm_data = (HV_BMS) ? 0x03 : 0x01;
	uart_command_t set_comm_ctrl = {
		.readWrite = SINGLE_WRITE,
		.dataLen = 1,
		.deviceAddress = 0x00,
		.registerAddress = COMM_CTRL,
		.data = {comm_data},
		.crc = {0x00, 0x00}
	};

	res = uart_sendCommand(&set_comm_ctrl);
	if(res != UART_SUCCESS) {
		return false;
	}
	DWT_Delay_ms(10);

	// Resync OTP registers with dummy reads
	otpSync.readWrite = BMS_READ;
	otpSync.data[0] = 0;

	for(int i = 0; i < 8; i++) {
		otpSync.registerAddress = OTP_ECC_DATAIN1 + i;
		res = uart_sendCommand(&otpSync);
		if(res != UART_SUCCESS) {
			return false;
		}
		DWT_Delay_ms(10);
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
	uint8_t dataToSend = 0x06;
	return sendUartWrite(ADC_CTRL1, &dataToSend, 1);
}

// Enable however many cells are in series in one segment
bool enableNumCells() {
	uint8_t dataToSend = CELL_NUM - 0x06;
	return sendUartWrite(ACTIVE_CELL, &dataToSend, 1);
}

// Enable all GPIO registers and TSREF for thermistor biasing
bool enableGPIOPins() {
	//enableTSref
	uint8_t dataToSend = 0x01;
	if (!sendUartWrite(CONTROL2, &dataToSend, 1))
			return false;

	if (HV_BMS) {
    // configures GPIO 5 and 6 as analog input
    dataToSend = 0x12;
    if (!sendUartWrite(GPIO_CONF3, &dataToSend, 1))
        return false;

    // configures GPIO 7 and 8 as analog input
    dataToSend = 0x12;
    if (!sendUartWrite(GPIO_CONF4, &dataToSend, 1))
        return false;

    //enable MUX outputs as low initially
    dataToSend = 0x2D;
    if (!sendUartWrite(GPIO_CONF2, &dataToSend, 1))
		return false;
	}
	else if (LV_BMS) {
    // configures GPIO 5 and 6 as analog input
    dataToSend = 0x12;
    if (!sendUartWrite(GPIO_CONF1, &dataToSend, 1))
        return false;
				
    //enable MUX outputs as low initially
    dataToSend = 0x2D;
    if (!sendUartWrite(GPIO_CONF2, &dataToSend, 1))
		return false;
	}
}

// Enable command timeout so BQ sleeps turns off when car is off
void enableTimeout() {
	uint8_t dataToSend = 0x0B;
	sendUartWrite(COMM_TIMEOUT_CONF, &dataToSend, 1);
}

// Init function for all BMBs
void BMBInit() {
	turnOn();
	DWT_Delay_ms(1000);

  autoAddr();
	DWT_Delay_ms(100);

	enableNumCells();
	DWT_Delay_ms(100);

	enableGPIOPins();
	DWT_Delay_ms(100);

	enableMainADC();
	DWT_Delay_ms(100);

	enableTimeout();
	DWT_Delay_ms(100);

	txToRxDelay(10);
	DWT_Delay_ms(100);

	byteDelay(0x3F);
	DWT_Delay_ms(100);

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

	// Switches Mux
	bool res = sendUartWrite(GPIO_CONF2, &data, 1);
	return res;
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

/**
 * Initializes cell balancing timers
 * @return True if all UART commands succeeded, false otherwise
 */
void cellBalancingSetup() {
	// Set up cell balancing timers
	// Done in two sets because max register write is 8 :(

	uart_command_t balance_register = {
		.readWrite = BMS_WRITE, 
		.dataLen = CELL_NUM/2,
		.deviceAddress = 0xFF, //not used!
		.registerAddress = CB_CELL14_CTRL,
		.data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
		.crc = {0x00, 0x00}
	};
	uart_sendCommand(&balance_register);

	balance_register.registerAddress = CB_CELL7_CTRL;
	uart_sendCommand(&balance_register);

	uint8_t dataToSend = 0x00; // TODO: what is this value supposed to be?
	sendUartWrite(BAL_CTRL1, &dataToSend, 1);
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
		.readWrite = BMS_READ,
		.dataLen = 1,
		.deviceAddress = 0xFF, //not used!
		.registerAddress = BAL_STAT,
		.data = {1}, 
		.crc = {0xFF, 0xFF}
	};

	uart_response_t response[BMS_NUM] = {0}; 

	// Critical section used so UART RX is not preempted
	taskENTER_CRITICAL();

	cmr_uart_result_t res = uart_sendCommand(&getBalStatus);
	if (res != UART_SUCCESS)
		return -1; 

	//loop through each BMB and channel
	for(uint8_t i = BMS_NUM; i >= 1; i--) {
		uint8_t status = uart_receiveResponse(&response[i-1], 1); 
		if (status == 1) {
			return -1; 
		} 
	}
	taskEXIT_CRITICAL();
	
	// determines if we are done balancing
	bool doneBalancing = 1;
	for(uint8_t i = 0; i < BMB_NUM; i++) {
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

	for(int i = 0; i < BMB_NUM; i++) {
		// Selections for cells--0x04 to balance for 5 minute intervals
		uint8_t top_len; 

		// Balance cells above 8 
		if (CELL_NUM > 8) {
			top_len = CELL_NUM - 8;
		} else {
			top_len = 0; 
		}

		// Decide which cells to balance before starting balancing. Inits to 10s 
		uint8_t cell_selects[] = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 
			0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01}; 

		// Disable selected cells below threshold 
		uint16_t cell_thresh = thresh; 
		for (int j = 0; j < CELL_NUM; j ++) {
			//if we've already finished balancing once, threshold is mincell + 10 instead of mincell + 5
			if (firstBalDone[i][CELL_NUM-1-j]) {
				cell_thresh += 5; 
			}
			//check to see which ones we turn don't need to balance 
			if ((BMSData[i].cellVoltages[CELL_NUM-1-j] < cell_thresh) || !set) {
				cell_selects[j] = 0x00;
			} else if (!firstBalDone[i][CELL_NUM-1-j]){ //first time we've finished balancing this cell 
				firstBalDone[i][CELL_NUM-1-j] = true; 
			}
		}

		// Board index by 0 but don't send to interface chip on HV_BMS
		uint8_t addr = (HV_BMS) ? i+1 : i; 

		//balance top length
		uart_command_t balance_register = {
			.readWrite = SINGLE_WRITE,
			.dataLen = top_len,
			.deviceAddress = addr,
			.registerAddress = TOP_CELL_CB_ADDR,
			.crc = {0x00, 0x00}
		};
		if (top_len > 0){
			memcpy(balance_register.data, cell_selects, top_len);
			uart_sendCommand(&balance_register);
		}

		//balance bottom 
		uint8_t bottom_len = 8; 
		balance_register.registerAddress = CB_CELL8_CTRL; 

		if (CELL_NUM < 8){
			bottom_len = CELL_NUM;
			balance_register.registerAddress = TOP_CELL_CB_ADDR; 
		}

		balance_register.dataLen = bottom_len; 
		if (HV_BMS) {balance_register.registerAddress = CB_CELL8_CTRL;}
		memcpy(balance_register.data, &(cell_selects[top_len]), bottom_len);
		uart_sendCommand(&balance_register); 
	}
		
	uint8_t toSend = 3;
	sendUartWrite(BAL_CTRL2, &toSend, 1);
}

void writeLED(bool set) {
	uint8_t enableLed = set ? 0 : 1;
	uint8_t data = 0x04 + enableLed;
	uint16_t gpio = (HV_BMS) ? GPIO_CONF1 : GPIO_CONF3;
  sendUartWrite(gpio, &data, 1);
}

void disableTimeout() {
	uart_command_t disable_timeout = {
			.readWrite = SINGLE_WRITE,
			.dataLen = 1,
			.deviceAddress = 0x00,
			.registerAddress = 0x2005,
			.data = {0x00},
			.crc = {0x00, 0x00}
	};
	uart_sendCommand(&disable_timeout);
}

void byteDelay(uint8_t delay) {
	if (delay > 0x3F) return;

	uint16_t addr = (HV_BMS) ? 0x29 : TX_HOLD_OFF
	uint8_t dataToSend = delay;
	sendUartWrite(addr, &dataToSend, 1);
}

void txToRxDelay(uint8_t delay) {
	uart_command_t tx_to_rx_delay = {
			.readWrite = SINGLE_WRITE,
			.dataLen = 1,
			.deviceAddress = 0x00,
			.registerAddress = 0x2003,
			.data = {delay},
			.crc = {0x00, 0x00}
	};
	uart_sendCommand(&tx_to_rx_delay);
}

void twoStop() {
	if (HV_BMS) {
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
		sendUartWrite(0x02, &dataToSend, 1);
	}
	else if (LV_BMS) {
		uint8_t dataToSend = 0b00111010;
		sendUartWrite(DEV_CONF, &dataToSend, 1);
	}
}

/**
 * @brief Send a UART "write" command to the device.
 *
 * Constructs a STACK_WRITE or SINGLE_WRITE uart_command_t with the provided register
 * address and data payload, then transmits it over UART (dependent on board).
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
static inline bool sendUartWrite(uint16_t registerAddress, 
										uint8_t* data, 
										uint8_t dataLen) {
	uart_command_t writeCmd = {
        .readWrite = BMS_WRITE,
        .dataLen = dataLen,
        .deviceAddress = 0xFF, //not used!
        .registerAddress = registerAddress,
        .crc = {0x00, 0x00}
	};

  memcpy(writeCmd.data, data, dataLen);

	cmr_uart_result_t res = uart_sendCommand(&stackWriteCmd);
  return (res == UART_SUCCESS);
}

/**
 * Reads voltage data of all VSENSE channels into BMSData
 * @return 0 on success, ASIC number on failure
 */
uint8_t pollAllVoltageData() {
  uint8_t toReadLen = CELL_NUM*2-1;
	uart_command_t read_voltage = {
		.readWrite = BMS_READ,
		.dataLen = 1,
		.deviceAddress = 0xFF, //not used!
		.registerAddress = TOP_CELL,
		.data = {toReadLen}, //reading high and low for cell 0 to CELL_NUM
		.crc = {0xFF, 0xFF}
	};

	uart_response_t response[BMS_NUM];
	// Critical section used so UART RX is not preempted
	taskENTER_CRITICAL();
	uart_sendCommand(&read_voltage);
		// Loop through each BMB and channel
		for(uint8_t i = BMS_NUM; i >= 1; i--) {

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
		for(uint8_t i = 0; i < BMS_NUM; i++) {
			for(uint8_t j = 0; j < CELL_NUM; j++) {
			uint8_t high_byte_data = response[i].data[2*j];
			uint8_t low_byte_data = response[i].data[2*j+1];

			BMSData[i].cellVoltages[CELL_NUM-j-1] = CELL_NUM(high_byte_data, low_byte_data);
		}
	}

	return 0;
}

/* Reads voltage data of all thermistor channels into BMSData */
void pollAllTemperatureData(int channel) {
	uart_command_t read_therms = {
		.readWrite = BMS_READ,
		.dataLen = 1,
		.deviceAddress = 0xFF, //not used!
		.registerAddress = GPIO5_HI,
		.data = {0x07},
		.crc = {0xFF, 0xFF}
	};

	taskENTER_CRITICAL();
	uart_sendCommand(&read_therms);

	uart_response_t response[BMS_NUM];

	for(uint8_t i = BMS_NUM; i >= 1; i--) {
		if(uart_receiveResponse(&response[i-1], 7) != 0) {
			// Loop through each GPIO channel
			setBMBErr(i-1, BMB_TEMP_READ_ERROR);
			BMBTimeoutCount[i-1]+=1;
			taskEXIT_CRITICAL();
			return;
		}
	}
	taskEXIT_CRITICAL();

	for(uint8_t i = 0; i < BMS_NUM; i++) {
		for(uint8_t k = 0; k < NUM_GPIO_CHANNELS; k++) {
			uint8_t cellNum = CHANNEL_GPIO_TO_CELL_MAP[channel][k];
			if (cellNum == 255)
					continue;

			uint8_t high_byte_data = response[i].data[2*k];
			uint8_t low_byte_data = response[i].data[2*k+1];
			int16_t cellTempVoltageReading = calculateTempVoltageReading(high_byte_data, low_byte_data);

			if (cellTempVoltageReading < 4990){
				BMSData[i].cellTemperaturesVoltageReading[cellNum] = cellTempVoltageReading;
			}
		}
	}
	return;
}