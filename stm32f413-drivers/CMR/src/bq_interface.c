#include <stdbool.h>
#include <string.h>
#include <stm32f4xx_hal.h>

#include <CMR/gpio.h>    
#include <CMR/uart.h>

#include "bq_interface.h"
#include "dwt.h"
#include "gpio.h"
#include "uart.h"

// only copy pasting similar functions for now: making notes about differences 
// notes start with ^^ so it is easy to find 


// ^^ lots of differences at the start 

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
		.readWrite = SINGLE_WRITE,
		.dataLen = 1,
		.deviceAddress = 0x00,
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
	DWT_Delay_ms(10);

	uart_command_t set_comm_ctrl = {
		.readWrite = SINGLE_WRITE,
		.dataLen = 1,
		.deviceAddress = 0x00,
		.registerAddress = COMM_CTRL,
		.data = {0x01},
		.crc = {0x00, 0x00}
	};


	res = uart_sendCommand(&set_comm_ctrl);
	if(res != UART_SUCCESS) {
		return false;
	}
	DWT_Delay_ms(10);

	// Resync OTP registers with dummy reads
	otpSync.readWrite = SINGLE_READ; // ^^ SINGLE_READ VS STACK_READ
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
	uart_command_t adcMsg = {
		.readWrite = BROADCAST_WRITE, // ^^ BROADCAST_WRITE VS STACK_WRITE
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
		.readWrite = BROADCAST_WRITE, // ^^ BROADCAST_WRITE VS STACK_WRITE
		.dataLen = 1,
		.deviceAddress = 0xFF, //not used!
		.registerAddress = ACTIVE_CELL,
		.data = {CELL_NUM-0x06}, // ^^ CELL_NUM-0x06 VS VSENSE_CHANNELS-0x06
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

    //enableTSref
    uint8_t dataToSend = 0x01;
    if (!sendUartStackWrite(CONTROL2, &dataToSend, 1))
        return false;

    // configures GPIO 5 and 6 as analog input
    dataToSend = 0x12;
    if (!sendUartStackWrite(GPIO_CONF3, &dataToSend, 1)) // ^^ GPIO_CONF3 VS GPIO_CONF1
        return false;

    // configures GPIO 7 and 8 as analog input
    dataToSend = 0x12;
    if (!sendUartStackWrite(GPIO_CONF4, &dataToSend, 1)) // ^^ GPIO_CONF4 VS Does not exist in LV BMS code
        // ^^ note the this doesn't exist in LV BMS code because it has less thermistors 
        return false;

    //enable MUX outputs as low initially
    dataToSend = 0x2D;
    if (!sendUartStackWrite(GPIO_CONF2, &dataToSend, 1))
      return false;

	return true;
}

// Enable command timeout so BQ sleeps turns off when car is off
void enableTimeout() {
	uart_command_t enable_timeout = {
			.readWrite = BROADCAST_WRITE, // ^^ BROADCAST_WRITE VS STACK_WRITE
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
	HAL_Delay(100);

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

For efficiency we choose to do as little computation as possible here and 
// just compute voltage. To convert from voltage to temperature we would need
// to do the Steinhart equation which is very expensive. However, since the
// Steinhart is strictly decreasing we are able to simply probe the voltage
// values for the hottest and coldest cells. The transfer function should be 
// on PCAN to convert to temperature for easy viewing
static int16_t calculateTempVoltageReading(uint8_t msb, uint8_t lsb) {
	int16_t voltage_mv = (uint16_t)((0.15259) * (((int16_t) msb << 8) | lsb));
    return (voltage_mv);
}

void cellBalancingSetup() {
	//set up cell balancing timers
	//done in two sets because max register write is 8 :(

	uart_command_t balance_register = {
		.readWrite = BROADCAST_WRITE, // ^^ BROADCAST_WRITE VS STACK_WRITE
		.dataLen = CELL_NUM/2, // ^^ CELL_NUM/2 VS VSENSE_CHANNELS/2
		.deviceAddress = 0xFF, //not used!
		.registerAddress = CB_CELL14_CTRL,
		.data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
		.crc = {0x00, 0x00}
	};
	uart_sendCommand(&balance_register);

	balance_register.registerAddress = CB_CELL7_CTRL;
	uart_sendCommand(&balance_register);

	//set duty cycle to switch between even and odd cells
	uart_command_t duty_cycle = {
		.readWrite = BROADCAST_WRITE, // ^^ BROADCAST_WRITE VS STACK_WRITE
		.dataLen = 1,
		.deviceAddress = 0xFF, //not used!
		.registerAddress = BAL_CTRL1,
		.data = {0x00}, //TODO what is this value supposed to be?
		.crc = {0x00, 0x00}
	};
	uart_sendCommand(&duty_cycle);
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
		.readWrite = BROADCAST_READ, // ^^ BROADCAST_READ VS STACK_READ
		.dataLen = 1,
		.deviceAddress = 0xFF, //not used!
		.registerAddress = BAL_STAT,
		.data = {1}, 
		.crc = {0xFF, 0xFF}
	};

	uart_response_t response[BOARD_NUM] = {0}; // ^^ BOARD_NUM-1 VS BOARD_NUM

	// Critical section used so UART RX is not preempted
	taskENTER_CRITICAL();

	cmr_uart_result_t res = uart_sendCommand(&getBalStatus);
	if (res != UART_SUCCESS)
		return -1; 

	//loop through each BMB and channel
	for(uint8_t i = BOARD_NUM; i >= 1; i--) { // ^^ BOARD_NUM-1 VS BOARD_NUM and i >= 1 vs i >= 0
		uint8_t status = uart_receiveResponse(&response[i-1], 1); 
		if (status == 1) {
			return -1; 
		} 
	}
	taskEXIT_CRITICAL();
	
	// determines if we are done balancing
	bool doneBalancing = 1;
	for(uint8_t i = 0; i < BOARD_NUM; i++) { // ^^ BOARD_NUM-1 VS BOARD_NUM
		if ((response[i].data[0] & 8) == 8) //CB_RUN is 1 
			doneBalancing = 0;
	}
	
	return doneBalancing; 

}

void cellBalancing(bool set, uint16_t thresh) {

	if (thresh >= 4250 || thresh <= 2450) {
		thresh = 3700;
	}

	// board index by 0 but don't send to interface chip
	for(int i = 0; i < BOARD_NUM-1; i++) { // ^^ BOARD_NUM VS BOARD_NUM-1
		// selections for cells--0x04 to balance for 5 minute intervals
		uint8_t top_len; 

		//balance cells above 8 
		if (VSENSE_CHANNELS > 8) { // ^^ CELL_NUM VS VSENSE_CHANNELS
			top_len = VSENSE_CHANNELS - 8; // ^^ CELL_NUM VS VSENSE_CHANNELS
		} else {
			top_len = 0; 
		}

		//decide which cells to balance before starting balancing. Inits to 10s 
		uint8_t cell_selects[] = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 
			0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01}; 

		// disable selected cells below threshold 

        // ^^ this for loop is written a little bit differently in the LV BMS code 
		uint16_t cell_thresh = thresh; 
		for (int j = 0; j < VSENSE_CHANNELS; j ++) {
			//if we've already finished balancing once, threshold is mincell + 10 instead of mincell + 5
			if (firstBalDone[i][VSENSE_CHANNELS-1-j]) {
				cell_thresh += 5; 
			}
			//check to see which ones we turn don't need to balance 
			if ((BMBData[i].cellVoltages[VSENSE_CHANNELS-1-j] < cell_thresh) || !set) {
				cell_selects[j] = 0x00;
			} else if (!firstBalDone[i][VSENSE_CHANNELS-1-j]){ //first time we've finished balancing this cell 
				firstBalDone[i][VSENSE_CHANNELS-1-j] = true; 
			}
		}

		//balance top length
		uart_command_t balance_register = {
			.readWrite = SINGLE_WRITE,
			.dataLen = top_len,
			.deviceAddress = i+1, // ^^ device address starts at 1 not 0
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

		if (VSENSE_CHANNELS < 8){ // ^^ CELL_NUM VS VSENSE_CHANNELS
			bottom_len = VSENSE_CHANNELS; // ^^ CELL_NUM VS VSENSE_CHANNELS
			balance_register.registerAddress = TOP_CELL_CB_ADDR; 
		}

		balance_register.dataLen = bottom_len; 
		balance_register.registerAddress = CB_CELL8_CTRL; // ^^ DNE in LV BMS code
		memcpy(balance_register.data, &(cell_selects[top_len]), bottom_len);
		uart_sendCommand(&balance_register); 
	}
		
	uint8_t toSend = 3;
	sendUartStackWrite(BAL_CTRL2, &toSend, 1); // ^^ sendUartStackWrite VS sendUartBroadcastWrite

}

void writeLED(bool set) {
	uint8_t enableLed = set ? 0 : 1;
    uint8_t data = 0x04 + enableLed;
    sendUartBroadcastWrite(GPIO_CONF3, &data, 1); // ^^ broadcast vs stack write and GPIO_CONF3 vs GPIO_CONF1
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

	uart_command_t byte_delay = {
			.readWrite = BROADCAST_WRITE, // ^^ BROADCAST_WRITE VS STACK_WRITE
			.dataLen = 1,
			.deviceAddress = 0xFF,
			.registerAddress = TX_HOLD_OFF, // ^^ 0x29 in hv bms code
			.data = {delay},
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
			.data = {delay},
			.crc = {0x00, 0x00}
	};
	uart_sendCommand(&tx_to_rx_delay);
}

void twoStop() {
    // ^^ this message not in LV BMS code because we use 1 stop bit instead of 2
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
			.readWrite = STACK_WRITE, // ^^ STACK_WRITE VS BROADCAST_WRITE
			.dataLen = 1,
			.deviceAddress = 0xFF,
			.registerAddress = 0x02, // DEV_CONF for lv bms
			.data = 0b00111010,
			.crc = {0x00, 0x00}
	};
	uart_sendCommand(&two_stop_stack);
}
// ^^ only in lv bms 
**
 * @brief Send a UART "broadcast write" command to the device.
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
        .dataLen = dataLen,
        .deviceAddress = 0xFF, //not used!
        .registerAddress = registerAddress,
        .crc = {0x00, 0x00}
    };

    memcpy(broadcastWriteCmd.data, data, dataLen);

    cmr_uart_result_t res = uart_sendCommand(&broadcastWriteCmd);
    return (res == UART_SUCCESS);
}

// ^^ only in hv bms
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
static inline bool sendUartStackWrite(  uint16_t registerAddress, 
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

void pollAllTemperatureData(int channel) {
    // ^^ very different, while still similar idt its worth copy pasting here 
}

uint8_t pollAllVoltageData() {
    // ^^ very different, while still similar idt its worth copy pasting here 
}

