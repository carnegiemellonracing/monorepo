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
BMB_Data_t BMBData;

uint16_t getVoltageData_mV(uint8_t cell_idx){
	BMBData.cellVoltages_mV[cell_idx];
}

uint16_t getTempData_centi_C(uint8_t cell_idx){
	BMBData.cellTemps_Centi_C[cell_idx];
}

// static void setBMBErr(uint8_t BMBIndex, BMB_UART_ERRORS err) {
// 	BMBErrs[BMBIndex] = err;
// }

// CHANNEL_GPIO_TO_CELL_MAP[i][j] yields the corresponding cell number for 
// ith mux setting and the jth GPIO channel. We choose to zero index the cell nums
uint8_t CHANNEL_GPIO_TO_CELL_MAP [4][NUM_GPIO_CHANNELS]  = {{0, 4},
                                                            {1, 5},
                                                            {2, 6},
                                                            {3, 255}};
																														

#define THERM_MV_TO_TEMP_NUM_ITEMS 101

/**
 * @brief Look up table thermistor voltage to to temp (C) (indexed by temp)
 * @todo Add link for this
 * 
 * goes from temp 0 to 100 C 
 */
static uint16_t thermMV_to_tempC[THERM_MV_TO_TEMP_NUM_ITEMS] = {
    27280, 26140, 25050, 24010, 23020, 22070, 21170, 20310, 19490, 18710,
    17960, 17250, 16570, 15910, 15290, 14700, 14130, 13590, 13070, 12570,
    12090, 11640, 11200, 10780, 10380, 10000, 9633, 9282, 8945, 8622,
    8312, 8015, 7730, 7456, 7194, 6942, 6700, 6468, 6245, 6031,
    5826, 5628, 5438, 5255, 5080, 4911, 4749, 4592, 4442, 4297,
    4158, 4024, 3895, 3771, 3651, 3536, 3425, 3318, 3215, 3115,
    3019, 2927, 2837, 2751, 2668, 2588, 2511, 2436, 2364, 2295,
    2227, 2163, 2100, 2039, 1981, 1924, 1869, 1817, 1765, 1716,
    1668, 1622, 1577, 1534, 1492, 1451, 1412, 1374, 1337, 1302,
    1267, 1234, 1201, 1170, 1139, 1110, 1081, 1053, 1027, 1001,
    975
}; 


//Forward Decelerations
void txToRxDelay(uint8_t delay);
void byteDelay(uint8_t delay);
static bool sendUartBroadcastWrite(
uint16_t registerAddress, uint8_t* data, uint8_t dataLen);
static uint16_t thermVoltage_to_temp_Centi_Deg(uint8_t msb, uint8_t lsb);

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

	DWT_Delay_ms(1000);
	autoAddr();

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
		res = uart_sendCommand(&hardReset);
		if(res != UART_SUCCESS) {
			return;
		}

		DWT_Delay_ms(200);
	}

    //Enables Send Shutdown
    uint8_t data = 0x40;
    res = sendUartBroadcastWrite(CONTROL1, &data, 1);
    if (!res) return;
	DWT_Delay_ms(1000);
}


/** Auto Address Function
 * This helper function will autoaddress a certain amount of BQ79616-Q1
 * chips. It runs the procedure exactly described in the datasheet and the TI
 * sample code.
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
	otpSync.readWrite = SINGLE_READ;
	otpSync.data[0] = 0;

	for(int i = 0; i < 8; i++) {
		otpSync.registerAddress = OTP_ECC_DATAIN1 + i;
		res = uart_sendCommand(&otpSync);
		if(res != UART_SUCCESS) {
			return false;
		}
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

bool cellBalancingSetup() {
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
		.data = {0x00}, //TODO what is this value supposed to be?
		.crc = {0x00, 0x00}
	};
	res = uart_sendCommand(&duty_cycle);


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
		.readWrite = BROADCAST_READ,
		.dataLen = 1,
		.deviceAddress = 0xFF, //not used!
		.registerAddress = BAL_STAT,
		.data = {1}, 
		.crc = {0xFF, 0xFF}
	};

	uart_response_t response[BOARD_NUM] = {0};

	// Critical section used so UART RX is not preempted
	taskENTER_CRITICAL();

	cmr_uart_result_t res = uart_sendCommand(&getBalStatus);
	if (res != UART_SUCCESS)
		return -1; 

	//loop through each BMB and channel
	for(uint8_t i = BOARD_NUM; i >= 1; i--) {
		uint8_t status = uart_receiveResponse(&response[i-1], 1); 
		if (status == 1) {
			return -1; 
		} 
	}
	taskEXIT_CRITICAL();
	
	// determines if we are done balancing
	bool doneBalancing = 1;
	for(uint8_t i = 0; i < BOARD_NUM; i++) {
		if ((response[i].data[0] & 8) == 8) //CB_RUN is 1 
			doneBalancing = 0;
	}
	
	return doneBalancing; 

}
void cellBalancing(bool set, uint16_t thresh) {
	cmr_uart_result_t res;

	if (thresh >= 4250 || thresh <= 2450) {
		thresh = 3700;
	}

	// board index by 0 but don't send to interface chip
	for(int i = 0; i < BOARD_NUM; i++) {
		// selections for cells--0x04 to balance for 10s minute intervals
		uint8_t top_len; 

		//balance cells above 8 
		if (CELL_NUM > 8) { 
			top_len = CELL_NUM - 8; 
		} else {
			top_len = 0; 
		}

		//decide which cells to balance before starting balancing. Inits to 10s 
		uint8_t cell_selects[] = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 
			0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01}; 

		// disable selected cells below threshold 
		for (int j = 0; j < CELL_NUM; j ++) {
			if ((BMBData.cellVoltages_mV[CELL_NUM-1-j] < thresh) || !set) {
				cell_selects[j] = 0x00;
			} 
		}

		//balance top length
		uart_command_t balance_register = {
			.readWrite = SINGLE_WRITE,
			.dataLen = top_len,
			.deviceAddress = i,
			.registerAddress = TOP_CELL_CB_ADDR,
			.crc = {0x00, 0x00}
		};

		if(top_len > 0){
			memcpy(balance_register.data, cell_selects, top_len);
			res = uart_sendCommand(&balance_register);
		}
		
		//balance bottom 8 
		uint8_t bottom_len = 8; 
		balance_register.registerAddress = CB_CELL8_CTRL; 

		if (CELL_NUM < 8){
			bottom_len = CELL_NUM; 
			balance_register.registerAddress = TOP_CELL_CB_ADDR; 
		}

		balance_register.dataLen = bottom_len; 
		memcpy(balance_register.data, &(cell_selects[top_len]), bottom_len); 
		res = uart_sendCommand(&balance_register); 
	}
		
	uint8_t toSend = 3;
	sendUartBroadcastWrite(BAL_CTRL2, &toSend, 1);

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
			.dataLen = 1,
			.deviceAddress = 0xFF,
			.registerAddress = TX_HOLD_OFF,
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
        .dataLen = dataLen,
        .deviceAddress = 0xFF, //not used!
        .registerAddress = registerAddress,
        .crc = {0x00, 0x00}
    };

    memcpy(broadcastWriteCmd.data, data, dataLen);

    cmr_uart_result_t res = uart_sendCommand(&broadcastWriteCmd);
    return (res == UART_SUCCESS);
}

void pollAllTemperatureData(int channel) {
    uint8_t bytesToRead = 2 * NUM_GPIO_CHANNELS;
	uart_command_t read_therms = {
		.readWrite = BROADCAST_READ,
		.dataLen = 1,
		.deviceAddress = 0xFF, //not used!
		.registerAddress = GPIO1_HI,
		.data = {bytesToRead-1},
		.crc = {0xFF, 0xFF}
	};

	taskENTER_CRITICAL();
	uart_sendCommand(&read_therms);

	uart_response_t response;
    if( uart_receiveResponse(&response, bytesToRead-1) != 0) {
        //loop through each GPIO channel
        // setBMBErr(i-1, BMB_TEMP_READ_ERROR);
	    // uart_sendCommand(&read_therms);
        taskEXIT_CRITICAL();
        return;
    }
	
	taskEXIT_CRITICAL();

    for(uint8_t k = 0; k < NUM_GPIO_CHANNELS; k++) {
        uint8_t cellNum = CHANNEL_GPIO_TO_CELL_MAP[channel][k];
        if (cellNum == 255)
            continue;

        uint8_t high_byte_data = response.data[2*k];
        uint8_t low_byte_data = response.data[2*k+1];
        int16_t cellTemp_centi_deg = thermVoltage_to_temp_Centi_Deg(high_byte_data, low_byte_data);
        
        BMBData.cellTemps_Centi_C[cellNum] = cellTemp_centi_deg;

    }
	return;
}


/**
 * @brief Converts thermistor voltage (mV) to temperature in centi-degrees Celsius.
 *
 * @param thermVolt_mV Measured voltage across the thermistor in millivolts.
 *
 * @return Temperature in centi-degrees Celsius (e.g., 2550 = 25.50°C). Returns 0 on out-of-bounds limits.
 */
static uint16_t thermVoltage_to_temp_Centi_Deg(uint8_t msb, uint8_t lsb){
    float pullup_ohms = 4700.0; 
    float v_in_mV = 5000.0; 
		uint16_t thermVolt_mV = (uint16_t)((0.15259) * (((int16_t) msb << 8) | lsb));

    //outside of upper bound 
    if (thermVolt_mV >= v_in_mV){
        return 0; 
    }

    float resistance = (pullup_ohms * thermVolt_mV) /  (v_in_mV - thermVolt_mV); 
    float temp_C; 

    //if thermVolt greater than max (don't allow negative temps)
    if (thermVolt_mV > thermMV_to_tempC[0]) {
        return 0; 
    }

    for (size_t i = 0; i < THERM_MV_TO_TEMP_NUM_ITEMS - 1; i++) {
        float table_resistance_lower = thermMV_to_tempC[i]; 
        float table_resistance_upper = thermMV_to_tempC[i+1]; 
        
        //temp is between i and i+1 (assume pretty much linear ratio)
        if (table_resistance_lower <= resistance && resistance < table_resistance_upper) {
            float decimal = INVLERP_SCALED(table_resistance_lower, table_resistance_upper, resistance, 1.0f);
            temp_C = i + decimal;
            return (uint16_t)(temp_C * 100); 
        }

    }
    // if we get to end of loop, voltage is less than lowest voltage in lut
    return THERM_MV_TO_TEMP_NUM_ITEMS * 100;
}

//return voltage data
uint8_t pollAllVoltageData() {
    static uint8_t bytesToRead = CELL_NUM*2-1;
    uart_command_t read_voltage = {
        .readWrite = BROADCAST_READ,
        .dataLen = 1,
        .deviceAddress = 0xFF, //not used!
        .registerAddress = TOP_CELL,
        .data = {bytesToRead}, //reading high and low for cell 0-CELL_NUM
        .crc = {0xFF, 0xFF}
    };

    uart_response_t response;

    // Critical section used so UART RX is not preempted
    taskENTER_CRITICAL();
    uart_sendCommand(&read_voltage);
    uint8_t status = uart_receiveResponse(&response, bytesToRead);
    if(status != 0) {
        //setBMBErr(i-1, BMB_VOLTAGE_READ_ERROR);
        //BMBTimeoutCount[i-1]+=1;
        // DWT_Delay_ms(10000);
        // RXTurnOnInit();
        // BMBInit();
        taskEXIT_CRITICAL();
        return 1;
    }
    taskEXIT_CRITICAL();

    // Handle writing data separately from receive so you don't miss a byte
    for(uint8_t j = 0; j < CELL_NUM; j++) {
        uint8_t high_byte_data = response.data[2*j];
        uint8_t low_byte_data = response.data[2*j+1];
        BMBData.cellVoltages_mV[CELL_NUM-j-1] = calculateVoltage(high_byte_data, low_byte_data);
    }
    return 0;
}