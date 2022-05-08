/*
 * BMB_task.c
 *
 *  Created on: Jun 8, 2020
 *      Author: vamsi
 */

#include "BMB_task.h"
#include "gpio.h"
#include "state_task.h"

extern volatile int BMBTimeoutCount[NUM_BMBS];

// Max valid thermistor temp, beyond which it is considered a short
static const int16_t THERM_MAX_TEMP = 850;
// Min valid thermistor temp, beyond which it is considered open
static const int16_t THERM_MIN_TEMP = -10;

static int16_t linearTemp(uint16_t ADC_lt);

//Fill in data to this array
static BMB_Data_t BMBData[NUM_BMBS];

// Returns temperature in 1/10th degC given ADC
// using LUT interpolation from the transfer function.
// See drive doc "18e CMR BMS Temperature Math" for LUT
static int16_t lutTemp(uint16_t ADC_lt) {
    const uint8_t LUT_SIZE = 18;
    const uint16_t lut[18][2] = {
        {8802, 850},
        {9930, 800},
        {11208, 750},
        {12657, 700},
        {14281, 650},
        {16112, 600},
        {18146, 550},
        {20408, 500},
        {22879, 450},
        {25575, 400},
        {28459, 350},
        {31533, 300},
        {34744, 250},
        {38019, 200},
        {41331, 150},
        {44621, 100},
        {47792, 50},
        {50833, 0},
    };

    // Check if input is out of LUT bounds
    // If so, return the boundary values
    if (ADC_lt < lut[0][0]) {
        return lut[0][1];
    }
    if (ADC_lt > lut[LUT_SIZE-1][0]) {
        return lut[LUT_SIZE-1][1];
    }

    // Modified LUT linear interpolation code from stack overflow
    uint8_t i;
    for(i = 0; i < LUT_SIZE-1; ++i){
        if (lut[i][0] <= ADC_lt && lut[i+1][0] >= ADC_lt){
            // Target value is between two LUT points
            uint16_t diffADC = ADC_lt - lut[i][0];
            uint16_t diffLUT = lut[i+1][0] - lut[i][0];

            return lut[i][1] + ((lut[i+1][1] - lut[i][1]) * diffADC) / diffLUT;
        }
    }

    // Something went wrong, return max temp
    return 850;
}

void setAllBMBsTimeout() {
    for (int i = 0; i < NUM_BMBS; i++) {
        BMBTimeoutCount[i] = BMB_TIMEOUT;
    }
}

void BMBInit() {
    // Period
    const TickType_t xPeriod = 1000 / BMB_SAMPLE_TASK_RATE;		// In ticks (ms)
    #define MAXRETRY 10
    int retryCount;
    for (retryCount = 0; retryCount < MAXRETRY; retryCount++) {
        cmr_gpioWrite(GPIO_BMB_POWER_ENABLE_L, 1);
        if (retryCount >= MAXRETRY - 1) {
            cmr_panic("Can't initialize BMBs");
        }

        HAL_Delay(1000);

        // Enable BMB IO power (active low)
        cmr_gpioWrite(GPIO_BMB_POWER_ENABLE_L, 0);

        // Wake BMB0
        // NOTE: This MUST happen immediately after power on
        cmr_gpioWrite(GPIO_BMB_WAKE_PIN, 1);
        // Wait then clear wake signal
        // BMB requires a pulse for wake
        HAL_Delay(100);
        cmr_gpioWrite(GPIO_BMB_WAKE_PIN, 0);

        
        HAL_Delay(100);

        // Initialize the slave UART interface
        // taskENTER_CRITICAL();
        cmr_uart_result_t retv = slave_uart_autoAddress();
        // taskEXIT_CRITICAL();

        if (retv != UART_SUCCESS) {
            // ERROR CASE: Could not auto address the slave boards
            setAllBMBsTimeout();
            continue;
        }

        // vTaskDelayUntil(&xLastWakeTime, xPeriod);

        retv = slave_uart_configureChannels();
        if (retv != UART_SUCCESS) {
            // ERROR CASE: The slaves were not able to configure their channels and OV/UV thresholds
            setAllBMBsTimeout();
            continue;
        }

        // Set communications timeout
        retv = slave_uart_broadcast_setBMBTimeout();
        if (retv != UART_SUCCESS) {
            // ERROR CASE: The slaves were not able to configure their channels and OV/UV thresholds
            setAllBMBsTimeout();
            continue;
        }

        // vTaskDelayUntil(&xLastWakeTime, xPeriod);

        // Initialize the slave UART sampling and GPIO per board
        for(int8_t boardNum = TOP_SLAVE_BOARD; boardNum >= 0; --boardNum) {
            // taskENTER_CRITICAL();
            retv = slave_uart_configureSampling(boardNum);
            if (retv != UART_SUCCESS) {
                // ERROR CASE: Could not configure sampling for slave boards
                BMBTimeoutCount[boardNum] = BMB_TIMEOUT;
                // taskEXIT_CRITICAL();
                continue;
            }

            // Configure GPIO as outputs for analog mux select line and LED
            retv = slave_uart_configureGPIODirection(BMB_GPIO_MUX_PIN | BMB_GPIO_LED_PIN, boardNum);
            // taskEXIT_CRITICAL();

            if (retv != UART_SUCCESS) {
                // ERROR CASE: The slaves were not able to configure the AFE
                BMBTimeoutCount[boardNum] = BMB_TIMEOUT;
                continue;
            }
            // vTaskDelayUntil(&xLastWakeTime, xPeriod);
        }

        break;
    }
}


void vBMBSampleTask(void *pvParameters) {

    
    BMBInit();

    // Index of the BMB we're currently sampling
    uint8_t BMBIndex = 0;
    // Whether or not select on the analog mux is asserted
    bool BMBActivityLEDEnable = false;

    // Previous wake time pointer
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        uart_response_t channelResponse = {0};
        cmr_uart_result_t uartRetv = UART_SUCCESS;

        // Sampling method #2: BQ Protocol p12

        // Tell all BMBs to sample their channels and store the results locally
        // taskENTER_CRITICAL();
        // Set the analog mux to sample the relevant half of the thermistors and set the status LED
        uint8_t BMBGPIOValues = (BMBActivityLEDEnable) ? (BMB_GPIO_LED_PIN) : 0;
        uartRetv = slave_uart_setGPIO(BMBGPIOValues, BMBIndex);
        if (uartRetv != UART_SUCCESS) {
            // ERROR CASE: We could not send the set GPIO command
            BMBTimeoutCount[BMBIndex]++;
        }

        // Sample all analog channels
        uartRetv = slave_uart_broadcast_sampleAndStore();

        if (uartRetv != UART_SUCCESS) {
            // ERROR CASE: We could not send the sample command
            BMBTimeoutCount[BMBIndex]++;
        }
        // taskEXIT_CRITICAL();

        // Retrieve the channel data from the device in question
        // taskENTER_CRITICAL();
        uartRetv = slave_uart_sampleDeviceChannels(BMBIndex, &channelResponse);
        // taskEXIT_CRITICAL();

        if(uartRetv != UART_SUCCESS ||
           (channelResponse.frameInit->responseBytes+1 <
           2*(VSENSE_CHANNELS_PER_BMB+TSENSE_CHANNELS_PER_MESSAGE))) {
            // ERROR CASE: We could not sample the BMB's channels correctly
            // or the response did not include the expect number of channels

            // TODO: add MIA checking for each BMB
            BMBTimeoutCount[BMBIndex]++;
        } else {
            BMBTimeoutCount[BMBIndex] = 0;
            // Retrieve each 16 bit cell voltage reading from the response
            for(uint8_t vChannel = 0; vChannel < VSENSE_CHANNELS_PER_BMB; ++vChannel) {
                uint32_t readAdcValue = (((uint32_t)channelResponse.data[2*vChannel])<<8) |
                                        ((uint32_t)channelResponse.data[2*vChannel+1]);
                
                //This is backwards for some reason.
                uint32_t volt = (5000*readAdcValue)/65535;
                float mult = 0.7f;
                BMBData[BMBIndex].cellVoltages[VSENSE_CHANNELS_PER_BMB - vChannel - 1] = mult * BMBData[BMBIndex].cellVoltages[VSENSE_CHANNELS_PER_BMB - vChannel - 1] + (1.0f-mult) * volt;
            }

            // Avg out cell 0 and cell 1
            uint16_t avg = BMBData[BMBIndex].cellVoltages[0] + BMBData[BMBIndex].cellVoltages[1];
            avg /= 2;
            BMBData[BMBIndex].cellVoltages[0] = avg;
            BMBData[BMBIndex].cellVoltages[1] = avg;


            // Retrieve each 16 bit temperature reading from the response
            for(uint8_t tChannel = 0; tChannel < TSENSE_CHANNELS_PER_MESSAGE; ++tChannel) {
                uint32_t readAdcValue = (((uint32_t)channelResponse.data[2*tChannel + 2*VSENSE_CHANNELS_PER_BMB])<<8) |
                                        ((uint32_t)channelResponse.data[2*tChannel+2*VSENSE_CHANNELS_PER_BMB+1]);
                uint8_t logicalThermIndex = TSENSE_CHANNELS_PER_MESSAGE - 1 - tChannel;

                // Temps indexed 4 to 11 are muxed
                // TODO: make #define
                if (BMBActivityLEDEnable && logicalThermIndex >= 4) {
                    logicalThermIndex = logicalThermIndex + 4;
                }
                
                //This is backwards for some reason.
                BMBData[BMBIndex].cellTemperatures[logicalThermIndex] = lutTemp((uint16_t)readAdcValue);

                // TODO set error conditions for bad temps
            }
        }

        uint16_t averageVoltage = 0;
        uint16_t cellsToBalance = 0x0000;

        for(uint16_t i = 0; i < VSENSE_CHANNELS_PER_BMB; i++) {
            averageVoltage += BMBData[BMBIndex].cellVoltages[i];
        }

        averageVoltage = averageVoltage/VSENSE_CHANNELS_PER_BMB;
        
        for(uint16_t i = 0; i < VSENSE_CHANNELS_PER_BMB; i++) {
            if(BMBData[BMBIndex].cellVoltages[i] - averageVoltage > 50){
                cellsToBalance |= 1 << i;
            }
        }
        
        // taskENTER_CRITICAL();
        if(getState() == CMR_CAN_HVC_STATE_CHARGE_CONSTANT_VOLTAGE){
            slave_uart_sendBalanceCmd(cellsToBalance, BMBIndex);
        }
        else{
            slave_uart_sendBalanceCmd(0x0000, BMBIndex);
        }
        // taskEXIT_CRITICAL();


        if (BMBIndex >= NUM_BMBS-1) {
            BMBIndex = 0;
            BMBActivityLEDEnable = !BMBActivityLEDEnable;
        } else {
            ++BMBIndex;
        }


        vTaskDelayUntil(&xLastWakeTime, 5);
    }
}

// Temperature Transfer Functions

// Returns temperature in 1/10th degC given ADC
// using a linear best fit of the transfer function.
// See drive doc "18e CMR BMS Temperature Math"
static int16_t linearTemp(uint16_t ADC_lt) {
    return (int16_t)((-2*((int32_t)(uint32_t)ADC_lt))/117 + 860);
}

// Lookup functions
uint8_t getBMBMaxTempIndex (uint8_t bmb_index){
    int16_t maxTemp = 0xFFFF;
    uint8_t cell_index = 0;
    for (uint8_t i = 0; i < TSENSE_CHANNELS_PER_BMB; i++) {
        int16_t temp = BMBData[bmb_index].cellTemperatures[i];
        if(temp > maxTemp) {
            maxTemp = temp;
            cell_index = i;
        }
    }
    return cell_index;
}

uint8_t getBMBMinTempIndex (uint8_t bmb_index){
    int16_t minTemp = 0x7FFF;
    uint8_t cell_index = 0;
    for (uint8_t i = 0; i < TSENSE_CHANNELS_PER_BMB; i++) {
        int16_t temp = BMBData[bmb_index].cellTemperatures[i];
        if(temp < minTemp) {
            minTemp = temp;
            cell_index = i;
        }
    }
    return cell_index;
}

uint8_t getBMBMaxVoltIndex (uint8_t bmb_index){
    uint16_t maxVoltage = 0;
    uint8_t cell_index = 0;
    for (uint8_t i = 0; i < VSENSE_CHANNELS_PER_BMB; i++) {
        uint16_t voltage = BMBData[bmb_index].cellVoltages[i];
        if(voltage > maxVoltage) {
            maxVoltage = voltage;
            cell_index = i;
        }
    }
    return cell_index;
}

uint8_t getBMBMinVoltIndex (uint8_t bmb_index){
    uint16_t minVoltage = 0xFFFF;
    uint8_t cell_index = 0;
    for (uint8_t i = 0; i < VSENSE_CHANNELS_PER_BMB; i++) {
        uint16_t voltage = BMBData[bmb_index].cellVoltages[i];
        if(voltage < minVoltage) {
            minVoltage = voltage;
            cell_index = i;
        }
    }
    return cell_index;
}

// Accessor Functions

int16_t getBMBTemp (uint8_t bmb_index, uint8_t cell_index) {
    return BMBData[bmb_index].cellTemperatures[cell_index];
}

uint16_t getBMBVoltage (uint8_t bmb_index, uint8_t cell_index) {
    return BMBData[bmb_index].cellVoltages[cell_index];
}

uint16_t getPackMaxCellVoltage() {
	uint16_t packMaxCellVoltage = 0;
	uint16_t maxCellVoltage = 0;
	uint8_t maxCellVoltageIndex;
	
	for (uint8_t bmb_index = 0; bmb_index < NUM_BMBS; bmb_index++) {
		// find highest cell voltage on current BMB, update packMaxCellVoltage if needed
		maxCellVoltageIndex = getBMBMaxVoltIndex(bmb_index);
		maxCellVoltage = BMBData[bmb_index].cellVoltages[maxCellVoltageIndex];
		
		if (maxCellVoltage > packMaxCellVoltage) {
			packMaxCellVoltage = maxCellVoltage;
		}
	}
	
	return packMaxCellVoltage;
}

uint16_t getPackMinCellVoltage() {
	uint16_t packMinCellVoltage = UINT16_MAX;
	uint16_t minCellVoltage = 0;
	uint8_t minCellVoltageIndex;

	for (uint8_t bmb_index = 0; bmb_index < NUM_BMBS; bmb_index++) {
		// find lowest cell temp on current BMB, update packMinCellVoltage if needed
		minCellVoltageIndex = getBMBMinVoltIndex(bmb_index);
		minCellVoltage = BMBData[bmb_index].cellVoltages[minCellVoltageIndex];
		
		if (minCellVoltage < packMinCellVoltage) {
			packMinCellVoltage = minCellVoltage;
		}
	}
	
	return packMinCellVoltage;
}

uint16_t getPackMaxCellTemp() {
    uint16_t packMaxCellTemp = 0;
    uint16_t maxCellTemp = 0;
    uint8_t maxCellTempIndex;

    for (uint8_t bmb_index = 0; bmb_index < NUM_BMBS; bmb_index++) {
        // find highest cell temp on current BMB, update packMaxCellTemp if needed
        maxCellTempIndex = getBMBMaxTempIndex(bmb_index);
        maxCellTemp = BMBData[bmb_index].cellTemperatures[maxCellTempIndex];

        if (maxCellTemp > packMaxCellTemp) {
            packMaxCellTemp = maxCellTemp;
        }
    }

    return packMaxCellTemp;
}

uint16_t getPackMinCellTemp() {
    uint16_t packMinCellTemp = UINT16_MAX;
    uint16_t minCellTemp = 0;
    uint8_t minCellTempIndex;

    for (uint8_t bmb_index = 0; bmb_index < NUM_BMBS; bmb_index++) {
        // find lowest cell temp on current BMB, update packMinCellTemp if needed
        minCellTempIndex = getBMBMinTempIndex(bmb_index);
        minCellTemp = BMBData[bmb_index].cellTemperatures[minCellTempIndex];

        if (minCellTemp < packMinCellTemp) {
            packMinCellTemp = minCellTemp;
        }
    }

    return packMinCellTemp;
}

void getBMSMinMaxCellVoltage(cmr_canBMSMinMaxCellVoltage_t *BMSMinMaxCellVoltage) {
	BMSMinMaxCellVoltage->minCellVoltage_mV = UINT16_MAX;
	BMSMinMaxCellVoltage->maxCellVoltage_mV = 0;
	
	uint16_t minCellVoltage;
	uint16_t maxCellVoltage;
	
	uint8_t minCellVoltageIndex;
	uint8_t maxCellVoltageIndex;
	
	for (uint8_t bmb_index = 0; bmb_index < NUM_BMBS; bmb_index++) {
		// find lowest cell temp on current BMB
		minCellVoltageIndex = getBMBMinVoltIndex(bmb_index);
		minCellVoltage = BMBData[bmb_index].cellVoltages[minCellVoltageIndex];
		
		// update struct if needed
		if (minCellVoltage < BMSMinMaxCellVoltage->minCellVoltage_mV) {
			BMSMinMaxCellVoltage->minCellVoltage_mV = (minCellVoltage);
			BMSMinMaxCellVoltage->minVoltageBMBNum = bmb_index;
			BMSMinMaxCellVoltage->minVoltageCellNum = minCellVoltageIndex;
		}
		
		// find highest cell voltage on current BMB
		maxCellVoltageIndex = getBMBMaxVoltIndex(bmb_index);
		maxCellVoltage = BMBData[bmb_index].cellVoltages[maxCellVoltageIndex];
		
		// update struct if needed
		if (maxCellVoltage > BMSMinMaxCellVoltage->maxCellVoltage_mV) {
			BMSMinMaxCellVoltage->maxCellVoltage_mV = (maxCellVoltage);
			BMSMinMaxCellVoltage->maxVoltageBMBNum = bmb_index;
			BMSMinMaxCellVoltage->maxVoltageCellNum = maxCellVoltageIndex;
		}
	}
}

void getBMSMinMaxCellTemperature(cmr_canBMSMinMaxCellTemperature_t *BMSMinMaxCellTemp) {
    BMSMinMaxCellTemp->minCellTemp_C = UINT16_MAX;
    BMSMinMaxCellTemp->maxCellTemp_C = 0;

    uint16_t minCellTemp;
    uint16_t maxCellTemp;

    uint8_t minCellTempIndex;
    uint8_t maxCellTempIndex;

    for (uint8_t bmb_index = 0; bmb_index < NUM_BMBS; bmb_index++) {
        // find lowest cell temp on current BMB
        minCellTempIndex = getBMBMinTempIndex(bmb_index);
        minCellTemp = BMBData[bmb_index].cellTemperatures[minCellTempIndex];

        // update struct if needed
        if (minCellTemp < BMSMinMaxCellTemp->minCellTemp_C) {
            BMSMinMaxCellTemp->minCellTemp_C = (minCellTemp);
            BMSMinMaxCellTemp->minTempBMBNum = bmb_index;
            BMSMinMaxCellTemp->minTempCellNum = minCellTempIndex;
        }

        // find highest cell voltage on current BMB
        maxCellTempIndex = getBMBMaxTempIndex(bmb_index);
        maxCellTemp = BMBData[bmb_index].cellTemperatures[maxCellTempIndex];

        // update struct if needed
        if (maxCellTemp > BMSMinMaxCellTemp->maxCellTemp_C) {
            BMSMinMaxCellTemp->maxCellTemp_C = (maxCellTemp);
            BMSMinMaxCellTemp->maxTempBMBNum = bmb_index;
            BMSMinMaxCellTemp->maxTempCellNum = maxCellTempIndex;
        }
    }
}

BMB_Data_t* getBMBData(uint8_t bmb_index) {
    return &(BMBData[bmb_index]);
}

int32_t getBattMillivolts() {
    int32_t totalPackCellVoltage = 0;

    for (uint8_t bmb_index = 0; bmb_index < NUM_BMBS; bmb_index++) {
        for (uint8_t i = 0; i < VSENSE_CHANNELS_PER_BMB; i++) {
            totalPackCellVoltage += (int32_t) BMBData[bmb_index].cellVoltages[i];
        }
    }

    return totalPackCellVoltage;
}
