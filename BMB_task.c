/*
 * BMB_task.c
 *
 *  Created on: Jun 8, 2020
 *      Author: vamsi
 */

#include "BMB_task.h"
#include "gpio.h"
#include "state_task.h"
#include <stdbool.h>
#include <math.h>               // math.h

extern volatile int BMBTimeoutCount[NUM_BMBS];
extern volatile int BMBErrs[NUM_BMBS];

uint16_t cellsToBalance[NUM_BMBS] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
		0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};

// Max valid thermistor temp, beyond which it is considered a short
static const int16_t THERM_MAX_TEMP = 850;
// Min valid thermistor temp, beyond which it is considered open
static const int16_t THERM_MIN_TEMP = -10;

static int16_t linearTemp(uint16_t ADC_lt);

//Fill in data to this array
static BMB_Data_t BMBData[NUM_BMBS];
// temporary raw ADC Values
static int16_t BMBADCResponse[NUM_MUX_CHANNELS][NUM_ADC_CHANNELS];

//lookup array based on mux and adc pinouts of BMB
static ADC_Mux_Channel_t ADCChannelLookupArr[NUM_ADC_CHANNELS][NUM_MUX_CHANNELS] = {
                                          {CELL1, FIXED2, CELL2, CELL3},
										  {FIXED1, THERM13, THERM14, THERM15},
										  {THERM10, THERM11, FIXED3, THERM12},
										  {CELL4, CELL5, CELL6, FIXED4},
										  {THERM7, THERM8, THERM9, FIXED5},
										  {CELL9, FIXED6, CELL8, CELL7},
										  {FIXED7, THERM6, THERM5, THERM4},
										  {THERM3, FIXED8, THERM2, THERM1}};


// Counter for how many times until we flash the LED
static const uint8_t LED_FLASH_COUNT = 10;
static uint8_t BMBFlashCounter = 0;

static bool failedToInitI2c = false;
bool startingPrecharge = false;
bool haltedI2C = false;

static const uint8_t LUT_SIZE = 18;
static const uint16_t lut[18][2] = { { 8802, 850 }, { 9930, 800 }, { 11208, 750 },
		{ 12657, 700 }, { 14281, 650 }, { 16112, 600 }, { 18146, 550 }, {
				20408, 500 }, { 22879, 450 }, { 25575, 400 },
		{ 28459, 350 }, { 31533, 300 }, { 34744, 250 }, { 38019, 200 }, {
				41331, 150 }, { 44621, 100 }, { 47792, 50 }, { 50833, 0 }, };

static const uint16_t idealFixedVals[8] = {645, 564, 483, 403, 322, 241, 161, 80};

static void setBMBErr(uint8_t BMBIndex, BMB_I2C_Errs_t err) {
	BMBErrs[BMBIndex] = err;
}

//takes in adc output and cell index to get voltage value
static int16_t adcOutputToVoltage(uint16_t ADC_val, int cell) {
	return ((((int32_t)ADC_val) * 4) * 11) / 10;
}

/**
 * @brief Converts thermistor resisistance to temperature in C
 *
 * @param B Beta value of thermistor
 * @param refResistance Thermistor resistance at refTemp
 * @param refTemp Reference temperature (in Celcius)
 * @param thermResistance Current Thermistor Resistance
 * @return Temperature based on thermistor resistance, in Celcius
 */
float thermistorCalc(float B, float refResistance, float refTemp, float thermResistance) {
    const float celciusToKelvin = 273.15f;
    float temp_K = (B * (refTemp + celciusToKelvin)) / (B + ((refTemp + celciusToKelvin) * log(thermResistance/refResistance)));
    return temp_K - celciusToKelvin;
}

uint16_t thermistorCalculate(uint16_t adcVal) {
    const float B = 3435.f;
    const float refResistance_Ohm = 10000.f;
    const float refTemp_C = 25.f;

    float adcmV = ((float)adcVal) * 4.f;

    float thermistorResistance_Ohm = 4700.f * (adcmV) / (5000.f - adcmV);

    float sensedTemp_C = thermistorCalc(B, refResistance_Ohm, refTemp_C, thermistorResistance_Ohm);
    return (uint16_t) (sensedTemp_C * 10);
}

//smooth out voltage data with ______ filter 
uint16_t filterADCData(uint16_t cumulativeValue, uint16_t newValue, float weightingAlpha) {
	return (uint16_t)(cumulativeValue * (1-weightingAlpha) + newValue * weightingAlpha);
}


//update corresponding voltage or temperature reading
void updateBMBData(uint16_t val, uint8_t adcChannel, uint8_t muxChannel, uint8_t bmb) {
	ADC_Mux_Channel_t indexToUpdate = ADCChannelLookupArr[adcChannel][muxChannel];
	if(indexToUpdate <= CELL9) {
		int16_t voltage = adcOutputToVoltage(val, indexToUpdate);
		if (voltage > 4400 || voltage < 2000) {
			// don't update anything
			BMBData[bmb].cellVoltages[indexToUpdate] = 3456;
		} else {
			BMBData[bmb].cellVoltages[indexToUpdate] = filterADCData(BMBData[bmb].cellVoltages[indexToUpdate], voltage, FILTER_ALPHA);
		}
	}
	if(CELL9 < indexToUpdate && indexToUpdate <= THERM15) {
		uint16_t temp = thermistorCalculate(val);
		if (temp > 600 || temp < 200) {
			BMBData[bmb].cellTemperatures[indexToUpdate - THERM1] = 278;
		} else {
			BMBData[bmb].cellTemperatures[indexToUpdate - THERM1] = thermistorCalculate(val);
		}
	}
	if (indexToUpdate >= FIXED1) {
		uint8_t fixedNum = indexToUpdate - FIXED1;
		// voltage divider, 470k on top, rest is 100k between each fixed
		if (val < idealFixedVals[fixedNum] - 50 || val > idealFixedVals[fixedNum] + 50) {
			//BMBTimeoutCount[bmb]++;
			//setBMBErr(bmb, BMB_FIXED_CHECK_ERR);
		}
	}
}

void setAllBMBsTimeout() {
	for (int i = 0; i < NUM_BMBS; i++) {
		BMBTimeoutCount[i] = BMB_TIMEOUT;
	}
}

void BMBInit() {
	i2cInit();
	bool succeeded = false;
	for (int i = 0; i < 10; i++) {
		if (!i2cInitChain()) {
			continue;
			//cmr_panic("Couldn't initialize I2C BMB Chain");
		} else {
			succeeded = true;
			break;
		}
	}
	if (!succeeded) {
		failedToInitI2c = true;
	}
}

bool sampleOneBMB(uint8_t BMBIndex, uint8_t BMBNum, uint8_t BMBSide) {
    if (!i2c_enableI2CMux(BMBNum, BMBSide)) {
    	BMBTimeoutCount[BMBIndex]++;
    	setBMBErr(BMBIndex, BMB_ENABLE_I2C_MUX_ERR);
        return false;
    }
    //select through each of the mux channels
    for (int channel = 0; channel < NUM_MUX_CHANNELS; channel++) {
        if (!i2c_select4MuxChannel(channel)) {
        	BMBTimeoutCount[BMBIndex]++;
        	setBMBErr(BMBIndex, BMB_SEL_4_MUX_ERR);
            return false;
        }
        // through each channel, input 8 adc channels
        if (!i2c_scanADC(BMBADCResponse[channel])) {
        	BMBTimeoutCount[BMBIndex]++;
        	setBMBErr(BMBIndex, BMB_SCAN_ADC_ERR);
            return false;
        }
    }
    // increment the counter or reset if we just flashed
    if (BMBFlashCounter >= LED_FLASH_COUNT) {
        // we got to threshold, blink this BMB
        if (!i2c_selectMuxBlink()) {
        	setBMBErr(BMBIndex, BMB_MUX_BLINK_ERR);
        	BMBTimeoutCount[BMBIndex]++;
            return false;
        }
        BMBFlashCounter = 0;
    } else {
        BMBFlashCounter++;
    }
    if (!(i2c_disableI2CMux(BMBNum))) {
    	BMBTimeoutCount[BMBIndex]++;
    	setBMBErr(BMBIndex, BMB_DISABLE_I2C_MUX_ERR);
        return false;
    }
    uint8_t enabled, side;
    if (!i2c_readI2CMux(BMBNum, &enabled, &side) || enabled) {
    	BMBTimeoutCount[BMBIndex]++;
    	setBMBErr(BMBIndex, BMB_DISABLE_I2C_MUX_ERR);
        return false;
    }
    return true;
}

//void doCellBalanceOneBMB(uint8_t BMBIndex) {
//    uint32_t totalVoltage = 0;
//    //loop through all cells and turn adc output to voltage
//    for (int j = 0; j < VSENSE_CHANNELS_PER_BMB; j++) {
//        //find difference between current TOTAL cell voltage - previous TOTAL cell voltage
//        totalVoltage += BMBData[BMBIndex].cellVoltages[j];
//    }
//
//    //Add cell balancing here
//    uint32_t averageVoltage = totalVoltage / VSENSE_CHANNELS_PER_BMB;
//    uint16_t cellsToBalance = 0xFFFF;
//
//    for (uint16_t i = 0; i < VSENSE_CHANNELS_PER_BMB; i++) {
//        if (BMBData[BMBIndex].cellVoltages[i] - averageVoltage > 50) {
//            cellsToBalance &= ~(1 << i);
//        }
//    }
//
//    //we have 9 bits, so split the cells into two 8 bit integers
//    //LSB of the higher 8 bits is the 9th cell balancer
//    uint8_t balanceCommands[2];
//    balanceCommands[0] = 0xFF & cellsToBalance;
//    balanceCommands[1] = (0xFF00 & cellsToBalance) >> 8;
//
//    //only send balance command when changing
//    //otherwise, make sure all the balancing is OFF
//    if (getState() == CMR_CAN_HVC_STATE_CHARGE_CONSTANT_VOLTAGE) {
//        if (!i2c_cellBalance(BMBIndex, balanceCommands[0],
//                balanceCommands[1])) {
//        	BMBTimeoutCount[BMBIndex]++;
//        }
//    } else {
//        if (!i2c_cellBalance(BMBIndex, 0, 0)) {
//        	BMBTimeoutCount[BMBIndex]++;
//        }
//    }
//}

bool doCellBalanceAllBMBs() {
	//TODO: Check for timeout
	for (int j = 0; j < NUM_BMBS; j++) {
		for (uint16_t i = 0; i < VSENSE_CHANNELS_PER_BMB; i++) {
			if (BMBData[j].cellVoltages[i]  > CELL_MAX_VOLTAGE_HI) {
				cellsToBalance[j] &= ~(1 << i);
			}
			if (BMBData[j].cellVoltages[i]  < CELL_MAX_VOLTAGE_LO) {
				cellsToBalance[j] |= (1 << i);
			}
		}
    }

    //we have 9 bits, so split the cells into two 8 bit integers
    //LSB of the higher 8 bits is the 9th cell balancer
    uint8_t balanceCommands[NUM_BMBS][2];
	for(int i = 0; i < NUM_BMBS; i++) {
		balanceCommands[i][0] = 0xFF & cellsToBalance[i];
		balanceCommands[i][1] = (0xFF00 & cellsToBalance[i]) >> 8;
	}

    //only send balance command when changing
    //otherwise, make sure all the balancing is OFF
	for(int i = 0; i < NUM_BMBS; i++) {
		int BMBNum = i/2;
		int BMBSide = i%2;
		if (!i2c_enableI2CMux(BMBNum, BMBSide)) {
			BMBTimeoutCount[i]++;
        	continue;
    	}
		if (getState() == CMR_CAN_HVC_STATE_CHARGE_CONSTANT_VOLTAGE) { //TODO: CHANGE THIS BACK
			if (!i2c_cellBalance(i, balanceCommands[0],
					balanceCommands[1])) {
				BMBTimeoutCount[i]++;
				continue;
			}
		} else {
			if (!i2c_cellBalance(i, 0xFF, 0xFF)) {
				BMBTimeoutCount[i]++;
				continue;
			}
		}
		if (!(i2c_disableI2CMux(BMBNum))) {
			BMBTimeoutCount[i]++;
        	continue;
    	}
	}
}

// calculate all the values for a single BMB
// this does converting to voltage, converting to temp, calculating avg
void calculateOneBMB(uint8_t BMBIndex) {
    for(int mux = 0; mux < NUM_MUX_CHANNELS; mux++) {
        for(int adc = 0; adc < NUM_ADC_CHANNELS; adc++) {
            // convert each bmb response to voltage or temperature
            updateBMBData(BMBADCResponse[mux][adc], adc, mux, BMBIndex);
            BMBADCResponse[mux][adc] = 0xFFFF;
        }
    }
}


void vBMBSampleTask(void *pvParameters) {

	BMBInit();

	// Previous wake time pointer
	TickType_t xLastWakeTime = xTaskGetTickCount();
	vTaskDelayUntil(&xLastWakeTime, 50);

	while (1) {
		for (uint8_t BMBIndex = 0; BMBIndex < NUM_BMBS; BMBIndex++) {
			if (failedToInitI2c) break;
			// if (startingPrecharge) {
			// 	haltedI2C = true;
			// 	break;
			// }
			// if (haltedI2C) {
			// 	if (!i2cInitChain()) {
			// 		failedToInitI2c = true;
			// 	}
			// 	haltedI2C = false;
			// }
			//since we treat each BMB side as an individual bmb
			//we just check whether the current bmb index is odd/even
			uint8_t BMBSide = BMBIndex % 2;
			// our actual BMB number, the physical board
			uint8_t BMBNum = BMBIndex / 2;

			//Sample BMBs
			// Get interesting crashes in list.c:192 with critical
			//taskENTER_CRITICAL();
			// Sample a single BMB (number and side fully)
			// this disables the mux at the end as well
			if (!sampleOneBMB(BMBIndex, BMBNum, BMBSide)) {
				// there was an error, so reset mux
				BMBTimeoutCount[BMBIndex]++;
				if (!(i2c_disableI2CMux(BMBNum))) {
					BMBTimeoutCount[BMBIndex]++;
				}
			} else {
				BMBTimeoutCount[BMBIndex] = 0;
			}
			//taskEXIT_CRITICAL();

			if(BMBTimeoutCount[BMBIndex] != 0) {
				// we had a timeout, continue onto next BMB
				continue;
			}

			// Calculate the values for this BMB
			calculateOneBMB(BMBIndex);

			//doCellBalanceOneBMB(BMBIndex); // WE DON'T DO THIS ANYMORE

		} // end for loop
		if (!failedToInitI2c && !startingPrecharge)
			doCellBalanceAllBMBs();
		//TickType_t temp = xTaskGetTickCount();
		if (xLastWakeTime+BMB_SAMPLE_TASK_RATE < xTaskGetTickCount()) {
			// we are already in the past trying to catch up, just give up and sleep for like 10ms
			xLastWakeTime = xTaskGetTickCount();
		}
		vTaskDelayUntil(&xLastWakeTime, BMB_SAMPLE_TASK_RATE);
	}
}

// Temperature Transfer Functions

// Returns temperature in 1/10th degC given ADC
// using a linear best fit of the transfer function.
// See drive doc "18e CMR BMS Temperature Math"
static int16_t linearTemp(uint16_t ADC_lt) {
	return (int16_t) ((-2 * ((int32_t) (uint32_t) ADC_lt)) / 117 + 860);
}

// Lookup functions
uint8_t getBMBMaxTempIndex(uint8_t bmb_index) {
	int16_t maxTemp = 0xFFFF;
	uint8_t cell_index = 0;
	for (uint8_t i = 0; i < TSENSE_CHANNELS_PER_BMB; i++) {
		int16_t temp = BMBData[bmb_index].cellTemperatures[i];
		if (temp > maxTemp) {
			maxTemp = temp;
			cell_index = i;
		}
	}
	return cell_index;
}

uint8_t getBMBMinTempIndex(uint8_t bmb_index) {
	int16_t minTemp = 0x7FFF;
	uint8_t cell_index = 0;
	for (uint8_t i = 0; i < TSENSE_CHANNELS_PER_BMB; i++) {
		int16_t temp = BMBData[bmb_index].cellTemperatures[i];
		if (temp < minTemp) {
			minTemp = temp;
			cell_index = i;
		}
	}
	return cell_index;
}

uint8_t getBMBMaxVoltIndex(uint8_t bmb_index) {
	uint16_t maxVoltage = 0;
	uint8_t cell_index = 0;
	for (uint8_t i = 0; i < VSENSE_CHANNELS_PER_BMB; i++) {
		uint16_t voltage = BMBData[bmb_index].cellVoltages[i];
		if (voltage > maxVoltage) {
			maxVoltage = voltage;
			cell_index = i;
		}
	}
	return cell_index;
}

uint8_t getBMBMinVoltIndex(uint8_t bmb_index) {
	uint16_t minVoltage = 0xFFFF;
	uint8_t cell_index = 0;
	for (uint8_t i = 0; i < VSENSE_CHANNELS_PER_BMB; i++) {
		uint16_t voltage = BMBData[bmb_index].cellVoltages[i];
		if (voltage < minVoltage) {
			minVoltage = voltage;
			cell_index = i;
		}
	}
	return cell_index;
}

// Accessor Functions

int16_t getBMBTemp(uint8_t bmb_index, uint8_t cell_index) {
	return BMBData[bmb_index].cellTemperatures[cell_index];
}

uint16_t getBMBVoltage(uint8_t bmb_index, uint8_t cell_index) {
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

void getBMSMinMaxCellVoltage(
		cmr_canBMSMinMaxCellVoltage_t *BMSMinMaxCellVoltage) {
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

void getBMSMinMaxCellTemperature(
		cmr_canBMSMinMaxCellTemperature_t *BMSMinMaxCellTemp) {
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
			totalPackCellVoltage +=
					(int32_t) BMBData[bmb_index].cellVoltages[i];
		}
	}

	return totalPackCellVoltage;
}
