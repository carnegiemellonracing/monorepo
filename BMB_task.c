/*
 * BMB_task.c
 *
 *  Created on: Jun 8, 2020
 *      Author: vamsi
 */

#include "BMB_task.h"
#include "gpio.h"
#include "state_task.h"
#include "i2c.h"
#include <stdbool.h>

extern volatile int BMBTimeoutCount[NUM_BMBS];

// Max valid thermistor temp, beyond which it is considered a short
static const int16_t THERM_MAX_TEMP = 850;
// Min valid thermistor temp, beyond which it is considered open
static const int16_t THERM_MIN_TEMP = -10;

static int16_t linearTemp(uint16_t ADC_lt);

//Fill in data to this array
static BMB_Data_t BMBData[NUM_BMBS];

// Counter for how many times until we flash the LED
static const uint8_t LED_FLASH_COUNT = 5;
static uint8_t BMBFlashCounter = 0;

//takes in adc output and cell index to get voltage value
static int16_t adcOutputToVoltage(uint16_t ADC_val, int cell) {
	const float resistorRatios[10];
	return ((ADC_val / 1023.0) * 4096) * resistorRatios[cell % 10];
}

// Returns temperature in 1/10th degC given ADC
// using LUT interpolation from the transfer function.
// See drive doc "18e CMR BMS Temperature Math" for LUT
static int16_t lutTemp(uint16_t ADC_lt) {
	const uint8_t LUT_SIZE = 18;
	const uint16_t lut[18][2] = { { 8802, 850 }, { 9930, 800 }, { 11208, 750 },
			{ 12657, 700 }, { 14281, 650 }, { 16112, 600 }, { 18146, 550 }, {
					20408, 500 }, { 22879, 450 }, { 25575, 400 },
			{ 28459, 350 }, { 31533, 300 }, { 34744, 250 }, { 38019, 200 }, {
					41331, 150 }, { 44621, 100 }, { 47792, 50 }, { 50833, 0 }, };

	// Check if input is out of LUT bounds
	// If so, return the boundary values
	if (ADC_lt < lut[0][0]) {
		return lut[0][1];
	}
	if (ADC_lt > lut[LUT_SIZE - 1][0]) {
		return lut[LUT_SIZE - 1][1];
	}

	// Modified LUT linear interpolation code from stack overflow
	uint8_t i;
	for (i = 0; i < LUT_SIZE - 1; ++i) {
		if (lut[i][0] <= ADC_lt && lut[i + 1][0] >= ADC_lt) {
			// Target value is between two LUT points
			uint16_t diffADC = ADC_lt - lut[i][0];
			uint16_t diffLUT = lut[i + 1][0] - lut[i][0];

			return lut[i][1] + ((lut[i + 1][1] - lut[i][1]) * diffADC) / diffLUT;
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
	i2cInit();
}

void vBMBSampleTask(void *pvParameters) {

	BMBInit();

	// Index of the BMB we're currently sampling
	uint8_t BMBIndex = 0;
	// Whether or not select on the analog mux is asserted

	// Previous wake time pointer
	TickType_t xLastWakeTime = xTaskGetTickCount();
	vTaskDelayUntil(&xLastWakeTime, 50);

	for (;;) {

		int16_t BMBADCResponse[8];

		//Sample BMBs
		taskENTER_CRITICAL();

		//loop through each side of the bmb
		for (uint8_t j = 0; j < 2; j++) {
			if (!i2c_enableI2CMux(j, BMBIndex)) {
				BMBTimeoutCount[BMBIndex] = BMB_TIMEOUT;
			} else {
				//select through each of the mux channels
				for (int channel = 0; channel < 3; channel++) {
					if (!i2c_select4MuxChannel(channel)
							|| !i2c_scanADC(BMBADCResponse)) {
						BMBTimeoutCount[BMBIndex] = BMB_TIMEOUT;
					} else {

						if (channel == 0) {
							BMBData[BMBIndex].cellVoltages[(j * 10) + 0] =
									BMBADCResponse[0];
							BMBData[BMBIndex].cellVoltages[(j * 10) + 4] =
									BMBADCResponse[1];
							BMBData[BMBIndex].cellVoltages[(j * 10) + 7] =
									BMBADCResponse[2];
						} else if (channel == 1) {
							BMBData[BMBIndex].cellVoltages[(j * 10) + 1] =
									BMBADCResponse[0];
							BMBData[BMBIndex].cellVoltages[(j * 10) + 5] =
									BMBADCResponse[1];
							BMBData[BMBIndex].cellVoltages[(j * 10) + 8] =
									BMBADCResponse[2];
						} else if (channel == 2) {
							BMBData[BMBIndex].cellVoltages[(j * 10) + 2] =
									BMBADCResponse[0];
							BMBData[BMBIndex].cellVoltages[(j * 10) + 6] =
									BMBADCResponse[1];
							BMBData[BMBIndex].cellVoltages[(j * 10) + 9] =
									BMBADCResponse[2];
						} else if (channel == 3) {
							BMBData[BMBIndex].cellVoltages[(j * 10) + 4] =
									BMBADCResponse[0];
						}
						if (channel < 3) {
							for (int temps = 0; temps < 5; temps++) {
								//TODO: ADD TEMP LUT
								BMBData[BMBIndex].cellTemperatures[(15 * j)
										+ (temps * 5) + channel] = lutTemp(
										(uint16_t) BMBADCResponse[temps + 3]);
							}
						}
					}

				}
			}
		}
		if (!(i2c_disableI2CMux(BMBIndex))) {
			BMBTimeoutCount[BMBIndex] = BMB_TIMEOUT;
		}

		taskEXIT_CRITICAL();

		uint32_t totalVoltage = 0;

		//loop through all cells and turn adc output to voltage

		for (int j = 0; j < VSENSE_CHANNELS_PER_BMB; j++) {
			BMBData[BMBIndex].cellVoltages[j] = adcOutputToVoltage(
					BMBData[BMBIndex].cellVoltages[j], j);
			//find difference between current TOTAL cell voltage - previous TOTAL cell voltage
			if (j != 0 || j != 10) {
				BMBData[BMBIndex].cellVoltages[j] =
						BMBData[BMBIndex].cellVoltages[j]
								- BMBData[BMBIndex].cellVoltages[j - 1];
			}
			totalVoltage += BMBData[BMBIndex].cellVoltages[j];
		}

		//Add cell balancing here
		uint32_t averageVoltage = totalVoltage / VSENSE_CHANNELS_PER_BMB;
		uint16_t cellsToBalance[2] = { 0x0000, 0x0000 };

		for (uint16_t i = 0; i < VSENSE_CHANNELS_PER_BMB; i++) {
			if (BMBData[BMBIndex].cellVoltages[i] - averageVoltage > 50
					&& (i != 0 && i != 10)) {
				cellsToBalance[i / 10] |= 1 << i;
			}
		}

		//we have 9 bits, so split the cells into two 8 bit integers
		//LSB of the higher 8 bits is the 9th cell balancer
		uint8_t balanceCommands[4];
		balanceCommands[0] = 0xFF & cellsToBalance[0];
		balanceCommands[1] = (0xFF00 & cellsToBalance[0]) >> 8;
		balanceCommands[2] = 0xFF & cellsToBalance[1];
		balanceCommands[3] = (0xFF00 & cellsToBalance[1]) >> 8;

		//only send balance command when changing
		//otherwise, make sure all the balancing is OFF
		if (getState() == CMR_CAN_HVC_STATE_CHARGE_CONSTANT_VOLTAGE) {
			i2c_enableI2CMux(0, BMBIndex);
			if (!i2c_cellBalance(BMBIndex, balanceCommands[0],
					balanceCommands[1])) {
				BMBTimeoutCount[BMBIndex] = BMB_TIMEOUT;
			}
			i2c_enableI2CMux(1, BMBIndex);
			if (!i2c_cellBalance(BMBIndex, balanceCommands[2],
					balanceCommands[3])) {
				BMBTimeoutCount[BMBIndex] = BMB_TIMEOUT;
			}
		} else {
			i2c_enableI2CMux(0, BMBIndex);
			if (!i2c_cellBalance(BMBIndex, 0, 0)) {
				BMBTimeoutCount[BMBIndex] = BMB_TIMEOUT;
			}
			i2c_enableI2CMux(1, BMBIndex);
			if (!i2c_cellBalance(BMBIndex, 0, 0)) {
				BMBTimeoutCount[BMBIndex] = BMB_TIMEOUT;
			}
		}

		if (BMBIndex < 8) {
			BMBIndex++;
		} else {
			BMBIndex = 0;
		}
		vTaskDelayUntil(&xLastWakeTime, 5);
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
