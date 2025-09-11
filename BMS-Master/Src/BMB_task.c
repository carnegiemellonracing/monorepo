/*
 * BMB_task.c
 *
 *  Created on: Jun 8, 2020
 *      Author: vamsi
 */

#include "BMB_task.h"
#include "gpio.h"
#include "can.h"
#include "state_task.h"
#include <stdbool.h>
#include <math.h>               // math.h

extern BMB_Data_t BMBData[BOARD_NUM-1];

extern volatile int BMBTimeoutCount[BOARD_NUM-1];
extern volatile int BMBErrs[BOARD_NUM-1];

#define BALANCE_EN true
#define BALANCE_DIS false
#define TO_IGNORE 6

static const uint8_t temp_to_ignore[] = { 2, 3, 4, 7, 11, 17, 18, 20, 21, 22,
		23, 25, 27, 30, 31, 35, 36, 37, 40, 43, 44, 45, 46, 47, 49, 50, 53, 59,
		60, 63, 70, 72, 73, 74, 77, 79, 84, 85, 87, 88, 89, 91, 94, 95, 96, 97,
		98, 101, 102, 104, 105, 109, 110, 115, 116, 119, 121, 125, 126, 129, 130,
		133, 134, 138};

// Use array to ignore some broken thermistor channels

bool check_to_ignore(uint8_t bmb_index, uint8_t channel) {
	for(int i = 0; i < sizeof(temp_to_ignore); i++) {
		if(bmb_index*14 + channel == temp_to_ignore[i]) {
			return true;
		}
	}
	return false;
}

// Check CANRX struct for balance command and threshold

bool getBalance(uint16_t *thresh) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	if(cmr_canRXMetaTimeoutError(&(canRXMeta[CANRX_BALANCE_COMMAND]), xLastWakeTime) < 0) {
		return false;
	}
	volatile cmr_canHVCBalanceCommand_t *balanceCommand = getPayload(CANRX_BALANCE_COMMAND);
	*thresh = balanceCommand->threshold;
	return balanceCommand->balanceRequest;
}


// Main sample task entry point for BMS

void vBMBSampleTask(void *pvParameters) {

	// Previous wake time pointer
	TickType_t xLastWakeTime = xTaskGetTickCount();
	vTaskDelayUntil(&xLastWakeTime, 50);
	bool prevStateBalance = false;
	bool ledToggle = false;
	uint16_t threshold = 0;
	uint32_t startTime;
	bool toReenable = false;

	// Main BMS control loop
	while (1) {

		// Balancing logic

		// If we get a balancing command and we weren't previously balancing, enable
		// cells to balance
		if (getBalance(&threshold) && !prevStateBalance) {
			cellBalancing(BALANCE_EN, threshold);
			prevStateBalance = true;
			startTime = xTaskGetTickCount();
		}

		// If the balancing command stops and we were balancing previously, disable
		// all balancing cells
		else if(!getBalance(&threshold) && prevStateBalance){
			cellBalancing(BALANCE_DIS, threshold);
			prevStateBalance = false;
		}

		// If 5 minutes has elapsed since balancing started, turn off balancing to
		// allow for rechecking of voltages
		else if((prevStateBalance && !toReenable) && (xTaskGetTickCount()-startTime)>300000) {
			cellBalancing(BALANCE_DIS, threshold);
			toReenable = true;
		}

		// After a 500 ms resting period, recheck the voltages
		// and enable whichever cells are above threshold
		else if(toReenable && (xTaskGetTickCount()-startTime)>300500) {
			cellBalancing(BALANCE_EN, threshold);
			startTime = xTaskGetTickCount();
			prevStateBalance = true;
			toReenable = false;
		}

		// Loop through the 4 different MUX channels and select a different one
		// We still monitor all voltages each channel switch
		for(uint8_t j = 0; j < 4; j++) {
			// Small delays put between all transaction

			setMuxOutput(j);

			xLastWakeTime = xTaskGetTickCount();
			vTaskDelayUntil(&xLastWakeTime, 10);

			uint8_t err = pollAllVoltageData();

			xLastWakeTime = xTaskGetTickCount();
			vTaskDelayUntil(&xLastWakeTime, 10);

			pollAllTemperatureData(j);

			xLastWakeTime = xTaskGetTickCount();
			vTaskDelayUntil(&xLastWakeTime, 10);

			writeLED(ledToggle);
			ledToggle = !ledToggle;

			xLastWakeTime = xTaskGetTickCount();
			vTaskDelayUntil(&xLastWakeTime, 100);
		}
	}
}


// Lookup functions
uint8_t getBMBMaxTempIndex(uint8_t bmb_index) {
	int16_t maxTemp = 0xFFFF;
	uint8_t cell_index = 0;
	for (uint8_t i = 0; i < TEMP_CHANNELS; i++) {
		int16_t temp = BMBData[bmb_index].cellTemperatures[i];
		if ((temp > maxTemp) && !check_to_ignore(bmb_index, i)) {
			maxTemp = temp;
			cell_index = i;
		}
	}
	return cell_index;
}

uint8_t getBMBMinTempIndex(uint8_t bmb_index) {
	int16_t minTemp = 0x7FFF;
	uint8_t cell_index = 0;
	for (uint8_t i = 0; i < TEMP_CHANNELS; i++) {
		int16_t temp = BMBData[bmb_index].cellTemperatures[i];
		if (temp < minTemp && !check_to_ignore(bmb_index, i)) {
			minTemp = temp;
			cell_index = i;
		}
	}
	return cell_index;
}

uint8_t getBMBMaxVoltIndex(uint8_t bmb_index) {
	uint16_t maxVoltage = 0;
	uint8_t cell_index = 0;
	for (uint8_t i = 0; i < VSENSE_CHANNELS; i++) {
		uint16_t voltage = BMBData[bmb_index].cellVoltages[i];
		if ((voltage > maxVoltage) && (voltage != 3456)) {
			maxVoltage = voltage;
			cell_index = i;
		}
	}
	return cell_index;
}

uint8_t getBMBMinVoltIndex(uint8_t bmb_index) {
	uint16_t minVoltage = 0xFFFF;
	uint8_t cell_index = 0;
	for (uint8_t i = 0; i < VSENSE_CHANNELS; i++) {
		uint16_t voltage = BMBData[bmb_index].cellVoltages[i];
		if ((voltage < minVoltage) && (voltage != 3456)) {
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

	for (uint8_t bmb_index = 0; bmb_index < BOARD_NUM-1; bmb_index++) {
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

	for (uint8_t bmb_index = 0; bmb_index < BOARD_NUM-1; bmb_index++) {
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

	for (uint8_t bmb_index = 0; bmb_index < BOARD_NUM-1; bmb_index++) {
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

	for (uint8_t bmb_index = 0; bmb_index < BOARD_NUM-1; bmb_index++) {
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

	for (uint8_t bmb_index = 0; bmb_index < BOARD_NUM-1; bmb_index++) {
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

	for (uint8_t bmb_index = 0; bmb_index < BOARD_NUM-1; bmb_index++) {
		// find lowest cell temp on current BMB
		minCellTempIndex = getBMBMinTempIndex(bmb_index);
		minCellTemp = BMBData[bmb_index].cellTemperatures[minCellTempIndex];

		// update struct if needed
		if (minCellTemp < BMSMinMaxCellTemp->minCellTemp_C) {
			BMSMinMaxCellTemp->minCellTemp_C = (minCellTemp);
			BMSMinMaxCellTemp->minTempBMBNum = bmb_index;
			BMSMinMaxCellTemp->minTempCellNum = minCellTempIndex;
		}

		// find highest cell temp on current BMB
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

//temp index 2,

BMB_Data_t* getBMBData(uint8_t bmb_index) {
	return &(BMBData[bmb_index]);
}

int32_t getBattMillivolts() {
	int32_t totalPackCellVoltage = 0;

	for (uint8_t bmb_index = 0; bmb_index < BOARD_NUM-1; bmb_index++) {
		for (uint8_t i = 0; i < VSENSE_CHANNELS; i++) {
			totalPackCellVoltage +=
					(int32_t) BMBData[bmb_index].cellVoltages[i];
		}
	}

	return totalPackCellVoltage;
}
