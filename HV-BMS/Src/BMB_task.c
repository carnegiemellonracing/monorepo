/*
 * BMB_task.c
 *
 *  Created on: Jun 8, 2020
 *      Author: vamsi
 */

#include "BMB_task.h"
#include "can.h"
#include <stdbool.h>
#include <math.h>               // math.h

extern BMB_Data_t BMBData[BOARD_NUM-1];

extern volatile int BMBTimeoutCount[BOARD_NUM-1];
extern volatile int BMBErrs[BOARD_NUM-1];
extern bool firstBalDone[BOARD_NUM-1][VSENSE_CHANNELS]; 

#define BALANCE_EN true
#define BALANCE_DIS false
#define TO_IGNORE 6

static const uint8_t temp_to_ignore[] = {143};

// Use array to ignore some broken thermistor channels

bool check_to_ignore(uint8_t bmb_index, uint8_t channel) {
	for(int i = 0; i < sizeof(temp_to_ignore); i++) {
		if(bmb_index*TEMP_CHANNELS + channel == temp_to_ignore[i]) {
			return true;
		}
	}
	return false;
}

// Check CANRX struct for balance command and threshold

bool isBalanceCommanded() {
	return false;
	// TickType_t xLastWakeTime = xTaskGetTickCount();

	// if(cmr_canRXMetaTimeoutError(&(canRXMeta[CANRX_BALANCE_COMMAND]), xLastWakeTime) < 0) {
	// 	return false;
	// }
	
	// volatile cmr_canHVCBalanceCommand_t *balanceCommand = getPayload(CANRX_BALANCE_COMMAND);
	// return balanceCommand->balanceRequest;
}


// Main sample task entry point for BMS

void vBMBSampleTask(void *pvParameters) {

	// Previous wake time pointer
	TickType_t xLastWakeTime = xTaskGetTickCount();
	vTaskDelayUntil(&xLastWakeTime, 50);
	bool currentlyBalancing = false;
	bool ledToggle = false;
	uint16_t threshold = getPackMinCellVoltage() + 5; 
	uint32_t settleTimer_ms;
	bool settlingTimerStarted = false;
	bool newBalCommand = true; 

	// Main BMS control loop
	while (1) {

		// Balancing logic

		// If we get a balancing command and we weren't previously balancing, enable
		// cells to balance
		if (isBalanceCommanded() && !currentlyBalancing) {
			//clear first balance done when we get a new balance command (everything starts over)
			if (newBalCommand){
				for(int i = 0; i < BOARD_NUM - 1; i++){
					for (int j = 0; j<VSENSE_CHANNELS; j++){
						firstBalDone[i][j] = false; 
					}
				}
				newBalCommand = false; 
			} 
			if(getBalDone()==1){
				cellBalancing(BALANCE_EN, threshold); 
				currentlyBalancing = true;
			} 
		}

		// If the balancing command stops and we were balancing previously, disable
		// all balancing cells
		else if(!isBalanceCommanded() && currentlyBalancing){
			cellBalancing(BALANCE_DIS, threshold);
			currentlyBalancing = false;
		}

		// Once balancing timers have expired we stop balancing
		else if(getBalDone()==1 && currentlyBalancing && !settlingTimerStarted) {
			settlingTimerStarted = true;
			settleTimer_ms = xTaskGetTickCount(); 
		}

		// After a 500 ms resting period, recheck the voltages
		// and enable whichever cells are above threshold
		else if (settlingTimerStarted && (xTaskGetTickCount()-settleTimer_ms)>500) {
			currentlyBalancing = false;
			settlingTimerStarted = false;
		}

		//waiting for new balance command 
		if (!isBalanceCommanded()){
			newBalCommand = true; 
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
		int16_t temp = BMBData[bmb_index].cellTemperaturesVoltageReading[i];
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
		int16_t temp = BMBData[bmb_index].cellTemperaturesVoltageReading[i];
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
		uint16_t voltage = BMBData[bmb_index].cellTemperaturesVoltageReading[i];
		if ((voltage > maxVoltage)) {
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
		if ((voltage < minVoltage)) {
			minVoltage = voltage;
			cell_index = i;
		}
	}
	return cell_index;
}

// Accessor Functions

int16_t getBMBTemp(uint8_t bmb_index, uint8_t cell_index) {
	return BMBData[bmb_index].cellTemperaturesVoltageReading[cell_index];
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
		maxCellTemp = BMBData[bmb_index].cellTemperaturesVoltageReading[maxCellTempIndex];

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
		minCellTemp = BMBData[bmb_index].cellTemperaturesVoltageReading[minCellTempIndex];

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
		minCellTemp = BMBData[bmb_index].cellTemperaturesVoltageReading[minCellTempIndex];

		// update struct if needed
		if (minCellTemp < BMSMinMaxCellTemp->minCellTemp_C) {
			BMSMinMaxCellTemp->minCellTemp_C = (minCellTemp);
			BMSMinMaxCellTemp->minTempBMBNum = bmb_index;
			BMSMinMaxCellTemp->minTempCellNum = minCellTempIndex;
		}

		// find highest cell temp on current BMB
		maxCellTempIndex = getBMBMaxTempIndex(bmb_index);
		maxCellTemp = BMBData[bmb_index].cellTemperaturesVoltageReading[maxCellTempIndex];

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
