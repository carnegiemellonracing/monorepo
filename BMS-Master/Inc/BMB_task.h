/*
 * BMB_task.h
 *
 *  Created on: Jun 8, 2020
 *      Author: vamsi
 */

#ifndef BMB_TASK_H_
#define BMB_TASK_H_

/* #include "stm32f413-drivers/CMR/can_types.h" */
#include <CMR/can_types.h>
#include <stdint.h>
#include <stdbool.h>
#include "bq_interface.h"


#define BMB_SAMPLE_TASK_RATE 100

#define CELL_MAX_VOLTAGE_HI 4250
#define CELL_MAX_VOLTAGE_LO 4150
#define VSENSE_CHANNELS 14
#define TEMP_CHANNELS 14

typedef struct BMB_Data_t{
    uint16_t cellVoltages[VSENSE_CHANNELS];
    int16_t cellTemperatures[TEMP_CHANNELS];
} BMB_Data_t;

void vBMBSampleTask(void* pvParameters);

uint8_t getBMBMaxTempIndex (uint8_t bmb_index);
uint8_t getBMBMinTempIndex (uint8_t bmb_index);
uint8_t getBMBMaxVoltIndex (uint8_t bmb_index);
uint8_t getBMBMinVoltIndex (uint8_t bmb_index);

int16_t getBMBTemp (uint8_t bmb_index, uint8_t cell_index);
uint16_t getBMBVoltage (uint8_t bmb_index, uint8_t cell_index);
uint16_t getPackMinCellVoltage();
uint16_t getPackMaxCellVoltage();
uint16_t getPackMinCellTemp();
uint16_t getPackMaxCellTemp();
uint16_t selfTestMux(uint8_t mux_index);
void getBMSMinMaxCellVoltage(cmr_canBMSMinMaxCellVoltage_t *BMSMinMaxCellVoltage);
void getBMSMinMaxCellTemperature(cmr_canBMSMinMaxCellTemperature_t *BMSMinMaxCellTemp);
BMB_Data_t* getBMBData (uint8_t bmb_index);
bool check_to_ignore(uint8_t bmb_index, uint8_t channel);

int32_t getBattMillivolts();

#endif /* BMB_TASK_H_ */
