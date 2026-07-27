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
#include <CMR/bq_interface.h>

#define BMB_SAMPLE_TASK_RATE 100

#define CELL_MAX_VOLTAGE_HI 4250
#define CELL_MAX_VOLTAGE_LO 4150
#define CELL_NUM 9
#define TEMP_CHANNELS 9

// bq_interface constants
#undef BOARD_NUM
#undef BMB_NUM
#undef CELL_NUM
#undef BMS_READ
#undef BMS_WRITE
#undef HV_BMS
#define BOARD_NUM 17
#define BMB_NUM 16 // minus HVC
#define CELL_NUM 9
#define BMS_READ STACK_READ
#define BMS_WRITE STACK_WRITE
#define HV_BMS true

typedef struct BMB_Data_t{
    uint16_t cellVoltages[CELL_NUM];
    int16_t cellTemperaturesVoltageReading[TEMP_CHANNELS];
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
BMB_Data_t* getBMBData (uint8_t bmb_index);
bool check_to_ignore(uint8_t bmb_index, uint8_t channel);

int32_t getBattMillivolts();

#endif /* BMB_TASK_H_ */
