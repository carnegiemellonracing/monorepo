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

#define NUM_BMBS 16
#define BMB_GPIO_MUX_PIN 0b000001
#define BMB_GPIO_LED_PIN 0b000100
#define BMB_GPIO_TEMP_FAULT_PIN 0b00000

#define VSENSE_CHANNELS_PER_BMB 9
#define TSENSE_CHANNELS_PER_BMB 30
#define TSENSE_CHANNELS_PER_MESSAGE 8

#define BMB_SAMPLE_TASK_RATE 10

//Define array indices for the ADC Channels
#define CELL1 0
#define CELL2 1
#define CELL3 2
#define CELL4 3
#define CELL5 4
#define CELL6 5
#define CELL7 6
#define CELL8 7
#define CELL9 8
#define THERM1 9
#define THERM2 10
#define THERM3 11
#define THERM4 12
#define THERM5 13
#define THERM6 14
#define THERM7 15
#define THERM8 16
#define THERM9 17
#define THERM10 18
#define THERM11 19
#define THERM12 20
#define THERM13 21
#define THERM14 22
#define THERM15 23
#define FIXED1 24
#define FIXED2 25
#define FIXED3 26
#define FIXED4 27
#define FIXED5 28
#define FIXED6 29
#define FIXED7 30
#define FIXED8 31

typedef struct BMB_Data_t{
    uint16_t cellVoltages[VSENSE_CHANNELS_PER_BMB];
    int16_t cellTemperatures[TSENSE_CHANNELS_PER_BMB];
} BMB_Data_t;

void vBMBSampleTask(void* pvParameters);
void BMBInit();

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

int32_t getBattMillivolts();

#endif /* BMB_TASK_H_ */
