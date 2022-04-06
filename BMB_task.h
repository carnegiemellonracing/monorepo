/*
 * BMB_task.h
 *
 *  Created on: Jun 8, 2020
 *      Author: vamsi
 */

#ifndef BMB_TASK_H_
#define BMB_TASK_H_

#include "FreeRTOS.h"
#include "slave_uart.h"
#include "task.h"
#include "stm32f413-drivers/CMR/can_types.h"

#define BMB_GPIO_MUX_PIN 0b000001
#define BMB_GPIO_LED_PIN 0b000100
#define BMB_GPIO_TEMP_FAULT_PIN 0b00000

#define VSENSE_CHANNELS_PER_BMB 9
#define TSENSE_CHANNELS_PER_BMB 12
#define TSENSE_CHANNELS_PER_MESSAGE 8

#define BMB_SAMPLE_TASK_RATE 10

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
void getBMSMinMaxCellVoltage(cmr_canBMSMinMaxCellVoltage_t *BMSMinMaxCellVoltage);
void getBMSMinMaxCellTemperature(cmr_canBMSMinMaxCellTemperature_t *BMSMinMaxCellTemp);
BMB_Data_t* getBMBData (uint8_t bmb_index);

int32_t getBattMillivolts();

#endif /* BMB_TASK_H_ */
