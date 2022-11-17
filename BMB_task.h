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

// #define NUM_BMBS 16
#define NUM_BMBS 1 // TODO: CHANGE THIS BACK
#define NUM_MUX_CHANNELS 4
#define NUM_ADC_CHANNELS 8
#define BMB_GPIO_MUX_PIN 0b000001
#define BMB_GPIO_LED_PIN 0b000100
#define BMB_GPIO_TEMP_FAULT_PIN 0b00000

#define VSENSE_CHANNELS_PER_BMB 9
#define TSENSE_CHANNELS_PER_BMB 15
#define TSENSE_CHANNELS_PER_MESSAGE 8

#define BMB_SAMPLE_TASK_RATE 10

//Define array indices for the ADC Channels

typedef enum {
    CELL1 = 0,
    CELL2,
    CELL3,
    CELL4,
    CELL5,
    CELL6,
    CELL7,
    CELL8,
    CELL9,
    THERM1,
    THERM2,
    THERM3,
    THERM4,
    THERM5,
    THERM6,
    THERM7,
    THERM8,
    THERM9,
    THERM10,
    THERM11,
    THERM12,
    THERM13,
    THERM14,
    THERM15,
    FIXED1,
    FIXED2,
    FIXED3,
    FIXED4,
    FIXED5,
    FIXED6,
    FIXED7,
    FIXED8
} ADC_Mux_Channel_t;

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
