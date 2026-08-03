/*
 * BMB_task.h
 *
 *  Created on: Jul 30, 2026
 *      Author: anvitaa 
 */

#ifndef BMB_TASK_H_
#define BMB_TASK_H_

#include <CMR/bq_interface.h>

// bq_interface constants
#undef BOARD_NUM
#undef BMB_NUM
#undef CELL_NUM
#undef BMS_READ
#undef BMS_WRITE
#undef HV_BMS
#undef LV_BMS
#define BOARD_NUM 1
#define BMB_NUM 1
#define CELL_NUM 7
#define BMS_READ BROADCAST_READ
#define BMS_WRITE BROADCAST_WRITE
#define HV_BMS false
#define LV_BMS true

//TODO change this
#define CELL_BALANCING_LOW_VOLTAGE 3.9

typedef struct BMB_Data_t{
    uint16_t cellVoltages[CELL_NUM];
    int16_t cellTemperaturesVoltageReading[CELL_NUM];
} BMB_Data_t;

#define TOP_CELL VCELL7_HI
#define TOP_CELL_CB_ADDR CB_CELL7_CTRL
#define NUM_GPIO_CHANNELS 2

#endif /* BMB_TASK_H_ */
