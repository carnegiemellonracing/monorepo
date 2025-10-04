/*
 * state_task.h
 *
 *  Created on: Jun 8, 2020
 *      Author: vamsi
 */

#include "FreeRTOS.h"
#include "task.h"

#ifndef STATE_TASK_H_
#define STATE_TASK_H_

/* Drivers */
#include <CMR/can_types.h>
#include "bms_relay.h"
#include "errors.h"
#include "can.h"
#include "sensors.h"


// External functions
cmr_canHVCState_t getState(void);


// Task functions
void vSetStateTask(void *pvParameters);


#endif /* STATE_TASK_H_ */
