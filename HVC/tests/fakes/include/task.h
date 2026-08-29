#ifndef TEST_FAKE_TASK_H
#define TEST_FAKE_TASK_H

#include "FreeRTOS.h"

TickType_t xTaskGetTickCount(void);
void vTaskDelayUntil(TickType_t *previousWakeTime, TickType_t timeIncrement);

#endif /* TEST_FAKE_TASK_H */
