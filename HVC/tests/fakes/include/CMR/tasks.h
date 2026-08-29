#ifndef TEST_FAKE_CMR_TASKS_H
#define TEST_FAKE_CMR_TASKS_H

#include <FreeRTOS.h>
#include <task.h>

typedef struct {
    TaskHandle_t handle;
} cmr_task_t;

#endif /* TEST_FAKE_CMR_TASKS_H */
