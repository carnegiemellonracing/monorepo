#ifndef TEST_FAKE_CMR_TASKS_H
#define TEST_FAKE_CMR_TASKS_H

#include <stdint.h>

typedef uint32_t TickType_t;
typedef unsigned int UBaseType_t;
typedef void *TaskHandle_t;
typedef void (*TaskFunction_t)(void *);

typedef struct {
    TaskHandle_t handle;
} cmr_task_t;

#define taskENTER_CRITICAL() do { } while (0)
#define taskEXIT_CRITICAL() do { } while (0)

void cmr_taskInit(
    cmr_task_t *task,
    const char *name,
    UBaseType_t priority,
    TaskFunction_t func,
    void *arg
);

TickType_t xTaskGetTickCount(void);
void vTaskDelayUntil(TickType_t *previousWakeTime, TickType_t timeIncrement);

#endif /* TEST_FAKE_CMR_TASKS_H */
