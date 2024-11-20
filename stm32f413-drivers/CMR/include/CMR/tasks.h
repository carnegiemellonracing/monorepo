/**
 * @file tasks.h
 * @brief FreeRTOS task wrapper interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CMR_TASKS_H
#define CMR_TASKS_H

#include <FreeRTOS.h>   // FreeRTOS interface
#include <task.h>       // Task interface

/**
 * @brief Represents a task.
 *
 * @note The contents of this struct are opaque to the library consumer.
 */
typedef struct {
    TaskHandle_t handle;    /**< @brief FreeRTOS task handle. */
#if (configSUPPORT_STATIC_ALLOCATION)
    StaticTask_t taskBuf;   /**< @brief FreeRTOS task storage. */

    /** @brief Task stack buffer. */
    StackType_t stackBuf[configMINIMAL_STACK_SIZE];
#endif
} cmr_task_t;

void cmr_taskInit(
    cmr_task_t *task,
    const char *name,
    UBaseType_t priority,
    TaskFunction_t func,
    void *arg
);
void cmr_taskDestroy(cmr_task_t *task);

TaskHandle_t cmr_taskHandle(const cmr_task_t *task);

#endif /* CMR_TASKS_H */

