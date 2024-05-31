/**
 * @file tasks.c
 * @brief FreeRTOS task wrapper implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "tasks.h"  // Interface to implement

/**
 * @brief Initializes a task.
 *
 * @param task The task to initialize.
 * @param name The task's name (a human-readable string describing the task).
 * @param priority The task's priority (larger is higher-priority).
 * @param func The task's entry point function.
 * @param arg The argument passed to the entry point function.
 */
void cmr_taskInit(
    cmr_task_t *task,
    const char *name,
    UBaseType_t priority,
    TaskFunction_t func,
    void *arg
) {
#if (configSUPPORT_STATIC_ALLOCATION)
    task->handle = xTaskCreateStatic(
        func,
        name,
        sizeof(task->stackBuf) / sizeof(task->stackBuf[0]),
        arg,
        priority,
        task->stackBuf,
        &task->taskBuf
    );
#elif (configSUPPORT_DYNAMIC_ALLOCATION)
    task->handle = xTaskCreate(
        func,
        name,
        configMINIMAL_STACK_SIZE
        arg,
        priority
    );
#else
#error "At least one of configSUPPORT_{STATIC,DYNAMIC}_ALLOCATION must be 1!"
#endif

    configASSERT(task->handle != NULL);
}

/**
 * @brief Destroys a task.
 *
 * @param task The task.
 */
void cmr_taskDestroy(cmr_task_t *task) {
    vTaskDelete(task->handle);
}

/**
 * @brief Gets a task's handle.
 *
 * @param task The task.
 *
 * @return The task's handle.
 */
TaskHandle_t cmr_taskHandle(const cmr_task_t *task) {
    return task->handle;
}

#if (configSUPPORT_STATIC_ALLOCATION)

/**
 * @brief Provides memory used by the FreeRTOS idle task.
 *
 * @see https://www.freertos.org/a00110.html#configSUPPORT_STATIC_ALLOCATION
 *
 * @param task[out] The task state buffer.
 * @param stack[out] The stack buffer.
 * @param stackLen[out] The stack buffer's length, in words (NOT bytes).
 */
void vApplicationGetIdleTaskMemory(
    StaticTask_t **task,
    StackType_t **stack,
    uint32_t *stackLen
) {
    /** @brief The task state buffer. */
    static StaticTask_t taskBuf;

    /** @brief The task's stack buffer. */
    static StackType_t stackBuf[configMINIMAL_STACK_SIZE];

    *task = &taskBuf;
    *stack = stackBuf;
    *stackLen = sizeof(stackBuf) / sizeof(stackBuf[0]);
}

#if (configUSE_TIMERS)

/**
 * @brief Provides memory used by the FreeRTOS timer task.
 *
 * @see https://www.freertos.org/a00110.html#configSUPPORT_STATIC_ALLOCATION
 *
 * @param task[out] The task state buffer.
 * @param stack[out] The stack buffer.
 * @param stackLen[out] The stack buffer's length, in words (NOT bytes).
 */
void vApplicationGetTimerTaskMemory(
    StaticTask_t **task,
    StackType_t **stack,
    uint32_t *stackLen
) {
    /** @brief The task state buffer. */
    static StaticTask_t taskBuf;

    /** @brief The task's stack buffer. */
    static StackType_t stackBuf[configMINIMAL_STACK_SIZE];

    *task = &taskBuf;
    *stack = stackBuf;
    *stackLen = sizeof(stackBuf) / sizeof(stackBuf[0]);
}

#endif /* configUSE_TIMERS */

#endif /* configSUPPORT_STATIC_ALLOCATION */

