/**
 * @file tasks.c
 * @brief FreeRTOS task wrapper implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "tasks.h"  // Interface to implement

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
    *stackLen = sizeof(stack) / sizeof(stack[0]);
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
    *stackLen = sizeof(stack) / sizeof(stack[0]);
}

#endif /* configUSE_TIMERS */

#endif /* configSUPPORT_STATIC_ALLOCATION */

