/*
 * @file tasks.h
 * @brief FreeRTOS task wrapper header.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef TASKS_H
#define TASKS_H

#include <FreeRTOS.h>   // FreeRTOS interface
#include <task.h>       // StaticTask_t, StackType_t

#if (configSUPPORT_STATIC_ALLOCATION)

void vApplicationGetIdleTaskMemory(
    StaticTask_t **task,
    StackType_t **stack,
    uint32_t *stackLen
);

#if (configUSE_TIMERS)
void vApplicationGetTimerTaskMemory(
    StaticTask_t **task,
    StackType_t **stack,
    uint32_t *stackLen
);
#endif /* configUSE_TIMERS */

#endif /* configSUPPORT_STATIC_ALLOCATION */


#endif /* TASKS_H */
