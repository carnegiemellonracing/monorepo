/**
 * @file camera.h
 * @brief Camera trigger interface.
 *
 * This module provides a periodic GPIO trigger for camera control with
 * dynamic delay capability via CAN message notifications.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CAMERA_H
#define CAMERA_H

#include <CMR/tasks.h>  // Task interface
#include "FreeRTOS.h"   // FreeRTOS types



extern TaskHandle_t cameraTrigger_taskHandle;


/**
 * @brief Initializes the camera trigger task.
 *
 * Creates a periodic task that generates GPIO trigger pulses at a fixed rate.
 * The GPIO pin goes HIGH for the specified exposure duration, then LOW.
 *
 * @param exposure_ms Camera exposure time in milliseconds.
 */
void cameraTriggerInit(TickType_t exposure_ms);

/**
 * @brief Delays the next camera trigger by a specified amount.
 *
 * Call this function from CAN RX ISR or task to delay the next trigger.
 * If the delay causes a trigger to be missed, it will wait for the
 * following trigger cycle.
 *
 * @param delay_ms Delay amount in milliseconds.
 * @param pxHigherPriorityTaskWoken Pointer to flag for context switch
 *                                   (required for ISR calls, NULL for task calls).
 *
 * @return pdPASS if notification sent successfully, pdFAIL otherwise.
 */
BaseType_t delayCameraTrigger(uint32_t delay_ms, BaseType_t *pxHigherPriorityTaskWoken);

#endif // CAMERA_H