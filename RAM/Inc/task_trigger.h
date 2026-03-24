
/**
 * @file task_trigger
 * @brief Camera trigger interface.
 *
 * This module provides a periodic GPIO trigger for camera control with
 * dynamic delay capability via CAN message notifications.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef TASK_TRIGGER_H
#define TASK_TRIGGER_H

#include <stdint.h>
/**
 * @brief Initializes the camera trigger task.
 *
 * Creates a periodic task that generates GPIO trigger pulses at a fixed rate.
 * The GPIO pin goes HIGH for the specified exposure duration, then LOW.
 * 
 *
 * @param exposure_ms Camera exposure time in milliseconds.
 */
void cameraTriggerInit();

void setDelay(int32_t delay);


#endif // TASK_TRIGGER_H
