/**
 * @file error.h
 * @brief Vehicle Safety Module error checking.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef ERROR_H
#define ERROR_H

#include <CMR/tasks.h>  // TickType_t, taskENTER_CRITICAL(), taskEXIT_CRITICAL()

#include "state.h"      // vsmStatus_t

extern const uint16_t brakePressureThreshold_PSI;

void updateCurrentErrors(volatile vsmStatus_t *vsmStatus, TickType_t lastWakeTime);
void updateCurrentWarnings(volatile vsmStatus_t *vsmStatus, TickType_t lastWakeTime);

#endif /* ERROR_H */
