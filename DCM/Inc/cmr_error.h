/**
 * @file error.h
 * @brief Board-specific error detection.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef ERROR_H
#define ERROR_H
#include <CMR/tasks.h>      // Task interface
#include <CMR/can_types.h>

void updateErrorsWarnings(cmr_canHeartbeat_t *heartbeat, TickType_t lastWakeTime);

#endif /* ERROR_H */
