/**
 * @file can.h
 * @brief Board-specific CAN interface.
 *
 */

#ifndef CAN_H
#define CAN_H

#include <CMR/platform.h>   // STM config
#include <CMR/can_types.h>  // CMR CAN types
#include <CMR/can_ids.h>    // CMR CAN IDs

void canInit(void);
void canDeinit(void);

#endif /* CAN_H */

