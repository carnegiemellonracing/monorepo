/**
 * @file can.h
 * @brief Board-specific CAN interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CAN_H
#define CAN_H

#include <CMR/can_types.h>  // CMR CAN types
#include <CMR/can_ids.h>    // CMR CAN IDs

extern volatile const cmr_canHeartbeat_t *canHeartbeatVSM;

void canInit(void);
void canTX(cmr_canID_t id, const void *data, size_t len);

#endif /* CAN_H */

