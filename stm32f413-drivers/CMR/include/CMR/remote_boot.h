/**
 * @file remote_boot.h
 * @brief Remote boot identifier information
 *
 * @author Carnegie Mellon Racing
 */
#ifndef CMR_REMOTE_BOOT_H
#define CMR_REMOTE_BOOT_H

#include <stdbool.h>


// setup requested restart
#define BLT_REQUESTED_RESTART_IDX 0u

// setup shared params system. This keeps static memory while the car is powered
void cmr_remoteBootInit(void);

// Get the current remote flash status
void cmr_requestRemoteFlash(void);
bool cmr_requestedRemoteFlash(void);
void cmr_resetRemoteFlash(void);
void cmr_checkMsgForRemoteFlash(uint32_t canID, uint8_t data[8]);

#endif /* CMR_REMOTE_BOOT_H */


