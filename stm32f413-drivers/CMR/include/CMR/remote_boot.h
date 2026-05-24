/**
 * @file remote_boot.h
 * @brief Remote boot identifier information
 *
 * @author Carnegie Mellon Racing
 */
#ifndef CMR_REMOTE_BOOT_H
#define CMR_REMOTE_BOOT_H

#include <stdbool.h>

// Setup build ids
#ifdef PROJECT_DCM
#define BLT_REMOTE_ID 0x01
#elif defined(PROJECT_DIM)
#define BLT_REMOTE_ID 0x02
#elif defined(PROJECT_HVBMS)
#define BLT_REMOTE_ID 0x03
#elif defined(PROJECT_HVC)
#define BLT_REMOTE_ID 0x04e
#elif defined(PROJECT_HVI)
#define BLT_REMOTE_ID 0x05
#elif defined(PROJECT_LVBMS)
#define BLT_REMOTE_ID 0x06
#elif defined(PROJECT_RAM)
#define BLT_REMOTE_ID 0x07
#else
#define BLT_REMOTE_ID 0x00
#endif

// setup requested restart
#define BLT_REQUESTED_RESTART_IDX 0u

void cmr_remoteBootInit(void);
void cmr_requestRemoteFlash(void);
bool cmr_requestedRemoteFlash(void);
void cmr_resetRemoteFlash(void);

#endif /* CMR_REMOTE_BOOT_H */


