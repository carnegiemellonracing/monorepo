/**
 * @file remote_boot.h
 * @brief Remote boot identifier information
 *
 * @author Carnegie Mellon Racing
 */
#ifndef CMR_REMOTE_BOOT_H
#define CMR_REMOTE_BOOT_H

// Setup build ids
#ifdef PROJECT_DCM
#define BLT_REMOTE_ID 0x01
#elif defined(PROJECT_DIM)
#define BLT_REMOTE_ID 0x02
#elif defined(PROJECT_HVBMS)
#define BLT_REMOTE_ID 0x03
#elif defined(PROJECT_HVC)
#define BLT_REMOTE_ID 0x04
#elif defined(PROJECT_HVI)
#define BLT_REMOTE_ID 0x05
#elif defined(PROJECT_LVBMS)
#define BLT_REMOTE_ID 0x06
#elif defined(PROJECT_RAM)
#define BLT_REMOTE_ID 0x07
#else
#define BLT_REMOTE_ID 0x00
#endif


#endif /* CMR_REMOTE_BOOT_H */


