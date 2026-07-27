/**
 * @file board_info.h
 * @brief Board identifier information
 *
 * @author Carnegie Mellon Racing
 */
#ifndef CMR_BOARD_INFO_H
#define CMR_BOARD_INFO_H

#include <stdbool.h>
#include <stdint.h>


typedef struct {
    uint32_t version;
    uint32_t board_id;
} board_info_t;

#define BOARD_INFO ((const board_info_t *)0x8000000)

// Get the current board info
uint32_t cmr_getVersion(void);
uint32_t cmr_getBoardId(void);

#endif /* CMR_BOARD_INFO_H */


