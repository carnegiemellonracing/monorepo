/**
 * @file board_info.c
 * @brief Board identifier information
 *
 * @author Carnegie Mellon Racing
 */

#include "CMR/board_info.h"

uint32_t cmr_getVersion(void) {
    return BOARD_INFO->version;
}

uint32_t cmr_getBoardId(void) {
    return BOARD_INFO->board_id;
}