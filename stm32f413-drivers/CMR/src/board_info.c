/**
 * @file board_info.c
 * @brief Board identifier information
 *
 * @author Carnegie Mellon Racing
 */

#include "CMR/board_info.h"
#include "CMR/panic.h"

bool cmr_validBoardInfo(void) {
    return BOARD_INFO->magic == BOARD_INFO_MAGIC;
}

void cmr_validateBoardInfo(void) {
    if (!cmr_validBoardInfo()) {
        cmr_panic("Invalid board info");
    }
}

uint32_t cmr_getVersion(void) {
    return BOARD_INFO->version;
}

uint32_t cmr_getBoardId(void) {
    return BOARD_INFO->board_id;
}

uint32_t cmr_getBootLoaderCanBusNum(void) {
    return BOARD_INFO->can_bus_num;
}