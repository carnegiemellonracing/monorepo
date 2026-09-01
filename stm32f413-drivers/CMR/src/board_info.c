/**
 * @file board_info.c
 * @brief Board identifier information
 *
 * @author Carnegie Mellon Racing
 */

#include "CMR/board_info.h"
#include "CMR/panic.h"
#include "CMR/platform.h"

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
    CAN_TypeDef *can_instance = cmr_getBootloaderCanIo().instance;
    if (can_instance == CAN1) {
        return 0;
    } else if (can_instance == CAN2) {
        return 1;
    } else if (can_instance == CAN3) {
        return 2;
    } else {
        cmr_panic("Invalid CAN instance in board info");
    }
}

cmr_gpioPin_t cmr_getBootloaderStatusLedPin(void) {
    cmr_gpioPinConfig_t* led_pin = BOARD_INFO->led_pin;
    return (cmr_gpioPin_t){.port = led_pin->port, .pin = led_pin->init.Pin};
}

cmr_canIo_t cmr_getBootloaderCanIo(void) {
    return *BOARD_INFO->can_io;
}
