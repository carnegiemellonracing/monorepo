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
#include <CMR/gpio.h>


typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t board_id;
    uint32_t can_bus_num;
    cmr_gpioPin_t led_pin;
    cmr_gpioPin_t can_tx_pin;
    cmr_gpioPin_t can_rx_pin;
} board_info_t;

#define BOARD_INFO ((const board_info_t *)0x8000000)
#define BOARD_INFO_MAGIC 0xB0A1D0A1

// Get the current board info
// ! This function will panic if invalid board info
void cmr_validateBoardInfo(void);
bool cmr_validBoardInfo(void);
uint32_t cmr_getVersion(void);
uint32_t cmr_getBoardId(void);
uint32_t cmr_getBootLoaderCanBusNum(void);

#endif /* CMR_BOARD_INFO_H */


