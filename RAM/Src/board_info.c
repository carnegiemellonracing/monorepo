
#include <CMR/board_info.h>
#include "gitcommit.h"
#include "gpio.h"
#include "can.h"

__attribute__((section(".board_info")))
const board_info_t board_info = {
    .magic = BOARD_INFO_MAGIC,
    .version = GIT_INFO,
    .board_id = 0x01,
    .led_pin = &gpioPinConfigs[GPIO_LED_STATUS],
    .can_io = &canIoPinConfig[CMR_CAN_BUS_VEH],
};
