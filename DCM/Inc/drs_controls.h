#ifndef DRS_CONTROLS_H
#define DRS_CONTROLS_H

#include <stdint.h>
#include <stdbool.h>
#include <CMR/can_types.h>
#include "can.h"
#include "motors.h"
#include "servo.h"
#include <CMR/config_screen_helper.h>
#include <stddef.h>
#include <CMR/can_ids.h>    // CMR CAN IDs


// ------------------------------------------------------------------------------------------------
// Public functions

const cmr_canCDCDRSStates_t *getDRSInfo();

void runDrsControls(
    cmr_canGear_t gear,
    cmr_canDrsMode_t drsMode,
    uint8_t throttlePos_u8,
    uint16_t brakePressurePsi_u8,
    int16_t swAngle_deg
);

#endif
