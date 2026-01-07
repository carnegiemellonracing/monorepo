/**
 * @file sensoric.c
 * @brief Board-specific senesoric implementation.
 *
 *
 * @author Carnegie Mellon Racing
 */

#include "sensoric.h"
#include <CMR/can_types.h> 
#include <math.h>

void sensoric_parse(uint16_t canID, volatile void *payload) {

    canVehicleRx_t sensoric_msg;

    volatile void *velocity = canVehicleGetPayload(CANRX_VEH_SENSORIC_VEL_ANG_POI);
}