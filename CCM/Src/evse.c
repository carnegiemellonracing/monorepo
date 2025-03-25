#include "evse.h"

evseState_t getEvseState(int32_t voltage) {
    switch (voltage) {
    case 3:
    case 6:
        return EVSE_READY;
    case 9:
        return EVSE_VEHICLE_DETECTED;
    default:
        return EVSE_ERROR;
    }
}

uint32_t getEvseCurrentLimit(int32_t dutyCycle) {
    /*
     * See Table 5 in J1772 201710 for these calculations.
     */
    if (dutyCycle < 10) {
        return 0;
    } else if (dutyCycle <= 85) {
        return dutyCycle * 6 / 10;
    } else if (dutyCycle <= 96) {
        return (dutyCycle - 64) * 5 / 2;
    } else {
        return 0;
    }
}
