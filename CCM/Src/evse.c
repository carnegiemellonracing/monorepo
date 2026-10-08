#include "evse.h"

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
