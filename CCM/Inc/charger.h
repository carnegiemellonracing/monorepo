/**
 * @file charger.h
 * @brief Charger interface
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CHARGER_H
#define CHARGER_H

#include <CMR/can_types.h>    // CMR CAN IDs
#include "state.h"

/** @brief Enumeration for chargers. */
typedef enum {
    CHARGER_ONE = 0,
    CHARGER_TWO,
    CHARGER_LEN
} charger_t;

void updateChargerCommands(cmr_CCMState_t state);
const cmr_canDilongCommand_t *getChargerCommand(charger_t charger);

#endif
