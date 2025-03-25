#include <cmr/can_types.h>
#include <CMR/gpio.h>
#include <stddef.h>
#include <stdbool.h>

#include "can.h"
#include "charger.h"
#include "evse.h"
#include "sensors.h"
#include "gpio.h"

const int MAX_CURRENT_DERATE = 1; // 1 A

const int CHARGER_VOLTAGE_BASE = 5930; // 583 V
const int CHARGER_VOLTAGE_OFFSET = 100; // 10 V
const int MV_TO_DV = 100; // milliVolts to deciVolts
const int MV_TO_V = 1000;

const int CHARGER_CURRENT_MAX = 80; // 8 A
const int CHARGER_SLOW_CHARGE_CURRENT_MAX = 20; // 2 A
const int CHARGER_TRICKLE_CURRENT_MAX = 10;
static uint32_t min(uint32_t a, uint32_t b) {
    return (a < b) ? a : b;
}

static cmr_canDilongCommand_t chargerCommands[CHARGER_LEN] = {
    [CHARGER_ONE] = {
        .maxVoltageHigh = 0,
        .maxVoltageLow  = 0,
        .maxCurrentHigh = 0,
        .maxCurrentLow  = 0,
        .chargerDisable = 1,
        .enableHeating  = 0
    },
    [CHARGER_TWO] = {
        .maxVoltageHigh = 0,
        .maxVoltageLow  = 0,
        .maxCurrentHigh = 0,
        .maxCurrentLow  = 0,
        .chargerDisable = 1,
        .enableHeating  = 0
    }
};

static int32_t maxChargerCurrent() {
    int32_t pilotDuty = cmr_sensorListGetValue(&sensorList, SENSOR_CH_PILOT_DUTY);
    int32_t evse_max_current = (int32_t) getEvseCurrentLimit(pilotDuty);

    float power_limit = 0;
    if (evse_max_current <= 15) {
        // Assume 120V source
        power_limit = evse_max_current * 120.0f * 0.9f;
    } else {
        // Assume 208V source
        power_limit = evse_max_current * 208.0f * 0.94f;
    }

    volatile cmr_canHVCPackVoltage_t* canPackVoltage = (void *) canVehicleRXMeta[CANRX_HVC_PACK_VOLTAGE].payload;
    int32_t packVoltage_mV = canPackVoltage->battVoltage_mV;

    float voltageLimit_V = ((float) packVoltage_mV) / 1000.0f + 10.0f;

    // Multiply by 10 to convert into dA which is what the DiLong expects
    int32_t currentLimit_dA = (int32_t) (    (power_limit / voltageLimit_V) * 10);

    //Make sure to limit
    currentLimit_dA = currentLimit_dA;

    return currentLimit_dA;
}

// Returns deciVolts
static uint16_t getChargerVoltage(charger_t charger) {
    volatile cmr_canHVCPackVoltage_t *canHVCPackVoltage = (void *) canVehicleRXMeta[CANRX_HVC_PACK_VOLTAGE].payload;
    uint16_t packVoltage = canHVCPackVoltage->hvVoltage_mV; //voltage is in mV
    uint16_t voltage = min(CHARGER_VOLTAGE_BASE,
                          (packVoltage / MV_TO_DV) + CHARGER_VOLTAGE_OFFSET);
#ifdef MCU_ONE
    volatile cmr_canHVCPackMinMaxCellVolages_t *canCellVoltage = (void *) canVehicleRXMeta[CANRX_HVC_CELL_VOLTAGE].payload;
    uint16_t maxCellVoltage_mV = canCellVoltage->maxCellVoltage_mV;

    volatile cmr_canHVCPackVoltage_t* canPackVoltage = (void *) canVehicleRXMeta[CANRX_HVC_PACK_VOLTAGE].payload;
    int32_t packVoltage_mV = canPackVoltage->hvVoltage_mV;


    if (maxCellVoltage_mV >= 4200) {
        voltage = (uint16_t) ((packVoltage_mV + 2000) / 100);
        voltage = min(voltage, 6000);
    } else {
        voltage = CHARGER_VOLTAGE_BASE;
    }
    if (charger == CHARGER_ONE) {
        /*
        * MCU_ONE's first charger is always on during charging.
        *
        * This voltage is lower than the other chargers in order to keep
        * charger one in constant voltage mode.
        */
        return voltage;
    } else {
#endif
        return voltage;
#ifdef MCU_ONE
    }
#endif
}
/**
 * @brief Return what current level the charger should be
 * @param charger
 * @return uint16_t Current
 */
static uint16_t getDesiredChargerCurrent(charger_t charger) {
	volatile cmr_canHVCPackVoltage_t *canHVCPackVoltage = (void *) canVehicleRXMeta[CANRX_HVC_PACK_VOLTAGE].payload;
	int32_t voltage = canHVCPackVoltage->battVoltage_mV/MV_TO_V;
    int32_t pilotDuty = cmr_sensorListGetValue(&sensorList, SENSOR_CH_PILOT_DUTY);
    int32_t evse_max_current = (int32_t) getEvseCurrentLimit(pilotDuty);


    if(charger == CHARGER_ONE)
    {
        if(evse_max_current <= 15)
        {
            return min(maxChargerCurrent(), CHARGER_CURRENT_MAX);
        }
        return min(maxChargerCurrent() / 2, CHARGER_CURRENT_MAX);
    }
    else //Charger 2 until more chargers added
    {
        if(evse_max_current <= 15)
        {
            return 0;
        }
       return min(maxChargerCurrent() / 2, CHARGER_CURRENT_MAX);
    }
}
const cmr_canDilongCommand_t *getChargerCommand(charger_t charger) {
    if (charger >= CHARGER_LEN) {
        return NULL;
    }

    return (const cmr_canDilongCommand_t *) &(chargerCommands[charger]);
}

static void setChargerCommand(charger_t charger, uint16_t maxVoltage_dV, uint16_t maxCurrent_dA, bool chargerDisable) {
    if (charger >= CHARGER_LEN) {
        return;
    }

    volatile cmr_canHVCHeartbeat_t *canHVCHeartbeat = (void *) canVehicleRXMeta[CANRX_HVC_HEARTBEAT].payload;
    uint8_t hvcState = canHVCHeartbeat->hvcState;
    uint8_t hvcMode = canHVCHeartbeat->hvcMode;

    if (hvcMode == CMR_CAN_HVC_MODE_ERROR) {
        cmr_canDilongCommand_t *command = &(chargerCommands[charger]);
        command->maxVoltageHigh = 0;
        command->maxVoltageLow = 0;
        command->maxCurrentHigh = 0;
        command->maxCurrentLow = 0;
        command->chargerDisable = true;
        command->enableHeating = 0; // Never enable heating

        cmr_gpioWrite(GPIO_CHARGE_ENABLE, 0);

        return;
    }

    cmr_canDilongCommand_t *command = &(chargerCommands[charger]);
    command->maxVoltageHigh = (maxVoltage_dV >> 8) & 0xFF;
    command->maxVoltageLow = (maxVoltage_dV >> 0) & 0xFF;
    command->maxCurrentHigh = (maxCurrent_dA >> 8) & 0xFF;
    command->maxCurrentLow = (maxCurrent_dA >> 0) & 0xFF;
    command->chargerDisable = chargerDisable;
    command->enableHeating = 0; // Never enable heating

    cmr_gpioWrite(GPIO_CHARGE_ENABLE, 1);
}

void updateChargerCommands(cmr_CCMState_t state) {
    switch (state) {
    case CMR_CCM_STATE_CHARGE:
        //get voltage first then pass into getDesiredChargerCurrent to get current
        setChargerCommand(CHARGER_ONE, getChargerVoltage(CHARGER_ONE), getDesiredChargerCurrent(CHARGER_ONE), false);
        setChargerCommand(CHARGER_TWO, getChargerVoltage(CHARGER_TWO), getDesiredChargerCurrent(CHARGER_TWO), false);
        break;
    case CMR_CCM_STATE_SLOW_CHARGE:
        setChargerCommand(CHARGER_ONE, CHARGER_VOLTAGE_BASE, CHARGER_SLOW_CHARGE_CURRENT_MAX, false);
        setChargerCommand(CHARGER_TWO, CHARGER_VOLTAGE_BASE, CHARGER_SLOW_CHARGE_CURRENT_MAX, false);
        break;
    case CMR_CCM_STATE_ERROR:
    case CMR_CCM_STATE_CLEAR_ERROR:
    case CMR_CCM_STATE_STANDBY:
    case CMR_CCM_STATE_CHARGE_REQ:
    case CMR_CCM_STATE_SHUTDOWN:
    default:
        setChargerCommand(CHARGER_ONE, 0, 0, true);
        setChargerCommand(CHARGER_TWO, 0, 0, true);
        break;
    }
}
