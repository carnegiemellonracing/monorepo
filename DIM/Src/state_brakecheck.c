typedef enum {
    BEGIN,
    SHUTDOWN_CIRCUIT_WAIT,
    TIME_CHECK,
    EBS_PRESSURE_CHECK,
    HYDRAULIC_PRESSURE_CHECK_1,
    TSAB_WAIT,
    MB_VALVES_SWITCH_ON_OFF,
    HYDRAULIC_PRESSURE_CHECK_2,
    MB_VALVES_SWITCH_OFF_ON,
    HYDRAULIC_PRESSURE_CHECK_3,
    READY,
    ERROR
} brakecheck_state;

brakecheck_state currState = BEGIN;
brakecheck_state nextState = BEGIN;

TickType_t lastWakeTime;

bool getShutdownCircuitClosed () {
    return true;
};

TickType_t getTime() {
    return xTaskGetTickCount();
}


#define EBS_PRESSURE_BAR_MIN 8.0
#define EBS_PRESSURE_BAR_MAX 10.0

/**
 * @brief checks ebs brake pressure between min and max bars in EBS_PRESSURE_CHECK state
 */
static bool getEBSPressureValid(){
    cmr_canDVPressureReadings_t* pressureReading = (cmr_canDVPressureReadings_t*) getPayload(CANRX_AS_PRESSURE_READING);
   bool brakes_good = pressureReading->ebsPressure_1_deci_bar > EBS_PRESSURE_BAR_MIN &&  
            pressureReading->ebsPressure_2_deci_bar > EBS_PRESSURE_BAR_MIN &&
            pressureReading->ebsPressure_1_deci_bar < EBS_PRESSURE_BAR_MAX &&  
            pressureReading->ebsPressure_2_deci_bar < EBS_PRESSURE_BAR_MAX;
    return brakes_good;
}

static bool checkHydraulicPressure(float min, float max, float min2, float max2) {
    return true;
}


void getNextState() {
    switch (currState) {
        BEGIN: {
            nextState = SHUTDOWN_CIRCUIT_WAIT;
            lastWakeTime = getTime();
            break;
        },
        SHUTDOWN_CIRCUIT_WAIT: {
            if (getShutdownCircuitClosed()) {
                nextState = TIME_CHECK;
            } else {
                nextState = SHUTDOWN_CIRCUIT_WAIT;
            }
            break;
        },
        TIME_CHECK: {
            if (getTime() - lastWakeTime < 50) {
                nextState = EBS_PRESSURE_CHECK;
            } else {
                nextState = ERROR;
            }
        },
        EBS_PRESSURE_CHECK: {
            if (getEBSPressureValid()) {
                nextState = HYDRAULIC_PRESSURE_CHECK_1;
            } else {
                nextState = ERROR;
            }
        },
        HYDRAULIC_PRESSURE_CHECK_1: {
            if (checkHydraulicPressure(60, 90, 30, 45)) {
                nextState = TSAB_WAIT;
            } else {
                nextState = ERROR;
            }
        },
        TSAB_WAIT: {
            if (TSActivated()) {
                nextState = MB_VALVES_SWITCH_ON_OFF;
            } else {
                nextState = TSAB_WAIT;
            }
        },

        



        default: {
            nextState = ERROR;
        }
    }
};