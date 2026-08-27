//Based on "DSF - Startup Control Check Flowchat"
#include <stdbool.h>
#include <gpio.h>
#include <sensors.h>
#include <can.h>

typedef enum {
    TSAB_WAIT,
    MB_VALVES_SWITCH_ON_OFF,
    WAIT_FOR_VALVES_1,
    HYDRAULIC_PRESSURE_CHECK_1,
    MB_VALVES_SWITCH_OFF_ON,
    WAIT_FOR_VALVES_2,
    HYDRAULIC_PRESSURE_CHECK_2,
    READY,
    EBS_ERROR
} brakecheck_state;

brakecheck_state currState = TSAB_WAIT;
brakecheck_state nextState = TSAB_WAIT;
TickType_t lastWakeTime;

static bool checkDSMS();

TickType_t getTime();

static bool checkEBSPressure();

static bool checkHydraulicPressure(float BP3_min, float BP3_max, float BP4_min, float BP4_max);

static bool checkShutdownCircuit();

static bool checkTSActivated();

#define EBS_PRESSURE_BAR_MIN 8.0
#define EBS_PRESSURE_BAR_MAX 10.0
#define VALVES_WAIT_TIME 5000



void getNextState() {
    switch (currState) {
        case TSAB_WAIT: {
            if (checkTSActivated()) { //checks is TS activation button pushed
                nextState = MB_VALVES_SWITCH_ON_OFF;
            } else {
                nextState = TSAB_WAIT;
            }
            break;
        }

        case MB_VALVES_SWITCH_ON_OFF:{
            cmr_gpiowrite(GPIO_VALVE_MB1, 1);
            cmr_gpiowrite(GPIO_VALVE_MB2, 0);
            nextState = WAIT_FOR_VALVES_1;
            lastWakeTime = getTime();
            break;
        }
        case WAIT_FOR_VALVES_1: {
            if (getTime() - VALVES_WAIT_TIME >= lastWakeTime) {
                nextState = HYDRAULIC_PRESSURE_CHECK_1;
            } else {
                nextState = WAIT_FOR_VALVES_1;
            }
            break;
        }
        case HYDRAULIC_PRESSURE_CHECK_1:{
            if (checkHydraulicPressure(90, 10000, 0, 1)){ //10000 just a big number, if pressure within bounds then good
               nextState = MB_VALVES_SWITCH_OFF_ON; 
            } else {
                nextState = EBS_ERROR;
            }
            break;
        }
        case MB_VALVES_SWITCH_OFF_ON:{
            cmr_gpiowrite(GPIO_VALVE_MB1, 0);
            cmr_gpiowrite(GPIO_VALVE_MB2, 1);
            nextState = WAIT_FOR_VALVES_2;
            lastWakeTime = getTime();
            break;
        }
        case WAIT_FOR_VALVES_2: {
            if (getTime() - VALVES_WAIT_TIME >= lastWakeTime) {
                nextState = HYDRAULIC_PRESSURE_CHECK_1;
            } else {
                nextState = WAIT_FOR_VALVES_1;
            }
            break;
        }
        case HYDRAULIC_PRESSURE_CHECK_2:{
            if (checkHydraulicPressure(0,1,45,10000)){ //10000 just a big number, if pressure within bounds then good
               nextState = MB_VALVES_SWITCH_OFF_ON; 
            } else {
                nextState = EBS_ERROR;
            }
            break;
        }
        case READY:{
            //what to do now?
            break;
        }
        case EBS_ERROR:{
            //What to do now?
            // do dim request error
            break;
        }
        default: {
            nextState = EBS_ERROR;
        }
    }
};

char runBrakeCheck() {
    getNextState();
    currState = nextState;
    if (currState == EBS_ERROR) {
        return -1;
    } else if (currState == READY) {
        return 1;
    } else {
        return 0;
    }
}

TickType_t getTime() {
    return xTaskGetTickCount();
}


static bool checkHydraulicPressure(float BP3_min, float BP3_max, float BP4_min, float BP4_max) {
    // front, BP3
    uint16_t brakePressureFront_PSI = (uint16_t)cmr_sensorListGetValue(&sensorList, SENSOR_CH_BPRES_PSI);

    // back, BP4
    cmr_canVSMSensors_t *vsmSensors = getPayload(CANRX_VSM_SENSORS);
    uint16_t brakePressureBack_PSI = vsmSensors->brakePressureRear_PSI;

    // convert to PSI
    BP3_min *= 14.504;
    BP3_max *= 14.504;
    BP4_min *= 14.504;
    BP4_max *= 14.504;

    // Is BP3 front wheel? VSM only checks that it's above 45 bar but diagram says 90 bar
    return BP3_min <= brakePressureFront_PSI && brakePressureFront_PSI <= BP3_max 
        && BP4_min <= brakePressureBack_PSI && brakePressureBack_PSI <= BP4_max;
}


static bool checkTSActivated(){
    return getEAB();
}