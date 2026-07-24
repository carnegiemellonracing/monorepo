//Based on "DSF - Startup Control Check Flowchat"
#include <stdbool.h>
#include <gpio.h>

typedef enum {
    IDLE,
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
    EBS_ERROR
} brakecheck_state;

brakecheck_state currState = IDLE;
brakecheck_state nextState = IDLE;
TickType_t lastWakeTime;

static bool checkDSMS();

TickType_t getTime();

static bool checkEBSPressure();

static bool checkHydraulicPressure(float BP3_min, float BP3_max, float BP4_min, float BP4_max);

static void toggleMB(bool MB1_state, bool MB2_state);

static bool checkShutdownCircuit();

static bool checkTSActivated();

#define EBS_PRESSURE_BAR_MIN 8.0
#define EBS_PRESSURE_BAR_MAX 10.0


void getNextState() {
    switch (currState) {
        case IDLE: {
            if (checkDSMS()){ //TODO: implement checkDSMS, if DSMS closed begin
                nextState = BEGIN;
            } else {
                nextState = IDLE;
            }
            break;
        }
        case BEGIN: {
            lastWakeTime = getTime(); //to begin watchdog (but doesnt exist rn)
            nextState = SHUTDOWN_CIRCUIT_WAIT; //Maybe we do not need to check shutdown circuit here
            break;
        }
        case SHUTDOWN_CIRCUIT_WAIT: {
            if (checkShutdownCircuit()) { //If circuit closed then go ahead to time_check
                nextState = TIME_CHECK;
            } else {
                nextState = SHUTDOWN_CIRCUIT_WAIT;
            }
            break;
        }
        case TIME_CHECK: {
            if (getTime() - lastWakeTime < 50) {
                nextState = EBS_PRESSURE_CHECK;
            } else {
                nextState = EBS_ERROR;
            }
            break;
        }
        case EBS_PRESSURE_CHECK: { //checks BP1, BP2
            if (checkEBSPressure()) { //if pressure is fine go to next check
                nextState = HYDRAULIC_PRESSURE_CHECK_1;
            } else {
                nextState = EBS_ERROR;
            }
            break;
        }
        case HYDRAULIC_PRESSURE_CHECK_1: {
            if (checkHydraulicPressure(60, 90, 30, 45)) { //check if pressure within limits 
                nextState = TSAB_WAIT;
            } else {
                nextState = EBS_ERROR;
            }
            break;
        }
        case TSAB_WAIT: {
            if (checkTSActivated()) { //checks is TS activation button pushed
                nextState = MB_VALVES_SWITCH_ON_OFF;
            } else {
                nextState = TSAB_WAIT;
            }
            break;
        }

        case MB_VALVES_SWITCH_ON_OFF:{
            toggleMB(1, 0); //Turn MB1 on, turn MB2 off //MB1, MB2 like a solenoid that control the brakes //TODO: does this needs to be blocking?
            nextState = HYDRAULIC_PRESSURE_CHECK_2;
            break;
        }
        case HYDRAULIC_PRESSURE_CHECK_2:{
            if (checkHydraulicPressure(90, 10000, 0, 1)){ //10000 just a big number, if pressure within bounds then good
               nextState = MB_VALVES_SWITCH_OFF_ON; 
            } else {
                nextState = EBS_ERROR;
            }
            break;
        }
        case MB_VALVES_SWITCH_OFF_ON:{
            toggleMB(false, true); //Turn MB1 off, turn MB2 on
            nextState = HYDRAULIC_PRESSURE_CHECK_3;
            break;
        }
        case HYDRAULIC_PRESSURE_CHECK_3:{
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
            break;
        }
        default: {
            nextState = EBS_ERROR;
        }
    }
};

TickType_t getTime() {
    return xTaskGetTickCount();
}

/**
 * @brief checks ebs brake pressure between min and max bars in EBS_PRESSURE_CHECK state
 */
static bool checkEBSPressure(){
    cmr_canDVPressureReadings_t* pressureReading = (cmr_canDVPressureReadings_t*) getPayload(CANRX_AS_PRESSURE_READING);
   bool brakes_good = pressureReading->ebsPressure_1_deci_bar > EBS_PRESSURE_BAR_MIN &&  
            pressureReading->ebsPressure_2_deci_bar > EBS_PRESSURE_BAR_MIN &&
            pressureReading->ebsPressure_1_deci_bar < EBS_PRESSURE_BAR_MAX &&  
            pressureReading->ebsPressure_2_deci_bar < EBS_PRESSURE_BAR_MAX;
    return brakes_good;
}

static bool checkTSActivated(){
    return getEAB();
}