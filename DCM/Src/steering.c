/**
 * @file steering.c
 * @brief CubeMars steering motor control.
 *
 * @author Vikram Mani and Ayush Garg
 */

#include <CMR/can_types.h>
#include <CMR/can_ids.h>
#include <stm32f4xx_hal.h> //Hal interface

#include <CMR/tasks.h> // Task interface
//#include <CMR/rcc.h> //

#include "steering.h"

#include "adc.h"
#include "can.h"   // Board-specific CAN interface
#include "math.h"   // Board-specific CAN interface

#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

static const int32_t MAX_CURRENT_MA = 60000;

/** @brief cubeMarsControl task. */
static cmr_task_t cubeMarsControlTask;

/** @brief Steering task period (milliseconds). */
static const TickType_t steeringPeriod_ms = 10;

/** @brief Steering task priority. */
static const uint32_t steeringPriority = 1;

const float minValue = -1530.0f;
const float maxValue = 1530.0f;

static bool inspectionActive = false;

// Forward declarations
float computeControlAction(float targetPosition, float currPosition);
float clampSteeringADCVal(float adcVal);

// TODO: fix
static int32_t getSteeringADC() {
    return clampSteeringADCVal(
        adcRead(ADC_CHANNEL_0)
    );
}

// Control Action Computation
float computeControlAction(float targetPosition, float currPosition)
{
    //function Statics used for derivative
    static float prevPosition = 0;
    static TickType_t prevTime = 0;
    static float filteredPosDerivative = 0.0f;

    //Constants for running PD controllers
    static float K_p = -2.5f;
    static float K_f = 0.0f; //feed forwards
    static float K_d = 0.0f; //derivative term
    const float filterAlpha = 0.1f;  //first order IIR filter alpha. Larger alpha is more filtering. Ranges 0-1
    static const float errorThresholdKF = 15.0f;
    static const bool constantsFromCAN = false;
    float mult;

    TickType_t currTime = xTaskGetTickCount();
    float currError = targetPosition - currPosition;
    float posDerivative = currPosition - prevPosition;
    //first order IIR filter
    filteredPosDerivative = filterAlpha*posDerivative + (1.0f - filterAlpha) * filteredPosDerivative;

    if(constantsFromCAN){
    	K_p = -1.0f*canGetK_p();
    	K_d = canGetK_d();
    	K_f = canGetK_f();
    }

    //ensures that feedforward is in the direction to reduce error
    if (fabs(currError) <= errorThresholdKF) mult = 0.0f;
    else if (currError < 0) mult = 1.0f;
    else mult = -1.0f;

    float controlAction = mult*K_f + K_p*currError + K_d*posDerivative;
    prevPosition = currPosition;
    prevTime = currTime;

    //ADDED FOR DEBUGGING
    if(fabs(controlAction) > 10000.0f) {
    	return controlAction;
    }

    if(fabs(controlAction) < 10.0f) return 0.0f;
    return controlAction;
}

static float computeInspectionTarget() {
    static float t = 0.0f;

    if (!inspectionActive) {
        t = 0.0f;
        inspectionActive = true;
    }

    const float dt = (float)steeringPeriod_ms / 1000.0f;
    const float T = 5.0f;
    const float A_deg = 4.25f * 360.0f;  // 1530 deg

    float target = A_deg * sinf(2.0f * M_PI * t / T);

    t += dt;
    if (t >= T) t -= T;

    return target;
}

float clampSteeringADCVal(float adcVal){
    if(adcVal < .5f * minValue) adcVal = adcVal + 4096.0f;
    adcVal = CLAMP(adcVal, minValue, maxValue);
    return adcVal;
}

/**
 *  - AS_DRIVING + DV_MISSION_INSPECTION: inspection sweep.
 *  - AS_DRIVING + any other DV mission: position-control motor to AS swangle.
 *  - else: freely spin motor.
 */
void runSteering() {
    volatile cmr_canHeartbeat_t      *heartbeatVSM = canVehicleGetPayload(CANRX_VEH_HEARTBEAT_VSM);
    volatile cmr_canDIMRequest_t     *reqDIM       = canVehicleGetPayload(CANRX_VEH_REQUEST_DIM);
    volatile cmr_canComputeSwangle_t *steerPayload = canDAQGetPayload(CANRX_AS_SWANGLE);

    cmr_canState_t state = heartbeatVSM->state;
    cmr_canGear_t  gear  = reqDIM->requestedGear;

    if (state != CMR_CAN_AS_DRIVING) {
        inspectionActive = false;
        // turn motor off
        cmr_canCubeMarsDutyCycle_t duty = { .duty_cycle = 0 };
        canExtendedTX(CMR_CAN_BUS_DAQ, CMR_CANID_EXTENDED_CUBEMARS_SET_DUTY, &duty, sizeof(duty), 2);
        return;
    }

    static float targetPosition = 0.0f;
    if (gear == CMR_CAN_GEAR_DV_MISSION_INSPECTION) {
        targetPosition = computeInspectionTarget();
    } else {
        inspectionActive = false;
        targetPosition = steerPayload->swangle;
    }

    int32_t current_output = computeControlAction(targetPosition, getSteeringADC());
    current_output = CLAMP(current_output, -MAX_CURRENT_MA, MAX_CURRENT_MA);
    cmr_canCubeMarsCurrentLoop_t current = { 
        .current_mA = int32_to_big(current_output)
    };

    canExtendedTX(CMR_CAN_BUS_DAQ, CMR_CANID_EXTENDED_CUBEMARS_SET_CURRENT, &current, sizeof(current), 2);
}

static void steeringTask(void *pvParameters) {
    (void) pvParameters;

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        runSteering();
        vTaskDelayUntil(&lastWakeTime, steeringPeriod_ms);
    }
}

void steeringInit() {
    cmr_taskInit(&cubeMarsControlTask, "steeringTask", steeringPriority, steeringTask, NULL);
}