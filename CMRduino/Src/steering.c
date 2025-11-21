/**
 * @file motorControl.c
 * @brief basic pdController implementation.
 *
 * @author Vikram Mani and Ayush Garg
 */

#include <CMR/can_types.h>
#include <CMR/can_ids.h>
#include <stm32f4xx_hal.h> //Hal interface

#include <CMR/tasks.h> // Task interface
//#include <CMR/rcc.h> //

#include "steering.h"

#include "can.h"   // Board-specific CAN interface
#include "math.h"   // Board-specific CAN interface

#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

/** @brief pdControl task. */
static cmr_task_t maxonControlTask;

/** @brief pdControl period (milliseconds). */
static const TickType_t maxonControlPeriod_ms = 10;

/** @brief pdControl priority. */
static const uint32_t maxonControlPriority = 1;

//Keep track of last time pdControl task was run
static TickType_t previousTickCount = 0;

const float minValue = -1530.0f;
const float maxValue = 1530.0f;

static bool inspectionClockStarted = false;
static TickType_t startTime = 0;

const int32_t centerPosition = 1000;
bool startMission = false;

//forward Declerations
float computeControlAction(float targetPosition, float currPosition);
float computeInspectionTarget();
float clampSteeringADCVal(float adcVal);
int rpmToERPM(int rpm);
int32_t canGetSteeringADC();

/**
 * @brief Task for compute control actions
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */

static void maxonControl(void *pvParameters) {
    (void) pvParameters;

    TickType_t lastWakeTime = xTaskGetTickCount();
    float targetPosition, currPosition;

    while(1){
        // if (canGetAMIMission() == MISSION_INSPECTION && canGetCurrentState() == CMR_CAN_AS_DRIVING){
        // 	targetPosition = computeInspectionTarget();
        // }
        // else{
        //     targetPosition = (float) canGetSteeringPosition();
        // }
        cmr_canCubeMarsData_t *data = (cmr_canCubeMarsData_t*)getPayload(CANRX_CUBEMARS_DATA);
        int16_t position = data->position_deg;
        if(!startMission) {
            if(position == centerPosition) {
                startMission = true;
                cmr_canCubeMarsSetOrigin_t origin = {
                    .origin = 1
                };
                canExtendedTX(CMR_CANID_EXTENDED_CUBEMARS_SET_ORIGIN_HERE, &origin, sizeof(origin), 100);
            } else {
                targetPosition = centerPosition;
            }
        } else {
            targetPosition = (int32_t)computeInspectionTarget();
        }
        currPosition = position;
        // currPosition = clampSteeringADCVal(currPosition);
        // targetPosition = clampSteeringADCVal(targetPosition);


        sendCubeMarsVelocity(rpmToERPM((int)computeControlAction(targetPosition, currPosition)));
        previousTickCount = lastWakeTime;
        vTaskDelayUntil(&lastWakeTime, maxonControlPeriod_ms);
    }
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


float computeInspectionTarget(){

	static const float amplitude = (maxValue - minValue)/2.0f;
	static const float average = (maxValue + minValue)/2.0f;
	static const float frequency_Hz = 1.0f / 3.0f;
	static const TickType_t swivelPeriod_ms = 30000;
	float targetPosition;

    if(!inspectionClockStarted){
        startTime = xTaskGetTickCount();
        inspectionClockStarted = true;
        return 3460.0f;
    }

    TickType_t currentTime_ms = (xTaskGetTickCount() - startTime);
            
    if (currentTime_ms < swivelPeriod_ms){
        targetPosition = amplitude*sinf((2.0f*(M_PI)*(frequency_Hz)/1000.0f) * ((float) currentTime_ms)) + average;
        uint8_t payLoad = 0;
        canTX(0x5EF, &payLoad, 1, 20);
        return targetPosition;
    }
    else{
      uint8_t payLoad = 1;
      canTX(0x5EF, &payLoad, 1, 20);
      return 3460.0f;
    }

}

int32_t canGetSteeringPos() {
    return 1000;
}

int rpmToERPM(int rpm) {
    return rpm * 21;
}

float clampSteeringADCVal(float adcVal){
    if(adcVal < .5f * minValue) adcVal = adcVal + 4096.0f;
    adcVal = CLAMP(adcVal, minValue, maxValue);
    return adcVal;
}

/**
 * @brief Initializes the dpControl task
 */
void maxonControlInit(){

    cmr_canCubeMarsData_t *data = (cmr_canCubeMarsData_t*)getPayload(CANRX_CUBEMARS_DATA);
    int16_t position = data->position_deg;
    if(position != centerPosition) {
        startMission = false;
    } else {
        startMission = true;
    }

    cmr_taskInit(
        &maxonControlTask,
        "maxonControl",
        maxonControlPriority,
        maxonControl,
        NULL);
}