/**
 * @file steering.c
 * @brief CubeMars steering motor control.
 *
 * @author Vikram Mani and Ayush Garg
 */

#include <CMR/can_types.h>
#include <CMR/can_ids.h>
#include <CMR/utils.h>


#include <CMR/tasks.h> // Task interface
//#include <CMR/rcc.h> //

#include "steering.h"

#include "adc.h"
#include "can.h"   // Board-specific CAN interface
#include "math.h"   // Board-specific CAN interface

#define MAX_CURRENT_MA 3000

/** @todo Cordinate with DV for these values
 *  @note This is average of left and right
 *  @todo This assumes symetricness which is not true
*/
#define MIN_STEERING_CENTI_DEG -2771.0f
#define MAX_STEERING_CENTI_DEG 2771.0f

#define INSPECTION_PERIOD_MS   5000.0f

/** @brief cubeMarsControl task. */
static cmr_task_t cubeMarsControlTask;

/** @brief Steering task period (milliseconds). */
static const TickType_t steeringPeriod_ms = 10;

/** @brief Steering task priority. */
static const uint32_t steeringPriority = 3;

static bool inspectionActive = false;

// Forward declarations
float computeControlAction(float targetPosition, float currPosition);

// Computes the swanlge in centidegrees from CAN
static float getSteeringAngle() {
    cmr_canFSMSWAngle_t *swangleData = canVehicleGetPayload(CANRX_VEH_DATA_FSM);
    // Weird due to overflow possibliliy
    int16_t swangle_centideg = (swangleData->steeringWheelAngle_millideg_FL / 10 
                                + swangleData->steeringWheelAngle_millideg_FR / 10) / 2;
    return CLAMP(MIN_STEERING_CENTI_DEG, (float) swangle_centideg, MAX_STEERING_CENTI_DEG);
}

// Control Action Computation
// Outputs in milli amps
float computeControlAction(float targetPosition_centi_deg, float currPosition_centi_deg)
{
    //function Statics used for derivative
    static float prevPosition_centi_deg = 0;
    static TickType_t prevTime_ms = 0;
    static float filteredPosDerivative = 0.0f;

    //Constants for running PD controllers
    static float K_f = 0.0f; //feed forwards
    static float K_p = -2.5f;
    static float K_d = 0.0f; //derivative term
    const float filterAlpha = 0.1f;  //first order IIR filter alpha. Larger alpha is more filtering. Ranges 0-1
    static const float errorThresholdKF = 15.0f;
    static const bool constantsFromCAN = false;
    float mult;

    TickType_t currTime_ms = xTaskGetTickCount();
    float currError = targetPosition_centi_deg - currPosition_centi_deg;
    float posDerivative = currPosition_centi_deg - prevPosition_centi_deg;
    //first order IIR filter
    filteredPosDerivative = filterAlpha*posDerivative + (1.0f - filterAlpha) * filteredPosDerivative;

    // if(constantsFromCAN){
    // 	K_p = -1.0f*canGetK_p();
    // 	K_d = canGetK_d();
    // 	K_f = canGetK_f();
    // }

    //ensures that feedforward is in the direction to reduce error
    if (fabs(currError) <= errorThresholdKF) mult = 0.0f;
    else if (currError < 0) mult = 1.0f;
    else mult = -1.0f;

    float controlAction_mA = mult*K_f + K_p*currError + K_d*posDerivative;
    prevPosition_centi_deg = currPosition_centi_deg;
    prevTime_ms = currTime_ms;
    controlAction_mA = CLAMP(-MAX_CURRENT_MA, controlAction_mA, MAX_CURRENT_MA);
    return controlAction_mA;
}

// computes the inspection mission target
static float computeInspectionTarget() {
    static float t = 0.0f;

    if (!inspectionActive) {
        t = 0.0f;
        inspectionActive = true;
    }

    const float dt = (float)steeringPeriod_ms / 1000.0f;
    const float double_amplitude_centi_deg = MIN_STEERING_CENTI_DEG + MAX_STEERING_CENTI_DEG;
    // Note not perfect sinusoid (center is not neccessairly 0)
    float target = double_amplitude_centi_deg * sinf(2.0f * M_PI * t / INSPECTION_PERIOD_MS) + MIN_STEERING_CENTI_DEG;

    t += dt;
    return target;
}

/**
 *  - AS_DRIVING + DV_MISSION_INSPECTION: inspection sweep.
 *  - AS_DRIVING + any other DV mission: position-control motor to AS swangle.
 *  - else: freely spin motor.
 */
void runSteering() {
    volatile cmr_canHeartbeat_t                 *heartbeatVSM = canVehicleGetPayload(CANRX_VEH_HEARTBEAT_VSM);
    volatile cmr_canDIMRequest_t                *reqDIM       = canVehicleGetPayload(CANRX_VEH_REQUEST_DIM);
    volatile cmr_canAutonomousControlAction_t   *controlAction= canDAQGetPayload(CANRX_DAQ_AUTONOMOUS_ACTION);

    cmr_canState_t state = heartbeatVSM->state;
    cmr_canGear_t  gear  = reqDIM->requestedGear;

    // if (state != CMR_CAN_AS_DRIVING) {
    //     inspectionActive = false;
    //     // turn motor off
    //     int32_t current = 0;
    //     sendCubeMarsMessage(CMR_CAN_BUS_DAQ, CMR_CANID_EXTENDED_CUBEMARS_SET_CURRENT, &current, sizeof(current), steeringPeriod_ms);
    //     return;
    // }

    float targetPosition_centi_deg = 0.0f;
    if (gear == CMR_CAN_GEAR_DV_MISSION_INSPECTION || true) {
        targetPosition_centi_deg = computeInspectionTarget();
    } else {
        inspectionActive = false;
        targetPosition_centi_deg = controlAction->steeringAngle_centi_deg;
    }
    targetPosition_centi_deg = CLAMP(MIN_STEERING_CENTI_DEG, targetPosition_centi_deg, MAX_STEERING_CENTI_DEG );

    int32_t current_output_mA = computeControlAction(targetPosition_centi_deg, getSteeringAngle());
    current_output_mA = CLAMP(-MAX_CURRENT_MA, current_output_mA, MAX_CURRENT_MA);
    sendCubeMarsMessage(CMR_CAN_BUS_DAQ, CMR_CANID_EXTENDED_CUBEMARS_SET_CURRENT, &current_output_mA, sizeof(current_output_mA), steeringPeriod_ms);
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