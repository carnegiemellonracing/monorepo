/**
 * @file motors.c
 * @brief DTI quad-inverter interface.
 *
 * @author Carnegie Mellon Racing
 */

// ------------------------------------------------------------------------------------------------
// Includes

#include "motors.h"         // Interface to implement
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <limits.h>
#include <stdlib.h>
#include <CMR/can_types.h>  // CMR CAN types
#include <CMR/config_screen_helper.h>
#include <CMR/fir_filter.h>
// #include "controls_23e.h"
#include "drs_controls.h"
#include "servo.h"
#include "can.h"
#include "daq.h"
#include "safety_filter.h"
#include "mc_power.h"
#include "pumps.h"
#include "fans.h"
#include "constants.h"
#include "controls.h"
#include "motors_helper.h"

// ------------------------------------------------------------------------------------------------
// Constants

/** @brief Motors command 200 Hz priority */
static const uint32_t motorsCommand_priority = 6;

/** @brief Motors command 200 Hz period (milliseconds) */
static const TickType_t motorsCommand_period_ms = 5;

/** @brief DAQ CAN Test period (milliseconds) */
static const TickType_t can10Hz_period_ms = 100;

// ------------------------------------------------------------------------------------------------
// Globals

/** @brief Motors command 200 Hz task. */
static cmr_task_t motorsCommand_task;

static cmr_task_t motorsTest_task;

/** @brief DAQ test type and HAL rand init **/
cmr_canDAQTest_t daqTest;

/** @brief Vehicle gear. */
static cmr_canGear_t gear = CMR_CAN_GEAR_SLOW;

/** @brief Setpoints for each inverter
 *  @note Indexed by motorLocation_t
 *  @note It will statically be defined to all 0s
 */
static cmr_DTISetpoints_t motorSetpoints[MOTOR_LEN];

/** @brief Set points for each inverter in DTI format
 *  @note Indexed by motorLocation_t
 *  @note It will statically be defined to all 0s
 */
static cmr_DTI_RX_Message_t DTI_RXMessage[MOTOR_LEN];

#define MAX_CURRENT_DECI_AMPS 850                        

cmr_canDAQTest_t getDAQTest() {
    return daqTest;
}

/* Global Variable to Initiate/Disable Torque Mode*/ 
bool isTorqueMode = false;

// ------------------------------------------------------------------------------------------------
// Private functions

static void motorsTest (void *pvParameters) {

    TickType_t lastWakeTime = xTaskGetTickCount();
    volatile cmr_canHeartbeat_t *heartbeatVSM   = canVehicleGetPayload(CANRX_VEH_HEARTBEAT_VSM);
    volatile cmr_canVSMStatus_t *vsm            = canVehicleGetPayload(CANRX_VSM_STATUS);
    volatile cmr_canFSMData_t   *dataFSM        = canVehicleGetPayload(CANRX_VEH_DATA_FSM);


    while (1) {
        volatile cmr_canFSMData_t *dataFSM = canVehicleGetPayload(CANRX_VEH_DATA_FSM);
        uint8_t throttlePos = dataFSM->throttlePosition;
        uint16_t setCurrent = (uint16_t)(((float)throttlePos * (float)MAX_CURRENT_DECI_AMPS / (float)UINT8_MAX));
        // setCurrent = setCurrent << 8 | ((setCurrent >> 8) & 0xFF);
        //enables motors to drive
        uint8_t driveEnable = 1;
        setPowerLimit(true, MOTOR_FL, 20.25f);

        if (vsm->internalState == CMR_CAN_VSM_STATE_INVERTER_EN || heartbeatVSM->state == CMR_CAN_HV_EN) {
            setCurrent = 0;
            mcCtrlOn();
            sendDTIMessage(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_BROADCAST_SET_DRIVE_EN, &driveEnable, sizeof(driveEnable), can10Hz_period_ms);
            sendDTIMessage(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_BROADCAST_SET_CURRENT, &setCurrent, sizeof(setCurrent), can10Hz_period_ms);
        }
        else if(heartbeatVSM->state == CMR_CAN_RTD){
            mcCtrlOn();
            sendDTIMessage(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_BROADCAST_SET_DRIVE_EN, &driveEnable, sizeof(driveEnable), can10Hz_period_ms);
            sendDTIMessage(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_BROADCAST_SET_CURRENT, &setCurrent, sizeof(setCurrent), can10Hz_period_ms);
        }
        else {
            mcCtrlOff();
        }

        vTaskDelayUntil(&lastWakeTime, motorsCommand_period_ms);
    }
}

/**
 * @brief Send Blank Command To Inverters
 */
void sendBlankCommand() {
    for (size_t i = 0; i < MOTOR_LEN; i++) {
        motorSetpoints[i].velocity_rpm     = 0;
        motorSetpoints[i].torqueLimPos_mNm = 0;
        motorSetpoints[i].torqueLimNeg_mNm = 0;
    }
}

/**
 * @brief Task for setting motors command.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void motorsCommand (
    void *pvParameters
) {
    (void) pvParameters;    // Placate compiler.

    /** @brief DRS Mode */
    cmr_canDrsMode_t drsMode = CMR_CAN_DRSM_CLOSED;

    // initialize filters
    initRetroactiveLimitFilters();

    cmr_canState_t prevState = CMR_CAN_GLV_ON;

    /** @brief Timer for temporarily blanking vel/torque commands
     *         on transition to RTD. Without this, the inverter may have
     *         non-zero torque/speed in the same message used to enable it,
     *         which would cause the inverter to refuse to enable.  */
    static TickType_t lastHvenTime = 0;

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        volatile cmr_canHeartbeat_t      *heartbeatVSM = canVehicleGetPayload(CANRX_VEH_HEARTBEAT_VSM);
        volatile cmr_canDIMRequest_t     *reqDIM       = canVehicleGetPayload(CANRX_VEH_REQUEST_DIM);
        volatile cmr_canFSMData_t        *dataFSM      = canVehicleGetPayload(CANRX_VEH_DATA_FSM);
        volatile cmr_canFSMSWAngle_t     *swangleFSM   = canVehicleGetPayload(CANRX_VEH_SWANGLE_FSM);
        volatile cmr_canHVCPackVoltage_t *voltageHVC   = canVehicleGetPayload(CANRX_VEH_VOLTAGE_HVC);
        volatile cmr_canHVCPackCurrent_t *currentHVC   = canVehicleGetPayload(CANRX_VEH_CURRENT_HVC);
        volatile cmr_canVSMStatus_t      *vsm          = canVehicleGetPayload(CANRX_VSM_STATUS);

        //transmit Coulombs using HVI sense
        integrateCurrent();

//         update DRS mode
        drsMode = reqDIM->requestedDrsMode;

        int32_t steeringWheelAngle_millideg = (swangleFSM->steeringWheelAngle_millideg_FL + swangleFSM->steeringWheelAngle_millideg_FR) / 2;
        // runDrsControls(reqDIM->requestedGear,
        //                 drsMode,
        //                 dataFSM    -> throttlePosition,
        //                 dataFSM    -> brakePressureFront_PSI
        //                 );
        // mcCtrlOn();

        switch (heartbeatVSM->state) {
            // Drive the vehicle in RTD
            case CMR_CAN_RTD: {
            	mcCtrlOn();
            	// fansOn();
            	pumpsOn();

                /** Drive Enable Initialized in can.c.  */

                // Blip (100ms) control message to zero torque/speed after transitioning
                // from HV_EN to RTD to make sure inverters receive clean enable

                // const bool blank_command = (lastHvenTime + 10000 > xTaskGetTickCount());
                const bool blank_command = (lastHvenTime + 100 > xTaskGetTickCount());
                if (blank_command) {
                    sendBlankCommand();
				}
                // else {
                //     int16_t set_current_fl = 40 << 8;
                //     int16_t set_current_fr = 40 << 8;
                //     int16_t set_current_rl = 40 << 8;
                //     int16_t set_current_rr = 40 << 8;

                //     //enables motors to drive
                //     uint8_t driveEnable = 1;
                //     sendDTIMessage(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_BROADCAST_SET_DRIVE_EN, &driveEnable, sizeof(driveEnable), can10Hz_period_ms);

                //     sendDTIMessage(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_FL_SET_CURRENT, &set_current_fl, sizeof(set_current_fl), can10Hz_period_ms);
                //     sendDTIMessage(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_FR_SET_CURRENT, &set_current_fr, sizeof(set_current_fr), can10Hz_period_ms);
                //     sendDTIMessage(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_RL_SET_CURRENT, &set_current_rl, sizeof(set_current_rl), can10Hz_period_ms);
                //     sendDTIMessage(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_RR_SET_CURRENT, &set_current_rr, sizeof(set_current_rr), can10Hz_period_ms);
                
                // }
                
                uint32_t au32_initial_ticks = DWT->CYCCNT;

                TickType_t startTime = xTaskGetTickCount();
                //taskENTER_CRITICAL(); /** @todo verify if this critical region is necessary */

                runControls(gear,
                		    dataFSM    -> torqueRequested,
                            dataFSM    -> brakePedalPosition_percent,
                            dataFSM    -> brakePressureFront_PSI,
                            swangleFSM->steeringWheelAngle_millideg_FL,
                            swangleFSM->steeringWheelAngle_millideg_FR,
                            voltageHVC -> hvVoltage_mV,
                            currentHVC -> instantCurrent_mA,
                            blank_command);
                //taskEXIT_CRITICAL();

                TickType_t endTime = xTaskGetTickCount();

                uint32_t total_ticks = DWT->CYCCNT - au32_initial_ticks;
                uint32_t microsecs = total_ticks*1000000/HAL_RCC_GetHCLKFreq();

                // Throttle pos is used instead of torque requested bc torque
                // requested is always 0 unless in RTD (this allows drivers to
                // test DRS implementation without being in RTD)

                // set status so DIM can see
                setControlsStatus(gear);

                break;
            }

            // Reset errors in HV_EN
            case CMR_CAN_HV_EN: {
            	mcCtrlOn();
            	// fansOn();
            	pumpsOn();
                sendBlankCommand();

                // set status so DIM can see
                setControlsStatus(reqDIM->requestedGear);
                // Store last timestamp
                lastHvenTime = xTaskGetTickCount();
                break;
            }

            // Also reset errors in GLV_ON
            case CMR_CAN_GLV_ON: {
                pumpsOff();
            	mcCtrlOff();

                if (vsm->internalState == CMR_CAN_VSM_STATE_INVERTER_EN) {
                    mcCtrlOn();
                }

            	// fansOff();
            	// pumpsOff();
                sendBlankCommand();

                // set status so DIM can see
                setControlsStatus(reqDIM->requestedGear);
                break;
            }

            // In all other states, disable inverters and do not reset errors
            default: {
                pumpsOff();
                mcCtrlOff();
                sendBlankCommand();
                break;
            }
        }

        // Update gear in transition from HV_EN to RTD
        if (prevState == CMR_CAN_HV_EN && heartbeatVSM->state == CMR_CAN_RTD) {
            gear = reqDIM->requestedGear;
            resetRetroactiveLimitFilters();
            initControls();
        }

        prevState = heartbeatVSM->state;
        vTaskDelayUntil(&lastWakeTime, motorsCommand_period_ms);
    }
}


// ------------------------------------------------------------------------------------------------
// Public functions

/**
 * @brief Initializes motor interface.
 */
void motorsInit (
    void
) {
    initControls();

    // Task creation.
    cmr_taskInit(
        &motorsCommand_task,
        "motorsCommand",
        motorsCommand_priority,
        motorsCommand,
        NULL
    );
    // cmr_taskInit(
    //     &motorsTest_task,
    //     "motorsTest",
    //     motorsCommand_priority,
    //     motorsTest,
    //     NULL
    // );
}

/**
 * @brief Sets positive torque limit for a motor.
 *
 * @param motor Which motor to set torque limit for.
 * @param torqueLimPos_Nm Desired positive torque limit.
 */
void setTorqueLimPos (
    motorLocation_t motor,
    float torqueLimPos_Nm
) {
    if (motor >= MOTOR_LEN) {
        return;
    }

    torqueLimPos_Nm = fmaxf(torqueLimPos_Nm, 0.0f);
    motorSetpoints[motor].torqueLimPos_mNm = torqueLimPos_Nm;
}

/**
 * @brief Sets negative torque limit for a motor.
 *
 * @param motor Which motor to set torque limit for.
 * @param torqueLimNeg_Nm Desired negative torque limit.
 */
void setTorqueLimNeg (
    motorLocation_t motor,
    float torqueLimNeg_Nm
) {
    if (motor >= MOTOR_LEN) {
        return;
    }

    torqueLimNeg_Nm = fminf(torqueLimNeg_Nm, 0.0f);
    motorSetpoints[motor].torqueLimNeg_mNm = torqueLimNeg_Nm;
}

/**
 * @brief Sets both positive and negative torque limits for all motors.
 *
 * @param torqueLimPos_Nm Desired positive torque limit.
 * @param torqueLimNeg_Nm Desired negative torque limit.
 */
void setTorqueLimsAllProtected (
    float torqueLimPos_Nm,
    float torqueLimNeg_Nm
) {
    setTorqueLimsAllDistProtected(torqueLimPos_Nm, torqueLimNeg_Nm, NULL, NULL);
}

/**
 * @brief Sets both positive and negative torque limits for all motors with over/undervolt protection.
 * @deprecated Please use setTorqueLimsProtected instead if possible
 * @note This is a wrapper of setTorqueLimsProtected
 *
 * @param torqueLimPos_Nm Max torque: upper-bounds the torque SF sends to the motors. MUST BE NON-NEGATIVE!
 * @param torqueLimNeg_Nm Min torque: lower-bounds the torque SF sends to the motors. MUST BE NON-POSITIVE!
 * @note The SF may decide to send any torque within the limits specified by torqueLimPos_Nm and torqueLimNeg_Nm.
 * @param distPos Coefficients that are multiplied onto torqueLimPos_Nm for each motor. NULL is treated as an even distribution. MUST BE NON-NEGATIVE!
 * @param distNeg Coefficients that are multiplied onto torqueLimNeg_Nm for each motor. NULL is treated as an even distribution. MUST BE NON-NEGATIVE!
 */
void setTorqueLimsAllDistProtected (
    float torqueLimPos_Nm,
    float torqueLimNeg_Nm,
    const cmr_loadDistribution_t *distPos,
    const cmr_loadDistribution_t *distNeg
) {
    const cmr_torqueDistributionNm_t torquesPos_Nm = {
        .fl = getLoadByIndex(distPos, MOTOR_FL) * torqueLimPos_Nm,
        .fr = getLoadByIndex(distPos, MOTOR_FR) * torqueLimPos_Nm,
        .rl = getLoadByIndex(distPos, MOTOR_RL) * torqueLimPos_Nm,
        .rr = getLoadByIndex(distPos, MOTOR_RR) * torqueLimPos_Nm
    };
    const cmr_torqueDistributionNm_t torquesNeg_Nm = {
        .fl = getLoadByIndex(distNeg, MOTOR_FL) * torqueLimNeg_Nm,
        .fr = getLoadByIndex(distNeg, MOTOR_FR) * torqueLimNeg_Nm,
        .rl = getLoadByIndex(distNeg, MOTOR_RL) * torqueLimNeg_Nm,
        .rr = getLoadByIndex(distNeg, MOTOR_RR) * torqueLimNeg_Nm
    };
    setTorqueLimsProtected(&torquesPos_Nm, &torquesNeg_Nm);
}

/**
 * @brief Sets both positive and negative torque limits for a motor.
 *
 * @param motor Which motor to set torque limits for.
 * @param torqueLimPos_Nm Desired positive torque limit.
 * @param torqueLimNeg_Nm Desired negative torque limit.
 */
void setTorqueLimsUnprotected (
    motorLocation_t motor,
    float torqueLimPos_Nm,
    float torqueLimNeg_Nm
) {
    if (motor >= MOTOR_LEN) {
        return;
    }

    torqueLimPos_Nm = fmaxf(torqueLimPos_Nm, 0.0f); // ensures torqueLimPos_Nm >= 0
    torqueLimNeg_Nm = fminf(torqueLimNeg_Nm, 0.0f); // ensures torqueLimNeg_Nm <= 0

    motorSetpoints[motor].torqueLimPos_mNm = torqueLimPos_Nm * 1000.0f;
    motorSetpoints[motor].torqueLimNeg_mNm = torqueLimNeg_Nm * 1000.0f;
}

/**
 * @brief Sets velocity setpoint for a motor.
 *
 * @param motor Which motor to set velocity for.
 * @param velocity_rpm Desired velocity.
 */
void setVelocityInt16 (
    motorLocation_t motor,
    int16_t velocity_rpm
) {
    if (motor >= MOTOR_LEN) {
        return;
    }

    if (velocity_rpm > maxSpeed_rpm) {
        velocity_rpm = maxSpeed_rpm;
    }

    if (velocity_rpm < -maxSpeed_rpm) {
        velocity_rpm = -maxSpeed_rpm;
    }

    motorSetpoints[motor].velocity_rpm = velocity_rpm;
}

/**
 * @brief Sets velocity setpoint for a motor.
 *
 * @param motor Which motor to set velocity for.
 * @param velocity_rpm Desired velocity.
 */
void setVelocityFloat (
    motorLocation_t motor,
    float velocity_rpm
) {
    velocity_rpm = fminf(velocity_rpm, (float)INT16_MAX);
    velocity_rpm = fmaxf(velocity_rpm, (float)INT16_MIN);
    setVelocityInt16(motor, (int16_t)velocity_rpm);
}

/**
 * @brief Sets velocity setpoint for a motor.
 *
 * @param motor Which motor to set velocity for.
 * @param velocity_rpm Desired velocity.
 */
void setVelocityInt16All (
    int16_t velocity_rpm
) {
    for (size_t motor = 0; motor < MOTOR_LEN; motor++) {
        setVelocityInt16(motor, velocity_rpm);
    }
}

/**
 * @brief Sets velocity setpoint for a motor.
 *
 * @param motor Which motor to set velocity for.
 * @param velocity_rpm Desired velocity.
 */
void setVelocityFloatAll (
    float velocity_rpm
) {
    for (size_t motor = 0; motor < MOTOR_LEN; motor++) {
        setVelocityFloat(motor, velocity_rpm);
    }
}

/**
 * @brief Sets torque setpoint for a motor.
 *
 * @param motor Which motor to set torque for.
 * @param torque Desired torque.
 */
void setTorque(
    motorLocation_t motor,
    float torque
){
    motorSetpoints[motor].torque_mNm = torque;
}

/**
 * @brief Initiates Torque Mode.
 */
void initiateTorqueMode()
{
    isTorqueMode = true;
}

/**
 * @brief Disables Torque Mode.
 */
void disableTorqueMode()
{
    isTorqueMode = false;
}

/**
 * @brief Calculate the torque budget for power-aware traction and yaw rate control.
 *
 * @return The torque upper- and lower-limits for a motor, which applies to every motor.
 */
cmr_torque_limit_t getTorqueBudget() {
	return getPreemptiveTorqueLimits();
}

/* @brief Sets the power limit for all motors or a specific motor
 */
void setPowerLimit(bool all, motorLocation_t motor, float powerLimit_kw) {
    volatile cmr_canHVSense_t *HVISense = canTractiveGetPayload(CANRX_HVI_SENSE);
    // float hvVoltage_V = ((float) HVISense->packVoltage_cV) / 100.f;
    float hvVoltage_V = 500.0f;
    uint16_t current = (int)((10.0f*((float)powerLimit_kw*1000.0f))/hvVoltage_V); // send current in deciamps
    // current = current << 8 | ((current >> 8) & 0xFF); 
    if(all) {
        sendDTIMessage(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_BROADCAST_SET_MAX_CURRENT, &current, sizeof(current), motorsCommand_period_ms);
    } else {
        switch(motor){
            case MOTOR_FL:
                sendDTIMessage(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_FL_SET_MAX_CURRENT, &current, sizeof(current), motorsCommand_period_ms);
                break;
            case MOTOR_FR:
                sendDTIMessage(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_FR_SET_MAX_CURRENT, &current, sizeof(current), motorsCommand_period_ms);
                break;
            case MOTOR_RL:
                sendDTIMessage(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_RL_SET_MAX_CURRENT, &current, sizeof(current), motorsCommand_period_ms);
                break;
            case MOTOR_RR:
                sendDTIMessage(CMR_CAN_BUS_TRAC, CMR_CANID_DTI_RR_SET_MAX_CURRENT, &current, sizeof(current), motorsCommand_period_ms);
                break;
        }
    }
}

/**
 * @brief Gets a read-only pointer to specified DTI inverter setpoints.
 *
 * @param motor Which motor to get setpoints for.
 *
 * @return Read-only pointer to requested setpoints.
 */
const cmr_DTI_RX_Message_t* getDTISetpoints(motorLocation_t motor) {
    if (motor >= MOTOR_LEN) {
        return NULL;
    }

    const cmr_DTISetpoints_t *src = (const cmr_DTISetpoints_t *) &(motorSetpoints[motor]);
    cmr_DTI_RX_Message_t *dst = &DTI_RXMessage[motor];
    dst->velocity_erpm     = (int16_t)src->velocity_rpm * pole_pairs;

    // Change to AC current lims
    dst->torqueLimPos_dA = (int16_t)torqueToCurrent (src->torqueLimPos_mNm);
    dst->torqueLimNeg_dA = (int16_t)torqueToCurrent (src->torqueLimNeg_mNm);

    dst->ACCurrent_dA = torqueToCurrent(src->torque_mNm);

    return (const cmr_DTI_RX_Message_t *) dst;
}