/**
 * @file motors.c
 * @brief AMK quad-inverter interface.
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

/** @brief DAQ test type and HAL rand init **/
cmr_canDAQTest_t daqTest;

/** @brief Vehicle gear. */
static cmr_canGear_t gear = CMR_CAN_GEAR_SLOW;

/** @brief Setpoints for each inverter
 *  @note Indexed by motorLocation_t
 */
static cmr_canAMKSetpoints_t motorSetpoints[MOTOR_LEN] = {
    [MOTOR_FL] = {
        .velocity_rpm = 0,
        .torqueLimPos = 0,
        .torqueLimNeg = 0
    },
    [MOTOR_FR] = {
        .velocity_rpm = 0,
        .torqueLimPos = 0,
        .torqueLimNeg = 0
    },
    [MOTOR_RL] = {
        .velocity_rpm = 0,
        .torqueLimPos = 0,
        .torqueLimNeg = 0
    },
    [MOTOR_RR] = {
        .velocity_rpm = 0,
        .torqueLimPos = 0,
        .torqueLimNeg = 0
    },
};

cmr_canDAQTest_t getDAQTest() {
    return daqTest;
}

// ------------------------------------------------------------------------------------------------
// Private functions

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

        switch (heartbeatVSM->state) {
            // Drive the vehicle in RTD
            case CMR_CAN_RTD: {
            	mcCtrlOn();
            	// fansOn();
            	pumpsOn();

                /** Drive Enable Initialized in can.c.  */

                // Blip (100ms) control message to zero torque/speed after transitioning
                // from HV_EN to RTD to make sure inverters receive clean enable
                const bool blank_command = (lastHvenTime + 100 > xTaskGetTickCount());
                if (blank_command) {
					for (size_t i = 0; i < MOTOR_LEN; i++) {
						motorSetpoints[i].velocity_rpm = 0;
						motorSetpoints[i].torqueLimPos = 0;
						motorSetpoints[i].torqueLimNeg = 0;
					}
				}

//                for (size_t i = 0; i < MOTOR_LEN; i++) {
//                    motorSetpoints[i].velocity_rpm = 300;
//                    motorSetpoints[i].torqueLimPos = 40;
//                    motorSetpoints[i].torqueLimNeg = -40;
//                }

                uint32_t au32_initial_ticks = DWT->CYCCNT;

                TickType_t startTime = xTaskGetTickCount();
                //taskENTER_CRITICAL(); /** @todo verify if this critical region is necessary */

                runControls(gear,
                		    dataFSM    -> torqueRequested,
                            dataFSM    -> brakePedalPosition,
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



                //canTX(CMR_CAN_BUS_VEH, 0x7F9, &microsecs, 4, 5);


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
                for (size_t i = 0; i < MOTOR_LEN; i++) {
                    motorSetpoints[i].velocity_rpm       = 0;
                    motorSetpoints[i].torqueLimPos = 0;
                    motorSetpoints[i].torqueLimNeg = 0;
                }

                // set status so DIM can see
                setControlsStatus(reqDIM->requestedGear);
                // Store last timestamp
                lastHvenTime = xTaskGetTickCount();
                break;
            }

            // Also reset errors in GLV_ON
            case CMR_CAN_GLV_ON: {
            	mcCtrlOff();

                if (vsm->internalState == CMR_CAN_VSM_STATE_INVERTER_EN) {
                    mcCtrlOn();
                } else

            	// fansOff();
            	pumpsOff();

                for (size_t i = 0; i < MOTOR_LEN; i++) {
                    motorSetpoints[i].velocity_rpm       = 0;
                    motorSetpoints[i].torqueLimPos = 0;
                    motorSetpoints[i].torqueLimNeg = 0;
                }
                // set status so DIM can see
                setControlsStatus(reqDIM->requestedGear);
                break;
            }

            // In all other states, disable inverters and do not reset errors
            default: {
                pumpsOn();
                pumpsOff();
                mcCtrlOff();
                set_optimal_control_with_regen(128, 10000, 10000);

                for (size_t i = 0; i < MOTOR_LEN; i++) {
                    motorSetpoints[i].velocity_rpm       = 0;
                    motorSetpoints[i].torqueLimPos = 0;
                    motorSetpoints[i].torqueLimNeg = 0;
                }
                break;
            }
        }

        // Update gear in transition from HV_EN to RTD
        if (prevState == CMR_CAN_HV_EN && heartbeatVSM->state == CMR_CAN_RTD) {
            gear = reqDIM->requestedGear;
            resetRetroactiveLimitFilters();
            initControls();

            // Generate new test ID
            daqTest = (rand() % 0x7Fu) & 0x7Fu;

            // Send message to start test on DAQ CAN
            daqTest = daqTest | 0x80; // Set MSB to one
            // canTX(
            //   CMR_CAN_BUS_DAQ, CMR_CANID_TEST_ID, &daqTest, sizeof(daqTest), can10Hz_period_ms
            // );
        }

        if (prevState == CMR_CAN_RTD && heartbeatVSM->state == CMR_CAN_HV_EN) {
            // Send message to stop test on DAQ CAN
            daqTest = daqTest & 0x7F; // Set MSB to zero
            // canTX(
            //   CMR_CAN_BUS_DAQ, CMR_CANID_TEST_ID, &daqTest, sizeof(daqTest), can10Hz_period_ms
            // );
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
    motorSetpoints[motor].torqueLimPos = convertNmToAMKTorque(torqueLimPos_Nm);
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
    motorSetpoints[motor].torqueLimNeg = convertNmToAMKTorque(torqueLimNeg_Nm);
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

    /** Check feasibility with field weakening (page 38 manual) */

    torqueLimPos_Nm = fmaxf(torqueLimPos_Nm, 0.0f); // ensures torqueLimPos_Nm >= 0
    torqueLimNeg_Nm = fminf(torqueLimNeg_Nm, 0.0f); // ensures torqueLimNeg_Nm <= 0

    motorSetpoints[motor].torqueLimPos = convertNmToAMKTorque(torqueLimPos_Nm);
    motorSetpoints[motor].torqueLimNeg = convertNmToAMKTorque(torqueLimNeg_Nm);
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
    motorSetpoints[motor].torque = torque;
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

/**
 * @brief Gets a read-only pointer to specified DTI inverter setpoints.
 *
 * @param motor Which motor to get setpoints for.
 *
 * @return Read-only pointer to requested setpoints.
 */
const cmr_canDTISetpoints_t *getDTISetpoints(motorLocation_t motor) {
    if (motor >= MOTOR_LEN) {
        return NULL;
    }

    return (const cmr_canDTISetpoints_t *) &(motorSetpoints[motor]);
}