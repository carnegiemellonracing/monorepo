/**
 * @file motors.c
 * @brief DTI inverter interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef MOTORS_H
#define MOTORS_H

// ------------------------------------------------------------------------------------------------
// Includes

#include <stdint.h>
#include <CMR/can_ids.h>    // CMR CAN IDs
#include <CMR/can_types.h> 
#include "motors_helper.h"

// ------------------------------------------------------------------------------------------------
// Type definitions

/** @brief Torque limits. */
typedef struct {
    float max_torque;
    float min_torque;
} cmr_torque_limit_t;

/** @brief DTI controls-facing motor setpoints struct.*/
typedef struct {
    float velocity_rpm;           /**< @brief Velocity setpoint (RPM). */
    float torqueLimPos_mNm;       /**< @brief Positive torque limit. */
    float torqueLimNeg_mNm;       /**< @brief Negative torque limit. */
    float torque_mNm;             /**< @brief Torque to motor */  
} cmr_DTISetpoints_t;

/** @brief DTI Facing motor setpoints struct*/
typedef struct{
    int32_t velocity_erpm;          /**< @brief Velocity setpoint (ERPM). */
    int16_t torqueLimPos_dA;        /**< @brief Positive torque limit. */
    int16_t torqueLimNeg_dA;        /**< @brief Negative torque limit. */
    int16_t ACCurrent_dA;     /**< @brief Negative torque limit. */      
} cmr_DTI_RX_Message_t;

// ------------------------------------------------------------------------------------------------
// Public function declarations

void motorsInit();
void setTorqueLimPos(motorLocation_t motor, float torqueLimPos_Nm);
void setTorqueLimNeg(motorLocation_t motor, float torqueLimNeg_Nm);
void setTorque(motorLocation_t motor, float torque);
void initiateTorqueMode();
void disableTorqueMode();
void setTorqueLimsUnprotected (motorLocation_t motor, float torqueLimPos_Nm, float torqueLimNeg_Nm);
void setTorqueLimsAllProtected(float torqueLimPos_Nm, float torqueLimNeg_Nm);
void setTorqueLimsAllDistProtected(float torqueLimPos_Nm, float torqueLimNeg_Nm, const cmr_loadDistribution_t *distPos, const cmr_loadDistribution_t *distNeg);
void setTorques (motorLocation_t motor, float torque_Nm);
void setTorquesAll (float torque_Nm);
void setVelocityInt16(motorLocation_t motor, int16_t velocity_rpm);
void setVelocityFloat(motorLocation_t motor, float velocity_rpm);
void setVelocityInt16All(int16_t velocity_rpm);
void setVelocityFloatAll(float velocity_rpm);
cmr_torque_limit_t getTorqueBudget();
const cmr_DTI_RX_Message_t *getDTISetpoints(motorLocation_t motor);
cmr_canDAQTest_t getDAQTest();
void setPowerLimit(bool all, motorLocation_t motor, float powerLimit_kw);

#endif /* MOTORS_H */
