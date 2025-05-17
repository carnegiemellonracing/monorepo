/**
 * @file motors.c
 * @brief AMK quad-inverter interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef MOTORS_H
#define MOTORS_H

// ------------------------------------------------------------------------------------------------
// Includes

#include <stdint.h>
#include <CMR/can_ids.h>    // CMR CAN IDs
#include "motors_helper.h"

// ------------------------------------------------------------------------------------------------
// Type definitions

/** @brief Torque limits. */
typedef struct {
    float max_torque;
    float min_torque;
} cmr_torque_limit_t;

// ------------------------------------------------------------------------------------------------
// Public function declarations

void motorsInit();
void setTorqueLimPos(motorLocation_t motor, float torqueLimPos_Nm);
void setTorqueLimNeg(motorLocation_t motor, float torqueLimNeg_Nm);
void setTorqueLimsUnprotected (motorLocation_t motor, float torqueLimPos_Nm, float torqueLimNeg_Nm);
void setTorqueLimsAllProtected(float torqueLimPos_Nm, float torqueLimNeg_Nm);
void setTorqueLimsAllDistProtected(float torqueLimPos_Nm, float torqueLimNeg_Nm, const cmr_loadDistribution_t *distPos, const cmr_loadDistribution_t *distNeg);
void setVelocityInt16(motorLocation_t motor, int16_t velocity_rpm);
void setVelocityFloat(motorLocation_t motor, float velocity_rpm);
void setVelocityInt16All(int16_t velocity_rpm);
void setVelocityFloatAll(float velocity_rpm);
cmr_torque_limit_t getTorqueBudget();
const cmr_canAMKSetpoints_t *getAMKSetpoints(motorLocation_t motor);

#endif /* MOTORS_H */
