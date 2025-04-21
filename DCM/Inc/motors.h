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

/** @brief Location-based CAN IDs for inverters. */
typedef enum {
    /*
    FR: AMK1 - 0x184, 0x283, 0x285
    RL: AMK2 - 0x185, 0x284, 0x286
    RR: AMK3 - 0x188, 0x287, 0x289
    FL: AMK4 - 0x189, 0x288, 0x28A
    */


    CMR_CANID_AMK_FR_ACT_1      = CMR_CANID_AMK_1_ACT_1,
    CMR_CANID_AMK_FR_ACT_2      = CMR_CANID_AMK_1_ACT_2,
    CMR_CANID_AMK_FR_SETPOINTS  = CMR_CANID_AMK_1_SETPOINTS,

    CMR_CANID_AMK_RR_ACT_1      = CMR_CANID_AMK_3_ACT_1,
    CMR_CANID_AMK_RR_ACT_2      = CMR_CANID_AMK_3_ACT_2,
    CMR_CANID_AMK_RR_SETPOINTS  = CMR_CANID_AMK_3_SETPOINTS,

    CMR_CANID_AMK_FL_ACT_1      = CMR_CANID_AMK_4_ACT_1,
    CMR_CANID_AMK_FL_ACT_2      = CMR_CANID_AMK_4_ACT_2,
    CMR_CANID_AMK_FL_SETPOINTS  = CMR_CANID_AMK_4_SETPOINTS,

    CMR_CANID_AMK_RL_ACT_1      = CMR_CANID_AMK_2_ACT_1,
    CMR_CANID_AMK_RL_ACT_2      = CMR_CANID_AMK_2_ACT_2,
    CMR_CANID_AMK_RL_SETPOINTS  = CMR_CANID_AMK_2_SETPOINTS
} amkCANID_t;

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
cmr_canDAQTest_t getDAQTest();

#endif /* MOTORS_H */
