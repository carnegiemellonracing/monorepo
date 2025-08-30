/**
 * @file motors_helper.h
 * @brief DTI inverter helper.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef MOTORS_HELPER_H
#define MOTORS_HELPER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <CMR/can_types.h>

// ------------------------------------------------------------------------------------------------
// Constants

extern const float torqueIncrement_Nm;
extern const float rpm_to_mV_factor;

// ------------------------------------------------------------------------------------------------
// Enums/Structs

/** @brief Enumeration for motor locations. */
typedef enum {
    MOTOR_FR = 0,   /**< @brief Front right. */
    MOTOR_RR,       /**< @brief Rear right. */
    MOTOR_FL,       /**< @brief Front left. */
    MOTOR_RL,       /**< @brief Rear left. */
    MOTOR_LEN       /**< @brief Number of motors. */
} motorLocation_t;

typedef struct {
    float fl;
    float fr;
    float rl;
    float rr;
} cmr_loadDistribution_t;

typedef struct {
    float fl;
    float fr;
    float rl;
    float rr;
} cmr_torqueDistributionNm_t;

// ------------------------------------------------------------------------------------------------
// Public Function Declarations

int16_t convertNmToAMKTorque(float torque_Nm);
volatile cmr_canDTI_TX_Erpm_t *getDTIErpm(motorLocation_t motor);
volatile cmr_canDTI_TX_Current_t *getDTICurrent(motorLocation_t motor);
volatile cmr_canDTI_TX_TempFault_t *getDTITempFault(motorLocation_t motor);
volatile cmr_canDTI_TX_IdIq_t *getDTIIdIq(motorLocation_t motor);
volatile cmr_canDTI_TX_ACLimits_t *getDTIACLimits(motorLocation_t motor);
volatile cmr_canDTI_TX_DCLimits_t *getDTIDCLimits(motorLocation_t motor);
volatile cmr_canDTI_TX_ControlStatus_t *getDTIControlStatus(motorLocation_t motor);
volatile cmr_canDTI_TX_IOStatus_t *getDTIIOStatus(motorLocation_t motor);
bool isMotorDataValid(motorLocation_t motor);
float rpmToRadps(float rpm);
float motorSpeedToWheelLinearSpeed_mps(float motor_speed_radps);
int16_t getMotorSpeed_rpm(motorLocation_t motor);
float getMotorSpeed_radps(motorLocation_t motor);
float getTotalMotorSpeed_radps();
float getTotalMotorSpeed_rpm();
float getMinMotorSpeed_radps();
float getMotorPower(motorLocation_t motor, float pack_voltage_V);
float getLoadByIndex(const cmr_loadDistribution_t *loads, size_t motor);
float getTorqueNmByIndex(const cmr_torqueDistributionNm_t *torques_Nm, size_t motor);
float getTorqueNmMax(const cmr_torqueDistributionNm_t *torques_Nm);
float getTorqueNmMin(const cmr_torqueDistributionNm_t *torques_Nm);
bool overVoltProtection();
float getMotorRegenerativeCapacity(int32_t rpm);

#endif /* MOTORS_HELPER_H */
