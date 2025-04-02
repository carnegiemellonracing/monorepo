#include <stdint.h>
#include <CMR/can_ids.h>    // CMR CAN IDs
#include "motors.h"

void setTorqueLimsProtected ( const cmr_torqueDistributionNm_t *torquesPos_Nm, const cmr_torqueDistributionNm_t *torquesNeg_Nm);
cmr_torque_limit_t getPreemptiveTorqueLimits();
void initRetroactiveLimitFilters();
void resetRetroactiveLimitFilters();
float getPowerLimit_W();

void setPowerLimit_kW(uint8_t power_limit_kW);

const cmr_canCDCSafetyFilterStates_t *getSafetyFilterInfo();
const cmr_canCDCMotorPower_t *getMotorPowerInfo();
