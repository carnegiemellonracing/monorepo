#include <stdint.h>
#include <CMR/can_ids.h>    // CMR CAN IDs

extern float power_upper_limit_W;
extern float power_safety_margin_W;

void setTorqueLimsProtected ( const cmr_torqueDistributionNm_t *torquesPos_Nm, const cmr_torqueDistributionNm_t *torquesNeg_Nm);
cmr_torque_limit_t getPreemptiveTorqueLimits();
void initRetroactiveLimitFilters();
void resetRetroactiveLimitFilters();
void setPowerLimit(uint8_t limit);

const cmr_canCDCSafetyFilterStates_t *getSafetyFilterInfo();
const cmr_canCDCMotorPower_t *getMotorPowerInfo();
