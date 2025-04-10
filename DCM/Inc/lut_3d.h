#ifndef LUT_H
#define LUT_H
#include "motors_helper.h"

typedef struct {
	float kappa;
    float Fx;
} kappaAndFx;

float getFxByKappaDownforceSlipangle(float kappa, float downforce_N, float slipangle_deg);

float getKappaByFxDownforceSlipangle(float downforce_N, float slipangle_deg, uint8_t throttlePos_u8, float target_Fx, bool assumeNoTurn);

float getKappaByFx(motorLocation_t motor, uint8_t throttlePos_u8, float target_Fx, bool assumeNoTurn);

float getTraction(motorLocation_t motor, uint8_t throttlePos_u8, bool assumeNoTurn);

kappaAndFx getKappaFxGlobalMax(motorLocation_t motor, uint8_t throttlePos_u8, bool assumeNoTurn);

kappaAndFx getKappaFxGlobalMaxAtDownforce(float downforce_N, uint8_t throttlePos_u8, bool assumeNoTurn);

float getMaxKappaCurrentState(motorLocation_t motor, bool assumeNoTurn);

float getBrakeKappa(motorLocation_t motor, uint16_t brakePressurePsi_u8, float deadband);

int32_t getBrakeMaxTorque_mNm(motorLocation_t motor, uint16_t brakePressurePsi_u8);

float test();

float getLUTMaxFx();

float get_fake_downforce(motorLocation_t motor);

#endif /* LUT_H */
