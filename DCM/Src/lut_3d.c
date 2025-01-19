#include <stdint.h>
#include <CMR/can_types.h>
#include "lut_3d.h"
#include "can.h"
#include <CMR/config_screen_helper.h>
#include <math.h>
#include "controls_23e.h"

// LUT Dimensions
#define LUT_IDX_N_ROWS_SLIPANGLE 21
#define LUT_IDX_N_COLS_FZ 41

// LUT Characteristics
static const float g = 9.81f;
static const float carWeight_N = 310.0f * g;
static const float driver_weight_N = 70.0f * g;
static const float minDownforce_N = carWeight_N / 4.0f * 0.20f;
static const float maxDownforce_N = carWeight_N / 4.0f * 1.70f;
static const float downforceSpacing_N = 30.0f;
static const float LUT_Fx_spacing_N = 19.0f;
static const float minSlipangle_deg = 0.0f;
static const float maxSlipangle_deg = 20.0f;
static const float slipangleSpacing_deg = 1.0f;
static const float LUT_granularity = 0.00001461697747;
static const float LUT_max_fx = 3154.0f;//3154.0f;

// Load 3D LUT, which is used by Fx-mapped TC
static const uint16_t LUT[] = {
    #include "LUT_3d.rawh"
};
static const uint32_t LUT_len = sizeof(LUT) / sizeof(LUT[0]);
static const uint32_t LUT_idx[LUT_IDX_N_ROWS_SLIPANGLE][LUT_IDX_N_COLS_FZ] = {
    #include "LUT_idxs.rawh"
};

// Load 2D LUT, which is used by torque-mapped TC
static const uint16_t LUT_2D[LUT_IDX_N_ROWS_SLIPANGLE][LUT_IDX_N_COLS_FZ] = {
    #include "LUTTorqueMapping.rawh"
};

// Tuneable Parameters
static const float cap_brake_psi = 150.0f; /** @brief Upper bound of break pressure for regen calculation */
static const bool useLoadCells = false;    // change this value to switch between loadcells and linpots

/**
 * @brief Returns the index of a load cell
 */
canDaqRX_t getLoadIndex(motorLocation_t motor) {
	switch (motor) {
		case MOTOR_FL:
			return CANRX_DAQ_LOAD_FL;
		case MOTOR_FR:
			return CANRX_DAQ_LOAD_FR;
		case MOTOR_RL:
			return CANRX_DAQ_LOAD_RL;
		case MOTOR_RR:
			return CANRX_DAQ_LOAD_RR;
		default:
			break;
	}
	return CANRX_DAQ_LOAD_FL; // Default to FL
}

/**
 * @brief returns max Fx
 */
float getLUTMaxFx() {
    return LUT_max_fx;
}

/**
 * @brief Return the horizontal (kappa) scale factor set by DIM
 */
static float getKappaScaleFactor() {
    float xScaleFactor = 1.0f;
    const bool retVal = getProcessedValue(&xScaleFactor, TC_LUT_Y_SCALE_INDEX, float_2_decimal);
    if (retVal) {
        return 0.85f;//xScaleFactor;
    }
    return 1.0f;
}

/**
 * @brief Return the vertical (Fz) scale factor set by DIM
 */
static float getFzScaleFactor() {
    float yScaleFactor = 1.0f;
    const bool retVal = getProcessedValue(&yScaleFactor, TC_LUT_X_SCALE_INDEX, float_2_decimal);
    if (retVal) {
        return yScaleFactor;
    }
    return 1.0f;
}

/**
 * @brief Return the break regen strength index set by DIM
 */
static uint8_t getBrakeRegenStrength() {
    uint8_t parallelRegenStrength = 0;
    const bool retVal = getProcessedValue(&parallelRegenStrength, PEDAL_REGEN_STRENGTH_INDEX, unsigned_integer);
    if (retVal) {
        return parallelRegenStrength;
    }
    return 0; 
}

/**
 * @brief Return the parallel regen enable flag set by DIM
 */
static bool shouldActivateParallelRegen() {
    bool activateParallelRegen = false;
    const bool retVal = false; //used to be config screen value
    if (retVal) {
        return activateParallelRegen;
    }
    return false;
}

static float getLinpotDownforce(canDaqRX_t leftRight, bool fronts, float springConstant, float pushrodAngle) {
    float downforce_N = carWeight_N / 4.0f;
    if (cmr_canRXMetaTimeoutWarn(&canDaqRXMeta[leftRight],  xTaskGetTickCount()) == 0) {
        volatile cmr_canDAQLinpot_t *linpotPayload = (volatile cmr_canDAQLinpot_t*) canDAQGetPayload(leftRight);
        const float springNominalLength_mm = 225.0f;
        float linpot_mm = 0.0f;
        if (fronts) {
            linpot_mm = springNominalLength_mm - ((float) linpotPayload->linpot_front_mm);
        } else {
            linpot_mm = springNominalLength_mm - ((float) linpotPayload->linpot_rear_mm);
        }
        float springForce_N = springConstant * linpot_mm; // TODO: check units
        downforce_N = springForce_N * cosf(pushrodAngle) / 4.0f;
    }
    return downforce_N;
}

float get_fake_downforce(motorLocation_t motor) {
    return (carWeight_N + driver_weight_N) / 4.0f;
}

/**
 * @brief Return downforce given motor location
 */
static float getDownforce(motorLocation_t motor) {

    if(!use_true_downforce)
        return get_fake_downforce(motor);

    float downforce_N = carWeight_N / 4.0f;
    if (useLoadCells) {
        const canDaqRX_t loadIndex = getLoadIndex(motor);
        // If we have valid loadcell data, use loadcell as downforce
        if (cmr_canRXMetaTimeoutWarn(&canDaqRXMeta[loadIndex],  xTaskGetTickCount()) == 0) {
            volatile cmr_canIZZELoadCell_t *downforcePayload = (volatile cmr_canIZZELoadCell_t*) canDAQGetPayload(loadIndex);
            downforce_N = downforcePayload->force_output_N;
        }
    } else {
        const float frontAngle_rad = ((float) M_PI) / 4;
        const float rearAngle_rad = 0.9f;
        const float frontSpringConstant = 225;
        const float rearSpringConstant = 300;
        if (motor == MOTOR_FL) {
            downforce_N = getLinpotDownforce(CANRX_DAQ_LINPOTS_LEFTS, true, frontSpringConstant, frontAngle_rad);
        } else if (motor == MOTOR_FR) {
            downforce_N = getLinpotDownforce(CANRX_DAQ_LINPOTS_RIGHTS, true, frontSpringConstant, frontAngle_rad);
        } else if (motor == MOTOR_RL) {
            downforce_N = getLinpotDownforce(CANRX_DAQ_LINPOTS_LEFTS, false, rearSpringConstant, rearAngle_rad);
        } else if (motor == MOTOR_RR) {
            downforce_N = getLinpotDownforce(CANRX_DAQ_LINPOTS_RIGHTS, false, rearSpringConstant, rearAngle_rad);
        }
    }

    // Multiply by the scale factor
    downforce_N = downforce_N * getFzScaleFactor();

    // Clamp the downforce to be valid for LUT
    if (downforce_N > maxDownforce_N) {
        downforce_N = maxDownforce_N;
    }
    if (downforce_N < minDownforce_N) {
        downforce_N = minDownforce_N;
    }

    return downforce_N;
}

/**
 * @brief Return slipangle collected from SBG
 */ 
static float getSlipangle() {
    float slipangle_deg = 0.0f;

    // Get slip angle from DAQ
    if (cmr_canRXMetaTimeoutWarn(&canDaqRXMeta[CANRX_DAQ_SBG_SLIPANGLE],  xTaskGetTickCount()) == 0) {
        volatile cmr_canSBGAutomotive_t *sbg_automotive = (volatile cmr_canSBGAutomotive_t*) canDAQGetPayload(CANRX_DAQ_SBG_SLIPANGLE);
        const float slip_angle_rad = ((float) sbg_automotive->angle_slip_rad) * 1e-4f;
        slipangle_deg = slip_angle_rad * 180.0f / ((float) M_PI);
    } 

    // Clamp slipangle to be valid for LUT
    if (slipangle_deg < 0) {
        slipangle_deg = -slipangle_deg;
    }
    if (slipangle_deg < minSlipangle_deg) {
        slipangle_deg = minSlipangle_deg;
    }
    if (slipangle_deg > maxSlipangle_deg) {
        slipangle_deg = maxSlipangle_deg;
    }

    return slipangle_deg;
}

// ******** ALTERNATE THROTTLE MAPPING BELOW ******** //

/**
 * @brief Return kappa interpolated between Fz values indexed at a specific slip angle.
*/
kappaAndFx interpKappa (
    uint32_t slipangle_ind,
    uint32_t Fz_ind,
    uint32_t Fz_ind_upp,
    uint32_t throttlePos_u8,
    float target_Fx,
    float downforce_N,
    float minDownforce_N,
    float downforceSpacing_N,
    float LUT_Fx_spacing_N
) {
    // Determine the start and end indices in LUT for given Fz
    uint32_t lut_start_idx = 0;
    uint32_t lut_end_idx = 0;
    if (slipangle_ind == 0) {
        if (Fz_ind == 0)
        {
            lut_start_idx = 0;
        } else
        {
            lut_start_idx = LUT_idx[slipangle_ind][Fz_ind-1]+1;
        }
        lut_end_idx = LUT_idx[slipangle_ind][Fz_ind];
    } else
    {
        if (Fz_ind == 0)
        {
            lut_start_idx = LUT_idx[slipangle_ind-1][LUT_IDX_N_COLS_FZ-1]+1;
        } else
        {
            lut_start_idx = LUT_idx[slipangle_ind][Fz_ind-1]+1;
        }
        lut_end_idx = LUT_idx[slipangle_ind][Fz_ind];
    }

    uint32_t lut_start_idx_upp = 0;
    uint32_t lut_end_idx_upp = 0;
    if (slipangle_ind == 0) {
        if (Fz_ind_upp == 0)
        {
            lut_start_idx_upp = 0;
        } else
        {
            lut_start_idx_upp = LUT_idx[slipangle_ind][Fz_ind_upp-1]+1;
        }
        lut_end_idx_upp = LUT_idx[slipangle_ind][Fz_ind_upp];
    } else
    {
        if (Fz_ind_upp == 0)
        {
            lut_start_idx_upp = LUT_idx[slipangle_ind-1][LUT_IDX_N_COLS_FZ-1]+1;
        } else
        {
            lut_start_idx_upp = LUT_idx[slipangle_ind][Fz_ind_upp-1]+1;
        }
        lut_end_idx_upp = LUT_idx[slipangle_ind][Fz_ind_upp];
    }

    // Interpolate for the kappas associated with the target_Fx
    const uint32_t lut_idx_add = (uint32_t) (target_Fx / LUT_Fx_spacing_N);

    uint32_t lut_idx_i1 = lut_start_idx + lut_idx_add;
    uint32_t lut_idx_i2 = lut_start_idx + lut_idx_add + 1;
    float Fx_i1 = ((float)(lut_idx_add * LUT_Fx_spacing_N));
    float Fx_i2 = ((float)((lut_idx_add + 1) * LUT_Fx_spacing_N));
    if (lut_idx_i1 >= lut_end_idx) {
        lut_idx_i1 = lut_end_idx;
        Fx_i1 = ((float)(lut_end_idx - lut_start_idx)) * ((float)(LUT_Fx_spacing_N));
    }
    if (lut_idx_i2 >= lut_end_idx) {
        lut_idx_i2 = lut_end_idx;
        Fx_i2 = ((float)(lut_end_idx - lut_start_idx)) * ((float)(LUT_Fx_spacing_N));
    }
    const float kappa = ((float)(LUT[lut_idx_i1])) + ((float)(LUT[lut_idx_i2] - LUT[lut_idx_i1])) * (target_Fx - (0.0f + ((float) lut_idx_add) * LUT_Fx_spacing_N)) / LUT_Fx_spacing_N;
    const float Fx = Fx_i1 + (Fx_i2 - Fx_i1) * (target_Fx - (0.0f + ((float) lut_idx_add) * LUT_Fx_spacing_N)) / LUT_Fx_spacing_N;

    uint32_t lut_idx_upp_i1 = lut_start_idx_upp + lut_idx_add;
    uint32_t lut_idx_upp_i2 = lut_start_idx_upp + lut_idx_add + 1;
    float Fx_upp_i1 = ((float)(lut_idx_add)) * LUT_Fx_spacing_N;
	float Fx_upp_i2 = ((float)(lut_idx_add + 1)) * LUT_Fx_spacing_N;
        if (lut_idx_upp_i1 >= lut_end_idx_upp) {
        lut_idx_upp_i1 = lut_end_idx_upp;
        Fx_upp_i1 = ((float)(lut_end_idx_upp - lut_start_idx_upp)) * LUT_Fx_spacing_N;
    }
    if (lut_idx_upp_i2 >= lut_end_idx_upp) {
        lut_idx_upp_i2 = lut_end_idx_upp;
        Fx_upp_i2 = ((float)(lut_end_idx_upp - lut_start_idx_upp)) * LUT_Fx_spacing_N;
    }
    const float kappa_upp = ((float)(LUT[lut_idx_upp_i1])) + ((float)(LUT[lut_idx_upp_i2] - LUT[lut_idx_upp_i1])) * (target_Fx - (0.0f + ((float) lut_idx_add) * LUT_Fx_spacing_N)) / LUT_Fx_spacing_N;
    const float Fx_upp = Fx_upp_i1 + (Fx_upp_i2 - Fx_upp_i1) * (target_Fx - (0 + ((float) lut_idx_add) * LUT_Fx_spacing_N)) / LUT_Fx_spacing_N;

    // Interpolate kappa along Fz dimension
    const float kappa_interped = kappa + (kappa_upp - kappa) * ((downforce_N - (minDownforce_N + ((float) Fz_ind) * downforceSpacing_N)) / downforceSpacing_N);
    const float Fx_achieved = Fx + (Fx_upp - Fx) * ((downforce_N - (minDownforce_N + ((float) Fz_ind) * downforceSpacing_N)) / downforceSpacing_N);
    const kappaAndFx result = {
        .kappa = kappa_interped,
        .Fx = Fx_achieved
    };
    return result;
}

/**
 * @brief Notimplemented
 * 
 * @param kappa 
 * @param downforce_N 
 * @param slipangle_deg 
 * @return float 
 */
float getFxByKappaDownforceSlipangle(float kappa, float downforce_N, float slipangle_deg) {
    return 0.0f;
}

/**
 * @brief Return kappa for a given motor with given downforce
*/
float getKappaByFxDownforceSlipangle (
    float downforce_N,
    float slipangle_deg,
    uint8_t throttlePos_u8,
    float target_Fx,
    bool assumeNoTurn
) {
    // const float downforce_N = getDownforce(motor);
    // const float slipangle_deg = getSlipangle();
    
    if (assumeNoTurn) {
        slipangle_deg = 0;
    }
    
    uint32_t Fz_ind = (downforce_N - minDownforce_N) / downforceSpacing_N;
    uint32_t Fz_ind_upp = Fz_ind + 1;

    if (Fz_ind >= LUT_IDX_N_COLS_FZ) {
        Fz_ind = LUT_IDX_N_COLS_FZ - 1;
    }
    if (Fz_ind_upp >= LUT_IDX_N_COLS_FZ) {
        Fz_ind_upp = LUT_IDX_N_COLS_FZ - 1;
    }
    
    uint32_t slipangle_ind = (slipangle_deg - minSlipangle_deg) / slipangleSpacing_deg;
    uint32_t slipangle_ind_upp = slipangle_ind + 1;

    if (slipangle_ind >= LUT_IDX_N_ROWS_SLIPANGLE) {
        slipangle_ind = LUT_IDX_N_ROWS_SLIPANGLE - 1;
    }
    if (slipangle_ind_upp >= LUT_IDX_N_ROWS_SLIPANGLE) {
        slipangle_ind_upp = LUT_IDX_N_ROWS_SLIPANGLE - 1;
    }

    // Get interpolated kappa at the lower slip angle
    // NOTE: due to the nature of max Fx decreasing as slip angle increases, we need to interp on the upper Fx first for this Fx correction to work
	const kappaAndFx kappa_and_Fx_interped_upp = interpKappa(slipangle_ind_upp, Fz_ind, Fz_ind_upp, throttlePos_u8, target_Fx, downforce_N, minDownforce_N, downforceSpacing_N, LUT_Fx_spacing_N);
	const float kappa_interped_upp = kappa_and_Fx_interped_upp.kappa;
	const float Fx_achieved = kappa_and_Fx_interped_upp.Fx;
	const float new_target_Fx = target_Fx - (Fx_achieved - target_Fx);
	const kappaAndFx kappa_and_Fx_interped = interpKappa(slipangle_ind, Fz_ind, Fz_ind_upp, throttlePos_u8, new_target_Fx, downforce_N, minDownforce_N, downforceSpacing_N, LUT_Fx_spacing_N);
	const float kappa_interped = kappa_and_Fx_interped.kappa;

    // Interpolate kappa along slip angle dimension
    const float kappa_result = kappa_interped + (kappa_interped_upp - kappa_interped) * (slipangle_deg - (minSlipangle_deg + ((float) slipangle_ind) * slipangleSpacing_deg)) / slipangleSpacing_deg;
    return kappa_result * LUT_granularity * getKappaScaleFactor();
}

/**
 * @brief Return kappa for a given motor with current downforce conditions
*/
float getKappaByFx (
    motorLocation_t motor,
    uint8_t throttlePos_u8,
    float target_Fx,
    bool assumeNoTurn
) {
    configASSERT(motor < MOTOR_LEN);

    const float downforce_N = getDownforce(motor);
    const float slipangle_deg = (assumeNoTurn) ? 0.0f : getSlipangle();

    uint32_t Fz_ind = (downforce_N - minDownforce_N) / downforceSpacing_N;
    uint32_t Fz_ind_upp = Fz_ind + 1;

    if (Fz_ind >= LUT_IDX_N_COLS_FZ) {
        Fz_ind = LUT_IDX_N_COLS_FZ - 1;
    }
    if (Fz_ind_upp >= LUT_IDX_N_COLS_FZ) {
        Fz_ind_upp = LUT_IDX_N_COLS_FZ - 1;
    }

    uint32_t slipangle_ind = (slipangle_deg - minSlipangle_deg) / slipangleSpacing_deg;
    uint32_t slipangle_ind_upp = slipangle_ind + 1;

    if (slipangle_ind >= LUT_IDX_N_ROWS_SLIPANGLE) {
        slipangle_ind = LUT_IDX_N_ROWS_SLIPANGLE - 1;
    }
    if (slipangle_ind_upp >= LUT_IDX_N_ROWS_SLIPANGLE) {
        slipangle_ind_upp = LUT_IDX_N_ROWS_SLIPANGLE - 1;
    }

    // Get interpolated kappa at the lower slip angle
    // NOTE: due to the nature of max Fx decreasing as slip angle increases, we need to interp on the upper Fx first for this Fx correction to work
	const kappaAndFx kappa_and_Fx_interped_upp = interpKappa(slipangle_ind_upp, Fz_ind, Fz_ind_upp, throttlePos_u8, target_Fx, downforce_N, minDownforce_N, downforceSpacing_N, LUT_Fx_spacing_N);
	const float kappa_interped_upp = kappa_and_Fx_interped_upp.kappa;
	const float Fx_achieved = kappa_and_Fx_interped_upp.Fx;
	const float new_target_Fx = target_Fx - (Fx_achieved - target_Fx);
	const kappaAndFx kappa_and_Fx_interped = interpKappa(slipangle_ind, Fz_ind, Fz_ind_upp, throttlePos_u8, new_target_Fx, downforce_N, minDownforce_N, downforceSpacing_N, LUT_Fx_spacing_N);
	const float kappa_interped = kappa_and_Fx_interped.kappa;

    // Interpolate kappa along slip angle dimension
    const float kappa_result = kappa_interped + (kappa_interped_upp - kappa_interped) * (slipangle_deg - (minSlipangle_deg + ((float) slipangle_ind) * slipangleSpacing_deg)) / slipangleSpacing_deg;
    return kappa_result * LUT_granularity * getKappaScaleFactor();
}

/**
 * @brief Return kappa interpolated between Fz values indexed at a specific slip angle
*/
float interpFx (
    uint32_t slipangle_ind,
    uint32_t Fz_ind,
    uint32_t Fz_ind_upp,
    uint32_t throttlePos_u8,
    float downforce_N, 
    float minDownforce_N,
    float downforceSpacing_N,
    float LUT_Fx_spacing_N
) {
    // Determine the start and end indices in LUT for given Fz
    uint32_t lut_start_idx = 0;
    uint32_t lut_end_idx = 0;
    if (slipangle_ind == 0) {
        if (Fz_ind == 0)
        {
            lut_start_idx = 0;
        } else
        {
            lut_start_idx = LUT_idx[slipangle_ind][Fz_ind-1]+1;
        }
        lut_end_idx = LUT_idx[slipangle_ind][Fz_ind];
    } else 
    {
        if (Fz_ind == 0)
        {
            lut_start_idx = LUT_idx[slipangle_ind-1][LUT_IDX_N_COLS_FZ-1]+1;
        } else
        {
            lut_start_idx = LUT_idx[slipangle_ind][Fz_ind-1]+1;
        }
        lut_end_idx = LUT_idx[slipangle_ind][Fz_ind];
    }   
    
    uint32_t lut_start_idx_upp = 0;
    uint32_t lut_end_idx_upp = 0;
    if (slipangle_ind == 0) {
        if (Fz_ind_upp == 0)
        {
            lut_start_idx_upp = 0;
        } else
        {
            lut_start_idx_upp = LUT_idx[slipangle_ind][Fz_ind_upp-1]+1;
        }
        lut_end_idx_upp = LUT_idx[slipangle_ind][Fz_ind_upp];
    } else 
    {
        if (Fz_ind_upp == 0)
        {
            lut_start_idx_upp = LUT_idx[slipangle_ind-1][LUT_IDX_N_COLS_FZ-1]+1;
        } else
        {
            lut_start_idx_upp = LUT_idx[slipangle_ind][Fz_ind_upp-1]+1;
        }
        lut_end_idx_upp = LUT_idx[slipangle_ind][Fz_ind_upp];
    }
    
    uint32_t lut_idx = (lut_end_idx - lut_start_idx) * ((uint32_t) throttlePos_u8) / ((uint32_t) (UINT8_MAX)) + lut_start_idx;
    if (lut_idx >= LUT_len) {
        lut_idx = LUT_len - 1;
    }
    uint32_t lut_idx_upp = (lut_end_idx_upp - lut_start_idx_upp) * ((uint32_t) throttlePos_u8) / ((uint32_t) (UINT8_MAX)) + lut_start_idx_upp;
    if (lut_idx_upp >= LUT_len) {
        lut_idx_upp = LUT_len - 1;
    }

    // Return Fx
    const float Fx = LUT_Fx_spacing_N * (float) (lut_idx - lut_start_idx);
    const float Fx_upp = LUT_Fx_spacing_N * (float) (lut_idx_upp - lut_start_idx_upp);
    const float Fx_interped = Fx + (Fx_upp - Fx) * ((downforce_N - (minDownforce_N + ((float) Fz_ind) * downforceSpacing_N)) / downforceSpacing_N);
    return Fx_interped;
}

/**
 * @brief Return kappa for a given motor with current downforce conditions
*/
float getTraction (
    motorLocation_t motor,
    uint8_t throttlePos_u8,
	bool assumeNoTurn
) {
    configASSERT(motor < MOTOR_LEN);

    const float downforce_N = getDownforce(motor);
    const float slipangle_deg = (assumeNoTurn) ? 0.0f : getSlipangle();
    
    uint32_t Fz_ind = (downforce_N - minDownforce_N) / downforceSpacing_N;
    uint32_t Fz_ind_upp = Fz_ind + 1;

    if (Fz_ind >= LUT_IDX_N_COLS_FZ) {
        Fz_ind = LUT_IDX_N_COLS_FZ - 1;
    }
    if (Fz_ind_upp >= LUT_IDX_N_COLS_FZ) {
        Fz_ind_upp = LUT_IDX_N_COLS_FZ - 1;
    }

    uint32_t slipangle_ind = (slipangle_deg - minSlipangle_deg) / slipangleSpacing_deg;
    uint32_t slipangle_ind_upp = slipangle_ind + 1;

    if (slipangle_ind >= LUT_IDX_N_ROWS_SLIPANGLE) {
        slipangle_ind = LUT_IDX_N_ROWS_SLIPANGLE - 1;
    }
    if (slipangle_ind_upp >= LUT_IDX_N_ROWS_SLIPANGLE) {
        slipangle_ind_upp = LUT_IDX_N_ROWS_SLIPANGLE - 1;
    }

    // Get interpolated two Fx interpolated along Fz
    const float Fx_interped = interpFx(slipangle_ind, Fz_ind, Fz_ind_upp, throttlePos_u8, downforce_N, minDownforce_N, downforceSpacing_N, LUT_Fx_spacing_N);
    const float Fx_interped_upp = interpFx(slipangle_ind_upp, Fz_ind, Fz_ind_upp, throttlePos_u8, downforce_N, minDownforce_N, downforceSpacing_N, LUT_Fx_spacing_N);

    // Interpolate Fx along slip angle dimension
    const float Fx_result = Fx_interped + (Fx_interped_upp - Fx_interped) * (slipangle_deg - (minSlipangle_deg + ((float) slipangle_ind) * slipangleSpacing_deg)) / slipangleSpacing_deg;
    return Fx_result;
}

// ******** INTUITIVE THROTTLE MAPPING BELOW ******** //

/**
 * @brief Helper function for getKappaByFx(), returns kappa interpolated between Fz values indexed at a specific slip angle
*/
kappaAndFx interpKappaIntuitive (
    uint32_t slipangle_ind,
    uint32_t Fz_ind,
    uint32_t Fz_ind_upp,
    float target_Fx,
    float downforce_N,
    float minDownforce_N,
    float downforceSpacing_N,
    float LUT_Fx_spacing_N
) {
    // Determine the start and end indices in LUT for given slipangle_ind and Fz_ind
    uint32_t lut_start_idx = 0;
    uint32_t lut_end_idx = 0;
    if (slipangle_ind == 0) { 
        if (Fz_ind == 0)
        {
            // if both slipangle_ind and Fz_ind are 0, then start idx is 0
            lut_start_idx = 0;
        } else
        {
            // if Fz_ind != 0 , then the start index is located at +1 of the previous index's value
            lut_start_idx = LUT_idx[slipangle_ind][Fz_ind-1]+1;
        }
        // the end idx is located at the slipangle_ind and Fz_ind
        lut_end_idx = LUT_idx[slipangle_ind][Fz_ind];
    } else 
    {
        if (Fz_ind == 0)
        {
            // if slipangle_ind != 0 but Fz_ind = 0, then the start idx is located at +1 of largest Fz_ind of the previous row
            lut_start_idx = LUT_idx[slipangle_ind-1][LUT_IDX_N_COLS_FZ-1]+1;
        } else
        {
            // if Fz_ind != 0 , then the start index is located at +1 of the previous index's value
            lut_start_idx = LUT_idx[slipangle_ind][Fz_ind-1]+1;
        }
        // the end idx is located at the slipangle_ind and Fz_ind
        lut_end_idx = LUT_idx[slipangle_ind][Fz_ind];
    }
    
    // Determine the start and end indices in LUT for given slipangle_ind and Fz_ind_upp
    uint32_t lut_start_idx_upp = 0;
    uint32_t lut_end_idx_upp = 0;
    if (slipangle_ind == 0) {
        if (Fz_ind_upp == 0)
        {
            // if both slipangle_ind and Fz_ind are 0, then start idx is 0
            lut_start_idx_upp = 0;
        } else
        {
            // if Fz_ind != 0 , then the start index is located at +1 of the previous index's value
            lut_start_idx_upp = LUT_idx[slipangle_ind][Fz_ind_upp-1]+1;
        }
        // the end idx is located at the slipangle_ind and Fz_ind
        lut_end_idx_upp = LUT_idx[slipangle_ind][Fz_ind_upp];
    } else 
    {
        if (Fz_ind_upp == 0)
        {
            // if slipangle_ind != 0 but Fz_ind = 0, then the start idx is located at +1 of largest Fz_ind of the previous row
            lut_start_idx_upp = LUT_idx[slipangle_ind-1][LUT_IDX_N_COLS_FZ-1]+1;
        } else
        {
            // if Fz_ind != 0 , then the start index is located at +1 of the previous index's value
            lut_start_idx_upp = LUT_idx[slipangle_ind][Fz_ind_upp-1]+1;
        }
        // the end idx is located at the slipangle_ind and Fz_ind
        lut_end_idx_upp = LUT_idx[slipangle_ind][Fz_ind_upp];
    }

    // Compute the idx add needs to locate the target_Fx for it's associated kappa
    const uint32_t lut_idx_add = (uint32_t) (target_Fx / LUT_Fx_spacing_N);

    // Interpolate for the kappas associated with the target_Fx for Fz_ind
    uint32_t lut_idx_i1 = lut_start_idx + lut_idx_add;
    uint32_t lut_idx_i2 = lut_start_idx + lut_idx_add + 1;
    float Fx_i1 = ((float)(lut_idx_add * LUT_Fx_spacing_N));
    float Fx_i2 = ((float)((lut_idx_add + 1) * LUT_Fx_spacing_N));
    if (lut_idx_i1 >= lut_end_idx) {
        lut_idx_i1 = lut_end_idx;
        Fx_i1 = ((float)(lut_end_idx - lut_start_idx)) * ((float)(LUT_Fx_spacing_N));
    }
    if (lut_idx_i2 >= lut_end_idx) {
        lut_idx_i2 = lut_end_idx;
        Fx_i2 = ((float)(lut_end_idx - lut_start_idx)) * ((float)(LUT_Fx_spacing_N));
    }
    float kappa = ((float)(LUT[lut_idx_i1])) + ((float)(LUT[lut_idx_i2] - LUT[lut_idx_i1])) * (target_Fx - (0.0f + ((float) lut_idx_add) * LUT_Fx_spacing_N)) / LUT_Fx_spacing_N;
    float Fx = Fx_i1 + (Fx_i2 - Fx_i1) * (target_Fx - (0.0f + ((float) lut_idx_add) * LUT_Fx_spacing_N)) / LUT_Fx_spacing_N;

    // Interpolate for the kappas associated with the target_Fx for Fz_ind_upp
    uint32_t lut_idx_upp_i1 = lut_start_idx_upp + lut_idx_add;
    uint32_t lut_idx_upp_i2 = lut_start_idx_upp + lut_idx_add + 1;
    float Fx_upp_i1 = ((float)(lut_idx_add)) * LUT_Fx_spacing_N;
	float Fx_upp_i2 = ((float)(lut_idx_add + 1)) * LUT_Fx_spacing_N;
        if (lut_idx_upp_i1 >= lut_end_idx_upp) {
        lut_idx_upp_i1 = lut_end_idx_upp;
        Fx_upp_i1 = ((float)(lut_end_idx_upp - lut_start_idx_upp)) * LUT_Fx_spacing_N;
    }
    if (lut_idx_upp_i2 >= lut_end_idx_upp) {
        lut_idx_upp_i2 = lut_end_idx_upp;
        Fx_upp_i2 = ((float)(lut_end_idx_upp - lut_start_idx_upp)) * LUT_Fx_spacing_N;
    }
    const float kappa_upp = ((float)(LUT[lut_idx_upp_i1])) + ((float)(LUT[lut_idx_upp_i2] - LUT[lut_idx_upp_i1])) * (target_Fx - (0.0f + ((float) lut_idx_add) * LUT_Fx_spacing_N)) / LUT_Fx_spacing_N;
    const float Fx_upp = Fx_upp_i1 + (Fx_upp_i2 - Fx_upp_i1) * (target_Fx - (0 + ((float) lut_idx_add) * LUT_Fx_spacing_N)) / LUT_Fx_spacing_N;

    // Interpolate kappa along Fz dimension
    const float kappa_interped = kappa + (kappa_upp - kappa) * ((downforce_N - (minDownforce_N + ((float) Fz_ind) * downforceSpacing_N)) / downforceSpacing_N);
    const float Fx_achieved = Fx + (Fx_upp - Fx) * ((downforce_N - (minDownforce_N + ((float) Fz_ind) * downforceSpacing_N)) / downforceSpacing_N);
    
    // Package the resulting kappa and the Fx associated with the Kappa
    const kappaAndFx result = {
        .kappa = kappa_interped,
        .Fx = Fx_achieved
    };
    return result;
}

/**
 * @brief The LUT Kappa and Fx for the throttle input
*/
kappaAndFx getKappaFxGlobalMax (
    motorLocation_t motor,
    uint8_t throttlePos_u8,
    bool assumeNoTurn
) {
    configASSERT(motor < MOTOR_LEN);

    float downforce_N = 0.0f;
    if (assumeNoTurn) {
        switch (motor) {
            case MOTOR_FL:
            case MOTOR_FR:
                // take the average of the downforce on front wheels
                downforce_N = (getDownforce(MOTOR_FL) + getDownforce(MOTOR_FR)) * 0.5f;
                break;

            case MOTOR_RL:
            case MOTOR_RR:
                // take the average of the downforce on rear wheels
                downforce_N = (getDownforce(MOTOR_RL) + getDownforce(MOTOR_RR)) * 0.5f;
                break;

            default:
                downforce_N = 0.0f;
                break;
        }
    } else {
        downforce_N = getDownforce(motor);
    }

    const float slipangle_deg = assumeNoTurn ? 0.0f : getSlipangle();

    // Get the Fx requested associated with throttlePos_u8
    const float target_Fx = LUT_max_fx * ((float) throttlePos_u8) / ((float) UINT8_MAX);
    
    // Covert the downforce to indexes for the LUT
    uint32_t Fz_ind = (downforce_N - minDownforce_N) / downforceSpacing_N;
    uint32_t Fz_ind_upp = Fz_ind + 1;
    
    // Ensure the Fz indices are within bounds
    if (Fz_ind >= LUT_IDX_N_COLS_FZ) {
        Fz_ind = LUT_IDX_N_COLS_FZ - 1;
    }
    if (Fz_ind_upp >= LUT_IDX_N_COLS_FZ) {
        Fz_ind_upp = LUT_IDX_N_COLS_FZ - 1;
    }

    // Convert slip angle to indexes for the lUT
    uint32_t slipangle_ind = (slipangle_deg - minSlipangle_deg) / slipangleSpacing_deg;
    uint32_t slipangle_ind_upp = slipangle_ind + 1;

    // Ensure the slip angle indices are within bounds
    if (slipangle_ind >= LUT_IDX_N_ROWS_SLIPANGLE) {
        slipangle_ind = LUT_IDX_N_ROWS_SLIPANGLE - 1;
    }
    if (slipangle_ind_upp >= LUT_IDX_N_ROWS_SLIPANGLE) {
        slipangle_ind_upp = LUT_IDX_N_ROWS_SLIPANGLE - 1;
    }

    // Get interpolated kappa at the two slip angles
    // NOTE: due to the nature of max Fx decreasing as slip angle increases, we need to interp on the upper slip angle first for this Fx correction to work
	const kappaAndFx kappa_and_Fx_interped_upp = interpKappaIntuitive(slipangle_ind_upp, Fz_ind, Fz_ind_upp, target_Fx, downforce_N, minDownforce_N, downforceSpacing_N, LUT_Fx_spacing_N);
	const float kappa_interped_upp = kappa_and_Fx_interped_upp.kappa;
	const float Fx_achieved = kappa_and_Fx_interped_upp.Fx; // This Fx comes from a higher slipangle condition 
	const float new_target_Fx = target_Fx - (Fx_achieved - target_Fx);
	const kappaAndFx kappa_and_Fx_interped = interpKappaIntuitive(slipangle_ind, Fz_ind, Fz_ind_upp, new_target_Fx, downforce_N, minDownforce_N, downforceSpacing_N, LUT_Fx_spacing_N);
	const float kappa_interped = kappa_and_Fx_interped.kappa;
    const float Fx_interped = kappa_and_Fx_interped.Fx;

    // Interpolate kappa between the slip angles
    const float kappa_result = kappa_interped + (kappa_interped_upp - kappa_interped) * (slipangle_deg - (minSlipangle_deg + ((float) slipangle_ind) * slipangleSpacing_deg)) / slipangleSpacing_deg;
    const float Fx_result = Fx_achieved + (Fx_interped - Fx_achieved) * (slipangle_deg - (minSlipangle_deg + ((float) slipangle_ind) * slipangleSpacing_deg)) / slipangleSpacing_deg; 
    const kappaAndFx result = {
        .kappa = kappa_result * LUT_granularity * getKappaScaleFactor(),
        .Fx = Fx_result
    };
    return result; /** @todo check value that gets returned */
}

// ********* THROTTLE TORQUE MAPPING BELOW ********* //

/**
 * @brief Get the kappa corresponding to max traction at current vehicle state
 */
float getMaxKappaCurrentState (
    motorLocation_t motor, 
    bool assumeNoTurn
) {
    configASSERT(motor < MOTOR_LEN);

    float downforce_N = 0.0f;
    if (assumeNoTurn) {
        switch (motor) {
            case MOTOR_FL:
            case MOTOR_FR:
                // take the average of the downforce on front wheels
                downforce_N = (getDownforce(MOTOR_FL) + getDownforce(MOTOR_FR)) * 0.5f;
                break;
            case MOTOR_RL:
            case MOTOR_RR:
                // take the average of the downforce on rear wheels
                downforce_N = (getDownforce(MOTOR_RL) + getDownforce(MOTOR_RR)) * 0.5f;
                break;

            default:
                downforce_N = 0.0f;
                break;
        }
    } else {
        downforce_N = getDownforce(motor);
    }

    const float slipangle_deg = assumeNoTurn ? 0.0f : getSlipangle();

    // Covert the downforce to indexes for the LUT
    uint32_t Fz_ind = (downforce_N - minDownforce_N) / downforceSpacing_N;
    uint32_t Fz_ind_upp = Fz_ind + 1;
    
    // Ensure the Fz indices are within bounds
    if (Fz_ind >= LUT_IDX_N_COLS_FZ) {
        Fz_ind = LUT_IDX_N_COLS_FZ - 1;
    }
    if (Fz_ind_upp >= LUT_IDX_N_COLS_FZ) {
        Fz_ind_upp = LUT_IDX_N_COLS_FZ - 1;
    }

    // Convert slip angle to indexes for the lUT
    uint32_t slipangle_ind = (slipangle_deg - minSlipangle_deg) / slipangleSpacing_deg;
    uint32_t slipangle_ind_upp = slipangle_ind + 1;

    // Ensure the slip angle indices are within bounds
    if (slipangle_ind >= LUT_IDX_N_ROWS_SLIPANGLE) {
        slipangle_ind = LUT_IDX_N_ROWS_SLIPANGLE - 1;
    }
    if (slipangle_ind_upp >= LUT_IDX_N_ROWS_SLIPANGLE) {
        slipangle_ind_upp = LUT_IDX_N_ROWS_SLIPANGLE - 1;
    }
    
    // interpolate kappa for the two slipangles
    float kappa_interped = ((float) LUT_2D[slipangle_ind][Fz_ind]) +  ((float)(LUT_2D[slipangle_ind][Fz_ind_upp] - LUT_2D[slipangle_ind][Fz_ind])) * ((downforce_N - (minDownforce_N + ((float) Fz_ind) * downforceSpacing_N)) / downforceSpacing_N);
    float kappa_interped_upp = ((float) LUT_2D[slipangle_ind_upp][Fz_ind]) +  ((float)(LUT_2D[slipangle_ind_upp][Fz_ind_upp] - LUT_2D[slipangle_ind_upp][Fz_ind])) * ((downforce_N - (minDownforce_N + ((float) Fz_ind) * downforceSpacing_N)) / downforceSpacing_N);

    // interpolate for the resulting kappa
    float kappa_result = kappa_interped + (kappa_interped_upp - kappa_interped) * (slipangle_deg - (minSlipangle_deg + ((float) slipangle_ind) * slipangleSpacing_deg)) / slipangleSpacing_deg;
    return kappa_result * LUT_granularity * getKappaScaleFactor();
}


// ******** BRAKING PARALLEL REGEN CODE BELOW ********** //

/**
 * @brief Get the kappa setpoint for brake parallel regen
*/
float getBrakeKappa(motorLocation_t motor, uint8_t brakePressurePsi_u8, float deadband) {
    const bool parallelRegen = shouldActivateParallelRegen();
    float brake_kappa = 0.0f;

    if (parallelRegen) {
        const float regen_percentage = (float)(getBrakeRegenStrength());
        const float adjusted_brake_pressure = (((float) brakePressurePsi_u8) - deadband);

        // Convert from measured brake pressure to intended brake amount on a uint8_scale 
        //  normalized_brake_pressure=255 indicates request of max brake torque available in LUT if achieveable as checked by getKappaMapped
        const float slope_deg = regen_percentage * (90.0f / 100.0f); // map from percentage [0,100] -> to slope [0,90]
        const float slope_rad = slope_deg * ((float) M_PI) / 180.0f;
        float mapped_brake_pressure = adjusted_brake_pressure * tanf(slope_rad);

        // clamp results between 0 and cap_brake_psi
        mapped_brake_pressure = fmaxf(mapped_brake_pressure, 0.0f);
        mapped_brake_pressure = fminf(mapped_brake_pressure, cap_brake_psi - deadband);

        // normalize -- to uint8 since getKappaByFx takes in a uint_8 throttlePos value
        const uint8_t normalized_brake_pressure = (uint8_t) (mapped_brake_pressure / (cap_brake_psi - deadband) * 255.0f);

        // get the kappa setpoint
        if (normalized_brake_pressure > 0) {
            brake_kappa = -(getKappaFxGlobalMax(motor, normalized_brake_pressure, false)).kappa; //TODO: Check if straight line bool can be always false
        }
    } else {
        brake_kappa = 0.0f;
    }
    return brake_kappa;
}
