#include <stdint.h>
#include <CMR/can_types.h>
#include "lut_3d.h"
#include "constants.h"
#include "can.h"
#include <CMR/config_screen_helper.h>
#include <math.h>
#include "controls.h"
#include "daq.h"

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
//    float yScaleFactor = 1.0f;
//    const bool retVal = getProcessedValue(&yScaleFactor, TC_LUT_X_SCALE_INDEX, float_2_decimal);
//    if (retVal) {
//        return yScaleFactor;
//    }
//    return 1.0f;

	// I don't trust the config screen value.
	return 0.4;
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

    float downforce_N = get_fake_downforce(motor);

    if(use_true_downforce) {

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
    //const bool parallelRegen = shouldActivateParallelRegen(); // make sure to set this to true for the branch
    float brake_kappa = 0.0f;

    const float regen_percentage = (float)(getBrakeRegenStrength()); // regen percentage requested by DIM
    const float adjusted_brake_pressure = (((float) brakePressurePsi_u8) - deadband);
    
    // check, dont allow negatives
    if ((adjusted_brake_pressure - deadband) < 0) {
        brake_kappa = 0.0f;
    }
    // brake pressure being translated into something that getKappaFxGlobalMax can use

    float scaled_brake_kappa = ((adjusted_brake_pressure - deadband) / (255 - deadband)) * regen_percentage;
    const uint8_t normalized_brake_pressure = scaled_brake_kappa * 255;

	// get the kappa setpoint
	if (normalized_brake_pressure > 0) {
		brake_kappa = -(getKappaFxGlobalMax(motor, normalized_brake_pressure, false)).kappa; //TODO: Check if straight line bool can be always false // what?
	} else {
        brake_kappa = 0.0f;
    }
    return brake_kappa;
}

/**
 * @brief Get the torque setpoint for brake parallel regen
*/
int32_t getBrakeMaxTorque_mNm(motorLocation_t motor, uint8_t brakePressurePsi_u8) {

    float desired_brake_torque_Nm = 0.0f;

    const float regen_strength = ((float) getBrakeRegenStrength()) * 0.01f;
    const float brake_strength = ((float) (brakePressurePsi_u8 - braking_threshold_psi)) / (UINT8_MAX - braking_threshold_psi);
    const float combined_strength = regen_strength * brake_strength;
    
    const float max_regen_torque_Nm = 42.0f;
    // combined_strength: [0, 1] -> [0Nm, 42Nm].
    // Since brake pressure caps at ~180 psi, brake_strength caps at ~0.65.
    // Hence, combined_strength caps at ~0.65, leading to a torque of ~27.3 Nm.
    // Of course this is more than allowed, and the DIM config is expected to be tuned to accommodate this.

    if (combined_strength <= 0)
        return 0;
    
    desired_brake_torque_Nm = combined_strength * max_regen_torque_Nm;

    const float max_brake_force_N = getKappaFxGlobalMax(motor, UINT8_MAX, false).Fx;
    const float max_brake_torque_Nm = (max_brake_force_N * effective_wheel_rad_m) / gear_ratio;
    // Clamping desired_brake_force_N as per tractive capacity.
    desired_brake_torque_Nm = fminf(desired_brake_torque_Nm, max_brake_torque_Nm);

    // Clamping desired_brake_torque_Nm as per motor limit.
    desired_brake_torque_Nm = fminf(desired_brake_torque_Nm, maxTorque_continuous_stall_Nm);

    return (int32_t)(-desired_brake_torque_Nm * 1000.0f);
}

float test() {
	return 42.42f;
}
