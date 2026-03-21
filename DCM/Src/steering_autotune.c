/**
 * @file steering_autotune.c
 * @brief Step-response autotuning for the CubeMars steering controller.
 *
 * Method: command an undamped (K_d=0) position step, measure the time to
 * first peak. For a double-integrator plant under PD control with K_d=0,
 * the closed-loop is undamped: t_peak = π/ω_n. From ω_n and the known
 * identification gain we back-calculate the plant gain K, then compute
 * new K_p / K_d for ζ=0.707 at the same natural frequency.
 *
 * Results are stored but NOT applied automatically — call
 * autotuneGetResults() then steeringSetGains() when ready.
 */

#include "steering_autotune.h"

#include <math.h>

#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

#define AT_SAMPLES       200        // 2 s of data at 10 ms/sample
#define AT_PERIOD_S      0.010f     // must match steeringPeriod_ms in steering.c
#define AT_STEP_SIZE     300.0f     // step size in ADC position units
#define AT_ID_KP         2.5f       // |K_p| used during identification
#define AT_CURRENT_LIMIT 8000       // mA cap during identification

// Position limits — match minValue / maxValue in steering.c
#define AT_POS_MIN (-1530.0f)
#define AT_POS_MAX  (1530.0f)

static struct {
    ATPhase_t phase;
    float     pos[AT_SAMPLES];
    uint32_t  n;
    float     target;
    float     initial;
    float     Kp_result;
    float     Kd_result;
    float     K_plant;
} at = { .phase = AT_IDLE };

void autotuneStart(float currPos) {
    at.initial   = currPos;
    at.target    = CLAMP(currPos + AT_STEP_SIZE, AT_POS_MIN, AT_POS_MAX);
    at.n         = 0;
    at.Kp_result = 0.0f;
    at.Kd_result = 0.0f;
    at.phase     = AT_COLLECTING;
}

static void autotuneAnalyze(void) {
    // Find first local maximum in the position trace
    uint32_t peak_idx = 0;
    for (uint32_t i = 1; i < AT_SAMPLES - 1; i++) {
        if (at.pos[i] > at.pos[i-1] && at.pos[i] > at.pos[i+1]) {
            peak_idx = i;
            break;
        }
    }
    if (peak_idx == 0) { at.phase = AT_FAILED; return; }

    // Verify the system actually moved
    if (fabsf(at.target - at.initial) < 10.0f) { at.phase = AT_FAILED; return; }

    // ω_n from peak time (undamped step response: t_peak = π / ω_n)
    float tp      = (float)peak_idx * AT_PERIOD_S;
    float omega_n = (float)M_PI / tp;

    // Back-calculate plant gain K from closed-loop ω_n:
    //   ω_n² = AT_ID_KP * K_plant  (magnitudes)
    float K_plant = (omega_n * omega_n) / AT_ID_KP;
    if (K_plant < 1e-6f) { at.phase = AT_FAILED; return; }

    at.K_plant = K_plant;

    // Desired poles: ζ = 0.707, same ω_n (conservative — don't increase bandwidth)
    const float zeta_d = 0.707f;
    at.Kp_result = -(omega_n * omega_n) / K_plant;  // negative per convention
    at.Kd_result =  (2.0f * zeta_d * omega_n) / K_plant;

    at.phase = AT_DONE;
}

int32_t autotuneTick(float currPos) {
    if (at.phase != AT_COLLECTING) return 0;

    // Stop driving if position hits the physical limits
    if (currPos <= AT_POS_MIN || currPos >= AT_POS_MAX) {
        at.phase = AT_FAILED;
        return 0;
    }

    at.pos[at.n++] = currPos;
    if (at.n >= AT_SAMPLES) {
        autotuneAnalyze();
        return 0;
    }

    // Drive toward step target using fixed identification gains (K_d=0 intentionally)
    float action = -AT_ID_KP * (at.target - currPos);
    return (int32_t)CLAMP(action, -AT_CURRENT_LIMIT, AT_CURRENT_LIMIT);
}

bool autotuneActive(void) {
    return at.phase == AT_COLLECTING;
}

ATPhase_t autotuneGetPhase(void) {
    return at.phase;
}

void autotuneGetResults(float *Kp_out, float *Kd_out) {
    *Kp_out = at.Kp_result;
    *Kd_out = at.Kd_result;
}

float autotuneGetKPlant(void) {
    return at.K_plant;
}
