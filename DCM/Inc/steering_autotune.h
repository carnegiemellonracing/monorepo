/**
 * @file steering_autotune.h
 * @brief Step-response autotuning for the CubeMars steering controller.
 *
 * Usage:
 *   1. autotuneStart(currPos)       — begin identification
 *   2. Each 10 ms tick: autotuneTick(currPos) — returns current command (mA)
 *   3. Poll autotuneGetPhase() for AT_DONE / AT_FAILED
 *   4. autotuneGetResults(&kp, &kd) — read suggested gains
 *   5. Manually inspect/adjust, then steeringSetGains(kp, kd) to apply
 */

#ifndef STEERING_AUTOTUNE_H
#define STEERING_AUTOTUNE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum { AT_IDLE, AT_COLLECTING, AT_DONE, AT_FAILED } ATPhase_t;

void      autotuneStart(float currPos);
int32_t   autotuneTick(float currPos);
bool      autotuneActive(void);
ATPhase_t autotuneGetPhase(void);
void      autotuneGetResults(float *Kp_out, float *Kd_out);
float     autotuneGetKPlant(void);

#endif /* STEERING_AUTOTUNE_H */
