/**
 * @file steering_dob.h
 * @brief Disturbance Observer (DOB) for the CubeMars steering controller.
 *
 * Estimates and cancels disturbance torques (friction, road feedback) acting
 * on the steering rack in real time, without requiring an explicit friction
 * model.
 *
 * Usage:
 *   1. Run autotuning to identify K_plant.
 *   2. dobInit(omega_q, K_plant) — choose omega_q < closed-loop bandwidth.
 *      A safe default: omega_q = omega_n / 3  (omega_n = sqrt(K_plant * |K_p|))
 *   3. Each tick: u_total = u_pd + dobTick(u_pd, pos)
 *   4. dobReset() if the controller is disabled or gains change.
 */

#ifndef STEERING_DOB_H
#define STEERING_DOB_H

#include <stdbool.h>
#include <stdint.h>

// Initialize the DOB with a chosen bandwidth and identified plant gain.
// omega_q : DOB filter bandwidth (rad/s) — keep below closed-loop bandwidth
// K_plant : plant gain from autotuneGetKPlant()
void  dobInit(float omega_q, float K_plant);

// Call each tick with the PD control output and current position.
// Returns the disturbance estimate (mA) to add to u_pd.
float dobTick(float u_pd, float pos);

void  dobReset(void);
bool  dobIsActive(void);

#endif /* STEERING_DOB_H */
