
#ifndef VE_H
#define VE_H

#include <stdint.h>
#include "can.h"

static const uint32_t ve_priority = 7;
static const TickType_t ve_period_ms = 5;

/**
 * @brief Car mass, including driver mass
 * Unit: kg
 */
static const float car_mass = 370;

/**
 * @brief Car weight, including driver weight
 * Unit: N
 */
static const float car_weight = car_mass * 9.81;

/**
 * @brief Aero drag coefficient
 * Unit: N / (m/s)^2
 * Aero drag = v^2 * Dcoe
 */
static const float aero_lift_coe = 0.806638;

/**
 * @brief Aero lift coefficient
 * Unit: N / (m/s)^2
 * Aero lift = v^2 * Lcoe
 */
static const float aero_drag_coe = 0.277213;

/**
 * @brief Distance between front and end axles
 * Unit: m
 * For calculating weight transfer from acceleration
 */
static const float wheelbase = 1.55;

/**
 * @brief Height of center of gravity above ground
 * Unit: m
 * For calculating weight transfer from acceleration
 */
static const float cg_height = 0.3;

/**
 * @brief Distance between tire axle and contact patch
 * Unit: m
 * Look into this. DAQ uses a different value.
 */
static const float effective_radius = 0.239;

// extern volatile cmr_canCDCVelocityEstimation_t velocity_estimator;

extern volatile cmr_canCDCVelocityEstimation1_t velocity_estimator1;
extern volatile cmr_canCDCVelocityEstimation2_t velocity_estimator2;

void veInit();
void veReset();

#endif
