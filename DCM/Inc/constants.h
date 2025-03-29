#ifndef __CONSTANTS_H
#define __CONSTANTS_H

#include <stdint.h>

/** @brief Maximum motor torque
 * motor datasheet: "maximum torque 'Mmax': 21 Nm"
*/
static const float maxTorque_Nm = 21.0f;

/** @brief Maximum motor speed
 * motor datasheet: "no-load-speed 'No': 18617"
 * motor datasheet: "mech. speed limit 'Nmax': 20000"
 * AMK racing kit summary p.16: "Speed setpoint values are limited to 30000 rpm"
*/
static const int16_t maxSpeed_rpm = 20000;

/** @brief Maximum motor torque in slow gear */
static const float maxSlowTorque_Nm = 5.0f;

static const float maxTorque_continuous_stall_Nm = 13.8f;

/** @brief Maximum motor torque in fast gear */
static const float maxFastTorque_Nm = maxTorque_continuous_stall_Nm; 

/** @brief Maximum motor speed in slow gear. Roughly 10 MPH.
 *  @details WolframAlpha query: (convert 10mph to meters/second) /
 *                               (((convert 18 inches to meters) / rev) * (pi revolutions / rev)) *
 *                               (60 seconds / 1 minute) * (15 revolutions / rev)
 */
static const int16_t maxSlowSpeed_rpm = 1500;

/** @brief Maximum motor speed in medium gear. Roughly 20m/s */
static const int16_t maxMediumSpeed_rpm = 13000;

/** @brief Maximum motor speed in fast gear */
static const int16_t maxFastSpeed_rpm = maxSpeed_rpm;

/** @brief horizontal distance between the center of the front wheels and the origin of the SBG frame in meters */
static const float chassis_a = 0.775f;

/** @brief horizontal distance between the center of the rear wheels and the origin of the SBG frame in meters*/
static const float chassis_b = 0.775f;

/** @brief horizontal distance between the front wheels in meters */
static const float chassis_w_f = 0.625; //Confirm with CAD people

/** @brief horizontal distance between the rear wheels in meters */
static const float chassis_w_r = 0.625; //Confirm with CAD people

// Gear ratio is defined as (TOP + BOTTOM) / (BOTTOM)
static const int32_t gear_ratio_top = 8784;
static const int32_t gear_ratio_bot = 621;

static const float gear_ratio = 13.93; //updated for 24e
static const float effective_wheel_dia_m = 0.43; /** @brief effective wheel diameter */
static const float effective_wheel_rad_m = effective_wheel_dia_m * 0.5f; /** @brief effective wheel radius */

static const double wheelbase_m = 1.55f;
static const double trackwidth_m = 1.30f;
static const double half_wheelbase_m = wheelbase_m * 0.5f;
static const double half_trackwidth_m= trackwidth_m * 0.5f;
static const double car_mass_kg = 370.0f;

static const float ac_max_capacity_J = 7 * 3.6e6; // 7 kWh. Replace with more accurate value.
static const float power_regulator_ac_margin_percentage = 0.15;
static const float power_regulator_ac_margin_J = ac_max_capacity_J * power_regulator_ac_margin_percentage;
static const float endurance_half_length_m = 0.5 * 22e3; // 11 km.
static const float odometer_correction_factor = 1.0; // Slip compensation.

#endif
