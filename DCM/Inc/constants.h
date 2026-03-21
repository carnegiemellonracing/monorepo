#ifndef __CONSTANTS_H
#define __CONSTANTS_H

#include <stdint.h>

/** @brief Maximum motor torque
 * motor datasheet: "Peak Torque - 31.6 Nm"
*/
static const float maxTorque_Nm = 31.6f;
static const float minTorqueLUTVal_Nm = 2.6f;

/** @brief Maximum motor speed
 * motor datasheet: "Nominal Speed - 13250 rpm"
 * motor datasheet: "Maximum Speed - 20000 rpm"
*/
static const int16_t maxSpeed_rpm = 20000;

/** @brief Maximum motor torque in slow gear */
static const float maxSlowTorque_Nm = 5.0f;

static const float maxTorque_continuous_stall_Nm = 31.6f;

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

//Accel constants
//General constants
static const float gear_ratio = 12.097;
static const float wheel_radius = 0.2032;
static const float slip_ratio_front = 0.12;
static const float slip_ratio_rear = 0.15;

//Pacejka tire coefficients
static const float pacejka_coeffs_605[4] = {11.6, 1.59, 1.7, 0.45};
static const float pacejka_coeffs_718[4] = {11.8, 1.55, 1.59, 0.45};
static const float pacejka_coeffs_954[4] = {10, 1.76, 1.66, 0.4};
static const float pacejka_coeffs_1160[4] = {8.4, 2.0, 1.6, 0.4};

//Torque saturations
static const float torque_min = 0.0;
static const float torque_max = 31.0;

//Power Limits
static const int total_power_limit = 80000;
static const float motor_eff = 0.92;

//PI controller gains
static const float kp = 80.0;
static const float ki = 40.0;
static const float dt = 0.001;

//Ramp function
static const float torque_ramp_up = 1000.0;
static const float torque_ramp_down = 1000.0;

//Launch/transition
static const float launch_speed = 1.0;
static const float blend_speed = 3.0;
static const float launch_torque_rear = 31.0;

static const float effective_wheel_rad_m = 0.205f; // meters
static const float effective_wheel_dia_m = 2 * effective_wheel_rad_m; // meters

static const double wheelbase_m = 1.55f;
static const double trackwidth_m = 1.30f;
static const double half_wheelbase_m = wheelbase_m * 0.5f;
static const double half_trackwidth_m= trackwidth_m * 0.5f;
static const double car_mass_kg = 280.0f;

static const uint16_t pole_pairs = 4;

typedef struct {
    int16_t current_Arms;  
    float torque_Nm;    
} MotorData;

#define DTI_TORQUE_CURRENT_LUT_LEN 13
/** 
 * @brief Lookup table mapping Drivetrain Innovation motor AC current to motor torque.
 *
 * Sourced from the F-MOT-A (2025) motor specification.
 */
static const MotorData DTI_torque_current_LUT[DTI_TORQUE_CURRENT_LUT_LEN] = {
    {5, 2.6f},
    {11, 5.6f},
    {16, 8.2f},
    {21, 10.6f},
    {26, 13.1f},
    {31, 15.5f},
    {36, 17.9f},
    {41, 20.4f},
    {46, 22.7f},
    {51, 25.0f},
    {56, 27.3f},
    {61, 29.5f},
    {66, 31.6f}
};

#endif
