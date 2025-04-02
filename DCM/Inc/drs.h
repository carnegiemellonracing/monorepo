/**
 * @file drs.h
 * @brief DRS Servo Interface
 *
 * @author Carnegie Mellon Racing
 */

// ------------------------------------------------------------------------------------------------
// Includes

#ifndef DRS_H
#define DRS_H
#include <stdbool.h>
#include <stddef.h>

#include <CMR/can_ids.h>    // CMR CAN IDs
#include <CMR/can_types.h>

// ------------------------------------------------------------------------------------------------
// Definitions 

#define DRS_DUTY_CYCLE_RANGE 8
#define DRS_CLOSED_ANGLE 80
#define DRS_OPENED_ANGLE 145

#define LAT_G_UPPER_THRESH 1.2
#define LAT_G_LOWER_THRESH 0.8

// ------------------------------------------------------------------------------------------------
// Public function declarations
void servoInit(void);
void setServoQuiet(void);
uint32_t angleToDutyCycle(int angle);
float calculate_latg(int16_t swAngle_millideg, float velocity_mps);
void processDRSControl(int16_t swAngle_millideg, bool braking, 
    bool traction_limited, bool skidpad);
void setDRS(bool open);
#endif
