/**
 * @file servo.h
 * @brief DRS Servo Interface
 *
 * @author Carnegie Mellon Racing
 */

// ------------------------------------------------------------------------------------------------
// Includes

#ifndef SERVO_H
#define SERVO_H
#include <stdbool.h>
#include <stddef.h>

#include <CMR/can_ids.h>    // CMR CAN IDs
#include <CMR/can_types.h>

// ------------------------------------------------------------------------------------------------
// Defintions 

#define DRS_DUTY_CYCLE_RANGE 8
#define DRS_MAX_DUTY_CYCLE 96
#define DRS_MIN_DUTY_CYCLE (DRS_MAX_DUTY_CYCLE - DRS_DUTY_CYCLE_RANGE)

#define DRS_OPEN_DUTY_CYCLE (DRS_DUTY_CYCLE_RANGE)  
#define DRS_CLOSED_DUTY_CYCLE (0)

// ------------------------------------------------------------------------------------------------
// Public function declarations
void servoInit(void);
void setServoQuiet(void);
void setDrsPosition(uint32_t angle);
#endif
