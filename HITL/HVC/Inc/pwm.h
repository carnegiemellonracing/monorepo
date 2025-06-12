/**
  * @file pwm.h
  * @brief Header for PWM interface on HAL
  * @author Carnegie Mellon Racing
**/
#ifndef PWM_H
#define PWM_H


#include <CMR/pwm.h>
#include <CMR/panic.h>
#include <stdlib.h>


//TODO: Change this value and add ones for each timer
#define TIMER_CLOCK_FREQ 1000000

void pwmReadInit(void);
void pwmRead(uint8_t tim, uint32_t *frequency, uint32_t *duty_cycle);

#endif