/**
 * @file pwm.h
 * @brief Board-specific PWM interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef PWM_H
#define PWM_H

#include <CMR/pwm.h>    // PWM interface

/**
 * @brief Represents a PWM channel.
 *
 * @warning New channels MUST be added before `PWM_LEN`.
 */
typedef enum {
	PWM_PUMP_RIGHT = 0,    /**< @brief Right pump. */
	PWM_PUMP_LEFT,         /**< @brief Left pump. */
	PWM_LEN     	  /**< @brief Total PWM pins. */
} pwm_t;

void pwmInit(void);
void pwmSetDutyCycle(pwm_t pin, uint32_t dutyCycle_pcnt);

#endif /* PWM_H */

