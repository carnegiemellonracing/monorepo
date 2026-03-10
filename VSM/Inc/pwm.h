/**
 * @file pwm.h
 * @brief Board-specific PWM interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef PWM_H
#define PWM_H

#include <CMR/pwm.h>    // ADC interface

/**
 * @brief Represents an ADC channel.
 *
 * @warning New channels MUST be added before `ADC_LEN`.
 */
typedef enum {
	PWM_GREEN = 0,    /**< @brief Green TSSI. */
	PWM_RED,          /**< @brief Red TSSI. */
    PWM_YELLOW,       /**< @brief Yellow ASSI. */
	PWM_BLUE,		  /**< @brief Blue ASSI */
	PWM_LEN     	  /**< @brief Total PWM pins. */
} pwm_t;

void pwmInit(void);
void pwmSetDutyCycle(pwm_t pin, uint32_t dutyCycle_pcnt);

#endif /* PWM_H */

