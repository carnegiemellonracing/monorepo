/**
 * @file adc.h
 * @brief Board-specific ADC interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef ADC_H
#define ADC_H

#include <CMR/adc.h>    // ADC interface

/**
 * @brief Represents an ADC channel.
 *
 * @warning New channels MUST be added before `ADC_LEN`.
 */
typedef enum {
	ADC_POWER_VSENSE = 0,	/**< @brief Board voltage sense.		*/
	ADC_POWER_ISENSE,		/**< @brief Board current sense.		*/
	ADC_LOGIC_VSENSE,		/**< @brief Logic voltage sense.		*/
	ADC_LOGIC_ISENSE,		/**< @brief Logic current sense.		*/
	ADC_BOARD_THERM_1,		/**< @brief On-board thermistor 1.		*/
	ADC_BOARD_THERM_2,		/**< @brief On-board thermistor 2.		*/
	ADC_RAD_THERM_1,		/**< @brief Pre-radiator thermistor.	*/
	ADC_RAD_THERM_2,		/**< @brief Post-radiator thermistor.	*/
	ADC_FAN_ISENSE,			/**< @brief	Isense from fan driver		*/
	ADC_LEN     /**< @brief Total ADC channels. */
} adcChannels_t;

extern cmr_adcChannel_t adcChannels[ADC_LEN];

void adcInit(void);

#endif /* ADC_H */

