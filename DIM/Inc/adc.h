/**
 * @file adc.h
 * @brief Board-specific ADC interface.
 *
 * @author Carnegie Mellon Racing
 */


#pragma once
#include <CMR/adc.h>  // ADC interface

/**
 * @brief Represents an ADC channel.
 *
 * @warning New channels MUST be added before `ADC_LEN`.
 */
typedef enum {
    ADC_TPOS_L = 0,         /**< @brief Sensor channel 1. - TPOS L */
    ADC_TPOS_R,             /**< @brief Sensor channel 2. - TPOS R */
    ADC_EBS_AIR_PRES_1,     /**< @brief Sensor channel 3. - EBS_TANK_PRES_1 */
    ADC_EBS_AIR_PRES_2,     /**< @brief Sensor channel 4. - EBS_TANK_PRES_2 */
    ADC_BPRES,              /**< @brief Sensor channel 5. - BPRES */
    ADC_SWANGLE,            /**< @brief Sensor channel 6. - SWANGLE */
    ADC_PADDLE,             /**< @brief Sensor Channel 7. - PADDLE */
    ADC_EBS_CURRENT_1,      /**< @brief Sensor channel 8. - EBS_CURRENT_1 */
    ADC_EBS_CURRENT_2,      /**< @brief Sensor channel 9. - EBS_CURRENT_2 */
    ADC_LEN                 /**< @brief Total ADC channels. */
} adcChannel_t;

void adcInit(void);
uint32_t adcRead(adcChannel_t channel);
