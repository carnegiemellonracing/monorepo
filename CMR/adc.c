/**
 * @file adc.c
 * @brief Analog-to-digtal converter wrapper implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "adc.h"    // Interface to implement

#ifdef HAL_ADC_MODULE_ENABLED

#include "rcc.h"    // cmr_rccADCClockEnable(), cmr_rccGPIOClockEnable()
#include "panic.h"  // cmr_panic()

/** @brief Timeout (in ms) for each ADC channel sample poll. */
static const uint32_t CMR_ADC_TIMEOUT_MS = 1;

/**
 * @brief Initializes the ADC.
 *
 * @warning It is undefined behavior to initialize the same HAL ADC instance
 * more than once!
 *
 * @param adc The ADC to initialize.
 * @param instance The HAL ADC instance (`ADCx` from `stm32f413xx.h`).
 */
void cmr_adcInit(cmr_adc_t *adc, ADC_TypeDef *instance) {
    *adc = (cmr_adc_t) {
        .handle = {
            .Instance = instance,

            // Configure ADC in discontinuous scan mode.
            // This will allow conversion of a series of channels one at a time.
            .Init = {
                .ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4,
                .Resolution = ADC_RESOLUTION_12B,
                .ScanConvMode = ENABLE,
                .ContinuousConvMode = DISABLE,
                .DiscontinuousConvMode = ENABLE,
                .NbrOfDiscConversion = 1,
                .ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE,
                .ExternalTrigConv = ADC_SOFTWARE_START,
                .DataAlign = ADC_DATAALIGN_RIGHT,
                .NbrOfConversion = CMR_ADC_CHANNELS,
                .DMAContinuousRequests = DISABLE,
                .EOCSelection = ADC_EOC_SINGLE_CONV
            }
        },

        .channelsUsed = 0,
        .channels = { { 0 } }
    };

    cmr_rccADCClockEnable(instance);

    if (HAL_ADC_Init(&adc->handle) != HAL_OK) {
        cmr_panic("HAL_ADC_Init() failed!");
    }
}

/**
 * @brief Adds a channel to sample.
 *
 * @note See RM0430 13.7 for minimum sample times.
 *
 * @warning It is undefined behavior to call this function with the same index
 * OR channel more than once!
 *
 * @param adc The ADC to configure.
 * @param channel The ADC channel to configure (`ADC_CHANNEL_x`, 0 <= x <= 15).
 * @param port Channel's GPIO port (`GPIOx` from `stm32f413xx.h`).
 * @param pin Channel's GPIO pin (`GPIO_PIN_x` from `stm32f4xx_hal_gpio.h`).
 * @param samplingTime Sampling time for ADC channel
 * (`ADC_SAMPLETIME_xCYCLES`, from `stm32f4xx_hal_adc.h`).
 *
 * @returns A reference to the configured channel.
 */
const cmr_adcChannel_t *cmr_adcAddChannel(
    cmr_adc_t *adc, uint32_t channel,
    GPIO_TypeDef *port, uint16_t pin,
    uint32_t samplingTime
) {
    if (channel > ADC_CHANNEL_15) {
        cmr_panic("Invalid ADC channel!");
    }

    size_t index = adc->channelsUsed;
    if (index >= CMR_ADC_CHANNELS) {
        cmr_panic("Too many ADC channels!");
    }

    ADC_ChannelConfTypeDef channelConfig = {
        .Channel = channel,
        .Rank = index + 1,  // HAL needs Rank to be from 1 to 16
        .SamplingTime = samplingTime,
        .Offset = 0     // reserved, set to 0
    };

    if (HAL_ADC_ConfigChannel(&adc->handle, &channelConfig) != HAL_OK) {
        cmr_panic("HAL_ADC_ConfigChannel() failed!");
    }

    cmr_rccGPIOClockEnable(port);

    // Configure the pin for analog use.
    GPIO_InitTypeDef pinConfig = {
        .Pin = pin,
        .Mode = GPIO_MODE_ANALOG,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_LOW,
        .Alternate = 0
    };
    HAL_GPIO_Init(port, &pinConfig);

    adc->channelsUsed++;
    return &adc->channels[index];
}

/**
 * @brief Samples new values from the ADC.
 *
 * @param adc The ADC to sample.
 */
void cmr_adcSample(cmr_adc_t *adc) {
    // ADC set up in discontinuous scan mode.
    // Each `HAL_ADC_Start()` call converts the next-highest-rank channel.
    for (uint32_t i = 0; i < adc->channelsUsed; i++) {
        HAL_ADC_Start(&adc->handle);
        HAL_ADC_PollForConversion(&adc->handle, CMR_ADC_TIMEOUT_MS);
        adc->channels[i].value = HAL_ADC_GetValue(&adc->handle);
    }
}

#endif /* HAL_ADC_MODULE_ENABLED */

