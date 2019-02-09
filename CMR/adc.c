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

/** @brief Array of adcChannel structs */
static cmr_adcChannel_t *adcChannels = NULL;

/** @brief Length of adcChannels array */
static size_t adcChannelsLen = 0;

/** Forward declaration */
static void cmr_adcConfigChannels(cmr_adc_t *adc);

/**
 * @brief Initializes the ADC.
 *
 * @warning It is undefined behavior to initialize the same HAL ADC instance
 * more than once!
 *
 * @param adc The ADC to initialize.
 * @param instance The HAL ADC instance (`ADCx` from `stm32f413xx.h`).
 * @param channels Array of adcChannel structs.
 * @param channelsLen Length of channels array.
 */
void cmr_adcInit(cmr_adc_t *adc, ADC_TypeDef *instance, cmr_adcChannel_t *channels,
                 const size_t channelsLen) {
    if (channelsLen > CMR_ADC_CHANNELS) {
        cmr_panic("Too many channels");
    }

    adcChannels = channels;
    adcChannelsLen = channelsLen;

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
        }
    };

    cmr_rccADCClockEnable(instance);

    if (HAL_ADC_Init(&adc->handle) != HAL_OK) {
        cmr_panic("HAL_ADC_Init() failed!");
    }

    cmr_adcConfigChannels(adc);
}

/**
 * @brief Initializes ADC channel sequence.
 *
 * @param adc The ADC to configure.
 */
static void cmr_adcConfigChannels(cmr_adc_t *adc) {
    if (adcChannels == NULL || adcChannelsLen == 0) {
        cmr_panic("Array not configured");
    }

    for (size_t i = 0; i < adcChannelsLen; i++) {
        if (adcChannels[i].channel > ADC_CHANNEL_15) {
            cmr_panic("Invalid ADC channel!");
        }

        ADC_ChannelConfTypeDef channelConfig = {
            .Channel = adcChannels[i].channel,
            .Rank = i + 1,  // HAL needs Rank to be from 1 to 16
            .SamplingTime = adcChannels[i].samplingTime,
            .Offset = 0     // reserved, set to 0
        };

        if (HAL_ADC_ConfigChannel(&adc->handle, &channelConfig) != HAL_OK) {
            cmr_panic("HAL_ADC_ConfigChannel() failed!");
        }

        // Configure the pin for analog use.
        cmr_rccGPIOClockEnable(adcChannels[i].port);

        GPIO_InitTypeDef pinConfig = {
            .Pin = adcChannels[i].pin,
            .Mode = GPIO_MODE_ANALOG,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW,
            .Alternate = 0
        };

        HAL_GPIO_Init(adcChannels[i].port, &pinConfig);
    }
}

/**
 * @brief Samples new values from the ADC.
 *
 * @param adc The ADC to sample.
 */
void cmr_adcSample(cmr_adc_t *adc) {
    // ADC set up in discontinuous scan mode.
    // Each `HAL_ADC_Start()` call converts the next-highest-rank channel.
    for (size_t i = 0; i < adcChannelsLen; i++) {
        HAL_ADC_Start(&adc->handle);
        HAL_ADC_PollForConversion(&adc->handle, CMR_ADC_TIMEOUT_MS);
        adcChannels[i].value = HAL_ADC_GetValue(&adc->handle);
    }
}

#endif /* HAL_ADC_MODULE_ENABLED */

