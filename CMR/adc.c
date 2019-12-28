/**
 * @file adc.c
 * @brief Analog-to-digital converter wrapper implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "adc.h"    // Interface to implement

#ifdef HAL_ADC_MODULE_ENABLED

#include "rcc.h"    // cmr_rccADCClockEnable(), cmr_rccGPIOClockEnable()
#include "panic.h"  // cmr_panic()

/** @brief Timeout (in ms) for each ADC channel sample poll. */
static const uint32_t CMR_ADC_TIMEOUT_MS = 1;

/** @brief ADC sampling priority. */
static const uint32_t cmr_adcSample_priority = 5;

/** @brief ADC sampling period (milliseconds). */
static const TickType_t cmr_adcSample_period_ms = 10;

/**
 * @brief Task for sampling the ADC.
 *
 * @param pvParameters (cmr_adc_t *) The ADC.
 *
 * @return Does not return.
 */
static void cmr_adcSample(void *pvParameters) {
    cmr_adc_t *adc = (cmr_adc_t *) pvParameters;

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        // ADC set up in discontinuous scan mode.
        // Each `HAL_ADC_Start()` call converts the next-highest-rank channel.
        for (size_t i = 0; i < adc->channelsLen; i++) {
            cmr_adcChannel_t *channel = &(adc->channels[i]);

            HAL_ADC_Start(&adc->handle);
            HAL_ADC_PollForConversion(&adc->handle, CMR_ADC_TIMEOUT_MS);
            channel->value = HAL_ADC_GetValue(&adc->handle);
        }

        vTaskDelayUntil(&lastWakeTime, cmr_adcSample_period_ms);
    }
}

/**
 * @brief Initializes ADC channel sequence.
 *
 * @param adc The ADC to configure.
 */
static void cmr_adcConfigChannels(cmr_adc_t *adc) {
    for (size_t i = 0; i < adc->channelsLen; i++) {
        const cmr_adcChannel_t *channel = adc->channels + i;
        if (channel->channel > ADC_CHANNEL_15) {
            cmr_panic("Invalid ADC channel!");
        }

        ADC_ChannelConfTypeDef channelConfig = {
            .Channel = channel->channel,
            .Rank = i + 1,  // HAL needs Rank to be from 1 to 16
            .SamplingTime = channel->samplingTime,
            .Offset = 0     // reserved, set to 0
        };

        if (HAL_ADC_ConfigChannel(&adc->handle, &channelConfig) != HAL_OK) {
            cmr_panic("HAL_ADC_ConfigChannel() failed!");
        }

        // Configure the pin for analog use.
        cmr_rccGPIOClockEnable(channel->port);

        GPIO_InitTypeDef pinConfig = {
            .Pin = channel->pin,
            .Mode = GPIO_MODE_ANALOG,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW,
            .Alternate = 0
        };

        HAL_GPIO_Init(channel->port, &pinConfig);
    }
}

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
void cmr_adcInit(
    cmr_adc_t *adc, ADC_TypeDef *instance,
    cmr_adcChannel_t *channels, const size_t channelsLen
) {
    if (channelsLen > CMR_ADC_CHANNELS) {
        cmr_panic("Too many channels");
    }

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
                .NbrOfConversion = channelsLen,
                .DMAContinuousRequests = DISABLE,
                .EOCSelection = ADC_EOC_SINGLE_CONV
            }
        },
        .channels = channels,
        .channelsLen = channelsLen
    };

    cmr_rccADCClockEnable(instance);

    if (HAL_ADC_Init(&adc->handle) != HAL_OK) {
        cmr_panic("HAL_ADC_Init() failed!");
    }

    cmr_adcConfigChannels(adc);

    cmr_taskInit(
        &adc->sampleTask,
        "ADC sample",
        cmr_adcSample_priority,
        cmr_adcSample,
        adc
    );
}

/**
 * @brief Reads the specified ADC channel.
 *
 * @param adc The ADC to read from.
 * @param channel The channel's index.
 *
 * @return The most recent sample for that channel.
 */
uint32_t cmr_adcRead(cmr_adc_t *adc, size_t channel) {
    return adc->channels[channel].value;
}

#endif /* HAL_ADC_MODULE_ENABLED */

