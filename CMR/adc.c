/**
 * @file adc.c
 * @brief Analog-to-digtal converter wrapper implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "adc.h"    // Interface to implement

/** @brief Timeout (in ms) for each ADC channel sample poll. */
static const uint32_t CMR_ADC_TIMEOUT_MS = 1;

/**
 * @brief Initializes the ADC.
 *
 * @param adc The ADC to initialize.
 * @param instance The HAL ADC instance (e.g. `ADC1`).
 *
 * @return 0 on success, or a negative error code.
 */
HAL_StatusTypeDef cmr_adcInit(cmr_adc_t *adc, ADC_TypeDef *instance) {
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
                .NbrOfConversion = NUM_ADC_CHANNELS,
                .DMAContinuousRequests = DISABLE,
                .EOCSelection = ADC_EOC_SINGLE_CONV
            }
        },

        channelsUsed = 0,
        samples = { 0 }
    };

    __HAL_RCC_ADC1_CLK_ENABLE();

    if (HAL_ADC_Init(&adc->handle) != HAL_OK) {
        return -1;
    }

    return 0;
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
 * @param index The sampling index.
 * @param channel The ADC channel to configure (`ADC_CHANNEL_x`, 0 <= x <= 15).
 * @param samplingTime Sampling time for ADC channel
 * (`ADC_SAMPLE_TIME_xCYCLES`, from `stm32f4xx_hal_adc.h`).
 * @param port Channel's GPIO port (`GPIOx` from `stm32f413xx.h`).
 * @param pin Channel's GPIO pin (`GPIO_PIN_x` from `stm32f4xx_hal_gpio.h`).
 *
 * @return 0 on success, or a negative error code.
 */
int cmr_adcAddChannel(cmr_adc_t *adc, size_t index,
                      uint32_t channel, uint32_t samplingTime,
                      GPIO_TypeDef *port, uint16_t pin) {
    if (channel > ADC_CHANNEL_15) {
        return -1;  // Invalid ADC channel.
    }

    if (adc->channelsUsed >= CMR_ADC_CHANNELS) {
        return -1;  // Too many channels.
    }

    ADC_ChannelConfTypeDef channelConfig = {
        .Channel = channel,
        .Rank = index + 1,  // HAL needs Rank to be from 1 to 16
        .SamplingTime = samplingTime,
        .Offset = 0     // reserved, set to 0
    };

    if (HAL_ADC_ConfigChannel(&adcHandle, &channelConfig) != HAL_OK) {
        return -1;
    }

    // Enable the appropriate port's GPIO clock.
    switch (port) {
        case GPIOA:
            __HAL_RCC_GPIOA_CLK_ENABLE();
            break;
        case GPIOB:
            __HAL_RCC_GPIOB_CLK_ENABLE();
            break;
        case GPIOC:
            __HAL_RCC_GPIOC_CLK_ENABLE();
            break;
        case GPIOD:
            __HAL_RCC_GPIOD_CLK_ENABLE();
            break;
        case GPIOE:
            __HAL_RCC_GPIOE_CLK_ENABLE();
            break;
        case GPIOF:
            __HAL_RCC_GPIOF_CLK_ENABLE();
            break;
        case GPIOG:
            __HAL_RCC_GPIOG_CLK_ENABLE();
            break;
        case GPIOH:
            __HAL_RCC_GPIOH_CLK_ENABLE();
            break;
    }

    // Configure the pin for analog use.
    GPIO_InitTypeDef pinConfig = {
        .Pin = pin,
        .Mode = GPIO_MODE_ANALOG,
        .Pull = GPIO_NOPULL,
        .Offset = 0     // Reserved; set to 0.
    };
    HAL_GPIO_Init(port, &pinConfig);

    adc->channelsUsed++;

    return 0;
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
        adc->samples[i] = HAL_ADC_GetValue(&adc->handle);
    }
}

/**
 * @brief Gets the latest value from the specified ADC channel.
 *
 * @param adc The ADC to access.
 * @param index The channel's index.
 *
 * @return The latest sampled value.
 */
uint32_t cmr_adcGet(const cmr_adc_t *adc, size_t index) {
    return adc->samples[index];
}

