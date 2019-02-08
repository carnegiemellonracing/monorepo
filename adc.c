/**
 * @file adc.c
 * @brief Board-specific ADC implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <FreeRTOS.h>       // FreeRTOS interface
#include <task.h>           // xTaskCreate()

#include "adc.h"    // Interface to implement


const cmr_adcChannel_t *adcChannels[ADC_LEN] = {
	[ADC_VSENSE] = NULL,
	[ADC_ISENSE] = NULL,
};

/** @brief Primary ADC. */
static cmr_adc_t adc;

/** @brief ADC sampling priority. */
static const uint32_t adcSamplePriority = 5;

/** @brief ADC sampling period (milliseconds). */
static const TickType_t adcSamplePeriod_ms = 10;

/**
 * @brief Task for sampling the ADC.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void adcSampleTask(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        cmr_adcSample(&adc);

        vTaskDelayUntil(&lastWakeTime, adcSamplePeriod_ms);
    }
}

/**
 * @brief Initializes the ADC interface.
 */
void adcInit(void) {
    // ADC initialization and channel configuration.
	// XXX edit me to match your pin configuration
    cmr_adcInit(&adc, ADC1);
    adcChannels[ADC_VSENSE] = cmr_adcAddChannel(
        &adc, ADC_CHANNEL_0,
        GPIOA, GPIO_PIN_0,
        ADC_SAMPLETIME_15CYCLES
    );
    adcChannels[ADC_ISENSE] = cmr_adcAddChannel(
        &adc, ADC_CHANNEL_1,
        GPIOA, GPIO_PIN_1,
        ADC_SAMPLETIME_15CYCLES
    );

    // Task creation.
    xTaskCreate(
        adcSampleTask, "adcSample",
        configMINIMAL_STACK_SIZE, NULL, adcSamplePriority, NULL
    );
}

