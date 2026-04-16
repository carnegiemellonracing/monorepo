/**
 * @file dac.c
 * @brief DAC Implementation
 * 
 * @note DAC should not be used at comp. Use this file just to set the resistor
 * divider for our reference voltage
 * @author Ayush Garg
 */

#include "dac.h"            // Interface to implement

#include <CMR/dac.h>         // DAC interface
#include <CMR/gpio.h>        // GPIO interface


/**
 * @brief Sets Up DAC for Brake Pressure Ref and Hall Effect Ref
 *
 */
void dacInit() {
    const uint32_t BRef_mV = 1000;
    const uint32_t CRef_mV = 1000;

    cmr_gpioPin_t dacPins[DAC_LEN] = {
        [DAC_B_REF] = {
            .port = GPIOA,
            .pin = GPIO_PIN_5 
        },
        [DAC_C_REF] = {
            .port = GPIOA,
            .pin = GPIO_PIN_4 
        }
    };
    cmr_dacInit(dacPins, DAC_LEN);
    cmr_dacSetValue(DAC_B_REF, BRef_mV);
    cmr_dacSetValue(DAC_C_REF, CRef_mV);
}