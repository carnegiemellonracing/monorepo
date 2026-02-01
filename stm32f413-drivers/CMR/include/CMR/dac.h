#pragma once

/**
 * @file dac.h
 * @brief DAC Interface
 *
 * @author Ayush Garg
 */

 #include "platform.h"
#include <CMR/gpio.h>  // gpio drivers
#include "stdint.h"

#ifdef HAL_DAC_MODULE_ENABLED
#ifdef F413

void cmr_dacInit(cmr_gpioPin_t* pins, size_t dacConfigsLen);
void cmr_dacSetValue(size_t pin, uint16_t voltage_mV);

#endif /* F413 */ 
#endif /* HAL_DAC_MODULE_ENABLED */ 

