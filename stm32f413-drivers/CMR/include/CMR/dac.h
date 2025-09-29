#pragma once

/**
 * @file dac.h
 * @brief DAC Interface
 *
 * @author Carnegie Mellon Racing
 */

#ifdef HAL_DAC_MODULE_ENABLED

/*
The DACs of the stms are always on the A port. For this reason it suffices to input
an array of the pins. For example if you wish to activate the DAC at pins PA4 and PA5
simply input an array of [4,5]
*/
void cmr_dacInit(uint16_t* pins, size_t dacConfigsLen);
void cmr_dacSetValue(uint16_t pin, uint16_t voltage_mV);

#endif HAL_DAC_MODULE_ENABLED

