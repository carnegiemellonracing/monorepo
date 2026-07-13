/**
 * @file main.c
 * @brief Firmware entry point.
 *
 * @author Carnegie Mellon Racing
 */

#include <stm32f4xx_hal.h>  // HAL interface

#include <CMR/panic.h>  // cmr_panic()
#include <CMR/rcc.h>    // RCC interface
#include <CMR/can.h>    // CAN interface
#include <CMR/adc.h>    // ADC interface
#include <CMR/gpio.h>   // GPIO interface
#include <CMR/tasks.h>  // Task interface

#include "adc.h"        // Board-specific ADC interface
#include "assi.h"
#include "can.h"        // Board-specific CAN interface
#include "dac.h"        // Board-specific DAC interface
#include "error.h"
#include "gpio.h"       // Board-specific GPIO interface
#include "sensors.h"    // Board-specific sensors interface
#include "statusLED.h"  // Status LED
#include "state.h"      // stateInit()
#include "pwm.h"
#include "tssi.h"       // TSSI control 


#define DEMCR      (*(volatile uint32_t*)0xE000EDFC)
#define ITM_LAR    (*(volatile uint32_t*)0xE0000FB0)
#define TPIU_SPPR  (*(volatile uint32_t*)0xE00400F0)
#define TPIU_ACPR  (*(volatile uint32_t*)0xE0040010)
#define TPIU_FFCR  (*(volatile uint32_t*)0xE0040304)
#define DWT_CTRL   (*(volatile uint32_t*)0xE0001000)
#define ITM_TCR    (*(volatile uint32_t*)0xE0000E80)
#define ITM_TER    (*(volatile uint32_t*)0xE0000E00)
#define ITM_STIM0 (*(volatile uint32_t*)0xE0000000)

void __attribute__((no_instrument_function))
__cyg_profile_func_enter(void *func, void *caller) {
    //while (!(ITM_TCR & 1)) {} // wait if ITM busy (rare)
    ITM_STIM0 = ((uint32_t)func & ~1u) | 1u; // LSB=1 → enter event
}

void __attribute__((no_instrument_function))
__cyg_profile_func_exit(void *func, void *caller) {
    ITM_STIM0 = ((uint32_t)func & ~1u) | 0u; // LSB=0 → exit event
}

void __attribute__((no_instrument_function))
itm_send_char(char c) {
    while (!(ITM_TCR & 1));           // wait until ITM enabled (should already be)
    while (ITM_STIM0 == 0);           // wait until FIFO has room (stimulus reg readable-as-0 means busy)
    *(volatile uint8_t*)&ITM_STIM0 = (uint8_t)c;  // write single byte to stimulus port 0
}

void __attribute__((no_instrument_function))
itm_send_string(const char *s) {
    while (*s) {
        itm_send_char(*s++);
    }
}

void trace_init(uint32_t core_clock_hz, uint32_t swo_baud) {
    DEMCR |= (1 << 24);              // TRCENA - enable trace subsystem
    ITM_LAR = 0xC5ACCE55;            // unlock ITM

    TPIU_SPPR = 2;                   // NRZ (UART) SWO mode
    TPIU_ACPR = (core_clock_hz / swo_baud) - 1;
    TPIU_FFCR &= ~(1 << 1);          // disable formatter (raw ITM/DWT only)

    ITM_TCR = (1 << 0)  |            // ITMENA
            (1 << 2)  |            // SYNCENA  <-- was missing
            (1 << 3)  |            // TXENA (DWT forwarding)
            (0x01 << 16);          // TraceBusID = 1
    ITM_TER = 0xFFFFFFFF;

    // DWT_CTRL — corrected bit positions:
    DWT_CTRL = (1 << 0)   |          // CYCCNTENA
               (0xF << 1) |          // POSTPRESET = 0xF (tap reload value)
               (0xF << 5) |          // POSTINIT   = 0xF (initial counter value)
               (0 << 9)   |          // CYCTAP = 0 -> tap at bit 6 (finer sample rate)
               (3 << 10)  |          // SYNCTAP = 3 (sync packet rate)
               (1 << 12)  |          // PCSAMPLENA  <-- the actual bit that was missing
               (1 << 16);            // EXCTRCENA (correct position this time)
}

/**
 * @brief Firmware entry point.
 *
 * Device configuration and task initialization should be performed here.
 *
 * @return Does not return.
 */
int main(void) {
    // System initialization.
    HAL_Init();
    cmr_rccSystemClockEnable();
    trace_init(HAL_RCC_GetHCLKFreq(), 1800000);

    // Peripheral configuration.
    gpioInit();
    
    // pwmInit();
    // canInit();
    // adcInit();
    // sensorsInit();
    // stateInit();
    // tssiInit();
    // assiInit();

    statusLEDInit();

    //vTaskStartScheduler();
    //cmr_panic("vTaskStartScheduler returned!");
    while (1) {
        itm_send_string("ITM ALIVE\r\n");
        HAL_Delay(500);
    }
}

