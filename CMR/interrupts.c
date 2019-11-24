/**
 * @file interrupts.c
 * @brief Mandatory common interrupt handler implementations.
 *
 * These definitions override the default handlers for each interrupt, which
 * spins forever.
 *
 * The default handlers are (weakly) defined in `CMSIS/startup_stm32f413xx.s`.
 *
 * @author Carnegie Mellon Racing
 */

#include <stm32l4xx_hal.h>  // HAL_IncTick()
#include <FreeRTOS.h>   // FreeRTOS interface
#include <task.h>       // xTaskGetSchedulerState()

/**
 * @brief Port-specific system tick handler; provided by FreeRTOS.
 *
 * @see FreeRTOS/portable/GCC/ARM_CM4F/port.c
 */
extern void xPortSysTickHandler(void);

/**
 * @brief System tick interrupt handler.
 */
void SysTick_Handler(void) {
    HAL_IncTick();  // Report tick to HAL.

    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return;
    }

    xPortSysTickHandler();  // Invoke FreeRTOS tick handler.
}

/**
 * @brief Non-maskable interrupt handler.
 */
void NMI_Handler(void) {
    // Nothing to do.
}

/**
 * @brief Debug monitor exception handler.
 */
void DebugMon_Handler(void) {
    // Nothing to do.
}

