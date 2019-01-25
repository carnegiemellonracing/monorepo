/**
 * @brief bsrr.c
 * @file GPIO "bit set/reset register" implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "bsrr.h"   // interface to implement

/**
 * @brief Sets and resets the specified pins on the given port.
 *
 * @param port GPIO port to modify (i.e. `GPIOx` from `stm32f413xx.h`);
 * @param set Bitwise-OR of `GPIO_PIN_x` (from `stm32f4xx_hal_gpio.h`) to set.
 * @param reset Bitwise-OR of `GPIO_PIN_x` to reset.
 */
void cmr_bsrr(GPIO_TypeDef *port, uint16_t set, uint16_t reset) {
    port->BSRR = (((uint32_t) reset) << 16) | (uint32_t) set;
}

