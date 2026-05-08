/**
 * @file gpio.h
 * @brief Board-specific GPIO interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef GPIO_H
#define GPIO_H

#include <CMR/gpio.h>   // GPIO interface
#include <stdbool.h>

void gpioInit(void);
void gpioDeinit(void);
void timedLedToggle(void);
bool getPushButton(void);

#endif /* GPIO_H */

