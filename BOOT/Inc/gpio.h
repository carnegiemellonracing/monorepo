/**
 * @file gpio.h
 * @brief Board-specific GPIO interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef GPIO_H
#define GPIO_H

#include <CMR/gpio.h>   // GPIO interface

void gpioInit(void);
void gpioDeinit(void);
void timedLedToggle(void);

#endif /* GPIO_H */

