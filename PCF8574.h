#ifndef I2C_H
#define I2C_H

#include <stdbool.h>
#include <stdint.h>

#define I2C_TIMEOUT 10

#define PCF8574_EXPANDER_ADDR 0b0100000 // {0}{1}{0}{0}{A2}{A1}{A0}
#define I2C_ROTSEL 3


bool PCF8574Init();
bool PCF8574Configure();

int PCF8574_expanderRead(uint8_t *data);
bool PCF8574_expanderWrite(uint8_t reg, uint8_t value);

uint8_t PCF8574_getButtonStatus();
uint8_t PCF8574_getRotaryStatus();

#endif
