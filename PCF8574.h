#ifndef I2C_H
#define I2C_H

#include <CMR/i2c.h>
#include <CMR/tasks.h>
#include <stdbool.h>
#include <stdint.h>

#define I2C_TIMEOUT 10

#define I2C_EXPANDER_ADDR 0b0100000
#define I2C_ROTSEL 3

bool i2cInit();

int i2c_expanderRead(uint8_t *data);
bool i2c_expanderWrite(uint8_t channel, uint8_t value);
static void i2c_updateIO(void *pvParameters);

uint8_t i2c_getButtonStatus();
uint8_t i2c_getRotaryStatus();

#endif
