#ifndef I2C_H
#define I2C_H

#include <stdbool.h>
#include <stdint.h>

#define I2C_TIMEOUT 10

#define TCA9554_EXPANDER_ADDR 0b0100000
#define I2C_ROTSEL 3

typedef enum {
    TCA9554_INPUT_PORT = 0x00,    /** Read input */
    TCA9554_OUTPUT_PORT,          /** Write output */
    TCA9554_POL_INV_PORT,         /** Write polarity inversion */
    TCA9554_CONFIG_PORT,          /** Write I/O direction config */
} TCA9554Cmd_t;


bool TCA9554Init();
bool TCA9554Configure();

int TCA9554_expanderRead(uint8_t reg,uint8_t *data);
bool TCA9554_expanderWrite(uint8_t reg, uint8_t value);

uint8_t TCA9554_getButtonStatus();
uint8_t TCA9554_getRotaryStatus();

#endif
