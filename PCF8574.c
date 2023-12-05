#include "PCF8574.h"

#include <CMR/i2c.h>
#include <CMR/tasks.h>
#include <stdbool.h>

cmr_i2c_t dim_i2c;

int PCF8574_expanderRead(uint8_t *data) {
    // Read Data from register address
    if (cmr_i2cRX(&dim_i2c, PCF8574_EXPANDER_ADDR, data, 1, I2C_TIMEOUT) != 0) {
        return -1;
    }
    return 0;
}

bool PCF8574_expanderWrite(uint8_t reg, uint8_t value) {
    uint8_t command;
    uint8_t currentRead;
    if ((PCF8574_expanderRead(&currentRead)) != 0) {
        return false;
    }

    if (value) {
        command = currentRead | (1 << reg);
    } else {
        command = currentRead & ~(1 << reg);
    }

    if (cmr_i2cTX(&dim_i2c, PCF8574_EXPANDER_ADDR, &command, sizeof(command), I2C_TIMEOUT) != 0) {
        return false;
    }

    return true;
}

bool PCF8574Init(void) {
    cmr_i2cInit(&dim_i2c, I2C3,
                I2C_CLOCK_LOW, 0,
                GPIOA, GPIO_PIN_8,   // clock but change pins per schem
                GPIOC, GPIO_PIN_9);  // data but change pins per schem

    if (HAL_I2CEx_ConfigAnalogFilter(&(dim_i2c.handle), I2C_ANALOGFILTER_DISABLE) != HAL_OK) {
        cmr_panic("Failed to disable analog filter");
    }
    /** Configure Digital filter
     */
    if (HAL_I2CEx_ConfigDigitalFilter(&(dim_i2c.handle), 15) != HAL_OK) {
        cmr_panic("Failed to enable digital filter");
    }

    return true;
}

bool PCF8574Configure(void) {
    return true;
}
