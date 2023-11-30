#include "TCA9554.h"
#include <stdbool.h>
#include <CMR/tasks.h>
#include <CMR/i2c.h>

cmr_i2c_t dim_i2c;

uint8_t buttonStatus;
uint8_t rotaryStatus;

static const TickType_t i2c_updateIO_period_ms = 100;

int TCA9554_expanderRead(uint8_t reg,uint8_t *data) {
	// Select Register
	if(cmr_i2cTX(&dim_i2c, TCA9554_EXPANDER_ADDR,&reg, 1, I2C_TIMEOUT) != 0) {
        return -1;
    }
	// Read Data from register address
    if(cmr_i2cRX(&dim_i2c, TCA9554_EXPANDER_ADDR,data, 1, I2C_TIMEOUT) != 0) {
        return -1;
    }
    return 0;
}

bool TCA9554_expanderWrite(uint8_t reg, uint8_t value) {

	uint8_t command;
	uint8_t currentRead;
	if((TCA9554_expanderRead(TCA9554_INPUT_PORT,&currentRead)) != 0){
		return false;
	}

	if (value) {
		command = currentRead | (1 << reg);
	} else {
		command = currentRead & ~(1 << reg);
	}

    if(cmr_i2cTX(&dim_i2c, TCA9554_EXPANDER_ADDR, &command, sizeof(command), I2C_TIMEOUT) != 0) {
        return false;
    }

    return true;
}

static void TCA9554_updateIO(void *pvParameters) {
	(void) pvParameters;

	TickType_t lastWakeTime = xTaskGetTickCount();

	while (1) {
		TCA9554_expanderWrite(I2C_ROTSEL, 0);
		uint8_t read1;
		uint8_t read2;
		int status = 0;
		status = TCA9554_expanderRead(TCA9554_INPUT_PORT,&read1);
		TCA9554_expanderWrite(I2C_ROTSEL, 1);
		status |= TCA9554_expanderRead(TCA9554_INPUT_PORT, &read2);
		vTaskDelayUntil(&lastWakeTime, i2c_updateIO_period_ms);
	}
}

bool TCA9554Init(void) {

    cmr_i2cInit(&dim_i2c, I2C3,
            I2C_CLOCK_LOW, 0,
                GPIOA, GPIO_PIN_8, // clock but change pins per schem
                GPIOC, GPIO_PIN_9); // data but change pins per schem

	if (HAL_I2CEx_ConfigAnalogFilter(&(dim_i2c.handle), I2C_ANALOGFILTER_DISABLE) != HAL_OK)
		{
		cmr_panic("Failed to disable analog filter");
		}
		/** Configure Digital filter
		*/
	if (HAL_I2CEx_ConfigDigitalFilter(&(dim_i2c.handle), 15) != HAL_OK)
		{
		  cmr_panic("Failed to enable digital filter");
		}

    return true;
}

bool TCA9554Configure(void) {
	int status = TCA9554_expanderWrite(TCA9554_CONFIG_PORT,0xFC);
	return status;
}
