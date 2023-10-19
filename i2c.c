#include "i2c.h"
#include <stdbool.h>


cmr_i2c_t dim_i2c;

uint8_t buttonStatus;
uint8_t rotaryStatus;

static cmr_task_t i2c_updateIO_task;
static const TickType_t i2c_updateIO_period_ms = 100;
static const uint32_t i2c_updateIO_priority = 3;

uint8_t i2c_expanderRead() {

	uint8_t data;

    if(cmr_i2cRX(&dim_i2c, I2C_EXPANDER_ADDR, &data, 1, I2C_TIMEOUT) != 0) {
        return false;
    }

    return data;
}

bool i2c_expanderWrite(uint8_t channel, uint8_t value) {

	uint8_t command;
	uint8_t currentRead = i2c_expanderRead();

	if (value) {
		command = currentRead | (1 << channel);
	} else {
		command = currentRead & ~(1 << channel);
	}

    if(cmr_i2cTX(&dim_i2c, I2C_EXPANDER_ADDR, &command, sizeof(command), I2C_TIMEOUT) != 0) {
        return false;
    }

    return true;
}

static void i2c_updateIO(void *pvParameters) {
	(void) pvParameters;

	TickType_t lastWakeTime = xTaskGetTickCount();

	while (1) {
		i2c_expanderWrite(I2C_ROTSEL, 0);
		uint8_t read1 = i2c_expanderRead();

		i2c_expanderWrite(I2C_ROTSEL, 1);
		uint8_t read2 = i2c_expanderRead();

		buttonStatus = read2 >> 4;
		rotaryStatus = ((read2 & 0xF) << 4) | (read1 & 0xF);

		vTaskDelayUntil(&lastWakeTime, i2c_updateIO_period_ms);
	}
}

bool i2cInit(void) {

    cmr_i2cInit(&dim_i2c, I2C3,
            I2C_CLOCK_LOW, 0,
                GPIOA, GPIO_PIN_8, // clock but change pins per schem
                GPIOC, GPIO_PIN_9); // data but change pins per schem

    cmr_taskInit(
        &i2c_updateIO_task,
        "i2c Update IO",
		i2c_updateIO_priority,
        i2c_updateIO,
        NULL
    );

    return true;
}
