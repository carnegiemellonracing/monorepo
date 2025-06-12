

#include "i2c.h"
#include <stdbool.h>


cmr_i2c_t hitl_i2c;


bool i2cInit(void) {

    cmr_i2cInit(&hitl_i2c, I2C1,
            I2C_CLOCK_LOW, 0,
                GPIOB, GPIO_PIN_6, // clock but change pins per schem
                GPIOB, GPIO_PIN_7); // data but change pins per schem

    return true;
}

uint16_t i2c_readI2CADC1(uint8_t channel) {

    // bit 7: (0: differential, 1: single-ended)
    // bits 4-6: channel
    // bit 3: internal voltage reference on/off
    // bit 2: A/D converter on/off
    // bits 0-1: unused
    uint8_t data[2];
    uint8_t command = (1 << 7) | (channel << 4) | 0b0100;

    // write-addressing to set slave ADC and channel
    if(cmr_i2cTX(&hitl_i2c, ADC1_ADDR, &command, 1, I2C_TIMEOUT) != 0) {
        return false;
    }

    // read-addressing to retrieve voltage
    if(cmr_i2cRX(&hitl_i2c, ADC1_ADDR, (uint8_t*)&data, 2, I2C_TIMEOUT) != 0) {
        return false;
    }

    return ((data[0] & 0xF) << 8 | data[1]);
}

uint16_t i2c_readI2CADC2(uint8_t channel) {

    // bit 7: (0: differential, 1: single-ended)
    // bits 4-6: channel
    // bit 3: internal voltage reference on/off
    // bit 2: A/D converter on/off
    // bits 0-1: unused
    uint8_t command = (1 << 7) | (channel << 4) | 0b0100;
    uint8_t data[2];

    // write-addressing to set slave ADC and channel
    if(cmr_i2cTX(&hitl_i2c, ADC2_ADDR, &command, 1, I2C_TIMEOUT) != 0) {
        return false;
    }

    // read-addressing to retrieve voltage
    if(cmr_i2cRX(&hitl_i2c, ADC2_ADDR, (uint8_t*)&data, 2, I2C_TIMEOUT) != 0) {
        return false;
    }

    return data[1] << 8 | data[0];
}


// channel: 0 to 7 inclusive representing DACs A through H
// value: 0 to 4096
bool i2c_writeI2CDAC1(uint8_t channel, uint8_t value) {

    uint8_t command[3];
    command[0] = (0b0011 << 4) | channel;
    command[1] = value;
    command[2] = 0;

    if(cmr_i2cTX(&hitl_i2c, DAC1_ADDR, (uint8_t*)&command, sizeof(command), I2C_TIMEOUT) != 0) {
        return false;
    }

    return true;
}

bool i2c_writeExtRefDAC1() {

    uint8_t command[1];
    command[0] = (0b0111 << 4) | 0b1111;

    if(cmr_i2cTX(&hitl_i2c, DAC1_ADDR, (uint8_t*)&command, sizeof(command), I2C_TIMEOUT) != 0) {
        return false;
    }

    return true;
}

// channel: 0 to 7 inclusive representing DACs A through H
// value: 0 to 4096
bool i2c_writeI2CDAC2(uint8_t channel, uint8_t value) {

	 uint8_t command[3];
	    command[0] = (0b0011 << 4) | channel;
	    command[1] = value;
	    command[2] = 0;

	    if(cmr_i2cTX(&hitl_i2c, DAC2_ADDR, (uint8_t*)&command, sizeof(command), I2C_TIMEOUT) != 0) {
	        return false;
	    }

	    return true;
}

void resetClock() {
    GPIO_InitTypeDef pinConfig = { //clock
        .Pin = GPIO_PIN_7,
        .Mode = GPIO_MODE_OUTPUT_OD,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH
    };

    HAL_GPIO_Init(GPIOB, &pinConfig);
    pinConfig.Pin = GPIO_PIN_8; //data
    pinConfig.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(GPIOB, &pinConfig);

    // TickType_t lastWakeTime = xTaskGetTickCount();

    for (int i = 0; i < 10; i++) {
        HAL_GPIO_WritePin(
            GPIOB, GPIO_PIN_7,
            GPIO_PIN_RESET
        );
    	// VTaskDelayUntil(&lastWakeTime, 5000);
        HAL_GPIO_WritePin(
            GPIOB, GPIO_PIN_7,
            GPIO_PIN_SET
        );
        // VTaskDelayUntil(&lastWakeTime, 5000);
    }

    pinConfig.Pin = GPIO_PIN_7; //clock
    pinConfig.Mode = GPIO_MODE_AF_OD;
    pinConfig.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &pinConfig);
    pinConfig.Pin = GPIO_PIN_8; //data
    HAL_GPIO_Init(GPIOB, &pinConfig);
}
