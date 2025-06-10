
#include <stm32f4xx_hal.h>  // HAL interface

#include <CMR/panic.h>  // cmr_panic()
#include <CMR/rcc.h>    // RCC interface
#include <CMR/tasks.h>  // Task interface

#include "i2c.h"
#include "gpio.h"
#include "can.h"
#include "pwm.h"





int main(void) {

	// Initialising interfaces: GPIO, PWM, CAN, I2C
	HAL_Init();
	cmr_rccSystemClockEnable();
	//gpioInit();
	i2cInit();
	//pwmReadInit();
	//canInit();


	// Setting outputs
	//cmr_gpioWrite(GPIO_MCU1, 0);
	//cmr_gpioWrite(DIGITAL_OUT_5V_MCU, 1);


	// Reading inputs to verify board
	i2c_writeExtRefDAC1();
	HAL_Delay(100);

	uint8_t val = 0;
	uint16_t y = 0;
	while (1) {
		i2c_writeI2CDAC1(15, val);
		HAL_Delay(100);
		y = i2c_readI2CADC1(0);
		HAL_Delay(100);
		val = val + 10;
	}

	// TODO:DIM TRANSITION BETWEEN STATES, DIM AND CAN STATES, GEAR SWITCH, SCREEN ( WHEN DONE)






	return 0;
}
