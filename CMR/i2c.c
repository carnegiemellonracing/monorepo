/**
 * @file i2c.c
 * @brief Driver for I2C.
 *
 * @author Carnegie Mellon Racing
 */
#include "i2c.h"    //interface
#include "panic.h"  //cmr_panic()
#include "rcc.h"    //cmr_rccGPIOClockEnable()
extern const I2C_CLOCK_LOW;
extern const I2C_CLOCK_HI;

/**
  * @brief I2C Transmission Function
  * @param devAddr The device address to send to
  * @param data The data to send
  * @param dataLength The length of the data
  * @retval int
  */
int cmr_i2cTX(uint16_t devAddr, uint8_t *data, int dataLength){
	//Shift the address by 1 per HAL library suggestion
	HAL_I2C_Master_Transmit(&hi2c1, devAddr << 1, new_data, dataLength, 0xFFFF);
	return 0;
}


/**
  * @brief I2C Initialization Function
  * @param i2c The I2C to initialize
  * @param instance The HAL I2C instance
  * @param clockSpeed The clock speed to initialize to 
  *         (either I2C_CLOCK_LOW or HI)
  * @param ownAddr User-defined own address
  * @param devAddr The array of device addresses
  * @param addrLen Length of device address array
  * @param i2cPort The I2C GPIO port
  * @param i2cPin The I2C GPIO pin
  * @retval None
  */
void cmr_i2cInit(
    cmr_i2c_t *i2c, I2C_Typedef *instance,
    uint32_t clockSpeed, uint32_t ownAddr,
    uint16_t *devAddr, const size_t addrLen
    GPIO_Typedef *i2cPort, uint32_t i2cPin
) {
	//TODO: error messages for what could go wrong here
    *i2c = (cmr_i2c_t) {
        .handle = {
            .Instance = instance,
            // Configure I2C in low frequency mode, copied from stm32bringups
            .Init = {
            		.ClockSpeed = clockSpeed,
            		.DutyCycle = I2C_DUTYCYCLE_2,
            		.OwnAddress1 = ownAddr,
            		.AddressingMode = I2C_ADDRESSINGMODE_7BIT,
            		.DualAddressMode = I2C_DUALADDRESS_DISABLE,
            		.OwnAddress2 = 0,
            		.GeneralCallMode = I2C_GENERALCALL_DISABLE,
            		.NoStretchMode = I2C_NOSTRETCH_DISABLE
            }
        },
        .devAddr = devAddr,
        .addrLen = addrLen
    };

     if (HAL_I2C_Init(&i2c->handle) != HAL_OK) {
        cmr_panic("HAL_I2C_Init() failed!");
    }
  
  cmr_rccGPIOClockEnable(i2cPort);

  /*Configure GPIO pin Output Level */
  // ^^original annotations, not sure what this means exactly
  HAL_GPIO_WritePin(GPIOB, i2cPin, GPIO_PIN_RESET);


// Format copied from can.c, data copied from stm32-bringups
  GPIO_InitTypeDef pinConfig = {
    .Pin = i2cPin,
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_LOW
  }

   HAL_GPIO_Init(i2cPin, &pinConfig);

}


