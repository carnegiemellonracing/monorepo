/**
 * @file i2c.c
 * @brief Driver for I2C.
 *
 * @author Carnegie Mellon Racing
 */
#include "i2c.h"    //interface
#include "panic.h"  //cmr_panic()

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
  * @param devAddr The array of device addresses
  * @param addrLen Length of device address array
  * @retval None
  */
void cmr_i2cInit(
    cmr_i2c_t *i2c, I2C_Typedef *instance,
    uint16_t *devAddr, const size_t addrLen
) {
	//TODO: error messages for what could go wrong here

    *adc = (cmr_adc_t) {
        .handle = {
            .Instance = instance,
            // Configure I2C in low frequency mode, copied from stm32bringups
            .Init = {
            		.ClockSpeed = 100000,
            		.DutyCycle = I2C_DUTYCYCLE_2,
            		.OwnAddress1 = 0,
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

}


