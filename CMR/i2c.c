/**
 * @file i2c.c
 * @brief Driver for I2C.
 *
 * @author Carnegie Mellon Racing
 */
#include "i2c.h"    // Interface to implement

#ifdef HAL_I2C_MODULE_ENABLED

#include "panic.h"  // cmr_panic()
#include "rcc.h"    // cmr_rccGPIOClockEnable(), cmr_rccI2CClockEnable()

const uint32_t I2C_CLOCK_LOW = 100000;
const uint32_t I2C_CLOCK_HI = 400000;

/**
  * @brief I2C Transmission Function
  *
  * @param i2c I2C instance to transmit on.
  * @param devAddr The device address to send to
  * @param data The data to send
  * @param dataLength The length of the data
  * @param timeout_ms Amount of time to wait in milliseconds.
  *
  * @retval 0 upon success, or otherwise a negative error code
  */
int cmr_i2cTX(cmr_i2c_t *i2c, uint16_t devAddr, uint8_t *data,
              size_t dataLength, uint32_t timeout_ms) {
    // Shift the address by 1 per HAL library suggestion
    HAL_StatusTypeDef txStatus = HAL_I2C_Master_Transmit(
        &(i2c->handle), devAddr << 1, data, dataLength, timeout_ms
    );

    if (txStatus != HAL_OK) {
    //HAL_ERROR    = 0x01U,
    //HAL_BUSY     = 0x02U,
    //HAL_TIMEOUT  = 0x03U
        return txStatus;
    }

    return 0;
}

/**
  * @brief I2C Reception Function
  *
  * @param i2c I2C instance to receive on.
  * @param devAddr Target device address
  * @param data The data to receive
  * @param dataLength The length of the data
  * @param timeout_ms Amount of time to wait in milliseconds.
  *
  * @retval 0 upon success, or otherwise a negative error code
  */
int cmr_i2cRX(cmr_i2c_t *i2c, uint16_t devAddr, uint8_t *data,
              size_t dataLength, uint32_t timeout_ms) {
    // Shift the address by 1 per HAL library suggestion
    HAL_StatusTypeDef rxStatus = HAL_I2C_Master_Receive(
        &(i2c->handle), devAddr << 1, data, dataLength, timeout_ms
    );


    if (rxStatus != HAL_OK) {
        return -1;
    }

    return 0;
}


/**
  * @brief I2C Initialization Function
  *
  * @param i2c The I2C to initialize
  * @param instance The HAL I2C instance
  * @param clockSpeed The clock speed to initialize to
  *         (either I2C_CLOCK_LOW or I2C_CLOCK_HI)
  * @param ownAddr User-defined own address
  * @param i2cPort The I2C GPIO port
  * @param i2cPin The I2C GPIO pin
  *
  * @retval None
  */
void cmr_i2cInit(
    cmr_i2c_t *i2c, I2C_TypeDef *instance,
    uint32_t clockSpeed, uint32_t ownAddr,
    GPIO_TypeDef *i2cClkPort, uint32_t i2cClkPin,
    GPIO_TypeDef *i2cDataPort, uint32_t i2cDataPin
) {
    *i2c = (cmr_i2c_t) {
        .handle = {
            .Instance = instance,
            .Init = {
//                .ClockSpeed = clockSpeed,
//                .DutyCycle = I2C_DUTYCYCLE_2,
                .OwnAddress1 = ownAddr,
                .AddressingMode = I2C_ADDRESSINGMODE_7BIT,
                .DualAddressMode = I2C_DUALADDRESS_DISABLE,
                .OwnAddress2 = 0,
                .GeneralCallMode = I2C_GENERALCALL_DISABLE,
                .NoStretchMode = I2C_NOSTRETCH_DISABLE
            }
        }
    };
    _platform_i2cInit(i2c, instance, clockSpeed, ownAddr);

    cmr_rccI2CClockEnable(instance);
    cmr_rccGPIOClockEnable(i2cClkPort);
    cmr_rccGPIOClockEnable(i2cDataPort);

    if (HAL_I2C_Init(&(i2c->handle)) != HAL_OK) {
        cmr_panic("HAL_I2C_Init() failed!");
    }

    // TODO: Init GPIO with CMR drivers instead of HAL
    GPIO_InitTypeDef pinConfig = {
        .Pin = i2cClkPin,
        .Mode = GPIO_MODE_AF_OD,
        .Pull = GPIO_PULLUP,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
        .Alternate = GPIO_AF4_I2C1
    };

    HAL_GPIO_Init(i2cClkPort, &pinConfig);
    pinConfig.Pin = i2cDataPin;
    HAL_GPIO_Init(i2cDataPort, &pinConfig);
}

#endif /* HAL_I2C_MODULE_ENABLED */

