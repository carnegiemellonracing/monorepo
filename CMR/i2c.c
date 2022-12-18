/**
 * @file i2c.c
 * @brief Driver for I2C.
 *
 * @author Carnegie Mellon Racing
 */
#include "i2c.h"    // Interface to implement

#ifdef HAL_I2C_MODULE_ENABLED

#include "panic.h"  // cmr_panic()
#include "dma.h"    // dma interface
#include "rcc.h"    // cmr_rccGPIOClockEnable(), cmr_rccI2CClockEnable()
                    //

static void I2C_ClearBusyFlagErratum(cmr_i2c_t* i2c, uint32_t timeout);

typedef struct {
    /** @brief The associated handle, or `NULL` if not configured. */
    I2C_HandleTypeDef *handle;
} cmr_i2cDevice_t;

const uint32_t I2C_CLOCK_LOW = 100000;
const uint32_t I2C_CLOCK_HI = 400000;

/**
 * @brief All I2C device configurations, indexed by interface.
 *
 * There is only 2 I2C interface on the STM32F413.
 */
static cmr_i2cDevice_t cmr_i2cDevices[2];

/**
 * @brief I2C interrupt handler.
 */
void I2C1_EV_IRQHandler(void) {
    HAL_I2C_EV_IRQHandler(cmr_i2cDevices[0].handle);
}

/**
 * @brief I2C interrupt handler.
 */
void I2C2_EV_IRQHandler(void) {
    HAL_I2C_EV_IRQHandler(cmr_i2cDevices[1].handle);
}
/**
 * @brief I2C interrupt handler.
 */
void I2C1_ER_IRQHandler(void) {
    HAL_I2C_ER_IRQHandler(cmr_i2cDevices[0].handle);
}

/**
 * @brief I2C interrupt handler.
 */
void I2C2_ER_IRQHandler(void) {
    HAL_I2C_ER_IRQHandler(cmr_i2cDevices[1].handle);
}

// void I2C1_DMA_IRQHandler(void)
// {
//   HAL_DMA_IRQHandler(cmr_i2cDevices[0].handle->hdmarx);
//   HAL_DMA_IRQHandler(cmr_i2cDevices[0].handle->hdmatx);
// }

// void I2C2_DMA_IRQHandler(void)
// {
//   HAL_DMA_IRQHandler(cmr_i2cDevices[1].handle->hdmarx);
//   HAL_DMA_IRQHandler(cmr_i2cDevices[1].handle->hdmatx);
// }

/**
 * @brief Handles I2C completion for the given port.
 *
 * @warning The handle must have been configured through this library!
 *
 * @param handle The HAL I2C handle.
 */
static void cmr_i2cDoneCallback(I2C_HandleTypeDef *handle) {
    char *addr = (void *) handle;
    cmr_i2c_t *i2c = (void *) (addr - offsetof(cmr_i2c_t, handle));

    // Indicate completion.
    BaseType_t higherWoken;
    if (xSemaphoreGiveFromISR(i2c->doneSem, &higherWoken) != pdTRUE) {
        cmr_panic("I2C done semaphore released more than once!");
    }
    portYIELD_FROM_ISR(higherWoken);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *handle) {
    cmr_i2cDoneCallback(handle);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *handle) {
    cmr_i2cDoneCallback(handle);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *handle) {
	cmr_i2cDoneCallback(handle);
}

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
        return -1;
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
                .ClockSpeed = clockSpeed,
                .DutyCycle = I2C_DUTYCYCLE_2,
                .OwnAddress1 = ownAddr,
                .AddressingMode = I2C_ADDRESSINGMODE_7BIT,
                .DualAddressMode = I2C_DUALADDRESS_DISABLE,
                .OwnAddress2 = 0,
                .GeneralCallMode = I2C_GENERALCALL_DISABLE,
                .NoStretchMode = I2C_NOSTRETCH_DISABLE
            }
        }
    };

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

/**
  * @brief I2C DMA Initialization Function
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
void cmr_i2cDmaInit(
    cmr_i2c_t *i2c, I2C_TypeDef *instance,
    DMA_Stream_TypeDef *txDmaStream, uint32_t txDmaChannel,
    DMA_Stream_TypeDef *rxDmaStream, uint32_t rxDmaChannel,
    uint32_t clockSpeed, uint32_t ownAddr,
    GPIO_TypeDef *i2cClkPort, uint32_t i2cClkPin,
    GPIO_TypeDef *i2cDataPort, uint32_t i2cDataPin
) {
    *i2c = (cmr_i2c_t) {
        .i2cClkPort = i2cClkPort,
        .i2cClkPin = i2cClkPin,
        .i2cDataPort = i2cDataPort,
        .i2cDataPin = i2cDataPin,
        .handle = {
            .Instance = instance,
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
        .dmatx_handle = {
            .Instance = txDmaStream,
            .Init = {
                .Direction = DMA_MEMORY_TO_PERIPH,
				.Channel = txDmaChannel,
                .PeriphInc = DMA_PINC_DISABLE,
                .MemInc = DMA_MINC_ENABLE,
                .PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
                .MemDataAlignment = DMA_MDATAALIGN_BYTE,
                .Mode = DMA_NORMAL,
                .Priority = DMA_PRIORITY_LOW,
                .FIFOMode = DMA_FIFOMODE_DISABLE,
                .FIFOThreshold = DMA_FIFO_THRESHOLD_FULL,
                .MemBurst = DMA_MBURST_SINGLE,
                .PeriphBurst = DMA_PBURST_SINGLE
            }
        },
        .dmarx_handle = {
            .Instance = rxDmaStream,
            .Init = {
                .Direction = DMA_PERIPH_TO_MEMORY,
				.Channel = rxDmaChannel,
                .PeriphInc = DMA_PINC_DISABLE,
                .MemInc = DMA_MINC_ENABLE,
                .PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
                .MemDataAlignment = DMA_MDATAALIGN_BYTE,
                .Mode = DMA_NORMAL,
                .Priority = DMA_PRIORITY_LOW,
                .FIFOMode = DMA_FIFOMODE_DISABLE,
                .FIFOThreshold = DMA_FIFO_THRESHOLD_FULL,
                .MemBurst = DMA_MBURST_SINGLE,
                .PeriphBurst = DMA_PBURST_SINGLE
            }
        }

    };

    i2c->doneSem = xSemaphoreCreateBinaryStatic(&(i2c->doneSemBuf));
    configASSERT(i2c->doneSem != NULL);

    cmr_rccGPIOClockEnable(i2cClkPort);
    cmr_rccGPIOClockEnable(i2cDataPort);
    cmr_rccI2CClockEnable(instance);

    if (HAL_I2C_Init(&(i2c->handle)) != HAL_OK) {
        cmr_panic("HAL_I2C_Init() failed!");
    }

    // TODO: Init GPIO with CMR drivers instead of HAL
    GPIO_InitTypeDef pinConfig = {
        .Pin = i2cClkPin,
        .Mode = GPIO_MODE_AF_OD,
        .Pull = GPIO_PULLUP,
        .Speed = GPIO_SPEED_FREQ_HIGH,
        .Alternate = GPIO_AF4_I2C1
    };

    HAL_GPIO_Init(i2cClkPort, &pinConfig);
    pinConfig.Pin = i2cDataPin;
    HAL_GPIO_Init(i2cDataPort, &pinConfig);

    cmr_dmaInit(&(i2c->dmatx_handle));
    __HAL_LINKDMA(&i2c->handle,hdmatx,i2c->dmatx_handle);

    cmr_dmaInit(&(i2c->dmarx_handle));
    __HAL_LINKDMA(&i2c->handle,hdmarx,i2c->dmarx_handle);

    if (instance == I2C1) {
        cmr_i2cDevices[0].handle = &(i2c->handle);
        // HAL_NVIC_SetPriority(I2C1_DMA_IRQn, 7, 0);
        // HAL_NVIC_EnableIRQ(I2C1_DMA_IRQn);
        HAL_NVIC_SetPriority(I2C1_EV_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
        HAL_NVIC_SetPriority(I2C1_ER_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
    } else if (instance == I2C2) {
    	cmr_i2cDevices[1].handle = &(i2c->handle);
    	// HAL_NVIC_SetPriority(I2C2_DMA_IRQn, 7, 0);
    	// HAL_NVIC_EnableIRQ(I2C2_DMA_IRQn);
        HAL_NVIC_SetPriority(I2C2_EV_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
        HAL_NVIC_SetPriority(I2C2_ER_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
    } else {
        cmr_panic("Unexpected I2C instance!");
    }
}

int cmr_i2cIsReady(cmr_i2c_t *i2c) {
    return HAL_I2C_GetState(&(i2c->handle)) == HAL_I2C_STATE_READY;
}

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
int cmr_i2cDmaTX(cmr_i2c_t *i2c, uint16_t devAddr, uint8_t *data,
              size_t dataLength, uint32_t timeout_ms) {
    // IMPORTANT: If we don't check this and an existing transaction is
    // going on, very bad things happen
	configASSERT(HAL_I2C_GetState(&(i2c->handle)) == HAL_I2C_STATE_READY);
	configASSERT(i2c->handle.hdmatx->State == HAL_DMA_STATE_READY);
	configASSERT(i2c->handle.hdmarx->State == HAL_DMA_STATE_READY);
    // Shift the address by 1 per HAL library suggestion
    HAL_StatusTypeDef txStatus = HAL_I2C_Master_Transmit_DMA(
        &(i2c->handle), devAddr << 1, data, dataLength 
    );

    if (txStatus != HAL_OK) {
    	if (__HAL_I2C_GET_FLAG(&(i2c->handle), I2C_FLAG_BUSY) != RESET) {
    		//I2C_ClearBusyFlagErratum(i2c, 1000);
    		SET_BIT(i2c->handle.Instance->CR1, I2C_CR1_STOP);
    		//I2C_WaitOnFlagUntilTimeout(&(i2c->handle), I2C_FLAG_BUSY, SET, 1000);
    	}
        return -1;
    }

    if (xSemaphoreTake(i2c->doneSem, timeout_ms) != pdTRUE) {
        return -2;
    }
    configASSERT(HAL_I2C_GetState(&(i2c->handle)) == HAL_I2C_STATE_READY);

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
int cmr_i2cDmaRX(cmr_i2c_t *i2c, uint16_t devAddr, uint8_t *data,
              size_t dataLength, uint32_t timeout_ms) {
    // IMPORTANT: If we don't check this and an existing transaction is
    // going on, very bad things happen
	configASSERT(HAL_I2C_GetState(&(i2c->handle)) == HAL_I2C_STATE_READY);
	configASSERT(i2c->handle.hdmatx->State == HAL_DMA_STATE_READY);
	configASSERT(i2c->handle.hdmarx->State == HAL_DMA_STATE_READY);
    // Shift the address by 1 per HAL library suggestion
    HAL_StatusTypeDef rxStatus = HAL_I2C_Master_Receive_DMA(
        &(i2c->handle), devAddr << 1, data, dataLength 
    );


    if (rxStatus != HAL_OK) {
    	if (__HAL_I2C_GET_FLAG(&(i2c->handle), I2C_FLAG_BUSY) != RESET) {
    		//I2C_ClearBusyFlagErratum(i2c, 1000);
    		SET_BIT(i2c->handle.Instance->CR1, I2C_CR1_STOP);
    		//I2C_WaitOnFlagUntilTimeout(&(i2c->handle), I2C_FLAG_BUSY, SET, 1000);
    	}
        return -1;
    }

    if (xSemaphoreTake(i2c->doneSem, timeout_ms) != pdTRUE) {
        return -2;
    }
    configASSERT(HAL_I2C_GetState(&(i2c->handle)) == HAL_I2C_STATE_READY);

    return 0;
}

static int wait_for_gpio_state_timeout(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state, uint32_t timeout)
 {
    uint32_t Tickstart = HAL_GetTick();
    int ret = 0;
    /* Wait until flag is set */
    for(;(state != HAL_GPIO_ReadPin(port, pin)) && (0 == ret);)
    {
        /* Check for the timeout */
        if (timeout != HAL_MAX_DELAY)
        {
            if ((timeout == 0U) || ((HAL_GetTick() - Tickstart) > timeout))
            {
                ret = -1;
            }
            else
            {
            }
        }
        asm("nop");
    }
    return ret;
}

static void I2C_ClearBusyFlagErratum(cmr_i2c_t* i2c, uint32_t timeout)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    I2C_HandleTypeDef *handle = &(i2c->handle);
    GPIO_TypeDef *I2C1_SCL_GPIO_Port = i2c->i2cClkPort;
    GPIO_TypeDef *I2C1_SDA_GPIO_Port = i2c->i2cDataPort;
    uint32_t I2C1_SDA_Pin = i2c->i2cDataPin;
    uint32_t I2C1_SCL_Pin = i2c->i2cClkPin;

    // 1. Clear PE bit.
    CLEAR_BIT(handle->Instance->CR1, I2C_CR1_PE);

    //  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    HAL_I2C_DeInit(handle);

    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull = GPIO_NOPULL;

    GPIO_InitStructure.Pin = i2c->i2cClkPin;
    HAL_GPIO_Init(I2C1_SCL_GPIO_Port, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = i2c->i2cDataPin;
    HAL_GPIO_Init(I2C1_SDA_GPIO_Port, &GPIO_InitStructure);

    // 3. Check SCL and SDA High level in GPIOx_IDR.
    HAL_GPIO_WritePin(I2C1_SDA_GPIO_Port, I2C1_SDA_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C1_SCL_GPIO_Port, I2C1_SCL_Pin, GPIO_PIN_SET);

    wait_for_gpio_state_timeout(I2C1_SCL_GPIO_Port, I2C1_SCL_Pin, GPIO_PIN_SET, timeout);
    wait_for_gpio_state_timeout(I2C1_SDA_GPIO_Port, I2C1_SDA_Pin, GPIO_PIN_SET, timeout);

    // 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C1_SDA_GPIO_Port, I2C1_SDA_Pin, GPIO_PIN_RESET);

    // 5. Check SDA Low level in GPIOx_IDR.
    wait_for_gpio_state_timeout(I2C1_SDA_GPIO_Port, I2C1_SDA_Pin, GPIO_PIN_RESET, timeout);

    // 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C1_SCL_GPIO_Port, I2C1_SCL_Pin, GPIO_PIN_RESET);

    // 7. Check SCL Low level in GPIOx_IDR.
    wait_for_gpio_state_timeout(I2C1_SCL_GPIO_Port, I2C1_SCL_Pin, GPIO_PIN_RESET, timeout);

    // 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C1_SCL_GPIO_Port, I2C1_SCL_Pin, GPIO_PIN_SET);

    // 9. Check SCL High level in GPIOx_IDR.
    wait_for_gpio_state_timeout(I2C1_SCL_GPIO_Port, I2C1_SCL_Pin, GPIO_PIN_SET, timeout);

    // 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(I2C1_SDA_GPIO_Port, I2C1_SDA_Pin, GPIO_PIN_SET);

    // 11. Check SDA High level in GPIOx_IDR.
    wait_for_gpio_state_timeout(I2C1_SDA_GPIO_Port, I2C1_SDA_Pin, GPIO_PIN_SET, timeout);

    // 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
    GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStructure.Alternate = GPIO_AF4_I2C1;

    GPIO_InitStructure.Pin = I2C1_SCL_Pin;
    HAL_GPIO_Init(I2C1_SCL_GPIO_Port, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = I2C1_SDA_Pin;
    HAL_GPIO_Init(I2C1_SDA_GPIO_Port, &GPIO_InitStructure);

    // 13. Set SWRST bit in I2Cx_CR1 register.
    SET_BIT(handle->Instance->CR1, I2C_CR1_SWRST);
    asm("nop");

    /* 14. Clear SWRST bit in I2Cx_CR1 register. */
    CLEAR_BIT(handle->Instance->CR1, I2C_CR1_SWRST);
    asm("nop");

    /* 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register */
    SET_BIT(handle->Instance->CR1, I2C_CR1_PE);
    asm("nop");

    // Call initialization function.
    HAL_I2C_Init(handle);
}

#endif /* HAL_I2C_MODULE_ENABLED */

