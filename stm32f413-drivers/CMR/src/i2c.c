/**
 * @file i2c.c
 * @brief Driver for I2C.
 *
 * @author Carnegie Mellon Racing
 */
#include <CMR/i2c.h>    // Interface to implement

#ifdef HAL_I2C_MODULE_ENABLED

#include <CMR/panic.h>  // cmr_panic()
#include <CMR/dma.h>    // dma interface
#include <CMR/rcc.h>    // cmr_rccGPIOClockEnable(), cmr_rccI2CClockEnable()
#include <stddef.h>     // for offsetof (needed for slave i2c)

// macro for getting the struct of a field.
#define container_of(ptr, type, member) \
    ((type *)((char *)(ptr) - offsetof(type, member)))

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

/**
 * @brief Handles I2C completion for the given port.
 *
 * @warning The handle must have been configured through this library!
 *
 * @param handle The HAL I2C handle.
 */
static void cmr_i2cDoneCallback(I2C_HandleTypeDef *handle, uint32_t error) {
    char *addr = (void *) handle;
    cmr_i2c_t *i2c = (void *) (addr - offsetof(cmr_i2c_t, handle));

    i2c->error = error;

    // Indicate completion.
    BaseType_t higherWoken;
    if (xSemaphoreGiveFromISR(i2c->doneSem, &higherWoken) != pdTRUE) {
        cmr_panic("I2C done semaphore released more than once!");
    }
    portYIELD_FROM_ISR(higherWoken);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *handle) {
    cmr_i2cDoneCallback(handle, 0);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *handle) {
    cmr_i2cDoneCallback(handle, 0);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *handle) {
	cmr_i2cDoneCallback(handle, HAL_I2C_GetError(handle));
}

/**
 * @brief Returns the AF for the given I2C instance and GPIO port.
 * @param instance The I2C instance.
 * @param port The GPIO port.
 * @param pin The GPIO pin.
 * @return The AF for the given I2C instance and GPIO port.
 */
uint32_t i2cGPIOAF(I2C_TypeDef *instance, GPIO_TypeDef *port, uint32_t pin) {
    #ifdef F413
    if (instance == I2C1) {
        return GPIO_AF4_I2C1;
    }
	else if (instance == I2C2) {
		if (port == GPIOB && (pin == GPIO_PIN_10 || pin == GPIO_PIN_11)) {
			return GPIO_AF4_I2C2;
		} else if (port == GPIOB) {
			return GPIO_AF9_I2C2;
		} else {
			return GPIO_AF4_I2C2;
		}
    } else if (instance == I2C3) {
		if (port == GPIOA) {
			return GPIO_AF4_I2C3;
		} else if (port == GPIOB) {
			return GPIO_AF9_I2C3;
		} else {
			return GPIO_AF4_I2C3;
		}
    }
    else {
    	cmr_panic("Invalid I2C instance!");
    }
    //TODO: FIX H7 I2C AFs
    #elif defined(H725)
    return GPIO_AF4_I2C4;
    #endif
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
  * @brief I2C Mem Reception Function
  *
  * @param i2c I2C instance to receive on.
  * @param devAddr Target device address
  * @param memaddress Target register address
  * @param data The data to receive
  * @param dataLength The length of the data
  * @param timeout_ms Amount of time to wait in milliseconds.
  *
  * @retval 0 upon success, or otherwise a negative error code
  */
int cmr_i2cMemRX(cmr_i2c_t *i2c, uint16_t devAddr, uint16_t memaddress, uint8_t memsize, uint8_t *data,
              size_t dataLength, uint32_t timeout_ms) {
    // Shift the address by 1 per HAL library suggestion
    HAL_StatusTypeDef rxStatus = HAL_I2C_Mem_Read(
        &(i2c->handle), devAddr << 1, memaddress, memsize, data, dataLength, timeout_ms
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

    if (HAL_I2CEx_ConfigAnalogFilter(&(i2c->handle), I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
    cmr_panic("bruh");
    }

    /** Configure Digital filter
     */
    if (HAL_I2CEx_ConfigDigitalFilter(&(i2c->handle), 0) != HAL_OK)
    {
        cmr_panic("bruh");
    }
    // TODO: Init GPIO with CMR drivers instead of HAL
    GPIO_InitTypeDef pinConfig = {
        .Pin = i2cClkPin,
        .Mode = GPIO_MODE_AF_OD,
        .Pull = GPIO_PULLUP,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
        .Alternate = i2cGPIOAF(instance, i2cClkPort, i2cClkPin)
    };

    HAL_GPIO_Init(i2cClkPort, &pinConfig);
    pinConfig.Pin = i2cDataPin;
    pinConfig.Alternate = i2cGPIOAF(instance, i2cDataPort, i2cDataPin);
    HAL_GPIO_Init(i2cDataPort, &pinConfig);
}

/**
  * @brief I2C Slave Additional Initialization Function
  *
  * @param i2c The I2C to initialize
  * @param instance The HAL I2C instance
  *
  * @retval None
  */
void cmr_i2cSlaveInit(
    cmr_i2c_t *i2c, I2C_TypeDef *instance
) {
    // enable listen mode
    if (HAL_I2C_EnableListen_IT(&(i2c->handle)) != HAL_OK) {
        cmr_panic("Failed to enable I2C listen mode!");
    }
    // register the handle so that HAL_I2C_EV_IRQHandler interrupt can function
    if (instance == I2C1) {
        cmr_i2cDevices[0].handle = &(i2c->handle);
    } else if (instance == I2C2) {
    	cmr_i2cDevices[1].handle = &(i2c->handle);
    } else {
        cmr_panic("Unexpected I2C instance!");
    }
}


/**
  * @brief Slave Address Match callback. If the device address sent by the master matches with the device address of the STM32 I2C, 
  * an interrupt will again trigger and the address callback is called.
  *
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  TransferDirection Master request Transfer Direction (Write/Read), value of @ref I2C_XferDirection_definition
  * @param  AddrMatchCode Address Match Code
  * @retval None
  */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
    cmr_i2c_t *i2c = container_of(hi2c, cmr_i2c_t, handle);

	if(TransferDirection == I2C_DIRECTION_TRANSMIT) {
        //HAL_StatusTypeDef HAL_I2C_Slave_Seq_Transmit_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions)
		HAL_I2C_Slave_Seq_Receive_IT(hi2c, i2c->slaveRxBuff, i2c->slaveRxLen, I2C_FIRST_AND_LAST_FRAME);
	} else {
		HAL_I2C_Slave_Seq_Transmit_IT(hi2c, i2c->slaveTxBuff, i2c->slaveTxLen, I2C_FIRST_AND_LAST_FRAME);
	}
}

/** @brief  Slave Tx Transfer completed callback. (Slave Tx to master finished)
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) 
{
    HAL_I2C_EnableListen_IT(hi2c);
}

/**
  * @brief  Slave Rx Transfer completed callback. (Slave Rx from master finished)
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_EnableListen_IT(hi2c);
}

/**
  * @brief  Slave Tx Transfer initializing data.
  * @param  i2c Pointer to cmr_i2c_t struct.
  * @param  txData Pointer to data buffer to be transmitted.
  * @param  len Length of data buffer.
  *
  * @retval 0 upon success, or otherwise a negative error code
  */
int cmr_i2cSlaveTx(cmr_i2c_t *i2c, uint8_t *txData, size_t len)
{
    i2c->slaveTxBuff = txData;
    i2c->slaveTxLen = len;
    return 0; // bypassing status check for now
}

/**
  * @brief  Slave Rx Transfer initializing data.
  * @param  i2c Pointer to cmr_i2c_t struct
  * @param  rxData Pointer to buffer for recieving data.
  * @param  len Length of data buffer.
  *
  * @retval 0 upon success, or otherwise a negative error code
  */
int cmr_i2cSlaveRx(cmr_i2c_t *i2c, uint8_t *rxData, size_t len)
{
    i2c->slaveRxBuff = rxData;
    i2c->slaveRxLen = len;
    return 0; // bypassing status check for now
}


#ifdef F413
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
        .error = 0,
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

    GPIO_InitTypeDef pinConfig = {
        .Pin = i2cClkPin,
        .Mode = GPIO_MODE_AF_OD,
        .Pull = GPIO_PULLUP,
        .Speed = GPIO_SPEED_FREQ_HIGH,
        .Alternate = i2cGPIOAF(instance, i2cClkPort, i2cClkPin)
    };

    HAL_GPIO_Init(i2cClkPort, &pinConfig);
    pinConfig.Pin = i2cDataPin;
    pinConfig.Alternate = i2cGPIOAF(instance, i2cDataPort, i2cDataPin);
    HAL_GPIO_Init(i2cDataPort, &pinConfig);

    cmr_dmaInit(&(i2c->dmatx_handle));
    __HAL_LINKDMA(&i2c->handle,hdmatx,i2c->dmatx_handle);

    cmr_dmaInit(&(i2c->dmarx_handle));
    __HAL_LINKDMA(&i2c->handle,hdmarx,i2c->dmarx_handle);

    if (instance == I2C1) {
        cmr_i2cDevices[0].handle = &(i2c->handle);
        HAL_NVIC_SetPriority(I2C1_EV_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
        HAL_NVIC_SetPriority(I2C1_ER_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
    } else if (instance == I2C2) {
    	cmr_i2cDevices[1].handle = &(i2c->handle);
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
  * @retval 0 upon success, or otherwise an error code
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
        return -1;
    }

    if (xSemaphoreTake(i2c->doneSem, timeout_ms) != pdTRUE) {
        return -2;
    }

    if (i2c->error != 0) {
        uint32_t savedError = i2c->error;
        i2c->error = 0;
        return savedError;
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
        return -1;
    }

    if (xSemaphoreTake(i2c->doneSem, timeout_ms) != pdTRUE) {
        return -2;
    }

    if (i2c->error != 0) {
        uint32_t savedError = i2c->error;
        i2c->error = 0;
        return savedError;
    }

    configASSERT(HAL_I2C_GetState(&(i2c->handle)) == HAL_I2C_STATE_READY);

    return 0;
}
#endif /* F413 */

#endif /* HAL_I2C_MODULE_ENABLED */

