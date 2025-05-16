#include <CMR/h725.h>      // Interface to implement

#ifdef H725

#ifdef HAL_FDCAN_MODULE_ENABLED

static uint32_t HAL_RCC_FDCAN_CLK_ENABLED=0;

/** @brief CAN interrupt configuration. */
typedef struct {
    FDCAN_HandleTypeDef *handle;  /**< @brief The handle. */
} cmr_canInterrupt_t;

/**
 * @brief All CAN interrupt configurations, indexed by port.
 *
 * There are 3 CAN controllers on the STM32H725 (CAN1, CAN2, CAN3).
 *
 * @note This array maps CAN1 to index 0, CAN2 to 1, etc.
 */
static cmr_canInterrupt_t cmr_canInterrupts[3];

/**
 * @brief Instantiates the macro for each CAN interface.
 *
 * @param f The macro to instantiate.
 */
#define CAN_FOREACH(f) \
    f(1)               \
        f(2)           \
            f(3)

/**
 * @brief Defines interrupt handlers for each CAN interface.
 *
 * @param can The CAN interface number.
 */

void FDCAN1_IT0_IRQHandler(void) {
	HAL_FDCAN_IRQHandler(cmr_canInterrupts[0].handle);
}

void FDCAN1_IT1_IRQHandler(void) {
	HAL_FDCAN_IRQHandler(cmr_canInterrupts[0].handle);
}
void FDCAN2_IT0_IRQHandler(void) {
	HAL_FDCAN_IRQHandler(cmr_canInterrupts[1].handle);
}

void FDCAN2_IT1_IRQHandler(void) {
	HAL_FDCAN_IRQHandler(cmr_canInterrupts[1].handle);
}
void FDCAN3_IT0_IRQHandler(void) {
	HAL_FDCAN_IRQHandler(cmr_canInterrupts[2].handle);
}

void FDCAN3_IT1_IRQHandler(void) {
	HAL_FDCAN_IRQHandler(cmr_canInterrupts[2].handle);
}

/**
 * @brief Determines the GPIO alternate function for the given CAN interface.
 *
 * @param can The CAN interface.
 * @param port The GPIO port.
 *
 * @return The GPIO alternate function.
 */
uint32_t _platform_FDcanGPIOAF(FDCAN_GlobalTypeDef *instance, GPIO_TypeDef *port) {
    switch ((uintptr_t) instance) {
        case FDCAN1_BASE:
            switch ((uintptr_t) port) {
                case GPIOA_BASE:
                	return GPIO_AF9_FDCAN1;
                case GPIOB_BASE:
                	return GPIO_AF9_FDCAN1;
                case GPIOD_BASE:
                	return GPIO_AF9_FDCAN1;
                case GPIOH_BASE:
                    return GPIO_AF9_FDCAN1;
                default:
                    cmr_panic("Unknown/unsupported GPIO port!");
            }
        case FDCAN2_BASE:
            switch ((uintptr_t) port) {
                case GPIOB_BASE:
                    return GPIO_AF9_FDCAN2;
                default:
                    cmr_panic("Unknown/unsupported GPIO port!");
            }
        case FDCAN3_BASE:
            switch ((uintptr_t) port) {
                case GPIOD_BASE:
                    return GPIO_AF5_FDCAN3;
                case GPIOF_BASE:
                	return GPIO_AF2_FDCAN3;
                case GPIOG_BASE:
                    return GPIO_AF2_FDCAN3;
                default:
                    cmr_panic("Unknown/unsupported GPIO port!");
            }
        default:
            cmr_panic("Unknown CAN instance!");
    }
}



/**
 * @brief Fills in platform-specific defaults
 *
 * @warning It is undefined behavior to initialize the same HAL CAN instance
 * more than once!
 *
 * @param can The interface to initialize.
 * @param instance The HAL CAN instance (`CANx` from `stm32f413xx.h`).
 * @param rxMeta Metadata for periodic messages to receive.
 * @param rxMetaLen Number of periodic receive messages.
 * @param rxCallback Callback for other messages received, or `NULL` to ignore.
 * @param rxPort Receiving GPIO port (`GPIOx` from `stm32f413xx.h`).
 * @param rxPin Receiving GPIO pin (`GPIO_PIN_x` from `stm32f4xx_hal_gpio.h`).
 * @param txPort Transmitting GPIO port.
 * @param txPin Transmitting GPIO pin.
 *
 */


void _platform_FDCANInit(
    cmr_can_t *can, FDCAN_GlobalTypeDef *instance,
    cmr_canRXMeta_t *rxMeta, size_t rxMetaLen,
    cmr_canRXCallback_t rxCallback
) {
    *can = (cmr_can_t) {
        .handle = {
            .Instance = instance,
            .Init = {
                /*
                 * These values can be calculated using "tools" -> "nets config" -> "advanced"
                 * in PCAN Explorer
                 */
                .FrameFormat = FDCAN_FRAME_CLASSIC,
                .Mode = FDCAN_MODE_NORMAL,
                .AutoRetransmission = ENABLE,
                .TransmitPause = DISABLE,
                .ProtocolException = DISABLE,
                .NominalPrescaler = 1,
                .NominalSyncJumpWidth = 2,
                .NominalTimeSeg1 = 41,
                .NominalTimeSeg2 = 8,
                .MessageRAMOffset = 0,
                // Extra options generated for FDCAN
                .StdFiltersNbr = 128,
                .ExtFiltersNbr = 0,
                .RxFifo0ElmtsNbr = 64,
                .RxFifo0ElmtSize = FDCAN_DATA_BYTES_8,
                .RxFifo1ElmtsNbr = 64,
                .RxFifo1ElmtSize = FDCAN_DATA_BYTES_8,
                .RxBuffersNbr = 0,
                .RxBufferSize = FDCAN_DATA_BYTES_8,
                .TxEventsNbr = 0,
                .TxBuffersNbr = 0,
                .TxFifoQueueElmtsNbr = 32,
                .TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION,
                .TxElmtSize = FDCAN_DATA_BYTES_8
            }
        },

        .rxMeta = rxMeta,
        .rxMetaLen = rxMetaLen,
        .rxCallback = rxCallback
    };

    if(instance == FDCAN2) {
    	can->handle.Init.MessageRAMOffset = 853; //partitioning RAM size equally based on 2560 RAM Word Size
    }
    else if(instance == FDCAN3) {
    	can->handle.Init.MessageRAMOffset = 1706;
    }

    can->txSem = xSemaphoreCreateMutexStatic(&can->txSemBuf);

    configASSERT(can->txSem != NULL);


    // Configure interrupts.
    size_t canIdx;
    IRQn_Type IT0;
    IRQn_Type IT1;
    switch ((uintptr_t) instance) {
#define CAN_INTERRUPT_CONFIG(num) \
        case FDCAN ## num ## _BASE: \
            canIdx = num - 1; \
            IT0 = FDCAN ## num ## _IT0_IRQn ; \
            IT1 = FDCAN ## num ## _IT1_IRQn ; \
            break;
CAN_FOREACH(CAN_INTERRUPT_CONFIG)
#undef CAN_INTERRUPT_CONFIG
        default:
            cmr_panic("Unknown CAN instance!");
    }

    cmr_canInterrupts[canIdx] = (cmr_canInterrupt_t) {
        .handle = &can->handle
    };
    HAL_NVIC_SetPriority(IT0, 5, 0);
    HAL_NVIC_SetPriority(IT1, 5, 0);
    HAL_NVIC_EnableIRQ(IT0);
    HAL_NVIC_EnableIRQ(IT1);
}

/**
* 2. FDCAN_FILTER_DUAL:
*    - Accepts messages with IDs exactly matching either FilterID1 OR FilterID2
*    - Example: FilterID1=0x100, FilterID2=0x200 accepts only IDs 0x100 and 0x200
*
* 3. FDCAN_FILTER_MASK:
*    - Accepts messages where (received_id & FilterID2) == (FilterID1 & FilterID2)
*    - FilterID1 is the ID to match
*    - FilterID2 is the mask that specifies which bits to compare (1=compare, 0=ignore)
*    - Example: FilterID1=0x100, FilterID2=0x7F0 compares only bits 4-10
*               This would accept IDs like 0x100, 0x101, ..., 0x10F
*/
/**
 * @brief Configures a filter bank with 4 CAN IDs to filter.
 *
 * @param can The CAN interface to configure.
 * @param filters The filter configuration(s).
 * @param filtersLen The number of filters. Must be less than
 * `CMR_CAN_FILTERBANKS`.
 */
void _platform_canFilter(
    cmr_can_t *can, const cmr_canFilter_t *filters, size_t filtersLen
) {
    if (filtersLen >= CMR_CAN_FILTERBANKS) {
        cmr_panic("Too many filter banks!");
    }

    for (size_t i = 0; i < filtersLen; i++) {
        const cmr_canFilter_t *filter = &filters[i];

        // Determine filter configuration based on rxFIFO
        uint32_t filterConfig = (filter->rxFIFO == FDCAN_RX_FIFO0) ?
                               FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;

        // Determine filter type based on isMask
        uint32_t filterType = filter->isMask ? FDCAN_FILTER_MASK : FDCAN_FILTER_DUAL;

        // Use IDs directly without shifting
        uint32_t id1 = filter->ids[0];
        uint32_t id2 = filter->ids[1];

        FDCAN_FilterTypeDef config = {
            .IdType = FDCAN_STANDARD_ID,
            .FilterIndex = i,
            .FilterType = filterType,
            .FilterConfig = filterConfig,
            .FilterID1 = id1,
            .FilterID2 = id2,
            .IsCalibrationMsg = 0
        };

        if (HAL_FDCAN_ConfigFilter(&can->handle, &config) != HAL_OK) {
            cmr_panic("HAL_FDCAN_ConfigFilter() failed!");
        }
    }
}

/**
 * @brief Enables the specified CAN interface's clock.
 *
 * @param instance The HAL CAN instance.
 */

void _platform_rccFDCanClockEnable() {
	if(HAL_RCC_FDCAN_CLK_ENABLED == 0) {
		RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
	    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
	    PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_HSE;
	    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
	    	cmr_panic("Clock source failed!");
	    }
		__HAL_RCC_FDCAN_CLK_ENABLE();
		HAL_RCC_FDCAN_CLK_ENABLED++;
	}
}

#endif /* HAL_CAN_MODULE_ENABLED */

#ifdef HAL_RCC_MODULE_ENABLED

/**
 * @brief Configures the system and peripheral clocks, using
 * an external clock.
 *
 * @note Generated by STM32Cube. Sets System Clock to 550 MHz
 * Peripheral clocks are at 137.5 MHz.
 */
void _platform_rccSystemClockEnable(void)  {
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	  /** Supply configuration update enable
	  */
	  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

	  /** Configure the main internal regulator output voltage
	  */
	  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

	  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

	  /** Macro to configure the PLL clock source
	  */
	  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

	  /** Initializes the RCC Oscillators according to the specified parameters
	  * in the RCC_OscInitTypeDef structure.
	  */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	  RCC_OscInitStruct.PLL.PLLM = 2;
	  RCC_OscInitStruct.PLL.PLLN = 44;
	  RCC_OscInitStruct.PLL.PLLP = 1;
	  RCC_OscInitStruct.PLL.PLLQ = 3;
	  RCC_OscInitStruct.PLL.PLLR = 2;
	  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
	  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	  RCC_OscInitStruct.PLL.PLLFRACN = 0;
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
		cmr_panic("osc init failed!");
	  }

	  /** Initializes the CPU, AHB and APB buses clocks
	  */
	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
	                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	  {
	    cmr_panic("clock init failed!");
	  }
}

/**
 * @brief Configures the system and peripheral clocks,
 * using an internal clock.
 *
 * @note Generated by STM32Cube. Sets System Clock to 550 MHz, with APB
 * Peripheral Clocks at 137.5 MHz.
 */
void _platform_rccSystemInternalClockEnable(void)  {
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    // Configure the main internal regulator output voltage

    /** Supply configuration update enable
    */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    /** Configure the main internal regulator output voltage
    */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    // Initializes the CPU, AHB and APB busses clocks
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = 64;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 34;
    RCC_OscInitStruct.PLL.PLLP = 1;
    RCC_OscInitStruct.PLL.PLLQ = 3;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 3072;


    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        cmr_panic("HAL_RCC_OscConfig() failed!");
    }

    // Initializes the CPU, AHB and APB busses clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                                  |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        cmr_panic("HAL_RCC_OscConfig() failed!");
    }
}
#endif /* HAL_RCC_MODULE_ENABLED */

#ifdef HAL_GPIO_MODULE_ENABLED

/**
 * @brief Enables the specified GPIO port's clock.
 *
 * @param port The GPIO port.
 */
void _platform_rccGPIOClockEnable(GPIO_TypeDef *port) {
    switch ((uintptr_t) port) {
        case GPIOA_BASE:
            __HAL_RCC_GPIOA_CLK_ENABLE();
            break;
        case GPIOB_BASE:
            __HAL_RCC_GPIOB_CLK_ENABLE();
            break;
        case GPIOC_BASE:
            __HAL_RCC_GPIOC_CLK_ENABLE();
            break;
        case GPIOD_BASE:
            __HAL_RCC_GPIOD_CLK_ENABLE();
            break;
        case GPIOE_BASE:
            __HAL_RCC_GPIOE_CLK_ENABLE();
            break;
        case GPIOF_BASE:
            __HAL_RCC_GPIOF_CLK_ENABLE();
            break;
        case GPIOG_BASE:
            __HAL_RCC_GPIOG_CLK_ENABLE();
            break;
        case GPIOH_BASE:
            __HAL_RCC_GPIOH_CLK_ENABLE();
            break;
    }
}
#endif /* HAL_GPIO_MODULE_ENABLED */


#ifdef HAL_ADC_MODULE_ENABLED
/**
 * @brief Enables the specified ADC's clock.
 *
 * @param instance The HAL ADC instance.
 */

static uint32_t HAL_RCC_ADC_CLK_ENABLED=0;

void _platform_rccADCClockEnable(ADC_TypeDef *instance) {
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
	 PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	    PeriphClkInitStruct.PLL3.PLL3M = 2;
	    PeriphClkInitStruct.PLL3.PLL3N = 15;
	    PeriphClkInitStruct.PLL3.PLL3P = 2;
	    PeriphClkInitStruct.PLL3.PLL3Q = 2;
	    PeriphClkInitStruct.PLL3.PLL3R = 16;
	    PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_3;
	    PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
	    PeriphClkInitStruct.PLL3.PLL3FRACN = 2950;
	    PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL3;
	    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	    {
	     cmr_panic("  ");
	    }

	if(HAL_RCC_ADC_CLK_ENABLED == 0) {
		__HAL_RCC_ADC12_CLK_ENABLE();
        __HAL_RCC_ADC3_CLK_ENABLE();
		HAL_RCC_ADC_CLK_ENABLED++;
	}
}

ADC_ChannelConfTypeDef _platform_adcChannelConfig(const cmr_adcChannel_t *channel, uint32_t rank) {
    ADC_ChannelConfTypeDef channelConfig = {
        .Channel = channel->channel,
        .Rank = rank,  // HAL needs Rank to be from 1 to 16
        .SamplingTime = channel->samplingTime,
        .Offset = 0,     // reserved, set to 0
        .SingleDiff = ADC_SINGLE_ENDED,
        .OffsetNumber = ADC_OFFSET_NONE
    };

    return channelConfig;
}

GPIO_InitTypeDef _platform_adcPinConfig(const cmr_adcChannel_t *channel) {
    GPIO_InitTypeDef pinConfig = {
        .Pin = channel->pin,
        .Mode = GPIO_MODE_ANALOG,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_LOW,
        .Alternate = 0
    };

    return pinConfig;
}

/**
 * @brief Platform-specifc adc initialization
 *
 *  @param adc The ADC to initialize.
 */
void _platform_adcInit(cmr_adc_t *adc, ADC_TypeDef *instance, cmr_adcChannel_t *channels, const size_t channelsLen) {
    *adc =  (cmr_adc_t) {
        .handle = {
            .Instance = instance,

            // Configure ADC in discontinuous scan mode.
            // This will allow conversion of a series of channels one at a time.
            .Init = {
            		.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1,
            		.Resolution = ADC_RESOLUTION_12B,
            		.ScanConvMode = ADC_SCAN_ENABLE,
                    .DataAlign = ADC3_DATAALIGN_RIGHT,
            		.EOCSelection = ADC_EOC_SINGLE_CONV,
            		.LowPowerAutoWait = DISABLE,
            		.ContinuousConvMode = DISABLE,
            		.NbrOfConversion = channelsLen,
            		.DiscontinuousConvMode = ENABLE,
            		.NbrOfDiscConversion = 1,
            		.ExternalTrigConv = ADC_SOFTWARE_START,
            		.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE,
                    .DMAContinuousRequests = DISABLE,
                    .SamplingMode = ADC_SAMPLING_MODE_NORMAL,
            		.ConversionDataManagement = ADC_CONVERSIONDATA_DR,
            		.Overrun = ADC_OVR_DATA_PRESERVED,
            		.LeftBitShift = ADC_LEFTBITSHIFT_NONE,
					.OversamplingMode = DISABLE
            }
        },
        .channels = channels,
        .channelsLen = channelsLen
    };
}

void _platform_adcPoll(cmr_adc_t *adc, uint32_t adcTimeout) {
    // ADC set up in discontinuous scan mode.
    // Each `HAL_ADC_Start()` call converts the next-highest-rank channel.
    for (size_t i = 0; i < adc->channelsLen; i++) {
        cmr_adcChannel_t *channel = &(adc->channels[i]);

        HAL_ADC_Start(&adc->handle);
        HAL_ADC_PollForConversion(&adc->handle, adcTimeout);
        channel->value = HAL_ADC_GetValue(&adc->handle);
    }
}
#endif /* HAL_ADC_MODULE_ENABLED */





#ifdef HAL_FLASH_MODULE_ENABLED

/**
 * @brief Instantiates the macro for each flash sector.
 *
 * The parameter are, in order: sector number, corresponding base address, and cooresponding size.
 * The sectors are defined at HAL @ref FLASHEx_Sectors.
 * The sector addresses and sizes are defined in the STM32H725 reference manual.
 *
 * @param f The macro to instantiate.
 */
#define SECTOR_FOREACH(f)                                                                           \
    f(FLASH_SECTOR_0, 0x08000000, 0x20000)                                                           \
        f(FLASH_SECTOR_1, 0x08020000, 0x20000)                                                       \
            f(FLASH_SECTOR_2, 0x08040000, 0x20000)                                                   \
                f(FLASH_SECTOR_3, 0x08060000, 0x20000)                                               \
                    f(FLASH_SECTOR_4, 0x08080000, 0x20000)                                          \
                        f(FLASH_SECTOR_5, 0x080A0000, 0x20000)                                      \
                            f(FLASH_SECTOR_6, 0x080C0000, 0x20000)                                  \
                                f(FLASH_SECTOR_7, 0x080E0000, 0x20000)

static void *getSectorBase(uint32_t sector)
{
#define GET_ADDR(sec, base, ...) \
    if (sector == sec)           \
    {                            \
        return (void *)base;     \
    }
    SECTOR_FOREACH(GET_ADDR)
#undef GET_ADDR

    cmr_panic("Invalid sector!");
}

static size_t getSectorSize(uint32_t sector)
{
#define GET_SIZE(sec, base, size) \
    if (sector == sec)            \
    {                             \
        return size;              \
    }
    SECTOR_FOREACH(GET_SIZE)
#undef GET_SIZE

    cmr_panic("Invalid sector!");
}

/**
 * @brief Initializes the configuration system with a base address.
 *
 * @param config The interface to initalize.
 * @param cache The config cache to use.
 * @param cacheLen The size of the config cache to use.
 * @param sector The flash sector to use. See the HAL documentation @ref FLASHEx_Sectors.
 */
void _platform_configInit(cmr_config_t *config, volatile uint32_t *cache, size_t cacheLen, uint32_t sector) {
    config->cache = cache;
    config->cacheLen = cacheLen;
    config->flashSector = sector;
    config->flashStart = (volatile uint32_t *)getSectorBase(sector);
    config->flashSize = getSectorSize(sector);;

    if (config->cacheLen > config->flashSize) {
        cmr_panic("Config cache is larger than sector!");
    }

    cmr_configPull(config);
}

/**
 * @brief Commits the local config cache to flash.
 *
 * @param config The interface to use.
 */
void _platform_configCommit(cmr_config_t *config) {
    if (HAL_FLASH_Unlock() != HAL_OK) {
        return;
    }

    // Clears all the error bits. See the HAL documentation @ref FLASH_Flag_definition.
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP |
            FLASH_FLAG_OPERR |
            FLASH_FLAG_WRPERR |
            FLASH_FLAG_PGSERR);

    FLASH_EraseInitTypeDef eraseInit = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Sector = config->flashSector,
        .NbSectors = 1,
        .VoltageRange = VOLTAGE_RANGE_3,
    };

    // // Use HAL_FLASHEx_Erase instead of HAL_FLASH_Erase, as it clears the FLASH control register.
    uint32_t error;
    if (HAL_FLASHEx_Erase(&eraseInit, &error) != HAL_OK) {
        cmr_panic("Flash erase failed!");
    }

    size_t idx = 0;
    while (idx < config->cacheLen)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, (uint32_t)(config->flashStart + idx),
                              config->cache[idx]) == HAL_OK)
        {
            idx++;
        }
    }

    HAL_FLASH_Lock();
}

#endif /* HAL_FLASH_MODULE_ENABLED */

#ifdef HAL_TIM_MODULE_ENABLED
void _platform_rccTIMClockEnable(TIM_TypeDef *instance)
{
    switch ((uintptr_t)instance)
    {
    case TIM1_BASE:
        __HAL_RCC_TIM1_CLK_ENABLE();
        break;
    case TIM2_BASE:
        __HAL_RCC_TIM2_CLK_ENABLE();
        break;
    case TIM3_BASE:
        __HAL_RCC_TIM3_CLK_ENABLE();
        break;
    case TIM4_BASE:
        __HAL_RCC_TIM4_CLK_ENABLE();
        break;
    case TIM5_BASE:
        __HAL_RCC_TIM5_CLK_ENABLE();
        break;
    case TIM6_BASE:
        __HAL_RCC_TIM6_CLK_ENABLE();
        break;
    case TIM7_BASE:
        __HAL_RCC_TIM7_CLK_ENABLE();
        break;
    case TIM8_BASE:
        __HAL_RCC_TIM8_CLK_ENABLE();
        break;
    case TIM12_BASE:
        __HAL_RCC_TIM12_CLK_ENABLE();
        break;
    case TIM13_BASE:
        __HAL_RCC_TIM13_CLK_ENABLE();
        break;
    case TIM14_BASE:
        __HAL_RCC_TIM14_CLK_ENABLE();
        break;
    case TIM15_BASE:
        __HAL_RCC_TIM15_CLK_ENABLE();
        break;
    case TIM16_BASE:
        __HAL_RCC_TIM16_CLK_ENABLE();
        break;
    case TIM17_BASE:
        __HAL_RCC_TIM17_CLK_ENABLE();
        break;
    case TIM23_BASE:
        __HAL_RCC_TIM23_CLK_ENABLE();
        break;
    case TIM24_BASE:
        __HAL_RCC_TIM24_CLK_ENABLE();
        break;
    }
}
#endif /* HAL_TIM_MODULE_ENABLED */

#ifdef HAL_I2C_MODULE_ENABLED
void _platform_i2cInit(cmr_i2c_t *i2c, I2C_TypeDef *instance, uint32_t clockSpeed, uint32_t ownAddr) {
    *i2c = (cmr_i2c_t) {
        .handle = {
            .Instance = instance,
            .Init = {
				 .Timing = 0x00601A5C,
				 .OwnAddress1 = 0,
				 .AddressingMode = I2C_ADDRESSINGMODE_7BIT,
				 .DualAddressMode = I2C_DUALADDRESS_DISABLE,
				 .OwnAddress2 = 0,
				 .OwnAddress2Masks = I2C_OA2_NOMASK,
				 .GeneralCallMode = I2C_GENERALCALL_DISABLE,
				 .NoStretchMode = I2C_NOSTRETCH_DISABLE
            }
        }
    };
}

void _platform_i2cClockInit(I2C_TypeDef *instance) {
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
	switch ((uintptr_t) instance) {
	        case I2C1_BASE:
	        	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
				PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C1CLKSOURCE_D2PCLK1;
				if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
				{
				  cmr_panic("i2c clock failed!");
				}
				__HAL_RCC_I2C1_CLK_ENABLE();
	            break;
	        case I2C2_BASE:
	        	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
				PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C2CLKSOURCE_D2PCLK1;
				if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
				{
				  cmr_panic("i2c clock failed!");
				}
				__HAL_RCC_I2C2_CLK_ENABLE();
				break;
	        case I2C3_BASE:
	        	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C3;
				PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C3CLKSOURCE_D2PCLK1;
				if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
				{
				  cmr_panic("i2c clock failed!");
				}
				__HAL_RCC_I2C3_CLK_ENABLE();
	            break;
	        case I2C4_BASE:
	        	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C4;
				PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C4CLKSOURCE_D3PCLK1;
				if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
				{
				  cmr_panic("i2c clock failed!");
				}
				__HAL_RCC_I2C4_CLK_ENABLE();
				break;
	        case I2C5_BASE:
				PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C5;
				PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C5CLKSOURCE_D2PCLK1;
				if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
				{
				  cmr_panic("i2c clock failed!");
				}
				__HAL_RCC_I2C5_CLK_ENABLE();
				break;
	    }
}
#endif /* HAL_I2C_MODULE_ENABLED */

#endif /* H725 */