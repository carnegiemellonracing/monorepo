#include "l431.h"      // Interface to implement

#ifdef L431

#ifdef HAL_CAN_MODULE_ENABLED

/** @brief Total number of hardware TX mailboxes. */
static const size_t CAN_TX_MAILBOXES = 1;

/** @brief CAN interrupt configuration. */
typedef struct {
    CAN_HandleTypeDef *handle;  /**< @brief The handle. */
} cmr_canInterrupt_t;

/**
 * @brief All CAN interrupt configurations, indexed by port.
 *
 * This design is a hold-over from the F413, which has multiple CAN ports.
 */
static cmr_canInterrupt_t cmr_canInterrupts[1];

/**
 * @brief Instantiates the macro for each CAN interface.
 *
 * @param f The macro to instantiate.
 */
#define CAN_FOREACH(f) \
    f(1)

/**
 * @brief Defines interrupt handlers for each CAN interface.
 *
 * @param can The CAN interface number.
 */
#define CAN_IRQ_HANDLERS(can) \
    void CAN ## can ## _TX_IRQHandler(void) { \
        HAL_CAN_IRQHandler(cmr_canInterrupts[can - 1].handle); \
    } \
    \
    void CAN ## can ## _RX0_IRQHandler(void) { \
        HAL_CAN_IRQHandler(cmr_canInterrupts[can - 1].handle); \
    } \
    \
    void CAN ## can ## _RX1_IRQHandler(void) { \
        HAL_CAN_IRQHandler(cmr_canInterrupts[can - 1].handle); \
    } \
    \
    void CAN ## can ## _SCE_IRQHandler(void) { \
        HAL_CAN_IRQHandler(cmr_canInterrupts[can - 1].handle); \
    }
CAN_FOREACH(CAN_IRQ_HANDLERS)
#undef CAN_IRQ_HANDLERS

/**
 * @brief Determines the GPIO alternate function for the given CAN interface.
 *
 * @param can The CAN interface.
 * @param port The GPIO port.
 *
 * @return The GPIO alternate function.
 */
uint32_t _platform_canGPIOAF(CAN_TypeDef *instance, GPIO_TypeDef *port) {
    switch ((uintptr_t) instance) {
        case CAN1_BASE:
            switch ((uintptr_t) port) {
                case GPIOA_BASE:
                case GPIOD_BASE:
                    return GPIO_AF9_CAN1;
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
 * TODO: Handle new bitrate parameter like f413.c
 */
void _platform_canInit(
    cmr_can_t *can, CAN_TypeDef *instance,
    cmr_canBitRate_t bitRate,
    cmr_canRXMeta_t *rxMeta, size_t rxMetaLen,
    cmr_canRXCallback_t rxCallback,
    GPIO_TypeDef *rxPort, uint16_t rxPin,
    GPIO_TypeDef *txPort, uint16_t txPin
) {
    *can = (cmr_can_t) {
        .handle = {
            .Instance = instance,
            .Init = {
                /*
                 * These values can be calculated using "tools" -> "nets config" -> "advanced"
                 * in PCAN Explorer
                 */
                .Prescaler = 2,
                .Mode = CAN_MODE_NORMAL,
                .SyncJumpWidth = CAN_SJW_1TQ,
                .TimeSeg1 = CAN_BS1_13TQ,
                .TimeSeg2 = CAN_BS2_2TQ,
                .TimeTriggeredMode = DISABLE,
                .AutoBusOff = ENABLE,
                .AutoWakeUp = DISABLE,
                .AutoRetransmission = ENABLE,
                .ReceiveFifoLocked = DISABLE,
                .TransmitFifoPriority = DISABLE
            }
        },

        .rxMeta = rxMeta,
        .rxMetaLen = rxMetaLen,
        .rxCallback = rxCallback
    };

    can->txSem = xSemaphoreCreateCountingStatic(
        CAN_TX_MAILBOXES, CAN_TX_MAILBOXES, &can->txSemBuf
    );
    configASSERT(can->txSem != NULL);

    // Configure interrupts.
    size_t canIdx;
    IRQn_Type irqTX;
    IRQn_Type irqRX0;
    IRQn_Type irqRX1;
    IRQn_Type irqSCE;
    switch ((uintptr_t) instance) {
#define CAN_INTERRUPT_CONFIG(num) \
        case CAN ## num ## _BASE: \
            canIdx = num - 1; \
            irqTX = CAN ## num ## _TX_IRQn; \
            irqRX0 = CAN ## num ## _RX0_IRQn; \
            irqRX1 = CAN ## num ## _RX1_IRQn; \
            irqSCE = CAN ## num ## _SCE_IRQn; \
            break;
CAN_FOREACH(CAN_INTERRUPT_CONFIG)
#undef CAN_INTERRUPT_CONFIG
        default:
            cmr_panic("Unknown CAN instance!");
    }

    cmr_canInterrupts[canIdx] = (cmr_canInterrupt_t) {
        .handle = &can->handle
    };
    HAL_NVIC_SetPriority(irqTX, 5, 0);
    HAL_NVIC_SetPriority(irqRX0, 5, 0);
    HAL_NVIC_SetPriority(irqRX1, 5, 0);
    HAL_NVIC_SetPriority(irqSCE, 5, 0);
    HAL_NVIC_EnableIRQ(irqTX);
    HAL_NVIC_EnableIRQ(irqRX0);
    HAL_NVIC_EnableIRQ(irqRX1);
    HAL_NVIC_EnableIRQ(irqSCE);
}

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
        const cmr_canFilter_t *filter = filters + i;

        uint32_t bank = i;

        uint32_t filterMode = filter->isMask
            ? CAN_FILTERMODE_IDMASK
            : CAN_FILTERMODE_IDLIST;

        // In 16 bit ID list mode, FilterIdHigh, FilterIdLow, FilterMaskIdHigh,
        // and FilterMaskIdLow all serve as a whitelist of left-aligned 11-bit
        // CAN IDs.
        // See RM0430 32.7.4 Fig. 387.
        const uint16_t CMR_CAN_ID_FILTER_SHIFT = 5;
        CAN_FilterTypeDef config = {
            .FilterIdHigh           = filter->ids[0] << CMR_CAN_ID_FILTER_SHIFT,
            .FilterIdLow            = filter->ids[1] << CMR_CAN_ID_FILTER_SHIFT,
            .FilterMaskIdHigh       = filter->ids[2] << CMR_CAN_ID_FILTER_SHIFT,
            .FilterMaskIdLow        = filter->ids[3] << CMR_CAN_ID_FILTER_SHIFT,
            .FilterFIFOAssignment   = filter->rxFIFO,
            .FilterBank             = bank,
            .FilterMode             = filterMode,
            .FilterScale            = CAN_FILTERSCALE_16BIT,
            .FilterActivation       = ENABLE,
            .SlaveStartFilterBank   = CMR_CAN_FILTERBANKS
        };

        if (HAL_CAN_ConfigFilter(&can->handle, &config) != HAL_OK) {
            cmr_panic("HAL_CAN_ConfigFilter() failed!");
        }
    }
}

#endif /* HAL_CAN_MODULE_ENABLED */

#ifdef HAL_RCC_MODULE_ENABLED

/**
 * @brief Configures the system and peripheral clocks, using
 * an external clock.
 *
 * @note Generated by STM32Cube. Sets System Clock to 16 MHz
 * All clocks are also at 16MHz.
 */
void _platform_rccSystemClockEnable(void)  {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the CPU, AHB and APB busses clocks
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        cmr_panic("HAL_RCC_OscConfig() failed!");
    }
    /** Initializes the CPU, AHB and APB busses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        cmr_panic("HAL_RCC_OscConfig() failed!");
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        cmr_panic("HAL_RCC_OscConfig() failed!");
    }
    /** Configure the main internal regulator output voltage
     */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
        cmr_panic("HAL_RCC_OscConfig() failed!");
    }
}

/**
 * @brief Configures the system and peripheral clocks,
 * using an internal clock.
 *
 * @note Generated by STM32Cube. Sets System Clock to 16 MHz.
 * TODO: Check if updates to f413.c need to be reflected here
 */
void _platform_rccSystemInternalClockEnable(void)  {
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    // Initializes the CPU, AHB and APB busses clocks
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 20;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        cmr_panic("HAL_RCC_OscConfig() failed!");
    }

    // Initializes the CPU, AHB and APB busses clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        cmr_panic("HAL_RCC_OscConfig() failed!");
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
    PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
    PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
    PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
    PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
    PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
    PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV8;
    PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        cmr_panic("HAL_RCC_PeriphCLKConfig() failed!");
    }

    /** Configure the main internal regulator output voltage
    */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
        cmr_panic("HAL_RCC_ControlVoltageScaling() failed!");
    }
}

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
void _platform_rccADCClockEnable(ADC_TypeDef *instance) {
    switch ((uintptr_t) instance) {
        case ADC1_BASE:
            __HAL_RCC_ADC_CLK_ENABLE();
            break;
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
        .Mode = GPIO_MODE_ANALOG_ADC_CONTROL,
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
                .ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4,
                .Resolution = ADC_RESOLUTION_12B,
                .ScanConvMode = ENABLE,
                .ContinuousConvMode = DISABLE,
                .DiscontinuousConvMode = ENABLE,
                .NbrOfDiscConversion = 1,
                .ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE,
                .ExternalTrigConv = ADC_SOFTWARE_START,
                .DataAlign = ADC_DATAALIGN_RIGHT,
                .NbrOfConversion = channelsLen,
                .DMAContinuousRequests = DISABLE,
                .EOCSelection = ADC_EOC_SINGLE_CONV,
                .Overrun = ADC_OVR_DATA_PRESERVED,
                .OversamplingMode = DISABLE
            }
        },
        .channels = channels,
        .channelsLen = channelsLen
    };
}
#endif /* HAL_ADC_MODULE_ENABLED */

#ifdef HAL_CAN_MODULE_ENABLED
/**
 * @brief Enables the specified CAN interface's clock.
 *
 * @param instance The HAL CAN instance.
 */
void _platform_rccCANClockEnable(CAN_TypeDef *instance) {
    switch ((uintptr_t) instance) {
        case CAN1_BASE:
            __HAL_RCC_CAN1_CLK_ENABLE();
            break;
    }
}
#endif /* HAL_CAN_MODULE_ENABLED */

#endif /* HAL_RCC_MODULE_ENABLED */

#ifdef HAL_FLASH_MODULE_ENABLED

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
    sector = 0x08000000U + FLASH_PAGE_SIZE * sector;
    config->flashStart = (volatile uint32_t *) (void *) sector; /* Hack for now, there is only one sector on L431 */
    config->flashSize = FLASH_PAGE_SIZE;

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
            FLASH_FLAG_PGAERR |
            FLASH_FLAG_PGSERR);

    FLASH_EraseInitTypeDef eraseInit = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .Banks = FLASH_BANK_1,
        .Page = config->flashSector,
        .NbPages = 1,
    };

    // // Use HAL_FLASHEx_Erase instead of HAL_FLASH_Erase, as it clears the FLASH control register.
    uint32_t error;
    if (HAL_FLASHEx_Erase(&eraseInit, &error) != HAL_OK) {
        cmr_panic("Flash erase failed!");
    }

    size_t idx = 0;
    while (idx < config->cacheLen) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (uint32_t) (config->flashStart + idx),
                *(volatile uint64_t *) (&config->cache[idx])) == HAL_OK) {
            idx+=2;
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
    case TIM15_BASE:
        __HAL_RCC_TIM15_CLK_ENABLE();
        break;
    case TIM16_BASE:
        __HAL_RCC_TIM16_CLK_ENABLE();
        break;
    }
}
#endif /* HAL_TIM_MODULE_ENABLED */

#endif /* L413 */
