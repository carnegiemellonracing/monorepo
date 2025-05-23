#include <CMR/f413.h> // Interface to implement

#ifdef F413

#ifdef HAL_CAN_MODULE_ENABLED

/** @brief Total number of hardware TX mailboxes. */
static const size_t CAN_TX_MAILBOXES = 3;

/** @brief CAN interrupt configuration. */
typedef struct
{
    CAN_HandleTypeDef *handle; /**< @brief The handle. */
} cmr_canInterrupt_t;

/**
 * @brief All CAN interrupt configurations, indexed by port.
 *
 * There are 3 CAN controllers on the STM32F413 (CAN1, CAN2, CAN3).
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
#define CAN_IRQ_HANDLERS(can)                                  \
    void CAN##can##_TX_IRQHandler(void)                        \
    {                                                          \
        HAL_CAN_IRQHandler(cmr_canInterrupts[can - 1].handle); \
    }                                                          \
                                                               \
    void CAN##can##_RX0_IRQHandler(void)                       \
    {                                                          \
        HAL_CAN_IRQHandler(cmr_canInterrupts[can - 1].handle); \
    }                                                          \
                                                               \
    void CAN##can##_RX1_IRQHandler(void)                       \
    {                                                          \
        HAL_CAN_IRQHandler(cmr_canInterrupts[can - 1].handle); \
    }                                                          \
                                                               \
    void CAN##can##_SCE_IRQHandler(void)                       \
    {                                                          \
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
uint32_t _platform_canGPIOAF(CAN_TypeDef *instance, GPIO_TypeDef *port)
{
    switch ((uintptr_t)instance)
    {
    case CAN1_BASE:
        switch ((uintptr_t)port)
        {
        case GPIOA_BASE:
        case GPIOD_BASE:
            return GPIO_AF9_CAN1;
        case GPIOB_BASE:
            return GPIO_AF8_CAN1;
        default:
            cmr_panic("Unknown/unspported GPIO port!");
        }
    case CAN2_BASE:
        return GPIO_AF9_CAN2;
    case CAN3_BASE:
        return GPIO_AF11_CAN3;
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
                .Prescaler = 0,
                .Mode = CAN_MODE_NORMAL,
                .SyncJumpWidth = CAN_SJW_2TQ,
                .TimeSeg1 = CAN_BS1_6TQ,
                .TimeSeg2 = CAN_BS2_1TQ,
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

    // These numbers assume 48 MHz ABP1 peripheral clock frequency
    // 48 MHz / (6 + 1 + 1 time quanta) / Prescaler = bitRate
    switch (bitRate) {
        case CMR_CAN_BITRATE_250K:
            can->handle.Init.Prescaler = 24;
            break;
        case CMR_CAN_BITRATE_500K:
            can->handle.Init.Prescaler = 12;
            break;
        case CMR_CAN_BITRATE_1M:
            can->handle.Init.Prescaler = 6;
            break;
    }

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
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(irqTX);
    HAL_NVIC_EnableIRQ(irqRX0);
    HAL_NVIC_EnableIRQ(irqRX1);
    HAL_NVIC_EnableIRQ(irqSCE);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
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
    cmr_can_t *can, const cmr_canFilter_t *filters, size_t filtersLen)
{
    if (filtersLen >= CMR_CAN_FILTERBANKS)
    {
        cmr_panic("Too many filter banks!");
    }

    CAN_TypeDef *instance = can->handle.Instance;

    for (size_t i = 0; i < filtersLen; i++)
    {
        const cmr_canFilter_t *filter = filters + i;

        uint32_t bank = i;
        if (instance == CAN2) {
            // CAN2 uses banks 14-27.
            bank += CMR_CAN_FILTERBANKS;
        }

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

        if (HAL_CAN_ConfigFilter(&can->handle, &config) != HAL_OK)
        {
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
 * @note Generated by STM32Cube. Sets System Clock to 96 MHz, with only APB1
 * Peripheral Clocks at 48 MHz (APB1 Timer Clocks are still 96 MHz).
 */
void _platform_rccSystemClockEnable(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // Configure the main internal regulator output voltage
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    // Initializes the CPU, AHB and APB busses clocks
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 25;
    RCC_OscInitStruct.PLL.PLLN = 192;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        cmr_panic("HAL_RCC_OscConfig() failed!");
    }

    // Initializes the CPU, AHB and APB busses clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
    {
        cmr_panic("HAL_RCC_ClockConfig() failed!");
    }
}

/**
 * @brief Configures the system and peripheral clocks using the internal oscillator and PLL
 *
 * @note Generated by STM32Cube. Sets System Clock to 96 MHz, with only APB1
 * Peripheral Clocks at 48 MHz (APB1 Timer Clocks are still 96 MHz).
 */
void _platform_rccSystemInternalClockEnable(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    // Configure the main internal regulator output voltage
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    // Initializes the CPU, AHB and APB busses clocks
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 96;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        cmr_panic("HAL_RCC_OscConfig() failed!");
    }

    // Initializes the CPU, AHB and APB busses clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
        cmr_panic("HAL_RCC_ClockConfig() failed!");
    }
}

#ifdef HAL_GPIO_MODULE_ENABLED

/**
 * @brief Enables the specified GPIO port's clock.
 *
 * @param port The GPIO port.
 */
void _platform_rccGPIOClockEnable(GPIO_TypeDef *port)
{
    switch ((uintptr_t)port)
    {
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

ADC_ChannelConfTypeDef _platform_adcChannelConfig(const cmr_adcChannel_t *channel, uint32_t rank)
{
    ADC_ChannelConfTypeDef channelConfig = {
        .Channel = channel->channel,
        .Rank = rank, // HAL needs Rank to be from 1 to 16
        .SamplingTime = channel->samplingTime,
        .Offset = 0 // reserved, set to 0
    };

    return channelConfig;
}

GPIO_InitTypeDef _platform_adcPinConfig(const cmr_adcChannel_t *channel)
{
    GPIO_InitTypeDef pinConfig = {
        .Pin = channel->pin,
        .Mode = GPIO_MODE_ANALOG,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_LOW,
        .Alternate = 0};

    return pinConfig;
}

/**
 * @brief Enables the specified ADC's clock.
 *
 * @param instance The HAL ADC instance.
 */
void _platform_rccADCClockEnable(ADC_TypeDef *instance)
{
    switch ((uintptr_t)instance)
    {
    case ADC1_BASE:
        __HAL_RCC_ADC1_CLK_ENABLE();
        break;
    }
}

/**
 * @brief Platform-specifc adc initialization
 *
 *  @param adc The ADC to initialize.
 */
void _platform_adcInit(cmr_adc_t *adc, ADC_TypeDef *instance, cmr_adcChannel_t *channels, const size_t channelsLen)
{
    *adc = (cmr_adc_t){
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
                .EOCSelection = ADC_EOC_SINGLE_CONV}},
        .channels = channels,
        .channelsLen = channelsLen};
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

#ifdef HAL_CAN_MODULE_ENABLED
/**
 * @brief Enables the specified CAN interface's clock.
 *
 * @param instance The HAL CAN instance.
 */
void _platform_rccCANClockEnable(CAN_TypeDef *instance)
{
    switch ((uintptr_t) instance) {
        case CAN1_BASE:
            __HAL_RCC_CAN1_CLK_ENABLE();
            break;
        case CAN2_BASE:
            __HAL_RCC_CAN2_CLK_ENABLE();
            __HAL_RCC_CAN1_CLK_ENABLE();    // CAN2 also needs CAN1 clock.
            break;
        case CAN3_BASE:
            __HAL_RCC_CAN3_CLK_ENABLE();
            break;
    }
}
#endif /* HAL_CAN_MODULE_ENABLED */

#endif /* HAL_RCC_MODULE_ENABLED */

#ifdef HAL_FLASH_MODULE_ENABLED

/**
 * @brief Instantiates the macro for each flash sector.
 *
 * The parameter are, in order: sector number, corresponding base address, and cooresponding size.
 * The sectors are defined at HAL @ref FLASHEx_Sectors.
 * The sector addresses and sizes are defined in the STM32F413 reference manual.
 *
 * @param f The macro to instantiate.
 */
#define SECTOR_FOREACH(f)                                                                           \
    f(FLASH_SECTOR_0, 0x08000000, 0x4000)                                                           \
        f(FLASH_SECTOR_1, 0x08004000, 0x4000)                                                       \
            f(FLASH_SECTOR_2, 0x08008000, 0x4000)                                                   \
                f(FLASH_SECTOR_3, 0x0800C000, 0x4000)                                               \
                    f(FLASH_SECTOR_4, 0x08010000, 0x10000)                                          \
                        f(FLASH_SECTOR_5, 0x08020000, 0x20000)                                      \
                            f(FLASH_SECTOR_6, 0x08040000, 0x20000)                                  \
                                f(FLASH_SECTOR_7, 0x08060000, 0x20000)                              \
                                    f(FLASH_SECTOR_8, 0x08080000, 0x20000)                          \
                                        f(FLASH_SECTOR_9, 0x080A0000, 0x20000)                      \
                                            f(FLASH_SECTOR_10, 0x080C0000, 0x20000)                 \
                                                f(FLASH_SECTOR_11, 0x080E0000, 0x20000)             \
                                                    f(FLASH_SECTOR_12, 0x08100000, 0x20000)         \
                                                        f(FLASH_SECTOR_13, 0x08120000, 0x20000)     \
                                                            f(FLASH_SECTOR_14, 0x08140000, 0x20000) \
                                                                f(FLASH_SECTOR_15, 0x08160000, 0x20000)

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
void _platform_configInit(cmr_config_t *config, volatile uint32_t *cache, size_t cacheLen, uint32_t sector)
{
    config->cache = cache;
    config->cacheLen = cacheLen;
    config->flashSector = sector;
    config->flashStart = (volatile uint32_t *)getSectorBase(sector);
    config->flashSize = getSectorSize(sector);

    if (config->cacheLen > config->flashSize)
    {
        cmr_panic("Config cache is larger than sector!");
    }

    cmr_configPull(config);
}

/**
 * @brief Commits the local config cache to flash.
 *
 * @param config The interface to use.
 */
void _platform_configCommit(cmr_config_t *config)
{
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return;
    }

    // Clears all the error bits. See the HAL documentation @ref FLASH_Flag_definition.
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP |
                           FLASH_FLAG_OPERR |
                           FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR |
                           FLASH_FLAG_PGSERR);

    FLASH_EraseInitTypeDef eraseInit = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Sector = config->flashSector,
        .NbSectors = 1,
        .VoltageRange = VOLTAGE_RANGE_3,
    };

    // Use HAL_FLASHEx_Erase instead of HAL_FLASH_Erase, as it clears the FLASH control register.
    uint32_t error;
    if (HAL_FLASHEx_Erase(&eraseInit, &error) != HAL_OK)
    {
        cmr_panic("Flash erase failed!");
    }

    size_t idx = 0;
    while (idx < config->cacheLen)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)(config->flashStart + idx),
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
    case TIM9_BASE:
        __HAL_RCC_TIM9_CLK_ENABLE();
        break;
    case TIM10_BASE:
        __HAL_RCC_TIM10_CLK_ENABLE();
        break;
    case TIM11_BASE:
        __HAL_RCC_TIM11_CLK_ENABLE();
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
    }
}
#endif /* HAL_TIM_MODULE_ENABLED */

#ifdef HAL_I2C_MODULE_ENABLED
void _platform_i2cInit(cmr_i2c_t *i2c, I2C_TypeDef *instance, uint32_t clockSpeed, uint32_t ownAddr) {
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
}

void _platform_i2cClockInit(I2C_TypeDef *instance) {
	switch ((uintptr_t) instance) {
	        case I2C1_BASE:
	            __HAL_RCC_I2C1_CLK_ENABLE();
	            break;
	        case I2C2_BASE:
	            __HAL_RCC_I2C2_CLK_ENABLE();
	            break;
	        case I2C3_BASE:
	            __HAL_RCC_I2C3_CLK_ENABLE();
	            break;
	    }
}

#endif /* HAL_I2C_MODULE_ENABLED */

#endif /* F413 */
