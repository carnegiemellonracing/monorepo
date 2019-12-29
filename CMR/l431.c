#include "l431.h"      // Interface to implement

#ifdef HAL_CAN_MODULE_ENABLED

/** @brief Total number of hardware TX mailboxes. */
static const size_t CAN_TX_MAILBOXES = 3;

/** @brief CAN interrupt configuration. */
typedef struct {
    CAN_HandleTypeDef *handle;  /**< @brief The handle. */
} cmr_canInterrupt_t;

/**
 * @brief All CAN interrupt configurations, indexed by port.
 *
 * There are 3 CAN controllers on the STM32F413 (CAN1, CAN2, CAN3).
 *
 * @note This array maps CAN1 to index 0, CAN2 to 1, etc.
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
                    cmr_panic("Unknown/unspported GPIO port!");
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
 */
void _platform_canInit(
    cmr_can_t *can, CAN_TypeDef *instance,
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
                .Prescaler = 5,
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
 * @note Generated by STM32Cube. Sets System Clock to 96 MHz, with only APB1
 * Peripheral Clocks at 48 MHz (APB1 Timer Clocks are still 96 MHz).
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
      if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
      {
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

      if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
      {
          cmr_panic("HAL_RCC_OscConfig() failed!");
      }
      PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
      PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
      if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
      {
          cmr_panic("HAL_RCC_OscConfig() failed!");
      }
      /** Configure the main internal regulator output voltage
      */
      if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
      {
          cmr_panic("HAL_RCC_OscConfig() failed!");
      }
}

/**
 * @brief Configures the system and peripheral clocks,
 * using an internal clock.
 *
 * @note Generated by STM32Cube. Sets System Clock to 16 MHz.
 */
void _platform_rccSystemInternalClockEnable(void)  {
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    // Configure the main internal regulator output voltage
    //__HAL_RCC_PWR_CLK_ENABLE();
    //__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
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
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        cmr_panic("HAL_RCC_PeriphCLKConfig() failed!");
    }

    /** Configure the main internal regulator output voltage
    */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
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
