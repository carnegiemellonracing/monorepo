/**
 * @file uart.c
 * @brief Universal Asynchronous Receiver/Transmitter interface wrapper
 * implementation.
 *
 * The `HAL_UART_...Callback()` functions override weak implementations in the
 * HAL UART driver (`stm32f4xx_hal_uart.c`).
 *
 * @warning All callbacks are called from interrupt handlers!
 *
 * @author Carnegie Mellon Racing
 */

#include "uart.h"    // Interface to implement

#ifdef HAL_USART_MODULE_ENABLED
#ifdef HAL_UART_MODULE_ENABLED
#ifdef HAL_DMA_MODULE_ENABLED

#include "rcc.h"    // cmr_rccUSARTClockEnable(), cmr_rccGPIOClockEnable()
#include "dma.h"    // cmr_dmaInit()

/**
 * @brief Initializes the UART interface.
 *
 * @warning It is undefined behavior to initialize the same HAL UART or DMA
 * stream instances more than once!
 *
 * @param uart The UART interface to initialize.
 * @param instance The HAL USART instance (`USARTx` from `stm32f413xx.h`).
 * @param init The HAL UART configuration (see `stm32f4xx_hal_uart.h`).
 * @param rxPort Receiving GPIO port (`GPIOx` from `stm32f413xx.h`).
 * @param rxPin Receiving GPIO pin (`GPIO_PIN_x` from `stm32f4xx_hal_gpio.h`).
 * @param txPort Transmitting GPIO port.
 * @param txPin Transmitting GPIO pin.
 * @param rxDMA HAL DMA stream instance for receiving data (`DMAx_StreamN` from
 * `stm32f413xx.h`).
 * @param rxDMAChannel HAL DMA channel for receiving data (`DMA_CHANNEL_x` from
 * `stm32f4xx_hal_dma.h`).
 * @param txDMA HAL DMA stream instance for transmitting data.
 * @param txDMAChannel HAL DMA channel for transmitting data.
 */
void cmr_uartInit(
    cmr_uart_t *uart, USART_TypeDef *instance, const UART_InitTypeDef *init,
    GPIO_TypeDef *rxPort, uint16_t rxPin,
    GPIO_TypeDef *txPort, uint16_t txPin,
    DMA_Stream_TypeDef *rxDMA, uint32_t rxDMAChannel,
    DMA_Stream_TypeDef *txDMA, uint32_t txDMAChannel
) {
    *uart = (cmr_uart_t) {
        .handle = {
            .Instance = instance,
            .Init = *init
        },
        .rx = {
            .dma = {
                .Instance = rxDMA,
                .Init = {
                    .Channel = rxDMAChannel,
                    .Direction = DMA_PERIPH_TO_MEMORY,
                    .PeriphInc = DMA_PINC_DISABLE,
                    .MemInc = DMA_MINC_ENABLE,
                    .PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
                    .MemDataAlignment = DMA_MDATAALIGN_BYTE,
                    .Mode = DMA_NORMAL,
                    .Priority = DMA_PRIORITY_LOW,
                    .FIFOMode = DMA_FIFOMODE_DISABLE
                }
            }
        },
        .tx = {
            .dma = {
                .Instance = txDMA,
                .Init = {
                    .Channel = txDMAChannel,
                    .Direction = DMA_MEMORY_TO_PERIPH,
                    .PeriphInc = DMA_PINC_DISABLE,
                    .MemInc = DMA_MINC_ENABLE,
                    .PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
                    .MemDataAlignment = DMA_MDATAALIGN_BYTE,
                    .Mode = DMA_NORMAL,
                    .Priority = DMA_PRIORITY_LOW,
                    .FIFOMode = DMA_FIFOMODE_DISABLE
                }
            }
        }
    };

    uart->rx.doneSem = xSemaphoreCreateBinaryStatic(&uart->rx.doneSemBuf);
    configASSERT(uart->rx.doneSem != NULL);

    uart->tx.doneSem = xSemaphoreCreateBinaryStatic(&uart->tx.doneSemBuf);
    configASSERT(uart->tx.doneSem != NULL);

    cmr_rccUSARTClockEnable(instance);

    // Determine GPIO pin alternate.
    uint8_t pinAlternate;
    switch ((uintptr_t) instance) {
        case USART1_BASE:
            pinAlternate = GPIO_AF7_USART1;
            break;
        case USART2_BASE:
            pinAlternate = GPIO_AF7_USART2;
            break;
        case USART3_BASE:
            pinAlternate = GPIO_AF7_USART3;
            break;
        case UART4_BASE:
            pinAlternate = GPIO_AF11_UART4;
            break;
        case UART5_BASE:
            pinAlternate = GPIO_AF8_UART5;
            break;
        case UART7_BASE:
            pinAlternate = GPIO_AF8_UART7;
            break;
        case UART8_BASE:
            pinAlternate = GPIO_AF8_UART8;
            break;
        case UART9_BASE:
            pinAlternate = GPIO_AF11_UART9;
            break;
        case UART10_BASE:
            pinAlternate = GPIO_AF11_UART10;
            break;
        default:
            cmr_panic("Unknown USART instance!", instance);
    }

    // Configure pins.
    GPIO_InitTypeDef pinConfig = {
        .Pin = rxPin,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_PULLUP,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
        .Alternate = pinAlternate
    };
    HAL_GPIO_Init(rxPort, &pinConfig);
    pinConfig.Pin = txPin;
    HAL_GPIO_Init(txPort, &pinConfig);

    // Configure DMA.
    cmr_dmaInit(&uart->rx.dma);
    cmr_dmaInit(&uart->tx.dma);
    __HAL_LINKDMA(&uart->handle, hdmarx, uart->rx.dma);
    __HAL_LINKDMA(&uart->handle, hdmatx, uart->tx.dma);

    if (HAL_UART_Init(&uart->handle) != HAL_OK) {
        cmr_panic("HAL_UART_Init() failed!");
    }
}

#endif /* HAL_DMA_MODULE_ENABLED */
#endif /* HAL_UART_MODULE_ENABLED */
#endif /* HAL_USART_MODULE_ENABLED */

