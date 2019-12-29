/**
 * @file uart.h
 * @brief Universal Asynchronous Receiver/Transmitter interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CMR_UART_H
#define CMR_UART_H

#include "platform.h"       // HAL_UART_MODULE_ENABLED,
                            // HAL_USART_MODULE_ENABLED,
                            // HAL_DMA_MODULE_ENABLED,
                            // UART_HandleTypeDef, UART_InitTypeDef,
                            // USART_TypeDef,
                            // DMA_HandleTypeDef, DMA_Stream_TypeDef

#ifdef HAL_USART_MODULE_ENABLED
#ifdef HAL_UART_MODULE_ENABLED
#ifdef HAL_DMA_MODULE_ENABLED

#include <FreeRTOS.h>   // FreeRTOS interface
#include <semphr.h>     // Semaphore interface

/** @brief Maximum number of pending UART messages. */
#define CMR_UART_MSGS_MAX 16

/**
 * @brief Represents a UART message.
 *
 * @note The contents of this struct are opaque to the library consumer.
 */
typedef struct {
    SemaphoreHandle_t doneSem;     /**< @brief Done binary semaphore. */
    StaticSemaphore_t doneSemBuf;  /**< @brief Done semaphore storage. */

    int opts;       /**< @brief Options. */
    size_t len;     /**< @brief Length of data, in bytes. */
    void *data;     /**< @brief Data buffer. */
} cmr_uartMsg_t;

void cmr_uartMsgInit(cmr_uartMsg_t *msg);
void cmr_uartMsgDestroy(cmr_uartMsg_t *msg);
size_t cmr_uartMsgWait(cmr_uartMsg_t *msg);

/**
 * @brief Represents a UART interface.
 *
 * @note The contents of this struct are opaque to the library consumer.
 */
typedef struct {
    UART_HandleTypeDef handle;  /**< @brief HAL UART handle. */

    /** @brief Receive state. */
    struct {
        DMA_HandleTypeDef dma;  /**< @brief HAL DMA handle. */

        SemaphoreHandle_t dmaSem;       /**< @brief DMA binary semaphore. */
        StaticSemaphore_t dmaSemBuf;    /**< @brief DMA semaphore storage. */

        QueueHandle_t q;        /**< @brief Message queue. */
        StaticQueue_t qBuf;     /**< @brief Queue storage. */

        /** @brief Queue item storage. */
        cmr_uartMsg_t *qItemBuf[CMR_UART_MSGS_MAX];
    } rx;

    /** @brief Transmit state. */
    struct {
        DMA_HandleTypeDef dma;  /**< @brief HAL DMA handle. */

        SemaphoreHandle_t dmaSem;       /**< @brief DMA binary semaphore. */
        StaticSemaphore_t dmaSemBuf;    /**< @brief DMA semaphore storage. */

        QueueHandle_t q;        /**< @brief Message queue. */
        StaticQueue_t qBuf;     /**< @brief Queue storage. */

        /** @brief Queue item storage. */
        cmr_uartMsg_t *qItemBuf[CMR_UART_MSGS_MAX];
    } tx;
} cmr_uart_t;

/** @brief UART receive options. */
typedef enum {
    /** @brief No options specified. */
    CMR_UART_RXOPTS_NONE = 0,

    /**
     * @brief Abort receive when line becomes idle.
     *
     * The line is considered idle when no data is received for 1 "character
     * time"; see RM0430 p. 888.
     */
    CMR_UART_RXOPTS_IDLEABORT = (1 << 0)
} cmr_uartRXOpts_t;

void cmr_uartInit(
    cmr_uart_t *uart, USART_TypeDef *instance, const UART_InitTypeDef *init,
    GPIO_TypeDef *rxPort, uint16_t rxPin,
    GPIO_TypeDef *txPort, uint16_t txPin,
    DMA_Stream_TypeDef *rxDMA, uint32_t rxDMAChannel,
    DMA_Stream_TypeDef *txDMA, uint32_t txDMAChannel
);

void cmr_uartTX(
    cmr_uart_t *uart, cmr_uartMsg_t *msg, const void *data, size_t len
);

void cmr_uartRX(
    cmr_uart_t *uart, cmr_uartMsg_t *msg,
    void *data, size_t len, cmr_uartRXOpts_t opts
);

#endif /* HAL_DMA_MODULE_ENABLED */
#endif /* HAL_UART_MODULE_ENABLED */
#endif /* HAL_USART_MODULE_ENABLED */

#endif /* CMR_UART_H */

