/**
 * @file dma.c
 * @brief Direct memory access wrapper implementation.
 *
 * When a DMA stream completes, `HAL_DMA_IRQHandler()` needs to be called with
 * the appropriate `DMA_HandleTypeDef` for that stream. However, the HAL does
 * not provide a way for associating a particular DMA handle with the correct
 * stream interrupt handler. This is implemented here using a table of interrupt
 * configurations.
 *
 * The interrupt handler definitions override the default handlers for each
 * interrupt, which spins forever.
 *
 * The default handlers are (weakly) defined in `CMSIS/startup_stm32f413xx.s`.
 *
 * @author Carnegie Mellon Racing
 */

#include <CMR/dma.h>    // Interface to implement
#include <CMR/panic.h>  // cmr_panic()

#ifdef HAL_DMA_MODULE_ENABLED

/** @brief DMA interrupt configuration.  */
typedef struct {
    DMA_HandleTypeDef *handle;  /**< @brief The handle. */
} cmr_dmaInterrupt_t;

/**
 * @brief All DMA interrupt configurations, indexed by controller and stream.
 *
 * There are 2 DMA controllers on the STM32F413 (DMA1 and DMA2).
 *
 * There are 8 streams (0-7) per controller.
 *
 * @note This array maps DMA1 to index 0, DMA2 to 1, etc.
 */
static cmr_dmaInterrupt_t cmr_dmaInterrupts[2][8];

/**
 * @brief Instantiates the macro for each DMA stream on each controller.
 *
 * @param f The macro to instantiate.
 */
#ifdef F413
#define DMA_STREAM_FOREACH(f) \
    f(1, 0) \
    f(1, 1) \
    f(1, 2) \
    f(1, 3) \
    f(1, 4) \
    f(1, 5) \
    f(1, 6) \
    f(1, 7) \
    f(2, 0) \
    f(2, 1) \
    f(2, 2) \
    f(2, 3) \
    f(2, 4) \
    f(2, 5) \
    f(2, 6) \
    f(2, 7)
#endif

/**
 * @brief Creates a HAL/CMSIS DMA stream name.
 *
 * @param ctrl The DMA controller number.
 * @param stream The stream number.
 * @param suffix The suffix to append, if any.
 */
#ifdef F413
#define DMA_STREAM_NAME(ctrl, stream, suffix) \
    DMA ## ctrl ## _Stream ## stream ## suffix
#endif
/**
 * @brief Defines the IRQ handler for each DMA stream.
 *
 * @param ctrl The DMA controller number.
 * @param stream The stream number.
 */
#define DMA_IRQ_HANDLER(ctrl, stream) \
    void DMA_STREAM_NAME(ctrl, stream, _IRQHandler)(void) { \
        HAL_DMA_IRQHandler(cmr_dmaInterrupts[ctrl - 1][stream].handle); \
    }
DMA_STREAM_FOREACH(DMA_IRQ_HANDLER)
#undef DMA_IRQ_HANDLER

/**
 * @brief Initializes the given HAL DMA handle, including interrupt handler
 * configuration for the corresponding stream.
 *
 * @param handle The HAL DMA handle.
 */
void cmr_dmaInit(DMA_HandleTypeDef *handle) {
    size_t ctrlIndex;
    size_t streamIndex;
    IRQn_Type irqNum;

    // Configure interrupts.
    switch ((uintptr_t) handle->Instance) {
#define DMA_INTERRUPT_CONFIG(ctrl, stream) \
        case DMA_STREAM_NAME(ctrl, stream, _BASE): \
            ctrlIndex = ctrl - 1; \
            streamIndex = stream; \
            irqNum = DMA_STREAM_NAME(ctrl, stream, _IRQn); \
            break;
DMA_STREAM_FOREACH(DMA_INTERRUPT_CONFIG)
#undef DMA_INTERRUPT_CONFIG
        default:
            cmr_panic("Unknown DMA stream!");
    }
    cmr_dmaInterrupts[ctrlIndex][streamIndex] = (cmr_dmaInterrupt_t) {
        .handle = handle
    };
    HAL_NVIC_SetPriority(irqNum, 5, 0);
    HAL_NVIC_EnableIRQ(irqNum);

    // Enable DMA clocks.
    switch (ctrlIndex) {
        case 0:
            __HAL_RCC_DMA1_CLK_ENABLE();
            break;
        case 1:
            __HAL_RCC_DMA2_CLK_ENABLE();
            break;
    }

    if (HAL_DMA_Init(handle) != HAL_OK) {
        cmr_panic("HAL_DMA_Init() failed!");
    }
}

#endif /* HAL_DMA_MODULE_ENABLED */

