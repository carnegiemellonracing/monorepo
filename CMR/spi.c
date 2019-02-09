/**
 * @file spi.c
 * @brief Serial Peripheral Interface port wrapper implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "adc.c"    // Interface to implement

#ifdef HAL_SPI_MODULE_ENABLED

#include "rcc.h"    // cmr_rccSPIClockEnable(), cmr_rccGPIOClockEnable()
#include "panic.h"  // cmr_panic()

/**
 * @brief Initializes the SPI port.
 *
 * @warning It is undefined behavior to initialize the same HAL SPI instance
 * more than once!
 *
 * @param spi The SPI port to initialize.
 * @param instance The HAL SPI instance (`SPIx` from `stm32f413xx.h`).
 * @param init The HAL SPI configuration (see `stm32f4xx_hal_spi.h`).
 * @param pins Pin configuration.
 */
void cmr_spiInit(
    cmr_spi_t *spi, SPI_TypeDef *instance, const SPI_InitTypeDef *init,
    const cmr_spiPinConfig_t *pins
) {
    *spi = (cmr_spi_t) {
        .handle = {
            .Instance = SPI1,
            .Init = *init
        }
    };

    cmr_rccSPIClockEnable(instance);

    // Configure pins.
    for (size_t i = 0; i < sizeof(*pins) / sizeof(cmr_spiPin_t); i++) {
        const cmr_spiPin_t *pin = ((const cmr_spiPin_t *) pins) + i;

        GPIO_InitTypeDef pinConfig = {
            .Pin = pin->pin,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
            .Alternate = GPIO_AF5_SPI1,     // All SPI ports on AF5.
        };
        HAL_GPIO_Init(pin->port, &pinConfig);
    }

    if (HAL_SPI_Init(&spi->handle) != HAL_OK) {
        cmr_panic("HAL_SPI_Init() failed!");
    }
}

#endif /* HAL_SPI_MODULE_ENABLED */

