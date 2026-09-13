/**
 * @file i2c.c
 * @brief Board-specific I2C implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "i2c.h"                 // Interface to implement
#include <MLX90640_I2C_Driver.h> // MLX90640 I2C driver interface to implement

#define I2C_TIMEOUT 100

/** @brief Primary I2C interface. */
static cmr_i2c_t i2c;

/** @brief Raw byte buffer for big-endian word conversion. */
static uint8_t i2cRaw[832 * 2];

/**
 * @brief Initializes the I2C interface.
 */
void i2cInit(void) {
    cmr_i2cInit(
        &i2c, I2C1,
        I2C_CLOCK_HI, 0,
        GPIOB, GPIO_PIN_6,      // I2C1 SCL port/pin.
        GPIOB, GPIO_PIN_7       // I2C1 SDA port/pin.
    );
}

/**
 * @brief Reads `nWords` 16-bit big-endian words starting at `memAddr`.
 *
 * @return 0 on success, negative on error.
 */
int i2cReadWords(uint16_t devAddr, uint16_t memAddr, uint16_t *out, size_t nWords) {
    if (cmr_i2cMemRX(
            &i2c, devAddr, memAddr, I2C_MEMADD_SIZE_16BIT,
            i2cRaw, nWords * 2, I2C_TIMEOUT
        ) != 0) {
        return -1;
    }

    for (size_t i = 0; i < nWords; i++) {
        out[i] = ((uint16_t) i2cRaw[2 * i] << 8) | i2cRaw[2 * i + 1];
    }

    return 0;
}

/**
 * @brief Writes a 16-bit word to `memAddr`.
 *
 * @return 0 on success, negative on error.
 */
int i2cWriteWord(uint16_t devAddr, uint16_t memAddr, uint16_t data) {
    uint8_t cmd[4] = {
        (uint8_t)(memAddr >> 8), (uint8_t)(memAddr & 0xFF),
        (uint8_t)(data >> 8), (uint8_t)(data & 0xFF)
    };
    return cmr_i2cTX(&i2c, devAddr, cmd, sizeof(cmd), I2C_TIMEOUT);
}

/*
 * MLX90640 I2C driver glue.
 *
 * The Melexis MLX90640 library (mlx90640-library/) is hardware-agnostic and
 * calls out to the MLX90640_I2C* functions declared in MLX90640_I2C_Driver.h.
 * These are implemented here on top of the board's CMR I2C interface. All
 * MLX90640 register and RAM accesses are 16-bit big-endian words, matching
 * i2cReadWords()/i2cWriteWord() above.
 *
 * Return convention: the library treats 0 as success (MLX90640_NO_ERROR) and
 * any negative value as an error; i2cReadWords()/i2cWriteWord() already return
 * 0 on success and -1 on failure.
 */

/**
 * @brief Initializes the MLX90640 I2C bus.
 *
 * The bus is already brought up by i2cInit() during board startup, so this is
 * a no-op provided only to satisfy the library interface.
 */
void MLX90640_I2CInit(void) {
    // Bus initialized by i2cInit().
}

/**
 * @brief Reads `nMemAddressRead` 16-bit words starting at `startAddress`.
 *
 * @return 0 on success, negative on error.
 */
int MLX90640_I2CRead(
    uint8_t slaveAddr, uint16_t startAddress,
    uint16_t nMemAddressRead, uint16_t *data
) {
    return i2cReadWords(slaveAddr, startAddress, data, nMemAddressRead);
}

/**
 * @brief Writes a 16-bit word to `writeAddress`.
 *
 * @return 0 on success, negative on error.
 */
int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data) {
    return i2cWriteWord(slaveAddr, writeAddress, data);
}

/**
 * @brief Issues an I2C general-call reset (address 0x00, command 0x06).
 *
 * @return 0 on success, negative on error.
 */
int MLX90640_I2CGeneralReset(void) {
    uint8_t cmd = 0x06;
    return cmr_i2cTX(&i2c, 0x00, &cmd, sizeof(cmd), I2C_TIMEOUT);
}

/**
 * @brief Sets the I2C bus frequency.
 *
 * The CMR I2C interface fixes the bus clock at init time, so this is a no-op
 * provided only to satisfy the library interface.
 */
void MLX90640_I2CFreqSet(int freq) {
    (void) freq;    // Placate compiler.
}
