/**
 * @file i2c.h
 * @brief Board-specific I2C interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef I2C_H
#define I2C_H

#include <CMR/i2c.h>    // I2C interface

void i2cInit(void);
int i2cReadWords(uint16_t devAddr, uint16_t memAddr, uint16_t *out, size_t nWords);
int i2cWriteWord(uint16_t devAddr, uint16_t memAddr, uint16_t data);

#endif /* I2C_H */
