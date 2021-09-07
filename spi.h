/*
 * spi.h
 *
 *  Created on: Sep 7, 2021
 *      Author: vamsi
 */

#ifndef SPI_H_
#define SPI_H_

#include <CMR/spi.h>

int32_t getHVmillivolts();
int32_t getCurrentInstant();
int32_t getCurrentAverage();

void spiInit(void);

#endif /* SPI_H_ */
