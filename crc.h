/*
 * crc.h
 *
 *  Created on: Jul 26, 2020
 *      Author: vamsi
 */

#ifndef CRC_H_
#define CRC_H_

#include <stdint.h>

typedef uint16_t crc16_t;

void crcInit(void);
crc16_t calculateCRC(uint8_t message[], uint16_t numBytes);

#endif /* CRC_H_ */
