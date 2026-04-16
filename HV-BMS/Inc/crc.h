/*
 * crc.h
 *
 *  Created on: Jul 26, 2020
 *      Author: vamsi
 */

#ifndef CRC_H_
#define CRC_H_
#include "FreeRTOS.h"

uint16_t calculateCRC(uint8_t *pBuf, int nLen);


#endif /* CRC_H_ */
