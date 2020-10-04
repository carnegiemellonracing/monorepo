/*
 * crc.c
 *
 *  Created on: Jul 26, 2020
 *      Author: vamsi
 */
#include "crc.h"

#if (REFLECT_DATA == TRUE)
#undef  REFLECT_DATA
#define REFLECT_DATA(X)			((unsigned char) reflect((X), 8))
#else
#undef  REFLECT_DATA
#define REFLECT_DATA(X)			(X)
#endif

#if (REFLECT_REMAINDER == TRUE)
#undef  REFLECT_REMAINDER
#define REFLECT_REMAINDER(X)	((crc16_t) reflect((X), WIDTH))
#else
#undef  REFLECT_REMAINDER
#define REFLECT_REMAINDER(X)	(X)
#endif

// File static constants for calculating crc
static const uint8_t WIDTH = 8 * sizeof(crc16_t);
static const uint16_t TOPBIT = 1 << 15; //(WIDTH-1);
static const uint16_t POLYNOMIAL = 0x8005;

// Lookup table for each byte of CRC data
static crc16_t crcTable[256];

/*********************************************************************
 *
 * Function:    reflect()
 * 
 * Description: Reorder the bits of a binary sequence, by reflecting
 *				them about the middle position.
 *
 * Notes:		No checking is done that nBits <= 32.
 *
 * Returns:		The reflection of the original data.
 *
 *********************************************************************/
static unsigned long
reflect(unsigned long data, unsigned char nBits)
{
	unsigned long  reflection = 0x00000000;
	unsigned char  bit;

	/*
	 * Reflect the data about the center bit.
	 */
	for (bit = 0; bit < nBits; ++bit)
	{
		/*
		 * If the LSB bit is set, set the reflection of it.
		 */
		if (data & 0x01)
		{
			reflection |= (1 << ((nBits - 1) - bit));
		}

		data = (data >> 1);
	}

	return (reflection);

}	/* reflect() */

// If we do not want to create all of them in one large init, we can move caching of crc
//   data to calculation, and only calculate if seeing a new byte
void crcInit() {
	crc16_t remainder;
	
	for(int dividend = 0; dividend < 256; ++dividend) {
		remainder = dividend << (WIDTH - 8); // Start with the dividend followed by zeros
		
		// Perform modulo-2 division, one bit at a time
		for(uint8_t bit = 8; bit > 0; --bit) {
			if(remainder & TOPBIT) { // Try to divide the current data bit
				remainder = (remainder << 1) ^ POLYNOMIAL;
			}
			else {
				remainder = (remainder << 1);
			}
		}
		
		crcTable[dividend] = remainder; // Store the result into the table
	}
}

crc16_t calculateCRC(uint8_t message[], uint16_t numBytes) {
	
	uint8_t data;
	crc16_t crcRemainder = 0;

	for (int byte = 0; byte < numBytes; ++byte) {
		data = REFLECT_DATA(message[byte]) ^ (crcRemainder >> (WIDTH - 8));
		crcRemainder = crcTable[data] ^ (crcRemainder << 8);
	}

	//return crcRemainder;
	int FINAL_XOR_VALUE = 0x000;
	return (REFLECT_REMAINDER(crcRemainder) ^ FINAL_XOR_VALUE);
}

