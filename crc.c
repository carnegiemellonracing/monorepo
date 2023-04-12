/*
 * crc.c
 *
 *  Created on: Jul 26, 2020
 *      Author: vamsi
 */
#include "crc.h"



//lookup for reverse
unsigned char lookup[16] = {
0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf};

//CRC Mask polynomial
unsigned long long mask = 0x18005;

uint8_t reverse(uint8_t n) {
	return (lookup[n & 0b1111] << 4) | lookup[n >> 4];
}

unsigned long long concat_arr(uint8_t *values, uint8_t numBytes) {
	unsigned long long num = 0;
	for (int i = 0; i < numBytes; i++) {
		num = num << 8;
		num |= x[i];
	}
	return num;
}

uint8_t msb(unsigned long long v) {
	uint8_t r = 0;
	while (v >>= 1) {
		r++;
	}
	return r;
}

uint16_t crc(uint8_t *message, uint8_t len) {
	for (int i = 0; i < len; i++) {
		message[i] = reverse(message[i]);
	}
	message[0] = message[0] ^ 0xFF;
	message[1] = message[1] ^ 0xFF;


}
