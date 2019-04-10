#include <stdint.h>
#include <CMR/i2c.h>

#ifndef SEGMENTS_H
#define SEGMENTS_H

int sendSegmentLEDsMessage(uint8_t *data, size_t dataLength);

int sendSegmentLEDsCommand(uint8_t commandAddr, uint8_t aByte);

void initSegmentDisplays(cmr_i2c_t *i2c);

uint8_t addrBySegno(uint8_t segno);

void writeSegmentText(uint8_t *data, uint32_t len);

#endif /*SEGMENTS_H*/
