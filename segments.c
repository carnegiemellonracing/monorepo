#include "segments.h"
#include <stdint.h>

static const uint8_t SEGMENT_CONFIG	=	0x0C;
static const uint8_t DISPLAY_TEST	=	0x07;
static const uint8_t  MAX_BRIGHTNESS=	0x02;
static const uint8_t CONFIGURATION	=	0x04;
static const uint32_t TIMEOUT_LEN    =   100;

static cmr_i2c_t *i2c_instance;



/**
 * @brief Send segment LEDs a message
 *
 * @param commandAddr Command address
 * @param data Array of data to be sent
 * @param dataLength length of the data array
 *
 * @retval 0
 */
int sendSegmentLEDsMessage(uint8_t *data, size_t dataLength){
	const uint8_t devAddr = 0x60;
	cmr_i2cTX(i2c_instance, devAddr, data, dataLength, TIMEOUT_LEN);
	return 0;
}

/**
 * @brief Send segment LEDs a single byte command
 *
 * @param commandAddr Command address
 * @param aByte Value of command to be sent
 *
 * @retval 0
 */
int sendSegmentLEDsCommand(uint8_t commandAddr, uint8_t aByte){
    uint8_t byteData[2];
    byteData[0] = commandAddr;
    byteData[1] = aByte;
	sendSegmentLEDsMessage(byteData, 2);
	return 0;
}
/**
 * @brief Initialize segment displays
 *
 * @retval none
 */
void initSegmentDisplays(cmr_i2c_t *i2c) {
	i2c_instance = i2c;
	sendSegmentLEDsCommand(DISPLAY_TEST, 0x0);
	sendSegmentLEDsCommand(CONFIGURATION, 0x01);
	sendSegmentLEDsCommand(SEGMENT_CONFIG, 0xFF);
	sendSegmentLEDsCommand(MAX_BRIGHTNESS, 0xF);
}

/**
 * @brief Returns the address based on the segment
 *
 * @param segno Segment number
 *
 * @retval Segment address
 */
uint8_t addrBySegno(uint8_t segno) {
	uint8_t seg_addr;
	switch(segno) {
		case 0:
			seg_addr = 0x20;
			break;
		case 1:
			seg_addr = 0x21;
			break;
		case 2:
			seg_addr = 0x22;
			break;
		case 3:
			seg_addr = 0x23;
			break;
		case 4:
			seg_addr = 0x26;
			break;
		case 5:
			seg_addr = 0x27;
			break;
		case 6:
			seg_addr = 0x24;
			break;
		case 7:
			seg_addr = 0x25;
			break;
		default:
			return  -1;
	}
	return seg_addr;
}

/**
 * @brief Write text on the segment display
 *
 * @param data Array of data to write
 * @param len Length of data array
 *
 * @retval none
 */
void writeSegmentText(uint8_t *data, uint32_t len) {
	const uint8_t num_segs = 8;
	if (len > num_segs) {
		len = num_segs;
	}
	//text is right-aligned currently
	uint8_t segno = num_segs - len;
	uint32_t i;
	for (i = 0; i < len; i++) {
		sendSegmentLEDsCommand(addrBySegno(segno++), data[i]);
	}
	for(i = 0; i < num_segs - len; i++) {
		sendSegmentLEDsCommand(addrBySegno(i), ' ');
	}
}
