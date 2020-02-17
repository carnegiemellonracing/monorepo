/**
 *  @file parser.h
 *  @author Zach Pomper (zbp@cmu.edu)
 *  @brief CAN <-> JSON formatting <-> CBOR transmission
 *
 *  How to use this interface:
 *  Call parseData(canid, data, datalen) for each message with relevant
 *  sample data.
 *  Periodically call parserFmtMsg, use the raw_msg handle to send the
 *  message out.
 *  Call parserClearMsg after consuming the message.
 *
 *  @warning must be protected against multiple access externally
 *  (most likely between parseData and parserFmtMsg)
 *
 */

#ifndef _PARSER_H_
#define _PARSER_H_

#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>

int parseData(uint16_t id, uint8_t msg[], size_t len);
int parserInit(void);
void parserClearMsg(void);
ssize_t parserFmtMsg(void);


#define MAX_MESSAGE_LEN 2000
/* The formatted message formatted for transmission */
extern uint8_t raw_msg[MAX_MESSAGE_LEN];

#endif  /* _PARSER_H_ */
