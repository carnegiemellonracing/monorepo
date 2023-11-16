/**
 *  @file parser.h
 *  @author Zach Pomper (zbp@cmu.edu)
 *  @brief CAN <-> JSON formatting <-> CBOR transmission
 */

#ifndef _PARSER_H_
#define _PARSER_H_

#include <stdint.h>     /* integer types */
#include <stdlib.h>     /* size_t */
#include <stdbool.h>    /* bool */

void setSignalEnable(uint32_t kind, bool is_enabled);
int parseData(uint32_t bus, uint16_t id, const uint8_t msg[], size_t len);
void parserInit(void);

/**
 * @brief Hard maximum on the length of the signal vector produced by
 * the JSON configuration. (Used by the sampler).
 */
#define MAX_SIGNALS 80

extern int signals_parsed;

#endif  /* _PARSER_H_ */
