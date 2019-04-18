/**
 * @file segments.h
 * @brief Segment display interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef SEGMENTS_H
#define SEGMENTS_H

#include <stddef.h>     // size_t

void segmentsInit(void);
void segmentsWrite(char *data, size_t len);

#endif /* SEGMENTS_H */

