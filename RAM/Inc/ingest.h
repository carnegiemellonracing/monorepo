/**
 * @file ingest.h
 * @brief Deferred CAN message processing.
 *
 * The CAN RX interrupt only copies each message into its RX meta payload and
 * enqueues it here. The ingest task then drains the queue and hands every
 * message to parseData() outside of interrupt context.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef INGEST_H
#define INGEST_H

#include <stdint.h>     // integer types
#include <stddef.h>     // size_t

/**
 * @brief Number of CAN messages the RX queue can hold.
 *
 * @warning Must be a power of two (indices wrap with a mask).
 *
 * @note Worst case is ~4 msgs/ms per 500K bus, so ~12 msgs/ms across all three
 * buses. 1024 entries gives ~80 ms of headroom before messages are dropped.
 */
#define INGEST_QUEUE_LEN 1024

/**
 * @brief Maximum number of messages the ingest task parses before yielding.
 *
 * The task yields after this many messages or once the queue is empty,
 * whichever comes first.
 */
#define INGEST_MAX_MSGS_PER_RUN (INGEST_QUEUE_LEN / 2)

void ingestInit(void);
void ingestEnqueueFromISR(uint32_t bus, uint16_t canID, const void *data, size_t dataLen);
uint32_t ingestDroppedCount(void);

#endif /* INGEST_H */
