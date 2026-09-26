/**
 * @file ingest.c
 * @brief Deferred CAN message processing.
 *
 * Single-producer/single-consumer ring buffer between the CAN RX interrupts
 * (producer) and the ingest task (consumer).
 *
 * @note All CAN RX IRQs are configured at the same NVIC priority, so they
 * cannot preempt each other and together act as a single producer.
 *
 * @author Carnegie Mellon Racing
 */

#include <string.h>     // memcpy()

#include <stm32f4xx_hal.h>  // __DMB()
#include <CMR/tasks.h>  // Task interface

#include "ingest.h"    // Interface to implement
#include "parser.h"     // parseData()
// #include <CMR/rtc.h>
// #include "memorator.h"

_Static_assert(
    (INGEST_QUEUE_LEN & (INGEST_QUEUE_LEN - 1)) == 0,
    "INGEST_QUEUE_LEN must be a power of two"
);

/** @brief Ingest task priority. */
static const uint32_t ingest_priority = 5;

/** @brief Ingest period (milliseconds). */
static const TickType_t ingest_period_ms = 1;

/** @brief Ingest task. */
static cmr_task_t ingest_task;

/** @brief A queued CAN message. */
typedef struct {
    uint8_t bus;        /**< @brief Bus the message arrived on. */
    uint8_t len;        /**< @brief Payload length, in bytes. */
    uint16_t id;        /**< @brief CAN ID. */
    uint8_t data[8];    /**< @brief Payload data. */
} ingestMsg_t;

/** @brief RX message queue. */
static ingestMsg_t queue[INGEST_QUEUE_LEN];

/** @brief Next slot to write (only modified by the ISR). */
static volatile uint32_t queueHead = 0;

/** @brief Next slot to read (only modified by the ingest task). */
static volatile uint32_t queueTail = 0;

/** @brief Number of messages dropped because the queue was full. */
static volatile uint32_t droppedCount = 0;

/**
 * @brief Enqueues a received CAN message for the ingest task.
 *
 * @warning Called from an interrupt handler!
 *
 * @param bus The bus the message arrived on.
 * @param canID The message's CAN ID.
 * @param data The received data.
 * @param dataLen The received data's length.
 */
void ingestEnqueueFromISR(uint32_t bus, uint16_t canID, const void *data, size_t dataLen) {
    uint32_t head = queueHead;
    if (head - queueTail >= INGEST_QUEUE_LEN) {
        droppedCount++;
        return;
    }

    if (dataLen > sizeof(queue[0].data)) {
        dataLen = sizeof(queue[0].data);
    }

    ingestMsg_t *msg = &queue[head & (INGEST_QUEUE_LEN - 1)];
    msg->bus = (uint8_t) bus;
    msg->len = (uint8_t) dataLen;
    msg->id = canID;
    memcpy(msg->data, data, dataLen);

    // Publish only after the slot is fully written.
    __DMB();
    queueHead = head + 1;
}

/**
 * @brief Gets the number of messages dropped due to a full queue.
 *
 * @return Dropped message count.
 */
uint32_t ingestDroppedCount(void) {
    return droppedCount;
}

/**
 * @brief Task for parsing queued CAN messages.
 *
 * Parses up to `INGEST_MAX_MSGS_PER_RUN` messages, or until the queue is
 * empty, then yields for one period.
 *
 * @param pvParameters Ignored.
 *
 * @return Does not return.
 */
static void ingest(void *pvParameters) {
    (void) pvParameters;

    while (1) {
        uint32_t tail = queueTail;
        for (uint32_t n = 0; n < INGEST_MAX_MSGS_PER_RUN && tail != queueHead; n++) {
            const ingestMsg_t *msg = &queue[tail & (INGEST_QUEUE_LEN - 1)];

            int ret = parseData(msg->bus, msg->id, msg->data, msg->len);
            configASSERT(ret != 1);
            configASSERT(ret != 2);
            configASSERT(ret != 3);
            configASSERT(ret != 4);
            configASSERT(ret == 0);

            // RTC_TimeTypeDef timestamp = getRTCTime();
            // memoratorWrite(msg->id, timestamp, msg->len, (uint8_t *) msg->data);

            // Release the slot only after we are done reading it.
            __DMB();
            queueTail = ++tail;
        }

        // Re-anchor to now so we always block (and let lower priority tasks
        // run), even if parsing overran the period.
        TickType_t lastWakeTime = xTaskGetTickCount();
        vTaskDelayUntil(&lastWakeTime, ingest_period_ms);
    }
}

/**
 * @brief Initializes the ingest task.
 */
void ingestInit(void) {
    cmr_taskInit(
        &ingest_task,
        "ingest",
        ingest_priority,
        ingest,
        NULL
    );
}
