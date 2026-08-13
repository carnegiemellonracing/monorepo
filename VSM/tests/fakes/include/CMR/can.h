#ifndef TEST_FAKE_CMR_CAN_H
#define TEST_FAKE_CMR_CAN_H

#include <stdint.h>

#include <CMR/tasks.h>
#include <CMR/can_types.h>

typedef struct {
    const uint16_t canID;
    const TickType_t timeoutWarn_ms;
    const cmr_canWarn_t warnFlag;
    const TickType_t timeoutError_ms;
    const cmr_canError_t errorFlag;
    volatile TickType_t lastReceived_ms;
    volatile uint8_t payload[8];
} cmr_canRXMeta_t;

int cmr_canRXMetaTimeoutError(const cmr_canRXMeta_t *meta, TickType_t now_ms);

#endif /* TEST_FAKE_CMR_CAN_H */
