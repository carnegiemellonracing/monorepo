#ifndef TEST_FAKE_CMR_SENSORS_H
#define TEST_FAKE_CMR_SENSORS_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
    void *sensors;
    size_t sensorsLen;
} cmr_sensorList_t;

#endif /* TEST_FAKE_CMR_SENSORS_H */
