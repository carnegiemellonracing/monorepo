#ifndef TEST_FAKE_FREERTOS_H
#define TEST_FAKE_FREERTOS_H

#include <stdint.h>

typedef uint32_t TickType_t;
typedef unsigned int UBaseType_t;
typedef void *TaskHandle_t;
typedef void (*TaskFunction_t)(void *);

#define taskENTER_CRITICAL() do { } while (0)
#define taskEXIT_CRITICAL() do { } while (0)

#endif /* TEST_FAKE_FREERTOS_H */
