#ifndef TEST_FAKE_CMR_GPIO_H
#define TEST_FAKE_CMR_GPIO_H

#include <stddef.h>

typedef struct {
    void *port;
    unsigned int init;
} cmr_gpioPinConfig_t;

void cmr_gpioWrite(size_t pin, int value);
int cmr_gpioRead(size_t pin);

#endif /* TEST_FAKE_CMR_GPIO_H */
