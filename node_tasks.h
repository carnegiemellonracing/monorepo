#ifndef NODE_TASKS_H
#define NODE_TASKS_H

// ------------------------------------------------------------------------------------------------
// Includes

#include <stdint.h>

// ------------------------------------------------------------------------------------------------
// Task priorities

extern const uint32_t mcuStatusLEDTaskPriority;
extern const uint32_t adcTaskPriority;

// ------------------------------------------------------------------------------------------------
// Task prototypes

void mcuStatusLED_task(void *pvParameters);
void adc_task(void *pvParameters);

#endif /* NODE_TASKS_H */
