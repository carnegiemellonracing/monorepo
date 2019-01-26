#ifndef CAN_TASKS_H
#define CAN_TASKS_H

// ------------------------------------------------------------------------------------------------
// Includes

#include <stdint.h>

// ------------------------------------------------------------------------------------------------
// Task priorities

extern const uint32_t canTX100HzTaskPriority;
extern const uint32_t canTX10HzTaskPriority;
extern const uint32_t canRXTaskPriority;

// ------------------------------------------------------------------------------------------------
// Task prototypes

void canTX100Hz_task(void *pvParameters);
void canTX10Hz_task(void *pvParameters);
void canRX_task(void *pvParameters);

#endif /* CAN_TASKS_H */
