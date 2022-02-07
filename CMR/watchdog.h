/**
 * @file watchdog.h
 * @brief Window Watchdog wrapper implementation.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CMR_WWDG_H
#define CMR_WWDG_H

#include <stm32f4xx_hal.h>

#ifdef HAL_WWDG_MODULE_ENABLED

typedef struct {    
    WWDG_HandleTypeDef handle;      /**< @brief HAL WWDG handle. */
} cmr_wwdg_t;

void cmr_wwdgInit(cmr_wwdg_t *wwdg,
        const WWDG_InitTypeDef *init);

void cmr_wwdgStart(cmr_wwdg_t *wwdg);

void cmr_wwdgKick(cmr_wwdg_t *wwdg);

#endif /* HAL_WWDG_MODULE_ENABLED */

#endif /* CMR_WWDG_H */