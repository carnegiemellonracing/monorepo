/**
 * @file watchdog.c
 * @brief Window Watchdog wrapper implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "../watchdog.h"

#ifdef HAL_WWDG_MODULE_ENABLED

#include "../rcc.h"    // cmr_rccCANClockEnable()
#include "../panic.h"  // cmr_panic()


/** @brief Initialize the watchdog
 *
 *  @param wwdg     Pointer to wwdg interface
 *  @param init     Params to initialize watchdog with
 */
void cmr_wwdgInit(cmr_wwdg_t *wwdg, const WWDG_InitTypeDef *init) {
    // Check if the system has resumed from WWDG reset
    if(__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET)  {
        /* WWDGRST flag set */
        // TODO: Handle Case when Watchdog Resets MCU

        /* Clear reset flags */
        __HAL_RCC_CLEAR_RESET_FLAGS();
    }  else  {
        /* WWDGRST flag is not set */
    }

    // Configure the Watchdog
    *wwdg = (cmr_wwdg_t) {
        .handle = {
            .Instance = WWDG,
            .Init = *init
        }
    };

    __HAL_RCC_WWDG_CLK_ENABLE();
}


/** @brief Start the watchdog
 *
 *  @param wwdg Pointer to wwdg interface
 */
void cmr_wwdgStart(cmr_wwdg_t *wwdg) {
    // Start the watchdog
    if (HAL_WWDG_Init(&wwdg->handle) != HAL_OK) {
        cmr_panic("HAL_WWDG_Init() failed!");
    }
}

/** @brief Kick the watchdog
 *
 *  @param wwdg Pointer to wwdg interface
 */
void cmr_wwdgKick(cmr_wwdg_t *wwdg) {
    if (HAL_WWDG_Refresh(&wwdg->handle) != HAL_OK) {
        cmr_panic("HAL_WWDG_Refresh() failed!");
    }
}

#endif /* HAL_WWDG_MODULE_ENABLED */