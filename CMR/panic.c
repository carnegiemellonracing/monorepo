/**
 * @file panic.c
 * @brief Default panic implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include "panic.h"  // interface to implement

/**
 * @brief Default pre-panic setup implementation.
 *
 * Nothing happens by default; this should be overridden if custom behavior is
 * desired.
 */
void cmr_panicSetup(void) {
    // Nothing to do.
}

#ifdef __GNUC__
void cmr_panicSetup(void) __attribute__((weak));
#endif

/**
 * @brief Indicates a catastrophic, non-recoverable error has occurred.
 *
 * @param fmt The formatting string.
 * @param ... Additional arguments for formatting.
 *
 * @return Does not return.
 */
void cmr_panic(const char *fmt, ...) {
    (void) fmt;

    cmr_panic_setup();

    while (1) {
        continue;
    }
}

