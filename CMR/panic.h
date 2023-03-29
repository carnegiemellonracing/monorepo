/**
 * @file panic.h
 * @brief Catastrophic, non-recoverable error handling.
 *
 * The purpose of `cmr_panic()` is to indicate a catastrophic, non-recoverable
 * error has occurred. For driver initialization, this usually means an invalid
 * configuration has been specified. Otherwise, `panic()` MUST NEVER OCCUR,
 * except in debugging environments (e.g for catching assertion failures).
 *
 * The intended result of a `cmr_panic()` is a halted system. The default
 * implementation just busy-spins forever. However, boards can provide some
 * additional behavior by implementing `cmr_panicSetup()` (e.g., to illuminate a
 * status LED).
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CMR_PANIC_H
#define CMR_PANIC_H

void cmr_panicSetup(void);

void cmr_panic(const char *fmt, ...)
#ifdef __GNUC__
    __attribute__((noreturn))
#endif
    ;

#endif /* CMR_PANIC_H */

