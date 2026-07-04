/*
 * getopt.h - portable, vendorable getopt/getopt_long header
 *
 * Usage:
 *   #include "getopt.h"
 *
 * On POSIX-like systems (Linux, macOS, *BSD, Cygwin, etc.) this header
 * simply includes the platform's own <unistd.h> / <getopt.h>, which
 * already declare getopt() and, on most of those systems, getopt_long()
 * as well. No extra object file is needed there.
 *
 * On Windows (MSVC, or any toolchain that does not ship a getopt
 * implementation), this header instead declares a self-contained
 * drop-in replacement. Compile and link the accompanying getopt.c
 * together with your program to get working getopt()/getopt_long()/
 * getopt_long_only() calls.
 *
 * This lets application code call getopt() / getopt_long() the same
 * way on every platform without any #ifdef in the calling code.
 *
 * Public domain / CC0 - do whatever you like with this file.
 */

#ifndef GETOPT_H_INCLUDED
#define GETOPT_H_INCLUDED

#if defined(_WIN32) || defined(_WIN64)

#ifdef __cplusplus
extern "C" {
#endif

/* --- struct option / has_arg constants (GNU-compatible) --- */

#ifndef no_argument
#define no_argument        0
#endif
#ifndef required_argument
#define required_argument  1
#endif
#ifndef optional_argument
#define optional_argument  2
#endif

struct option {
    const char *name;    /* long option name, without leading "--" */
    int         has_arg; /* no_argument | required_argument | optional_argument */
    int        *flag;    /* if non-NULL, val is stored here and getopt_long returns 0 */
    int         val;     /* value to return, or to store in *flag */
};

/* --- getopt() state, matching the traditional POSIX globals --- */

extern char *optarg; /* argument for the option that was just parsed */
extern int   optind;  /* index of the next element of argv to process; starts at 1 */
extern int   opterr;  /* if non-zero (default), getopt prints its own error messages */
extern int   optopt;  /* set to the unrecognized/invalid option character */

/* --- function prototypes --- */

int getopt(int argc, char *const argv[], const char *optstring);

int getopt_long(int argc, char *const argv[], const char *optstring,
                 const struct option *longopts, int *longindex);

int getopt_long_only(int argc, char *const argv[], const char *optstring,
                      const struct option *longopts, int *longindex);

#ifdef __cplusplus
}
#endif

#else /* POSIX-like systems already provide getopt */

#include <unistd.h>

/* getopt_long()/getopt_long_only() and struct option live in <getopt.h>
 * on glibc-based Linux, Cygwin, and most other GNU-ish platforms.
 * BSD/macOS provide getopt_long() via <unistd.h> or <getopt.h> as well;
 * including it here if present is harmless where it also exists. */
#if defined(__linux__) || defined(__CYGWIN__) || defined(__GLIBC__) || \
    defined(__APPLE__) || defined(__FreeBSD__) || defined(__NetBSD__) || \
    defined(__OpenBSD__)
#include <getopt.h>
#endif

#endif /* _WIN32 || _WIN64 */

#endif /* GETOPT_H_INCLUDED */