/*
 * getopt.c - implementation of getopt()/getopt_long()/getopt_long_only()
 * for platforms that don't already provide them (Windows/MSVC).
 *
 * On POSIX-like systems this file compiles to an (almost) empty
 * translation unit, since getopt.h pulls in the system implementation
 * there instead. It is safe to always add getopt.c to your build on
 * every platform.
 *
 * Behavior notes:
 *  - Short options follow traditional getopt() semantics: optstring
 *    characters may be followed by ':' (required argument) or '::'
 *    (optional argument, GNU extension). A leading ':' in optstring
 *    requests "silent" error reporting (returns ':' instead of '?'
 *    for a missing argument).
 *  - Combined short options are supported ("-abc" == "-a -b -c") when
 *    none of them take an argument, and "-oARG" / "-o ARG" both work
 *    for options that do.
 *  - getopt_long() supports "--name", "--name=value", "--name value",
 *    and unambiguous prefix matching of long option names.
 *  - This implementation does NOT permute argv (it stops scanning for
 *    options as soon as it hits the first non-option argument), which
 *    matches classic POSIX getopt() behavior rather than GNU's
 *    argv-reordering behavior.
 *
 * Public domain / CC0 - do whatever you like with this file.
 */

#include "getopt.h"

#if defined(_WIN32) || defined(_WIN64)

#include <string.h>
#include <stdio.h>

char *optarg = NULL;
int   optind = 1;
int   opterr = 1;
int   optopt = 0;

/* Position within the current argv element when parsing grouped short
 * options like "-abc". Reset whenever we move to a new argv element. */
static int optpos = 1;

static void report_error(const char *argv0, const char *fmt, int ch)
{
    if (opterr) {
        fprintf(stderr, "%s: ", argv0 ? argv0 : "");
        fprintf(stderr, fmt, ch);
        fprintf(stderr, "\n");
    }
}

int getopt(int argc, char *const argv[], const char *optstring)
{
    const char *spec;
    int         silent = 0;

    optarg = NULL;

    if (optstring && optstring[0] == ':') {
        silent = 1;
        optstring++;
    }

    if (optind >= argc)
        return -1;

    /* Starting a fresh argv element */
    if (optpos == 1) {
        const char *arg = argv[optind];

        if (arg == NULL || arg[0] != '-' || arg[1] == '\0')
            return -1; /* not an option, or bare "-" (stdin marker) */

        if (arg[1] == '-' && arg[2] == '\0') {
            optind++; /* "--" ends option processing */
            return -1;
        }
    }

    {
        const char *arg = argv[optind];
        int         ch  = arg[optpos];

        spec = strchr(optstring ? optstring : "", ch);

        if (ch == ':' || spec == NULL) {
            optopt = ch;
            if (!silent)
                report_error(argv[0], "invalid option -- '%c'", ch);
            if (arg[optpos + 1] != '\0')
                optpos++;
            else {
                optind++;
                optpos = 1;
            }
            return '?';
        }

        if (spec[1] == ':') {
            /* Option takes an argument */
            if (arg[optpos + 1] != '\0') {
                /* Argument attached: "-oARG" */
                optarg = (char *)&arg[optpos + 1];
                optind++;
                optpos = 1;
            } else if (spec[2] == ':') {
                /* Optional argument, GNU-style: only if attached */
                optarg = NULL;
                optind++;
                optpos = 1;
            } else if (optind + 1 < argc) {
                /* Argument is the next argv element: "-o ARG" */
                optarg = (char *)argv[optind + 1];
                optind += 2;
                optpos = 1;
            } else {
                optopt = ch;
                if (!silent)
                    report_error(argv[0], "option requires an argument -- '%c'", ch);
                optind++;
                optpos = 1;
                return silent ? ':' : '?';
            }
        } else {
            /* No argument; may be grouped with more short options */
            if (arg[optpos + 1] != '\0') {
                optpos++;
            } else {
                optind++;
                optpos = 1;
            }
        }

        return ch;
    }
}

static int match_long_option(const char *name, const struct option *longopts,
                              int *exact_index, int *unique_index)
{
    size_t namelen = strlen(name);
    int    i;
    int    matches = 0;
    int    match_i = -1;

    for (i = 0; longopts[i].name != NULL; i++) {
        if (strcmp(longopts[i].name, name) == 0) {
            *exact_index = i;
            return 1; /* exact match wins immediately */
        }
        if (strncmp(longopts[i].name, name, namelen) == 0) {
            matches++;
            match_i = i;
        }
    }

    if (matches == 1) {
        *unique_index = match_i;
        return 1;
    }
    return matches > 1 ? -1 /* ambiguous */ : 0 /* no match */;
}

static int getopt_long_impl(int argc, char *const argv[], const char *optstring,
                             const struct option *longopts, int *longindex,
                             int long_only)
{
    optarg = NULL;

    if (optind >= argc)
        return -1;

    {
        const char *arg = argv[optind];
        const char *name;
        char       *eq;
        char        namebuf[256];
        int         exact_index = -1, unique_index = -1;
        int         found;

        if (arg == NULL || arg[0] != '-' || arg[1] == '\0')
            return -1;

        if (arg[0] == '-' && arg[1] == '-') {
            if (arg[2] == '\0') {
                optind++;
                return -1; /* "--" ends option processing */
            }
            name = arg + 2;
        } else if (long_only && arg[1] != '-') {
            name = arg + 1;
        } else {
            return getopt(argc, argv, optstring);
        }

        eq = strchr(name, '=');
        if (eq) {
            size_t len = (size_t)(eq - name);
            if (len >= sizeof(namebuf))
                len = sizeof(namebuf) - 1;
            memcpy(namebuf, name, len);
            namebuf[len] = '\0';
            name = namebuf;
        }

        found = match_long_option(name, longopts, &exact_index, &unique_index);

        if (found == -1) {
            if (opterr)
                fprintf(stderr, "%s: option '--%s' is ambiguous\n",
                        argv[0] ? argv[0] : "", name);
            optind++;
            return '?';
        }

        if (found == 0) {
            if (long_only)
                return getopt(argc, argv, optstring);
            if (opterr)
                fprintf(stderr, "%s: unrecognized option '--%s'\n",
                        argv[0] ? argv[0] : "", name);
            optind++;
            return '?';
        }

        {
            int idx = (exact_index >= 0) ? exact_index : unique_index;
            const struct option *opt = &longopts[idx];

            if (longindex)
                *longindex = idx;

            if (opt->has_arg == required_argument || opt->has_arg == optional_argument) {
                if (eq) {
                    optarg = eq + 1;
                    optind++;
                } else if (opt->has_arg == required_argument) {
                    if (optind + 1 < argc) {
                        optarg = (char *)argv[optind + 1];
                        optind += 2;
                    } else {
                        if (opterr)
                            fprintf(stderr, "%s: option '--%s' requires an argument\n",
                                    argv[0] ? argv[0] : "", opt->name);
                        optind++;
                        return '?';
                    }
                } else {
                    optarg = NULL;
                    optind++;
                }
            } else {
                optarg = NULL;
                optind++;
            }

            if (opt->flag) {
                *opt->flag = opt->val;
                return 0;
            }
            return opt->val;
        }
    }
}

int getopt_long(int argc, char *const argv[], const char *optstring,
                 const struct option *longopts, int *longindex)
{
    return getopt_long_impl(argc, argv, optstring, longopts, longindex, 0);
}

int getopt_long_only(int argc, char *const argv[], const char *optstring,
                      const struct option *longopts, int *longindex)
{
    return getopt_long_impl(argc, argv, optstring, longopts, longindex, 1);
}


#endif /* _WIN32 || _WIN64 */