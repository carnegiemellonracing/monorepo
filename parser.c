/**
 * @file parser.c
 * @brief Extracts relevant samples from incoming can messages and
 * queues them for sending offboard.
 *
 * Reads in can_fmt.rawh from project folder to describe
 * configuration. Run marshall.py to generate from JSON data.
 * JSON data follows format discussed here:
 * http://cmr.red/jira/browse/EN-131
 *
 * @author Carnegie Mellon Racing
 */

#include "parser.h"                         /* Interface */
#include "sample.h"                         /* Sample queuing */
#include <cJSON.h>                          /* JSON parsing */
#include <stdlib.h>                         /* NULL */
#include <stdint.h>                         /* Usual suspects */
#include <string.h>                         /* strncmp */

#ifndef NFLOAT /* Define this to disable floating point support */
#include <arm_neon.h>                       /* Floating types
                                             * (define NFLOAT for
                                             * local testing) */
#endif

/**
 * @brief Maximum length of a signal name in bytes.
 * @note Name is silently truncated on overrun.
 */
#define MAX_SIGNAL_NAME 30

/**
 * @brief Data type names are <= 9 bytes
 */
#define DT_NAME_MAX 10

/**
 * @brief Data type encapsulation for re-interpretation
 * (e.g. you specify f16 in the JSON config, but the incoming
 * signal sends as a u32).
 */
enum data_type {
    DT_INT8,            /**< @brief i8 datatype */
    DT_INT16,           /**< @brief i16 datatype */
    DT_INT32,           /**< @brief i32 datatype */
    DT_INT64,           /**< @brief i64 datatype */
    DT_UINT8,           /**< @brief u8 datatype */
    DT_UINT16,          /**< @brief u16 datatype */
    DT_UINT32,          /**< @brief u32 datatype */
    DT_UINT64,          /**< @brief u64 datatype */
    DT_FLOAT16,         /**< @brief f16 datatype */
    DT_FLOAT32,         /**< @brief f32 datatype */
    DT_FLOAT64,         /**< @brief f64 datatype */
    DT_UNK,             /**< @brief If you see this, something is wrong.*/
    DT_NUM              /**< @brief Number of data types. */
};

/**
 * @brief Parsed signal information.
 */
struct signal {
    uint32_t id;                        /**< @brief Backing can ID */
    size_t offset;                      /**< @brief Backing offset within
                                         * the relevant message */
    size_t len;                         /**< @brief Length of each sample
                                         * (in bytes) */
    char name[MAX_SIGNAL_NAME];         /**< @brief Signal name
                                         * (mostly useful for debugging
                                         * right now, but we have it anyways) */
    enum data_type dt;                  /**< @brief Type to reinterpret data */
    uint32_t kind;                      /**< @brief Index in the configured
                                         * signal vector, unique among
                                         * all signals parsed. */
};

/**
 * @brief Number of signals configured in the parsed vector.
 */
static int signals_parsed = 0;

/**
 * @brief Metadata on each signal in the vector (filled during parsing)
 */
static struct signal signal_map[MAX_SIGNALS];

/**
 * @brief How we read in the configuration file (use marshall.py to update).
 */
static const char JSON_STRING[] = {
  #include "can_fmt.rawh"
};

/**
 * @brief What each datatype is called in the configuration file.
 * @warning If a signal has a type not resident here,
 * we will never boot successfully.
 */
static const char dtNameMap[DT_NUM][DT_NAME_MAX] = {
    [DT_INT16]   = "i16",
    [DT_INT32]   = "i32",
    [DT_INT64]   = "i64",
    [DT_UINT8]   = "u8",
    [DT_UINT16]  = "u16",
    [DT_UINT32]  = "u32",
    [DT_UINT64]  = "u64",
    [DT_FLOAT16] = "f16",
    [DT_FLOAT32] = "f32",
};

/**
 * @brief According to the configured signal type, find the representing enum
 * @param s The type name
 * @return enum data_type Datatype representing this.
 */
enum data_type dtLookupStr(const char *s) {
    for (int i = 0; i < DT_NUM; i++) {
        if (!strncmp(dtNameMap[i], s, DT_NAME_MAX)) {
            return (enum data_type) i;
        }
    }
    return DT_UNK;
}

/**
 * @brief Parse a CAN message and enqueue it to be sent out later.
 * @param id The ID the message came in on.
 * @param msg The message data
 * @param len The received data length (in bytes)
 * @return int 0 on success, -1 on failure.
 */
int parseData(uint16_t id, const uint8_t msg[], size_t len) {
    struct sample sample;
    struct sample *s = &sample;

    struct signal *sigv[MAX_VAL_PER_SIG];
    int relevant_sigs = 0;
    for (int i = 0; i < signals_parsed; i++) {
        if (signal_map[i].id == id) {
            sigv[relevant_sigs++] = &signal_map[i];
        }

        if (relevant_sigs >= MAX_VAL_PER_SIG - 1) {
            /* There are more subscribing signals on this message
             * That we can handle.
             * This will never happen if the configuration
             * is valid, but I would still rather continue execution here
             * by dropping the rest of the (desired) samples. */
            break;
        }
    }

    if (!relevant_sigs) {
        /* No sample parsed, but that's ok */
        return 0;
    }

    for (size_t i = 0; i < relevant_sigs; i++) {
        struct signal *sig = sigv[i];

        if (sig->offset + sig->len > len) {
            /* Uh oh */
            return -1;
        }

        const void *v_pt = msg + sig->offset;
        switch(sig->dt) {
        case DT_INT8:
            s->v[i].i8 = *(int8_t *) v_pt;
            break;
        case DT_INT16:
            s->v[i].i16 = *(int16_t *) v_pt;
            break;
        case DT_INT32:
            s->v[i].i32 = *(int32_t *) v_pt;
            break;
        case DT_INT64:
            s->v[i].i64 = *(int64_t *) v_pt;
            break;
        case DT_UINT8:
            s->v[i].u8 = *(uint8_t *) v_pt;
            break;
        case DT_UINT16:
            s->v[i].u16 = *(uint16_t *) v_pt;
            break;
        case DT_UINT32:
            s->v[i].u32 = *(uint32_t *) v_pt;
            break;
        case DT_UINT64:
            s->v[i].u64 = *(uint64_t *) v_pt;
            break;
#ifndef NFLOAT /* Define this to disable floating point support */
        case DT_FLOAT16:
            s->v[i].f16 = *(float16_t *) v_pt;
            break;
        case DT_FLOAT32:
            s->v[i].f32 = *(float32_t *) v_pt;
            break;
        case DT_FLOAT64:
            s->v[i].f64 = *(double *) v_pt;
            break;
#endif
        default:
            /* Not entirely sure what the best course of action is here. */
            return -1;
        }

        s->sig[i] = (struct sample_info) {
            .kind = sig->kind,
            .len = sig->len,
        };
    }

    s->values_parsed = relevant_sigs;
    addSample(s);
    return 0;
}

/**
 * @brief Read in the configuration file for subsequent message parsing.
 * @return int
 */
void parserInit(void) {
    cJSON *json = cJSON_Parse(JSON_STRING);

    const cJSON *name_pt = cJSON_GetObjectItemCaseSensitive(json, "signals");

    uint32_t kind_count = 0;
    struct cJSON *cur;
    cJSON_ArrayForEach(cur, name_pt) {
        struct signal s = {
            .id     = 0,
            .len    = 0,
            .name   = "UNKNOWN_SIGNAL",
            .dt     = DT_UNK,
            .offset = 0,
            s.kind  = kind_count++,
        };
        if (cJSON_IsObject(cur)) {
            struct cJSON *id, *len, *name, *type, *offset;
            id   = cJSON_GetObjectItem(cur, "id");
            len  = cJSON_GetObjectItem(cur, "length");
            name = cJSON_GetObjectItem(cur, "name");
            type = cJSON_GetObjectItem(cur, "type");
            offset = cJSON_GetObjectItem(cur, "offset");
            if (cJSON_IsNumber(id)) {
                s.id = id->valueint;
            }

            if (cJSON_IsNumber(len)) {
                s.len = len->valueint;
            }

            if (cJSON_IsNumber(offset)) {
                s.offset = offset->valueint;
            }

            if (cJSON_IsString(name) && (name->valuestring != NULL)) {
                strncpy(s.name, name->valuestring, sizeof(s.name));
            }

            if (cJSON_IsString(type) && (type->valuestring != NULL)) {
                s.dt = dtLookupStr(type->valuestring);
            }

            signal_map[signals_parsed++] = s;
        }
    }

    /* Don't need this any more */
    cJSON_Delete(json);
}
