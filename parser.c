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
#include <arm_neon.h>                       /* Floating types */

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
    size_t out_len;                     /**< @brief Length of each sample
                                         * (in bytes) */
    size_t in_len;                      /**< @brief Length raw received data. */
    char name[MAX_SIGNAL_NAME];         /**< @brief Signal name
                                         * (mostly useful for debugging
                                         * right now, but we have it anyways) */
    enum data_type dt_in;               /**< @brief Type to reinterpret
                                         * input data */
    enum data_type dt_out;              /**< @brief Type to reinterpret
                                         * output data */
    uint32_t kind;                      /**< @brief Index in the configured
                                         * signal vector, unique among
                                         * all signals parsed. */
    double factor;                      /**< @brief Parsed conversion term.
                                         * 1. If not found. */
    double bias;                        /**< @brief Parsed term term.
                                         * 0. If not found. */
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
    [DT_FLOAT64] = "f64",
};

/** @brief Some type on which we can apply any conversion gain/bias.
 * It would probably be prudent to make this a floating type. */
typedef double sig_intermediary_val_t;

/**
 * @brief How large each type is in the configuration file.
 * @warning If a signal has a type not resident here,
 * we will never boot successfully.
 */
static const size_t dtSizeMap[DT_NUM] = {
    [DT_INT16]   = sizeof(int16_t),
    [DT_INT32]   = sizeof(int32_t),
    [DT_INT64]   = sizeof(int64_t),
    [DT_UINT8]   = sizeof(uint8_t),
    [DT_UINT16]  = sizeof(uint16_t),
    [DT_UINT32]  = sizeof(uint32_t),
    [DT_UINT64]  = sizeof(uint64_t),
    [DT_FLOAT16] = sizeof(float16_t),
    [DT_FLOAT32] = sizeof(float32_t),
    [DT_FLOAT64] = sizeof(double),
};

/**
 * @brief According to the configured signal type, find the representing enum
 * @param s The type name
 * @return enum data_type Datatype representing this.
 */
static enum data_type dtLookupStr(const char *s) {
    for (int i = 0; i < DT_NUM; i++) {
        if (!strncmp(dtNameMap[i], s, DT_NAME_MAX)) {
            return (enum data_type) i;
        }
    }
    return DT_UNK;
}

/**
 *  @brief Apply a signal conversion to an intermediary type.
 *
 *  @param s The signal to base conversion off of
 *  @param val The value to convert
 *  @return sig_intermediary_val_t The converted value
 */
static sig_intermediary_val_t signal_apply_conversion(
    struct signal *s, sig_intermediary_val_t val) {
    return (s->factor * val) + s->bias;
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

        if (sig->offset + sig->in_len > len) {
            /* Uh oh */
            return -1;
        }

        const void *v_pt = msg + sig->offset;
        uint8_t   u8   = 0U;
        uint16_t  u16  = 0U;
        uint32_t  u32  = 0U;
        uint64_t  u64  = 0ULL;
        int8_t    i8   = 0;
        int16_t   i16  = 0;
        int32_t   i32  = 0;
        int64_t   i64  = 0LL;
        float16_t f16  = 0.f;
        float32_t f32  = 0.f;
        double    f64  = 0.;

#define DT_FOREACH(f)                                                          \
    f(DT_INT8,    i8,  i8, int8_t)                                             \
    f(DT_INT16,   i16, i16, int16_t)                                           \
    f(DT_INT32,   i32, i32, int32_t)                                           \
    f(DT_INT64,   i64, i64, int64_t)                                           \
    f(DT_UINT8,   u8,  u8,  uint8_t)                                           \
    f(DT_UINT16,  u16, u16, uint16_t)                                          \
    f(DT_UINT32,  u32, u32, uint32_t)                                          \
    f(DT_UINT64,  u64, u64, uint64_t)                                          \
    f(DT_FLOAT16, f16, f16, float16_t)                                         \
    f(DT_FLOAT32, f32, f32, float32_t)                                         \
    f(DT_FLOAT64, f64, f64, double)

#define REINTERP_IN(dt_name, field, var, type)                                 \
        case dt_name:                                                          \
            /* Fill in var with the raw value */                               \
            var = *(type *) v_pt;                                              \
            /* Apply any conversion. We'll do this by casting                  \
             * To the intermediary type, running the conversion,               \
             * then casting back. */                                           \
            /* Note that it is more polite to do this here than in out the     \
             * output switch, as that is a O(N^2) case explosion               \
             * whereas this need only be O(N) */                               \
            var = (type) signal_apply_conversion(                              \
                sig,                                                           \
                (sig_intermediary_val_t) var                                   \
            );                                                                 \
                                                                               \
            break;

        switch(sig->dt_in) {

        DT_FOREACH(REINTERP_IN)

        default:
            /* Not entirely sure what the best course of action is here. */
            return -1;
        }

#undef REINTERP_IN
#define REINTERP_IN(dt_name, field, var, type)                                 \
        case dt_name:                                                          \
            s->v[i].field = (type) var;                                        \
            break;

#define REINTERP_OUT(dt_name, field_out, var, type_out)                        \
        case dt_name:                                                          \
            switch(sig->dt_in) {                                               \
            REINTERP_IN(DT_INT8,    field_out,  i8, type_out)                  \
            REINTERP_IN(DT_INT16,   field_out, i16, type_out)                  \
            REINTERP_IN(DT_INT32,   field_out, i32, type_out)                  \
            REINTERP_IN(DT_INT64,   field_out, i64, type_out)                  \
            REINTERP_IN(DT_UINT8,   field_out,  u8, type_out)                  \
            REINTERP_IN(DT_UINT16,  field_out, u16, type_out)                  \
            REINTERP_IN(DT_UINT32,  field_out, u32, type_out)                  \
            REINTERP_IN(DT_UINT64,  field_out, u64, type_out)                  \
            REINTERP_IN(DT_FLOAT16, field_out, f16, type_out)                  \
            REINTERP_IN(DT_FLOAT32, field_out, f32, type_out)                  \
            REINTERP_IN(DT_FLOAT64, field_out, f64, type_out)                  \
            default:                                                           \
                return -1;                                                     \
            }                                                                  \
            break;


        switch (sig->dt_out) {
        DT_FOREACH(REINTERP_OUT)
        default:
            return -1;
        }
#undef REINTERP_OUT
#undef REINTERP_IN
#undef DT_FOREACH

        s->sig[i] = (struct sample_info) {
            .kind = sig->kind,
            .len  = sig->out_len,
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
            .id      = 0,
            .out_len = 0,
            .in_len  = 0,
            .name    = "UNKNOWN_SIGNAL",
            .dt_in   = DT_UNK,
            .dt_out  = DT_UNK,
            .offset  = 0,
            .factor  = 1.,
            .bias    = 0.,
            .kind   = kind_count++,
        };
        if (cJSON_IsObject(cur)) {
            struct cJSON *id, *name, *intype, *outtype, *offset, *factor, *bias;
            id      = cJSON_GetObjectItem(cur, "id");
            name    = cJSON_GetObjectItem(cur, "name");
            intype  = cJSON_GetObjectItem(cur, "in_type");
            outtype = cJSON_GetObjectItem(cur, "out_type");
            offset  = cJSON_GetObjectItem(cur, "offset");
            factor  = cJSON_GetObjectItem(cur, "factor");
            bias    = cJSON_GetObjectItem(cur, "bias");
            if (cJSON_IsNumber(id)) {
                s.id = id->valueint;
            }

            if (cJSON_IsNumber(offset)) {
                s.offset = offset->valueint;
            }

            if (cJSON_IsNumber(factor)) {
                s.factor = factor->valuedouble;
            }

            if (cJSON_IsNumber(bias)) {
                s.bias   = bias->valuedouble;
            }

            if (cJSON_IsString(name) && (name->valuestring != NULL)) {
                strncpy(s.name, name->valuestring, sizeof(s.name));
            }

            if (cJSON_IsString(intype) && (intype->valuestring != NULL)) {
                s.dt_in   = dtLookupStr(intype->valuestring);
                s.in_len  = dtSizeMap[s.dt_in];
            }

            if (cJSON_IsString(outtype) && (outtype->valuestring != NULL)) {
                s.dt_out  = dtLookupStr(outtype->valuestring);
                s.out_len = dtSizeMap[s.dt_out];
            }

            signal_map[signals_parsed++] = s;
        }
    }

    /* Don't need this any more */
    cJSON_Delete(json);
}
