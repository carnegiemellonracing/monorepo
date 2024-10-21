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
#include <stdbool.h>                        /* bool */

/**
 * @brief Data type names are <= 9 bytes
 */
#define DT_NAME_MAX 10

/** @brief Call f on each internal datatype, with args: dt enum, abv. type name,
 *  real/compiler-facing type name. Note that this won't hit DT_UNK, as that's
 *  considered a 'bad' value (just a named one).
 */
#define DT_FOREACH(f)                                                          \
    f(DT_INT8,    i8,  int8_t)                                                 \
    f(DT_INT16,   i16, int16_t)                                                \
    f(DT_INT32,   i32, int32_t)                                                \
    f(DT_INT64,   i64, int64_t)                                                \
    f(DT_UINT8,   u8,  uint8_t)                                                \
    f(DT_UINT16,  u16, uint16_t)                                               \
    f(DT_UINT32,  u32, uint32_t)                                               \
    f(DT_UINT64,  u64, uint64_t)                                               \
    f(DT_FLOAT16, f16, __fp16)                                                 \
    f(DT_FLOAT32, f32, float)                                                  \
    f(DT_FLOAT64, f64, double)

/**
 * @brief Parsed signal information.
 */

/**
 * @brief Number of signals configured in the parsed vector.
 */
uint32_t signals_parsed = 0;

/**
 * @brief Metadata on each signal in the vector (filled during parsing)
 */
struct signal signal_map[MAX_SIGNALS];

/**
 * @brief How we read in the configuration file (use marshall.py to update).
 */
static const char JSON_STRING[] = {
 #include "can_fmt.rawh"
};

/**
 * @brief What each datatype is called in the configuration file.
 * @warning If a signal has a type not resident here, we will never boot
 * successfully.
 */
static const char dtNameMap[DT_NUM][DT_NAME_MAX] = {
#define INST_DT_NAME(dt_name, abv_name, type)   \
    [dt_name] = #abv_name,

    /* Each dt name in the JSON cfg is given by e.g. "f32" */
    DT_FOREACH(INST_DT_NAME)
#undef INST_DT_NAME
};

/**
 * @brief How large each type is in the configuration file.
 * @warning If a signal has a type not resident here, we will never boot
 * successfully.
 */
static const size_t dtSizeMap[DT_NUM] = {
#define INST_DT_SIZE(dt_name, abv_name, type)   \
    [dt_name] = sizeof(type),

    /* Each dt size is given by sizeof(type) */
    DT_FOREACH(INST_DT_SIZE)
#undef INST_DT_SIZE
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
    struct signal *s, sig_intermediary_val_t val
) {
    sig_intermediary_val_t ret = (s->factor * val) + s->bias;
    return ret;
}

void setSignalEnable(uint32_t kind, bool is_enabled) {
    signal_map[kind].enabled = is_enabled;
}

/**
 * @brief Parse a CAN message and enqueue it to be sent out later.
 * @param bus The ID of the bus the message came in on.
 * @param id The ID the message came in on.
 * @param msg The message data
 * @param len The received data length (in bytes)
 * @return int 0 on success, -1 on failure.
 */
int parseData(uint32_t bus, uint16_t id, const uint8_t msg[], size_t len) {
   struct sample sample;
   struct sample *s = &sample;

   struct signal *sigv[MAX_VAL_PER_SIG];
   size_t relevant_sigs = 0;
   for (size_t i = 0; i < signals_parsed; i++) {
       if (signal_map[i].id == id && signal_map[i].bus == bus && signal_map[i].enabled) {
           sigv[relevant_sigs++] = &signal_map[i];
       }

       if (relevant_sigs >= MAX_VAL_PER_SIG - 1) {
           /* There are more subscribing signals on this message That we can
            * handle. This will never happen if the configuration is valid, but
            * I would still rather continue execution here by dropping the rest
            * of the (desired) samples. */
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

       /* Generate a bunch of locals as holding variables before shunting into
        * the union. This isn't strictly necessary to do, but some types might
        * have *strange* conversion rules, so casting in/out of memory is safer
        * in my mind than a mem-mem copy. */
#define INST_LOCAL(dt_name, abv_name, type)   \
       type abv_name = 0;

       /* Each local is e.g. float32_t f32 = 0 */
      DT_FOREACH(INST_LOCAL)
#undef INST_LOCAL

       switch(sig->dt_in) {

#define REINTERP_IN(dt_name, abv_name, type)                                   \
       case dt_name:                                                          \
           /* Fill in abv_name with the raw value */                          \
           abv_name = *(type *) v_pt;                                         \
           break;


       DT_FOREACH(REINTERP_IN)

       default:
           /* Not entirely sure what the best course of action is here. */
           /* Note that this will get hit on DT_UNK as well. */
           return -1;
       }

#undef REINTERP_IN

   /* Each local is named abv_name, e.g. float32_t f32 */
#define REINTERP_IN(dt_name, field, var, type)                                 \
       case dt_name:                                                          \
           s->v[i].field = (type) signal_apply_conversion(                    \
               sig,                                                           \
               (sig_intermediary_val_t) var                                   \
           );                                                                 \
           break;

#define REINTERP_OUT(dt_name, abv_name, type_out)                              \
       case dt_name:                                                          \
           switch(sig->dt_in) {                                               \
           REINTERP_IN(DT_INT8,    abv_name,  i8, type_out)                   \
           REINTERP_IN(DT_INT16,   abv_name, i16, type_out)                   \
           REINTERP_IN(DT_INT32,   abv_name, i32, type_out)                   \
           REINTERP_IN(DT_INT64,   abv_name, i64, type_out)                   \
           REINTERP_IN(DT_UINT8,   abv_name,  u8, type_out)                   \
           REINTERP_IN(DT_UINT16,  abv_name, u16, type_out)                   \
           REINTERP_IN(DT_UINT32,  abv_name, u32, type_out)                   \
           REINTERP_IN(DT_UINT64,  abv_name, u64, type_out)                   \
           REINTERP_IN(DT_FLOAT16, abv_name, f16, type_out)                   \
           REINTERP_IN(DT_FLOAT32, abv_name, f32, type_out)                   \
           REINTERP_IN(DT_FLOAT64, abv_name, f64, type_out)                   \
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
 */
void parserInit(void) {
   cJSON *json = cJSON_Parse(JSON_STRING);

   const cJSON *name_pt = cJSON_GetObjectItemCaseSensitive(json, "signals");

   uint32_t kind_count = 0;
   struct cJSON *cur;
   cJSON_ArrayForEach(cur, name_pt) {
       struct signal s = {
           .bus     = 0,   /* Assumed 0 if not specified */
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
           .enabled = true,
       };
       if (cJSON_IsObject(cur)) {
           struct cJSON *id, *name, *intype, *outtype, *offset, *factor, *bias, *enabled;
           struct cJSON *bus;
           bus     = cJSON_GetObjectItem(cur, "bus");
           id      = cJSON_GetObjectItem(cur, "id");
           name    = cJSON_GetObjectItem(cur, "name");
           intype  = cJSON_GetObjectItem(cur, "in_type");
           outtype = cJSON_GetObjectItem(cur, "out_type");
           offset  = cJSON_GetObjectItem(cur, "offset");
           factor  = cJSON_GetObjectItem(cur, "factor");
           bias    = cJSON_GetObjectItem(cur, "bias");
           enabled = cJSON_GetObjectItem(cur, "enabled");


           if (cJSON_IsNumber(bus)) {
               s.bus = bus->valueint;
           }

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

           if (cJSON_IsNumber(enabled)) {
        	   s.enabled = enabled->valueint;
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