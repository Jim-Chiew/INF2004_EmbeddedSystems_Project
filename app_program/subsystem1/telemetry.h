/*
 * telemetry.h -- Buddy 1. Packs one sample into the binary frame (12 to 24
 * bytes, layout in data_layout.h) and reads it back. Plain C, no kernel calls.
 * Tasks keep natural-width values; packing happens only here, at send time.
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include "data_layout.h"

#define TELEM_OK         0
#define TELEM_ERR_ARG   (-1)   /* NULL pointer or buffer too small */
#define TELEM_ERR_RANGE (-2)   /* a value does not fit its field */

typedef struct {
    uint8_t  state;        /* 0-7 (dl_state_t)            */
    uint8_t  error;        /* 0/1                         */
    uint8_t  wifi_loss;    /* 0/1 status                  */
    uint8_t  low_batt;     /* 0/1 status                  */
    uint8_t  drop;         /* 0/1 messages were dropped   */
    uint8_t  hump;         /* 0/1                         */
    uint8_t  speed;        /* cm/s 0-62, 63 = invalid     */
    uint8_t  bc_dir;       /* 0-2 (3 spare)               */
    uint8_t  bc_err;       /* 0/1                         */
    uint8_t  line;         /* 0-3                         */
    uint8_t  dist;         /* cm 0-62, 63 = no echo       */
    uint16_t dir;          /* deg 0-359, 511 = invalid    */
    uint8_t  hump_h;       /* jolt, 0.02 g steps 0-128, 255 = invalid */
    int16_t  tilt;         /* deg -180..180, TILT_BAD     */
    int16_t  roll;         /* deg -180..180, TILT_BAD     */
    uint8_t  seq;          /* 0-15, set by comms          */
    uint8_t  obj_len;      /* measured object length, cm 0-62,
                              DL_OBJ_NONE (63) = none measured yet */
    /* Appended in frame version 3.  New fields go at the END: the tests and
       the frame builder set the ones above positionally. */
    uint8_t  errors;       /* DL_ERRF_* bitmap: every error the indicator shows */
    char     bc_text[DL_BC_TEXT_MAX + 1];
                           /* the decoded barcode code itself, '\0' ended, ""
                              when nothing has been read.  Only printable
                              characters go on the wire; the length is worked
                              out from the string, so there is no second field
                              to keep in step. */
} telemetry_sample_t;

#define TELEM_ANGLE_INVALID  INT16_MIN   /* "no tilt/roll reading" */

/* Write one frame into out (DL_HEAD_BYTES plus the barcode text, so
 * DL_FRAME_MIN_BYTES..DL_FRAME_MAX_BYTES).  *len, if given, is set to how many
 * bytes were written -- the caller must publish exactly that many, since the
 * length varies with the text.  Returns TELEM_OK or TELEM_ERR_*. */
int telemetry_pack(const telemetry_sample_t *s, uint8_t *out, uint32_t size,
                   uint32_t *len);

/* Read a frame back (PC tools and tests). Returns TELEM_OK or TELEM_ERR_*;
 * a wrong version byte, a length that does not match the text-length byte, or
 * a text longer than DL_BC_TEXT_MAX gives TELEM_ERR_ARG. */
int telemetry_unpack(const uint8_t *in, uint32_t len, telemetry_sample_t *s);

/* One command byte -> DL_CMD_START/STOP/RESET, or 0 if not valid. */
uint8_t telemetry_cmd_decode(const uint8_t *in, uint32_t len);

#endif