/* telemetry.c -- Buddy 1. Pack/unpack with masks and shifts (no C bit-fields). */
#include "telemetry.h"

/* Put a value into the payload / read it back (shift and width come from
 * data_layout.h).  The payload is a little-endian bit string: bit n lives in
 * byte n / 8, at bit n % 8.  Version 2 of the frame is 72 bits, so it no
 * longer fits one 64-bit word and the bits are placed one at a time; that
 * also lets a field straddle a byte boundary.  Five frames a second: the
 * handful of extra instructions costs nothing.
 *
 * Bit n ends up exactly where the old 64-bit-word code put it, so a field's
 * position on the wire has not moved. */
static void put(uint8_t *p, uint32_t v, uint32_t shift, uint32_t bits)
{
    uint32_t i;
    uint32_t bit;

    for (i = 0u; i < bits; i++) {
        if (((v >> i) & 1u) != 0u) {
            bit = shift + i;
            p[bit / DL_BITS_PER_BYTE] |=
                (uint8_t)(1u << (bit % DL_BITS_PER_BYTE));
        }
    }
}

static uint32_t get(const uint8_t *p, uint32_t shift, uint32_t bits)
{
    uint32_t i, v = 0u;
    uint32_t bit;

    for (i = 0u; i < bits; i++) {
        bit = shift + i;
        if ((p[bit / DL_BITS_PER_BYTE] &
             (uint8_t)(1u << (bit % DL_BITS_PER_BYTE))) != 0u) {
            v |= (1u << i);
        }
    }
    return v;
}

static int angle_ok(int16_t a)
{
    return (a == TELEM_ANGLE_INVALID) || ((a >= DL_TILT_MIN_DEG) && (a <= DL_TILT_MAX_DEG));
}

static uint32_t angle_enc(int16_t a)
{
    return (a == TELEM_ANGLE_INVALID) ? DL_TILT_INVALID : (uint32_t)(a + DL_TILT_OFFSET);
}

static int16_t angle_dec(uint32_t v)
{
    return (v == DL_TILT_INVALID) ? TELEM_ANGLE_INVALID : (int16_t)((int32_t)v - DL_TILT_OFFSET);
}

/* Every field must be inside its range (or be its invalid code). */
static int flags_ok(const telemetry_sample_t *s)
{
    return (s->error <= 1u) && (s->wifi_loss <= 1u) && (s->low_batt <= 1u) &&
           (s->drop <= 1u) && (s->hump <= 1u) && (s->bc_err <= 1u);
}

static int fields_ok(const telemetry_sample_t *s)
{
    if (!flags_ok(s) || (s->state > DL_STATE_MAX) || (s->bc_dir > DL_BC_DIR_MAX) || (s->line > DL_LINE_MAX) || (s->seq > DL_SEQ_MAX)) {
        return 0;
    }
    if ((s->speed > DL_SPEED_INVALID) || (s->dist > DL_DIST_INVALID)) { return 0; }
    if (s->obj_len > DL_OBJ_NONE) { return 0; }
    /* 360..510 would fit in 9 bits but is not a valid direction */
    if ((s->dir > DL_DIR_MAX_DEG) && (s->dir != DL_DIR_INVALID)) { return 0; }
    if ((s->hump_h > DL_HUMP_H_MAX_MM) && (s->hump_h != DL_HUMP_H_INVALID)) { return 0; }
    return angle_ok(s->tilt) && angle_ok(s->roll);
}

int telemetry_pack(const telemetry_sample_t *s, uint8_t *out, uint32_t size)
{
    uint8_t *p;
    uint32_t i;

    if ((!s) || (!out) || (size < DL_FRAME_BYTES)) { return TELEM_ERR_ARG; }

    if (!fields_ok(s)) { return TELEM_ERR_RANGE; }

    out[0] = (uint8_t)DL_VERSION;
    p = &out[1];
    for (i = 0u; i < DL_PAYLOAD_BYTES; i++) { p[i] = 0u; }  /* spare bits stay 0 */

    put(p, s->state, DL_STATE_SHIFT, DL_STATE_BITS);
    put(p, s->error, DL_ERROR_SHIFT, DL_ERROR_BITS);
    put(p, s->wifi_loss, DL_WIFI_LOSS_SHIFT, DL_WIFI_LOSS_BITS);
    put(p, s->low_batt, DL_LOW_BATT_SHIFT, DL_LOW_BATT_BITS);
    put(p, s->drop, DL_DROP_SHIFT, DL_DROP_BITS);
    put(p, s->hump, DL_HUMP_SHIFT, DL_HUMP_BITS);
    put(p, s->speed, DL_SPEED_SHIFT, DL_SPEED_BITS);
    put(p, s->bc_dir, DL_BC_DIR_SHIFT, DL_BC_DIR_BITS);
    put(p, s->bc_err, DL_BC_ERR_SHIFT, DL_BC_ERR_BITS);
    put(p, s->line, DL_LINE_SHIFT, DL_LINE_BITS);
    put(p, s->dist, DL_DIST_SHIFT, DL_DIST_BITS);
    put(p, s->dir, DL_DIR_SHIFT, DL_DIR_BITS);
    put(p, s->hump_h, DL_HUMP_H_SHIFT, DL_HUMP_H_BITS);
    put(p, angle_enc(s->tilt), DL_TILT_SHIFT, DL_TILT_BITS);
    put(p, angle_enc(s->roll), DL_ROLL_SHIFT, DL_ROLL_BITS);
    put(p, s->seq, DL_SEQ_SHIFT, DL_SEQ_BITS);
    put(p, s->obj_len, DL_OBJ_SHIFT, DL_OBJ_BITS);
    return TELEM_OK;
}

int telemetry_unpack(const uint8_t *in, uint32_t len, telemetry_sample_t *s)
{
    const uint8_t *p;

    if ((!in) || (!s) || (len != DL_FRAME_BYTES) || (in[0] != DL_VERSION)) {
        return TELEM_ERR_ARG;
    }
    p = &in[1];

    s->state = (uint8_t)get(p, DL_STATE_SHIFT, DL_STATE_BITS);
    s->error = (uint8_t)get(p, DL_ERROR_SHIFT, DL_ERROR_BITS);
    s->wifi_loss = (uint8_t)get(p, DL_WIFI_LOSS_SHIFT, DL_WIFI_LOSS_BITS);
    s->low_batt = (uint8_t)get(p, DL_LOW_BATT_SHIFT, DL_LOW_BATT_BITS);
    s->drop = (uint8_t)get(p, DL_DROP_SHIFT, DL_DROP_BITS);
    s->hump = (uint8_t)get(p, DL_HUMP_SHIFT, DL_HUMP_BITS);
    s->speed = (uint8_t)get(p, DL_SPEED_SHIFT, DL_SPEED_BITS);
    s->bc_dir = (uint8_t)get(p, DL_BC_DIR_SHIFT, DL_BC_DIR_BITS);
    s->bc_err = (uint8_t)get(p, DL_BC_ERR_SHIFT, DL_BC_ERR_BITS);
    s->line = (uint8_t)get(p, DL_LINE_SHIFT, DL_LINE_BITS);
    s->dist = (uint8_t)get(p, DL_DIST_SHIFT, DL_DIST_BITS);
    s->dir = (uint16_t)get(p, DL_DIR_SHIFT, DL_DIR_BITS);
    s->hump_h = (uint8_t)get(p, DL_HUMP_H_SHIFT, DL_HUMP_H_BITS);
    s->seq = (uint8_t)get(p, DL_SEQ_SHIFT, DL_SEQ_BITS);
    s->tilt = angle_dec(get(p, DL_TILT_SHIFT, DL_TILT_BITS));
    s->roll = angle_dec(get(p, DL_ROLL_SHIFT, DL_ROLL_BITS));
    s->obj_len = (uint8_t)get(p, DL_OBJ_SHIFT, DL_OBJ_BITS);
    return TELEM_OK;
}

uint8_t telemetry_cmd_decode(const uint8_t *in, uint32_t len)
{
    if ((!in) || (len != DL_CMD_BYTES)) { return 0u; }
    return ((in[0] >= DL_CMD_START) && (in[0] <= DL_CMD_RESET)) ? in[0] : 0u;
}