/*
 * telemetry.c  --  Buddy 1 (WiFi / communication / telemetry)
 * See telemetry.h.
 */
#include "telemetry.h"
#include <stddef.h>   /* NULL; this .c file does not include any kernel header */


/* A state name may only contain letters, digits and '_'. That keeps the
 * message valid JSON (no quotes or backslashes can sneak in). */
static int state_is_valid(const char *name)
{
    size_t i;

    for (i = 0u; i < TELEM_STATE_MAX; i++) {
        char c = name[i];

        if (c == '\0') {
            return (i > 0u) ? 1 : 0;        /* empty name is not allowed */
        }
        if (!(((c >= 'A') && (c <= 'Z')) ||
              ((c >= 'a') && (c <= 'z')) ||
              ((c >= '0') && (c <= '9')) ||
              (c == '_'))) {
            return 0;
        }
    }
    return 0;                               /* no end marker found */
}

/* ---- tiny text writer: no snprintf, because the micro T-Kernel image has no
 * C library system calls (snprintf pulls in _sbrk, _write, ... and the link
 * fails). 'len' counts the characters the message WOULD have, even past the
 * end of the buffer, exactly like snprintf's return value. ---- */
typedef struct {
    char     *buf;
    uint32_t  size;
    uint32_t  len;
} out_t;

static void put_char(out_t *o, char c)
{
    if ((o->len + 1u) < o->size) {          /* keep room for the end marker */
        o->buf[o->len] = c;
    }
    o->len++;
}

static void put_str(out_t *o, const char *s)
{
    while (*s != '\0') {
        put_char(o, *s);
        s++;
    }
}

static void put_u32(out_t *o, uint32_t v)
{
    char tmp[10];
    int  n = 0;
    do {
        tmp[n++] = (char)('0' + (v % 10u));
        v /= 10u;
    } while (v != 0u);
    while (n > 0) {
        put_char(o, tmp[--n]);
    }
}

static void put_i32(out_t *o, int32_t v)
{
    if (v < 0) {
        put_char(o, '-');
        put_u32(o, 0u - (uint32_t)v);       /* also right for INT32_MIN */
    } else {
        put_u32(o, (uint32_t)v);
    }
}

int telemetry_format(const telemetry_sample_t *s, char *buf, uint32_t size)
{
    out_t o;

    if ((s == NULL) || (buf == NULL) || (size == 0u)) {
        return TELEM_ERR_ARG;
    }
    if (!state_is_valid(s->state)) {
        return TELEM_ERR_ARG;
    }

    o.buf = buf;
    o.size = size;
    o.len = 0u;
    put_str(&o, "{\"ts\":");     put_u32(&o, s->ts_ms);
    put_str(&o, ",\"state\":\""); put_str(&o, s->state);
    put_str(&o, "\",\"spd\":");  put_i32(&o, s->speed_mm_s);
    put_str(&o, ",\"dist\":");   put_i32(&o, s->distance_mm);
    put_str(&o, ",\"hdg\":");    put_i32(&o, (int32_t)s->heading_deg_x10);
    put_str(&o, ",\"bat\":");    put_u32(&o, (uint32_t)s->battery_mv);
    put_char(&o, '}');

    buf[(o.len < size) ? o.len : (size - 1u)] = '\0';   /* end marker */

    if ((o.len > TELEM_MSG_MAX) || (o.len >= size)) {
        return TELEM_ERR_TOO_BIG;           /* too long, or buffer too small */
    }
    return (int)o.len;
}