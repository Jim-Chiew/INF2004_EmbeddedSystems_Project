/*
 * telemetry.c  
 * See telemetry.h.
 */
#include "telemetry.h"

#include <stdio.h>

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

int telemetry_format(const telemetry_sample_t *s, char *buf, size_t size)
{
    int len;

    if ((s == NULL) || (buf == NULL) || (size == 0u)) {
        return TELEM_ERR_ARG;
    }
    if (!state_is_valid(s->state)) {
        return TELEM_ERR_ARG;
    }

    /* snprintf never writes more than 'size' bytes. It returns the length the
     * message WOULD have had, so we can tell if it was cut short. */
    len = snprintf(buf, size,
                   "{\"ts\":%lu,\"state\":\"%s\",\"spd\":%ld,\"dist\":%ld,"
                   "\"hdg\":%d,\"bat\":%u}",
                   (unsigned long)s->ts_ms,
                   s->state,
                   (long)s->speed_mm_s,
                   (long)s->distance_mm,
                   (int)s->heading_deg_x10,
                   (unsigned)s->battery_mv);

    if (len < 0) {
        return TELEM_ERR_ARG;
    }
    if (((size_t)len > TELEM_MSG_MAX) || ((size_t)len >= size)) {
        return TELEM_ERR_TOO_BIG;           /* too long, or buffer too small */
    }
    return len;
}