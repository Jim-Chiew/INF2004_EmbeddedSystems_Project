/*
 * telemetry.h  --  Buddy 1 (WiFi / communication / telemetry)
 * Turns one 1 Hz sample into the text message that is published live
 * (FR-COM-07, FR-COM-13) or stored in trans_queue (FR-COM-09).
 *
 * Message format (JSON text, whole numbers only):
 *   {"ts":123456,"state":"LINE_FOLLOWING","spd":250,"dist":1830,"hdg":905,"bat":7420}
 *
 * Units (to be agreed with the other buddies, see Section 7):
 *   ts    milliseconds since power-up
 *   spd   millimetres per second
 *   dist  millimetres
 *   hdg   degrees x 10   (905 = 90.5 degrees)
 *   bat   millivolts     (7420 = 7.42 V)
 *
 * Plain C, no kernel or network calls.
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>   /* no <stddef.h>: it clashes with the micro T-Kernel size_t */

#define TELEM_MSG_MAX    256u   /* NFR-RES-03: message <= 256 bytes */
#define TELEM_STATE_MAX   24u   /* longest state name incl. the end marker */

#define TELEM_ERR_ARG    (-1)   /* NULL pointer or bad state name */
#define TELEM_ERR_TOO_BIG (-2)  /* message would not fit */

typedef struct {
    uint32_t ts_ms;
    char     state[TELEM_STATE_MAX]; /* letters, digits and '_' only */
    int32_t  speed_mm_s;
    int32_t  distance_mm;
    int16_t  heading_deg_x10;
    uint16_t battery_mv;
} telemetry_sample_t;

/* Write the message into buf (size bytes, including room for the end marker).
 * Returns the message length (without the end marker), or a TELEM_ERR_ value.
 * Never writes past buf[size - 1]. */
int telemetry_format(const telemetry_sample_t *s, char *buf, uint32_t size);

#endif /* TELEMETRY_H */