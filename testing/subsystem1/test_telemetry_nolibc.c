/*
 * test_telemetry_nolibc.c -- proves the hand-written message writer in
 * telemetry.c gives EXACTLY the same text as the snprintf version it replaced
 * (snprintf cannot be used on the micro T-Kernel image: it needs C library
 * system calls that the image does not have).
 *
 * Each check prints an ID S1-Tnn. Add new checks at the END.
 * Build and run from the repo root:
 *   gcc -std=c11 -Wall -Wextra -fsanitize=address,undefined -Iapp_program/subsystem1 \
 *       testing/subsystem1/test_telemetry_nolibc.c app_program/subsystem1/telemetry.c -o /tmp/t
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include "telemetry.h"

static int fails = 0, checks = 0;
#define CHECK(c, m) do { checks++; \
    if (c) { printf("PASS S1-T%02d %s\n", checks, m); } \
    else { printf("FAIL S1-T%02d %s\n", checks, m); fails++; } } while (0)

/* the old formatter, kept here as the reference */
static int reference(const telemetry_sample_t *s, char *buf, size_t size)
{
    return snprintf(buf, size,
                    "{\"ts\":%lu,\"state\":\"%s\",\"spd\":%ld,\"dist\":%ld,"
                    "\"hdg\":%d,\"bat\":%u}",
                    (unsigned long)s->ts_ms, s->state, (long)s->speed_mm_s,
                    (long)s->distance_mm, (int)s->heading_deg_x10,
                    (unsigned)s->battery_mv);
}

static uint32_t rnd32(void) { return ((uint32_t)rand() << 16) ^ (uint32_t)rand() ^ ((uint32_t)rand() << 29); }

int main(void)
{
    static const char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_";
    char a[300], b[300];
    long mismatches = 0, total = 0;
    srand(12345);

    /* 200000 random samples: same text, same length */
    for (long i = 0; i < 200000; i++) {
        telemetry_sample_t s;
        memset(&s, 0, sizeof s);
        s.ts_ms = rnd32();
        size_t n = 1u + (size_t)(rand() % (TELEM_STATE_MAX - 1u));
        for (size_t k = 0; k < n; k++) { s.state[k] = alphabet[rand() % (int)(sizeof alphabet - 1)]; }
        s.speed_mm_s = (int32_t)rnd32();
        s.distance_mm = (int32_t)rnd32();
        s.heading_deg_x10 = (int16_t)rnd32();
        s.battery_mv = (uint16_t)rnd32();
        int r = reference(&s, a, sizeof a);
        int m = telemetry_format(&s, b, sizeof b);
        total++;
        if (m != r || strcmp(a, b) != 0) { mismatches++; }
    }
    CHECK(mismatches == 0, "200000 random samples: same text as snprintf");

    /* extreme values */
    {
        telemetry_sample_t s;
        memset(&s, 0, sizeof s);
        strcpy(s.state, "X");
        s.ts_ms = UINT32_MAX; s.speed_mm_s = INT32_MIN; s.distance_mm = INT32_MAX;
        s.heading_deg_x10 = INT16_MIN; s.battery_mv = UINT16_MAX;
        int r = reference(&s, a, sizeof a);
        int m = telemetry_format(&s, b, sizeof b);
        CHECK(m == r && strcmp(a, b) == 0, "min/max values: same text as snprintf");
        s.ts_ms = 0; s.speed_mm_s = 0; s.distance_mm = 0; s.heading_deg_x10 = 0; s.battery_mv = 0;
        r = reference(&s, a, sizeof a); m = telemetry_format(&s, b, sizeof b);
        CHECK(m == r && strcmp(a, b) == 0, "all zero: same text as snprintf");
    }

    /* every buffer size from 1 to 120: never writes past the buffer, same verdict as the
     * length rule (TOO_BIG when the text does not fit with its end marker) */
    {
        telemetry_sample_t s;
        memset(&s, 0, sizeof s);
        strcpy(s.state, "LINE_FOLLOW");
        s.ts_ms = 123456; s.speed_mm_s = -250; s.distance_mm = 1830; s.heading_deg_x10 = 905; s.battery_mv = 7420;
        int full = reference(&s, a, sizeof a);
        int ok = 1;
        for (int size = 1; size <= 120; size++) {
            char guard[130];
            memset(guard, 0x5A, sizeof guard);
            int m = telemetry_format(&s, guard, (uint32_t)size);
            int expect = (full >= size) ? TELEM_ERR_TOO_BIG : full;
            if (m != expect) { ok = 0; }
            for (int k = size; k < 130; k++) {
                if (guard[k] != 0x5A) { ok = 0; }          /* nothing written after 'size' */
            }
            if (m > 0 && strlen(guard) != (size_t)m) { ok = 0; }
        }
        CHECK(ok, "sizes 1..120: right result, nothing written past the buffer");
    }

    /* refusals are unchanged */
    {
        telemetry_sample_t s;
        memset(&s, 0, sizeof s);
        strcpy(s.state, "OK");
        CHECK(telemetry_format(NULL, b, sizeof b) == TELEM_ERR_ARG, "NULL sample refused");
        CHECK(telemetry_format(&s, NULL, 10) == TELEM_ERR_ARG, "NULL buffer refused");
        CHECK(telemetry_format(&s, b, 0) == TELEM_ERR_ARG, "size 0 refused");
        strcpy(s.state, "a\"b");
        CHECK(telemetry_format(&s, b, sizeof b) == TELEM_ERR_ARG, "quote in state name refused");
        s.state[0] = '\0';
        CHECK(telemetry_format(&s, b, sizeof b) == TELEM_ERR_ARG, "empty state name refused");
        memset(s.state, 'A', sizeof s.state);       /* no end marker */
        CHECK(telemetry_format(&s, b, sizeof b) == TELEM_ERR_ARG, "state name without end marker refused");
    }

    printf("%d checks, %s (%d failed), %ld samples compared\n", checks, fails ? "FAILED" : "ALL PASSED", fails, total);
    return fails != 0;
}