/*
 * test_telemetry.c  
 * Runs on a PC. Test IDs match testing/subsystem1/TEST_LOG.md.
 */
#include <stdio.h>
#include <string.h>

#include "telemetry.h"

static int g_fail = 0;

#define CHECK(id, cond, desc)                                              \
    do {                                                                   \
        if (cond) {                                                        \
            printf("[TEST] %s PASS  %s\n", (id), (desc));                  \
        } else {                                                           \
            printf("[TEST] %s FAIL  %s   (line %d)\n", (id), (desc), __LINE__); \
            g_fail++;                                                      \
        }                                                                  \
    } while (0)

static telemetry_sample_t make(const char *state)
{
    telemetry_sample_t s;

    (void)memset(&s, 0, sizeof s);
    (void)strncpy(s.state, state, TELEM_STATE_MAX - 1u);
    s.ts_ms = 123456u;
    s.speed_mm_s = 250;
    s.distance_mm = 1830;
    s.heading_deg_x10 = 905;
    s.battery_mv = 7420u;
    return s;
}

int main(void)
{
    telemetry_sample_t s;
    char   buf[TELEM_MSG_MAX + 1u];
    char   guard[64 + 8];
    int    n;
    const char *expect =
        "{\"ts\":123456,\"state\":\"LINE_FOLLOWING\",\"spd\":250,\"dist\":1830,\"hdg\":905,\"bat\":7420}";

    s = make("LINE_FOLLOWING");
    n = telemetry_format(&s, buf, sizeof buf);
    CHECK("S1-U50", (n == (int)strlen(expect)) && (strcmp(buf, expect) == 0),
          "normal sample gives the expected message (FR-COM-07/09/13)");
    CHECK("S1-U51", (n > 0) && ((size_t)n <= TELEM_MSG_MAX), "message is at most 256 bytes (NFR-RES-03)");

    /* worst case: biggest numbers, longest state name */
    s = make("ABCDEFGHIJKLMNOPQRSTUVW");           /* 23 characters */
    s.ts_ms = 4294967295u;
    s.speed_mm_s = -2147483647 - 1;
    s.distance_mm = -2147483647 - 1;
    s.heading_deg_x10 = -32768;
    s.battery_mv = 65535u;
    n = telemetry_format(&s, buf, sizeof buf);
    CHECK("S1-U52", (n > 0) && ((size_t)n <= TELEM_MSG_MAX), "worst-case values still fit in 256 bytes");

    s = make("LINE_FOLLOWING");
    s.speed_mm_s = -40;
    n = telemetry_format(&s, buf, sizeof buf);
    CHECK("S1-U53", (n > 0) && (strstr(buf, "\"spd\":-40,") != NULL), "negative values are written correctly");

    s = make("IDLE");
    s.ts_ms = 0u; s.speed_mm_s = 0; s.distance_mm = 0; s.heading_deg_x10 = 0; s.battery_mv = 0u;
    n = telemetry_format(&s, buf, sizeof buf);
    CHECK("S1-U54", (n > 0) && (strstr(buf, "\"ts\":0,") != NULL) && (strstr(buf, "\"bat\":0}") != NULL), "all-zero sample works");

    /* buffer sizes: exact fit, one byte short */
    s = make("LINE_FOLLOWING");
    n = telemetry_format(&s, buf, sizeof buf);
    CHECK("S1-U55", telemetry_format(&s, buf, (size_t)n + 1u) == n, "buffer with room for the end marker works");
    CHECK("S1-U56", telemetry_format(&s, buf, (size_t)n) == TELEM_ERR_TOO_BIG, "buffer one byte too small is refused");

    /* no write past the buffer */
    (void)memset(guard, 'X', sizeof guard);
    n = telemetry_format(&s, guard, 16u);
    {
        int intact = 1;
        size_t i;
        for (i = 16u; i < sizeof guard; i++) {
            if (guard[i] != 'X') { intact = 0; }
        }
        CHECK("S1-U57", (n == TELEM_ERR_TOO_BIG) && intact, "small buffer: nothing written past its end");
    }

    CHECK("S1-U58", (telemetry_format(NULL, buf, sizeof buf) == TELEM_ERR_ARG) &&
                    (telemetry_format(&s, NULL, 10u) == TELEM_ERR_ARG) &&
                    (telemetry_format(&s, buf, 0u) == TELEM_ERR_ARG), "NULL pointers and size 0 are rejected");

    s = make("BAD\"STATE");
    CHECK("S1-U59", telemetry_format(&s, buf, sizeof buf) == TELEM_ERR_ARG, "state name with a quote is rejected");
    s = make("");
    CHECK("S1-U60", telemetry_format(&s, buf, sizeof buf) == TELEM_ERR_ARG, "empty state name is rejected");
    s = make("LINE_FOLLOWING");
    (void)memset(s.state, 'A', TELEM_STATE_MAX);    /* no end marker */
    CHECK("S1-U61", telemetry_format(&s, buf, sizeof buf) == TELEM_ERR_ARG, "state name without an end marker is rejected");

    printf("%s (%d failed)\n", g_fail ? "SOME TESTS FAILED" : "ALL TESTS PASSED", g_fail);
    return g_fail ? 1 : 0;
}