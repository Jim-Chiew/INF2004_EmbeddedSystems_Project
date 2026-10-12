/*
 * test_status_led.c 
 * Runs on a PC. Test IDs match testing/subsystem1/TEST_LOG.md.
 */
#include <stdio.h>

#include "status_led.h"

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

int main(void)
{
    uint32_t t;
    int      solid_ok = 1;
    int      ones = 0;
    int      toggles = 0;
    bool     prev = status_led_level(COM_DISCONNECTED, 0u);
    bool     cur;

    for (t = 0u; t < 10000u; t++) {
        if (!status_led_level(COM_CONNECTED, t) || !status_led_level(COM_SYNC, t)) {
            solid_ok = 0;
        }
    }
    CHECK("S1-U40", solid_ok, "solid ON in COM_CONNECTED and COM_SYNC for 10 s (FR-COM-05)");

    CHECK("S1-U41", status_led_level(COM_DISCONNECTED, 0u) && status_led_level(COM_DISCONNECTED, 249u),
          "COM_DISCONNECTED: ON for the first 250 ms");
    CHECK("S1-U42", !status_led_level(COM_DISCONNECTED, 250u) && !status_led_level(COM_DISCONNECTED, 499u),
          "COM_DISCONNECTED: OFF for the next 250 ms");
    CHECK("S1-U43", status_led_level(COM_DISCONNECTED, 500u), "pattern repeats every 500 ms");

    for (t = 1u; t <= 10000u; t++) {
        cur = status_led_level(COM_DISCONNECTED, t);
        if (cur != prev) {
            toggles++;
        }
        prev = cur;
    }
    CHECK("S1-U44", toggles == 40, "20 blinks (40 changes) in 10 s = 2 Hz (FR-COM-06)");

    for (t = 0u; t < 10000u; t++) {
        if (status_led_level(COM_DISCONNECTED, t)) {
            ones++;
        }
    }
    CHECK("S1-U45", ones == 5000, "50% duty cycle over 10 s");
    CHECK("S1-U46", !status_led_level((com_state_t)99, 0u), "unknown state: LED off");

    printf("%s (%d failed)\n", g_fail ? "SOME TESTS FAILED" : "ALL TESTS PASSED", g_fail);
    return g_fail ? 1 : 0;
}