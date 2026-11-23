/*
 * test_com_state.c  --  Buddy 1 unit tests for the COM state machine.
 * Runs on a PC. Test IDs match testing/subsystem1/TEST_LOG.md.
 */
#include <stdio.h>

#include "com_state.h"

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

static com_state_t step(bool wifi, bool mqtt, bool empty)
{
    com_inputs_t in;
    in.wifi_up = wifi;
    in.mqtt_up = mqtt;
    in.queue_empty = empty;
    return com_update(&in);
}

int main(void)
{
    com_inputs_t in;

    com_init();
    CHECK("S1-U20", com_get_state() == COM_DISCONNECTED, "starts in COM_DISCONNECTED at power-up");

    CHECK("S1-U21", step(true, false, true) == COM_DISCONNECTED, "WiFi up but MQTT down: stays COM_DISCONNECTED");
    CHECK("S1-U22", step(false, true, true) == COM_DISCONNECTED, "MQTT up but WiFi down: stays COM_DISCONNECTED");

    CHECK("S1-U23", step(true, true, false) == COM_SYNC, "both up, queue not empty: COM_SYNC (FR-COM-03)");
    CHECK("S1-U24", step(true, true, false) == COM_SYNC, "stays in COM_SYNC while queue has messages");
    CHECK("S1-U25", step(true, true, true) == COM_CONNECTED, "queue becomes empty: COM_CONNECTED (FR-COM-04)");
    CHECK("S1-U26", step(true, true, true) == COM_CONNECTED, "stays in COM_CONNECTED");

    CHECK("S1-U27", step(false, true, true) == COM_DISCONNECTED, "WiFi lost in COM_CONNECTED: COM_DISCONNECTED (FR-COM-01)");

    com_init();
    CHECK("S1-U28", step(true, true, true) == COM_CONNECTED, "both up, queue empty from DISCONNECTED: COM_CONNECTED (FR-COM-03)");
    CHECK("S1-U29", step(true, false, true) == COM_DISCONNECTED, "MQTT lost in COM_CONNECTED: COM_DISCONNECTED (FR-COM-01)");

    com_init();
    (void)step(true, true, false);
    CHECK("S1-U30", step(false, true, false) == COM_DISCONNECTED, "WiFi lost in COM_SYNC: COM_DISCONNECTED (FR-COM-01)");
    (void)step(true, true, false);
    CHECK("S1-U31", step(true, false, false) == COM_DISCONNECTED, "MQTT lost in COM_SYNC: COM_DISCONNECTED (FR-COM-01)");

    CHECK("S1-U32", step(true, true, true) == COM_CONNECTED, "reconnects with empty queue: COM_CONNECTED");
    CHECK("S1-U33", com_update(NULL) == COM_CONNECTED, "NULL inputs change nothing");
    in.wifi_up = false; in.mqtt_up = false; in.queue_empty = true;
    (void)com_update(&in);
    CHECK("S1-U34", com_get_state() == COM_DISCONNECTED, "com_get_state() matches the last update");

    printf("%s (%d failed)\n", g_fail ? "SOME TESTS FAILED" : "ALL TESTS PASSED", g_fail);
    return g_fail ? 1 : 0;
}