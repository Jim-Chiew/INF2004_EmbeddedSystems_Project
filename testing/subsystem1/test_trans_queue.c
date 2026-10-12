/*
 * test_trans_queue.c  --  Buddy 1 unit tests for the offline queue.
 * Runs on a PC (no Pico needed). Test IDs match testing/subsystem1/TEST_LOG.md.
 */
#include <stdio.h>
#include <string.h>

#include "trans_queue.h"

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

static int push_tag(uint8_t tag)
{
    uint8_t m[2];
    m[0] = tag;
    m[1] = 0xABu;
    return tq_push(m, 2u);
}

int main(void)
{
    tq_msg_t m;
    int      i;
    int      r;
    uint32_t old_seq;
    uint8_t  big[TQ_MSG_MAX + 1u];
    uint8_t  full[TQ_MSG_MAX];
    uint8_t  one = 1u;

    (void)memset(big, 0, sizeof big);
    (void)memset(full, 7, sizeof full);

    tq_init();
    CHECK("S1-U01", (tq_count() == 0u) && (tq_discarded() == 0u),
          "queue starts empty, discard counter 0");
    CHECK("S1-U02", (tq_peek_oldest(&m) == TQ_EMPTY) && (tq_ack_oldest(1u) == TQ_EMPTY),
          "peek and ack on an empty queue are refused");

    for (i = 1; i <= 100; i++) {
        r = push_tag((uint8_t)i);
        if (r != TQ_OK) {
            break;
        }
    }
    CHECK("S1-U03", (i == 101) && (tq_count() == 100u) && (tq_discarded() == 0u),
          "100 messages stored, none dropped (FR-COM-09)");

    r = push_tag(101u);
    CHECK("S1-U04", (r == TQ_DROPPED_OLDEST) && (tq_count() == 100u) && (tq_discarded() == 1u),
          "101st message drops the oldest and counts it (FR-COM-10, FR-COM-11)");
    CHECK("S1-U05", (tq_peek_oldest(&m) == TQ_OK) && (m.data[0] == 2u),
          "oldest message is now number 2");

    (void)push_tag(102u);
    CHECK("S1-U06", (tq_discarded() == 2u) && (tq_count() == 100u),
          "discard counter keeps counting");

    CHECK("S1-U07", (tq_push(NULL, 2u) == TQ_ERR_ARG) && (tq_push(&one, 0u) == TQ_ERR_ARG),
          "NULL pointer and zero length are rejected");
    CHECK("S1-U08", tq_push(big, (uint16_t)(TQ_MSG_MAX + 1u)) == TQ_ERR_TOO_BIG,
          "257-byte message rejected (NFR-RES-03)");
    CHECK("S1-U09", tq_push(full, (uint16_t)TQ_MSG_MAX) == TQ_DROPPED_OLDEST,
          "exactly 256 bytes accepted (NFR-RES-03)");

    (void)tq_peek_oldest(&m);
    old_seq = m.seq;
    (void)push_tag(200u);                  /* overflow replaces the oldest */
    CHECK("S1-U10", tq_ack_oldest(old_seq) == TQ_ERR_STALE,
          "ack of a message that was already dropped is refused");
    CHECK("S1-U11", tq_discarded() > 0u,
          "a refused ack does not reset the discard counter");

    {
        int      n = 0;
        uint32_t prev = 0u;
        int      ordered = 1;
        int      counter_kept = 1;

        while (tq_peek_oldest(&m) == TQ_OK) {
            if (m.seq <= prev) {
                ordered = 0;
            }
            prev = m.seq;
            if (tq_ack_oldest(m.seq) != TQ_OK) {
                break;
            }
            n++;
            if ((tq_count() > 0u) && (tq_discarded() == 0u)) {
                counter_kept = 0;
            }
        }
        CHECK("S1-U12", (n == 100) && ordered,
              "100 messages sent oldest first, in order");
        CHECK("S1-U13", counter_kept,
              "discard counter kept until the last message is acked");
        CHECK("S1-U14", (tq_count() == 0u) && (tq_discarded() == 0u),
              "counter reset only after the queue is fully sent (FR-COM-12)");
    }

    tq_init();
    for (i = 0; i < 100; i++) {
        (void)push_tag((uint8_t)i);
    }
    for (i = 0; i < 60; i++) {
        (void)tq_peek_oldest(&m);
        (void)tq_ack_oldest(m.seq);
    }
    for (i = 0; i < 60; i++) {
        (void)push_tag((uint8_t)(100 + i));
    }
    {
        int     ok = (tq_count() == 100u);
        int     k;
        uint8_t expect;

        for (k = 0; (k < 100) && ok; k++) {
            expect = (k < 40) ? (uint8_t)(60 + k) : (uint8_t)(100 + (k - 40));
            (void)tq_peek_oldest(&m);
            if (m.data[0] != expect) {
                ok = 0;
            }
            (void)tq_ack_oldest(m.seq);
        }
        CHECK("S1-U15", ok && (tq_discarded() == 0u),
              "order is kept when the queue wraps around");
    }

    printf("%s (%d failed)\n", g_fail ? "SOME TESTS FAILED" : "ALL TESTS PASSED", g_fail);
    return g_fail ? 1 : 0;
}