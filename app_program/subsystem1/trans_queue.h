/*
 * trans_queue.h 
 * Offline telemetry queue: FR-COM-09 to FR-COM-12, NFR-RES-03.
 * Plain C, no kernel calls, so it can be tested on a PC.
 * Callers must serialise access with a mutex (NFR-MNT-02).
 */
#ifndef TRANS_QUEUE_H
#define TRANS_QUEUE_H

#include <stdint.h>

#define TQ_CAPACITY   100u   /* FR-COM-09: up to 100 messages */
#define TQ_MSG_MAX    256u   /* NFR-RES-03: message <= 256 bytes */

#define TQ_OK              0
#define TQ_DROPPED_OLDEST  1    /* stored, but oldest was discarded (FR-COM-10) */
#define TQ_ERR_ARG        (-1)
#define TQ_ERR_TOO_BIG    (-2)
#define TQ_EMPTY          (-3)
#define TQ_ERR_STALE      (-4)  /* oldest message is no longer the one peeked */

typedef struct {
    uint32_t seq;               /* increases for every stored message */
    uint16_t len;
    uint8_t  data[TQ_MSG_MAX];
} tq_msg_t;

void     tq_init(void);

/* Store a message at the back. If full, discard the oldest and count it
 * (FR-COM-10, FR-COM-11) and return TQ_DROPPED_OLDEST. */
int      tq_push(const uint8_t *data, uint16_t len);

/* Copy the oldest message into *out without removing it. */
int      tq_peek_oldest(tq_msg_t *out);

/* Remove the oldest message, only if its seq matches. When this removes the
 * last message the discard counter is reset to 0 (FR-COM-12). */
int      tq_ack_oldest(uint32_t seq);

uint16_t tq_count(void);
uint32_t tq_discarded(void);    /* FR-COM-11 */

#endif /* TRANS_QUEUE_H */