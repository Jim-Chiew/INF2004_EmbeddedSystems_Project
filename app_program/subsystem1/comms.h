/*
 * comms.h  --  Buddy 1 (WiFi / MQTT communication and telemetry)
 *
 * The one interface the rest of the team uses.
 *
 *   comms_init(cfg)        set up (does not block, does not touch the radio)
 *   comms_send(sample)     put one telemetry sample in the send queue
 *   comms_get_command()    read START / STOP / RESET sent from the broker
 *   comms_led_task()       drive the status LED from the COM state
 *   comms_get_state()      COM_DISCONNECTED / COM_SYNC / COM_CONNECTED
 *   comms_get_stats()      counters (acked, discarded, queued, commands)
 *   comms_now_ms()         milliseconds since comms_init(), for ts
 *   comms_task()           the network work: reconnect, send, receive
 *
  * WHO CALLS WHAT
 *
 *   The team build is the plain Pico SDK with one polled main loop.
 *     - comms_init() is called once at start-up.
 *     - comms_task() is called every ~10 ms. It is the only function that
 *       touches lwIP. It never blocks.
 *     - comms_send() is called by the telemetry code, about once per second.
 *     - comms_led_task() is called every ~10 ms.
 *     - comms_get_command() is called by the Nav code, at least every 20 ms.
 *     - comms_get_state() and comms_get_stats() can be called from anywhere.
 *   See testing/subsystem1/bench/buddy1_mqtt.c for a working loop.
  * WHY THE SPLIT: comms_send() only puts the message in a small inbox (lock
 * free, never blocks). comms_task() moves it into the 100-message queue and
 * sends it. So the sending code and comms_task() never share the queue and
 * need no mutex when they run in different tasks. Likewise the commands go
 * from comms_task() to comms_get_command() through a small ring.
 *
 * Rules:
 *  - comms_send(): ONE task only. comms_get_command(): ONE task only.
 *  - comms_task() never blocks. A WiFi or broker problem never stops the
 *    caller (FR-COM-02).
 *  - This component never touches the motors. A command is only reported
 *    through comms_get_command(). Nav decides what to do with it.
 *
 * The pure logic is in trans_queue, com_state, status_led and telemetry,
 * which are tested on a PC. comms.c (the engine) is tested on a PC with a
 * fake radio and a fake broker. The only part that touches real hardware is
 * the small port, bench/comms_port_sdk.c.
 */
#ifndef COMMS_H
#define COMMS_H

#include <stdbool.h>
#include <stdint.h>

#include "com_state.h"
#include "telemetry.h"

/* MQTT topics. Change them here only. See COMMS_FORMAT.md. */
#define COMMS_TOPIC_TELEMETRY  "robocar/car1/telemetry"   /* car -> broker */
#define COMMS_TOPIC_CMD        "robocar/car1/cmd"          /* broker -> car */

#define COMMS_RETRY_MS   5000u   /* FR-COM-02: retry every 5 s */
#define COMMS_ACK_TIMEOUT_MS 3000u  /* FR-COM-01: no broker ack in 3 s = link lost.
                                    * 0.9 s gave false "link lost" about once per 40 s
                                    * on the bench WiFi (late acks), 3 s gave 1 in 10 min.
                                    * A sample is published at least once a second,
                                    * so the link loss is seen within about 1 s + 3 s. */

/* comms_send() results */
#define COMMS_OK              0
#define COMMS_ERR_NOT_READY (-20)   /* comms_init() not done (or no network in this build) */
#define COMMS_ERR_BUSY      (-21)   /* inbox full: sample dropped and counted */
/* other negative values: TELEM_ERR_ARG / TELEM_ERR_TOO_BIG (bad sample) */

typedef enum {
    COMMS_CMD_NONE = 0,
    COMMS_CMD_START,
    COMMS_CMD_STOP,
    COMMS_CMD_RESET
} comms_cmd_t;

typedef struct {
    const char *wifi_ssid;       /* required */
    const char *wifi_password;   /* required */
    const char *broker_ip;       /* required, e.g. "192.168.1.10" */
    uint16_t    broker_port;     /* normally 1883 */
    const char *client_id;       /* required, unique on the broker */
    const char *username;        /* NULL = anonymous (bench only) */
    const char *password;        /* NULL = anonymous */
} comms_config_t;
/* The strings are not copied. They must stay valid while the car runs.
   * comms_init() only checks them. comms_task() starts the join. */

typedef struct {
    uint32_t sent_acked;      /* messages the broker has confirmed */
    uint32_t discarded;       /* samples lost to overflow (queue or inbox full) */
    uint16_t queued;          /* messages waiting to be sent */
    uint32_t cmds_accepted;   /* valid START/STOP/RESET received */
    uint32_t cmds_rejected;   /* anything else received on the command topic */
} comms_stats_t;

/* Set up the component. Returns true if it worked. Returns straight away.
 * Call once, before the first comms_send(). */
bool comms_init(const comms_config_t *cfg);

/* Format one sample as JSON and put it in the inbox. Never blocks.
 *   COMMS_OK             accepted
 *   COMMS_ERR_BUSY       inbox full: this sample was dropped (and counted)
 *   COMMS_ERR_NOT_READY  comms_init() not done
 *   other < 0            bad sample (TELEM_ERR_ARG, TELEM_ERR_TOO_BIG)
 * When the 100-message queue overflows later, the OLDEST message is dropped
 * and counted in comms_stats_t.discarded (FR-COM-09/10/11). */
int comms_send(const telemetry_sample_t *s);

/* The network work. See "WHO CALLS WHAT" above. Does not block. */
void comms_task(void);

/* Returns the oldest unread command, or COMMS_CMD_NONE. Each command is
 * returned once. Holds up to 3 unread commands. */
comms_cmd_t comms_get_command(void);

/* Drives the status LED from the COM state (FR-COM-05, FR-COM-06).
 * Call every ~10 ms from one task. */
void comms_led_task(void);

com_state_t comms_get_state(void);
void        comms_get_stats(comms_stats_t *out);

/* Milliseconds since comms_init(). Use it for telemetry_sample_t.ts_ms. */
uint32_t comms_now_ms(void);

#endif /* COMMS_H */