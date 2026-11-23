/*
 * comms.h  --  Buddy 1 (WiFi / MQTT communication and telemetry)
 *
 * The one interface the rest of the team uses.
 *
 *   comms_init(cfg)        set up (does not block, does not touch the radio)
 *   comms_send(sample)     put one telemetry sample in the send queue
 *   comms_send_barcode(t)  publish one decoded barcode text (its own topic)
 *   comms_get_command()    read START / STOP / RESET sent from the broker
 *   comms_led_task()       drive the status LED from the COM state
 *   comms_get_state()      COM_DISCONNECTED / COM_SYNC / COM_CONNECTED
 *   comms_get_stats()      counters (acked, discarded, queued, commands)
 *   comms_now_ms()         milliseconds since comms_init(), for ts
 *   comms_task()           the network work: reconnect, send, receive
 *
 * WHO CALLS WHAT
 *
 *   micro T-Kernel build (the team's base):
 *     - comms_task() is called by the WiFi task (cyw43_utk_task). That task is
 *       the only owner of lwIP, so all network work happens there. YOU DO NOT
 *       CALL comms_task().
 *     - comms_init() is called once from subsystem1_init().
 *     - comms_send(), comms_send_barcode(), comms_send_obstacle() and
 *       comms_led_task() are called from Telemetry_Comms_Task (one task only).
 *     - comms_get_command() is called by the Nav task (one task only).
 *     - comms_get_state() and comms_get_stats() can be called from any task.
 *
 *   Plain Pico SDK build (comms_link.c runs these on core 1):
 *     - one loop calls comms_send(), comms_send_barcode(), comms_task(),
 *       comms_led_task() and comms_get_command() every ~10 ms.
 *
 * WHY THE SPLIT: comms_send() only puts the message in a small inbox (lock
 * free, never blocks). comms_task() moves it into the 100-message queue and
 * sends it. So the sending task and the WiFi task never share the queue and
 * need no mutex. Likewise the commands go from comms_task() to
 * comms_get_command() through a small ring.
 *
 * Rules:
 *  - comms_send(), comms_send_barcode() and comms_send_obstacle(): the same
 *    ONE task only (they share the head of one text ring).
 *    comms_get_command(): ONE task only.
 *  - comms_task() never blocks. A WiFi or broker problem never stops the
 *    caller (FR-COM-02).
 *  - This component never touches the motors. A command is only reported
 *    through comms_get_command(). Nav decides what to do with it.
 *
 * The pure logic is in trans_queue, com_state, status_led and telemetry,
 * which are tested on a PC. comms.c (the engine) is tested on a PC with a
 * fake radio and a fake broker. The two small "ports" (comms_port_utk.c and
 * comms_port_sdk.c) are the only parts that touch real hardware.
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
#define COMMS_TOPIC_BARCODE    "robocar/car1/barcode"      /* car -> broker: decoded text */
#define COMMS_TOPIC_OBSTACLE   "robocar/car1/obstacle"     /* car -> broker: measured object */
#define COMMS_TOPIC_STATUS     "robocar/car1/status"       /* car -> broker: online / offline */
#define COMMS_TOPIC_HUMP       "robocar/car1/hump"         /* car -> broker: hump height */

/* Hump height, one message per hump one wheel rolls over (subsystem 4's
 * hump_height.h), plain ASCII key=value pairs:
 *     h=12 s=R t=5.7
 *   h  height, mm
 *   s  R or L: the wheel that went up; C: the car pitched instead (not one
 *      wheel), so h is only a rough figure
 *   t  the peak tilt it was worked out from, degrees */

/* Whether the car is connected, without any continuous messages. Both are
 * RETAINED on COMMS_TOPIC_STATUS, so anyone subscribing later is told at once:
 *   "online"   published by the car once after every connect
 *   "offline"  the car's Last Will: the BROKER publishes it by itself when the
 *              car vanishes (power off, out of range), once the keep-alive runs
 *              out -- about 1.5 x KEEP_ALIVE_S, so roughly 15 s. */
#define COMMS_STATUS_ONLINE    "online"
#define COMMS_STATUS_OFFLINE   "offline"

/* Barcode text messages. The 64-bit frame only has room for the LEFT/RIGHT
 * direction, so the decoded text goes to COMMS_TOPIC_BARCODE as plain ASCII
 * without the '*'s, once per barcode read, followed by a space and the
 * direction that code commands: "A LEFT", "B RIGHT", "S FORWARD", or
 * "X NONE" for a code that commands nothing (task_telemetry.c builds it). */
#define COMMS_BARCODE_MAX    20u   /* longest text comms_send_barcode() takes:
                                      12 characters of decoded text (the most
                                      a code holds, S1_BC_TEXT_MAX), a space,
                                      and "FORWARD", the longest direction */

/* The telemetry frame is a fixed 64 bits with every field allocated, so an
 * obstacle measurement cannot go in it.  Like the barcode text, it gets its
 * own topic, published once per scan as plain ASCII key=value pairs:
 *     d=142 w=184 n=133 l=10 r=-25 s=L
 *   d  how far away the obstacle was when it stopped the car, mm
 *   w  object length across the car's path, mm ("none" if nothing was found;
 *      then only d and s follow)
 *   n  distance to its closest point, mm
 *   l  its left-most angle, degrees (+ve = left of straight ahead)
 *   r  its right-most angle
 *   s  side with more room to pass: L or R
 * Both topics share one ring and one publish in flight, so a long burst of
 * barcodes can push an obstacle message out (it is counted, see the stats). */
#define COMMS_TEXT_MAX       48u   /* longest text any of these topics takes */
#define COMMS_BARCODE_SLOTS   7u   /* texts that can wait while the link is down */

#define COMMS_RETRY_MS   5000u   /* FR-COM-02: retry every 5 s */
#define SUBSCRIBE_RETRY_MS 1000u /* retry a failed subscribe after 1 s */
#define KEEP_ALIVE_S     10u     /* MQTT keep-alive; the requirement says 1 s, see Version C */
#define COMMS_ACK_TIMEOUT_MS 3000u  /* FR-COM-01: no broker ack in 0.9 s = link lost.
                                    * A sample is published at least once a second,
                                    * so the link loss is seen within about 1 s + 0.9 s. */

#define COMMS_CMD_SETTLE_MS 300u   /* commands in the first 300 ms after subscribing are
                                    * ignored (stale retained/replayed START). A real
                                    * command sent in this window is lost too: resend it. */

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
 * On the micro T-Kernel build the WiFi task joins the network itself with
 * WIFI_SSID / WIFI_PASSWORD from config/wifi_secrets.h, so wifi_ssid and
 * wifi_password are only checked, not used. */

typedef struct {
    uint32_t sent_acked;      /* messages the broker has confirmed */
    uint32_t discarded;       /* samples lost to overflow (queue or inbox full) */
    uint16_t queued;          /* messages waiting to be sent */
    uint32_t cmds_accepted;   /* valid START/STOP/RESET received */
    uint32_t cmds_rejected;   /* anything else received on the command topic */
    uint32_t barcodes_sent;   /* barcode texts the broker has confirmed */
    uint32_t barcodes_dropped; /* barcode texts refused: COMMS_BARCODE_SLOTS were waiting */
    uint32_t obstacles_sent;   /* obstacle measurements the broker has confirmed */
    uint32_t obstacles_dropped;/* obstacle measurements refused: no free slot */
    uint32_t humps_sent;       /* hump heights the broker has confirmed */
    uint32_t humps_dropped;    /* hump heights refused: no free slot */
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

/* Publish the text of one decoded barcode (e.g. "A LEFT") on COMMS_TOPIC_BARCODE,
 * QoS 1, not retained. Never blocks. The text is copied. It is removed only
 * after the broker confirms it; while the link is down up to
 * COMMS_BARCODE_SLOTS texts wait and go out after the reconnect, oldest first.
 *   COMMS_OK             accepted
 *   COMMS_ERR_BUSY       COMMS_BARCODE_SLOTS texts waiting: this one dropped (and counted)
 *   COMMS_ERR_NOT_READY  comms_init() not done
 *   TELEM_ERR_ARG        NULL or empty text
 *   TELEM_ERR_RANGE      longer than COMMS_BARCODE_MAX characters (not cut short) */
int comms_send_barcode(const char *text);

/* Publish one obstacle measurement on COMMS_TOPIC_OBSTACLE (format above).
 * Same rules, same ring and the same ONE calling task as
 * comms_send_barcode(): COMMS_OK, COMMS_ERR_NOT_READY, COMMS_ERR_BUSY,
 * TELEM_ERR_ARG or TELEM_ERR_RANGE. */
int comms_send_obstacle(const char *text);

/* Publish one hump height on COMMS_TOPIC_HUMP (format above).  Same rules,
 * same ring and the same ONE calling task as comms_send_barcode(). */
int comms_send_hump(const char *text);

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

/* Milliseconds since comms_init(). (Not sent: the frame has no timestamp.) */
uint32_t comms_now_ms(void);

#endif /* COMMS_H */