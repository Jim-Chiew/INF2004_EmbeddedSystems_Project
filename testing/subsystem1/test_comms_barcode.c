/*
 * test_comms_barcode.c -- PC test of the barcode text messages in comms.c:
 * comms_send_barcode() and the publish on COMMS_TOPIC_BARCODE.
 *
 * Same fakes as test_comms_task.c: stubs/fake_port.h (clock, radio, lwIP lock)
 * and stubs/lwip/apps/mqtt.h (a fake broker the test controls). The fake broker
 * remembers only the LAST publish, so most checks keep the telemetry queue
 * empty. It does NOT test the real radio or a real broker.
 *
 * Each check prints an ID S1-Pnn in the order it runs. Add new checks at the END.
 *
 * Build and run from the repo root:
 *   gcc -std=c11 -Wall -Wextra -fsanitize=address,undefined -DCOMMS_HOST_TEST \
 *       -Itesting/subsystem1/stubs -Iapp_program/subsystem1 \
 *       testing/subsystem1/test_comms_barcode.c \
 *       app_program/subsystem1/trans_queue.c app_program/subsystem1/com_state.c \
 *       app_program/subsystem1/status_led.c app_program/subsystem1/telemetry.c \
 *       -o /tmp/test_comms_barcode
 *   /tmp/test_comms_barcode
 */
#include <stdio.h>
#include <string.h>
#include "fake_port.h"
#include "../../app_program/subsystem1/comms.c"

#define FK_CONNECT_CLOSED 256   /* MQTT_CONNECT_DISCONNECTED in lwIP */

static int fails = 0;
static int checks = 0;
#define CHECK(c, m) do { checks++; \
    if (c) { printf("PASS S1-P%02d %s\n", checks, m); } \
    else { printf("FAIL S1-P%02d %s\n", checks, m); fails++; } } while (0)

/* run the task for ms of fake time, 10 ms at a time */
static void step(uint32_t ms)
{
    for (uint32_t i = 0; i < ms / 10u; i++) {
        fk_now_ms += 10;
        comms_task();
    }
}

/* connect to the broker: accept, then confirm the subscribe */
static void broker_up(void)
{
    step(20);
    fk_conn_cb(&fk_client, NULL, MQTT_CONNECT_ACCEPTED);
    step(20);
    fk_sub_cb(NULL, ERR_OK);
    step(20);
}

/* the broker answers the LAST publish */
static void broker_ack(err_t e) { fk_pub_cb(fk_pub_arg, e); }

/* the last publish was exactly this text on the barcode topic */
static int last_pub_is(const char *text)
{
    size_t n = strlen(text);
    return (strcmp(fk_pub_topic, COMMS_TOPIC_BARCODE) == 0) &&
           (fk_pub_len == (int)n) && (memcmp(fk_pub_payload, text, n) == 0);
}

static uint32_t bc_sent(void) { comms_stats_t st; comms_get_stats(&st); return st.barcodes_sent; }
static uint32_t bc_dropped(void) { comms_stats_t st; comms_get_stats(&st); return st.barcodes_dropped; }
static uint16_t queued(void) { comms_stats_t st; comms_get_stats(&st); return st.queued; }

/* "N0", "N1", ... (comms_send_barcode() copies the text) */
static const char *num(unsigned i)
{
    static char b[8];
    snprintf(b, sizeof b, "N%u", i);
    return b;
}

int main(void)
{
    comms_config_t cfg = { "ssid", "pw", "10.0.0.2", 1883, "car1", NULL, NULL };
    int pubs;

    /* ---- A: arguments ---- */
    CHECK(comms_send_barcode("A") == COMMS_ERR_NOT_READY, "barcode before init is refused (NOT_READY)");
    CHECK(comms_init(&cfg), "init succeeds");
    CHECK(comms_send_barcode(NULL) == TELEM_ERR_ARG, "NULL text refused");
    CHECK(comms_send_barcode("") == TELEM_ERR_ARG, "empty text refused");
    {
        /* One character over COMMS_BARCODE_MAX */
        char too_long[COMMS_BARCODE_MAX + 2];
        memset(too_long, 'A', COMMS_BARCODE_MAX + 1);
        too_long[COMMS_BARCODE_MAX + 1] = '\0';
        CHECK(comms_send_barcode(too_long) == TELEM_ERR_RANGE,
              "COMMS_BARCODE_MAX + 1 characters refused, not cut short");
    }
    CHECK(bc_dropped() == 0, "refused texts are not counted as dropped");

    /* ---- B: waits while offline, goes out after the connect ---- */
    CHECK(comms_send_barcode("A") == COMMS_OK, "text accepted while offline");
    step(1000);
    CHECK(fk_publishes == 0, "nothing published while offline");
    fk_link = PORT_LINK_UP;
    broker_up();
    CHECK(fk_publishes == 1 && last_pub_is("A"), "after the connect \"A\" is published on robocar/car1/barcode");
    CHECK(fk_pub_qos == 1 && fk_pub_retain == 0, "QoS 1, not retained");
    step(500);
    CHECK(fk_publishes == 1 && bc_sent() == 0, "in flight: not sent again, not counted before the ack");

    /* ---- C: the ack removes it ---- */
    broker_ack(ERR_OK); step(20);
    CHECK(bc_sent() == 1, "broker ack: counted as sent");
    step(500);
    CHECK(fk_publishes == 1, "and it is not sent again");

    /* ---- D: a failed ack keeps the text ---- */
    CHECK(comms_send_barcode("B") == COMMS_OK, "second text accepted");
    step(20);
    CHECK(fk_publishes == 2 && last_pub_is("B"), "\"B\" published");
    broker_ack(ERR_BUF); step(20);
    CHECK(fk_publishes == 3 && last_pub_is("B") && bc_sent() == 1,
          "failed ack: the same text is sent again, not counted");
    broker_ack(ERR_OK); step(20);
    CHECK(bc_sent() == 2, "the next ack counts it");

    /* ---- E: order kept ---- */
    CHECK(comms_send_barcode("L") == COMMS_OK && comms_send_barcode("R") == COMMS_OK, "two texts accepted");
    step(20);
    CHECK(last_pub_is("L"), "the older one goes first");
    broker_ack(ERR_OK); step(20);
    CHECK(last_pub_is("R"), "then the next");
    broker_ack(ERR_OK); step(20);
    CHECK(bc_sent() == 4, "both counted");

    /* ---- F: WiFi drops while a text is in flight ---- */
    pubs = fk_publishes;
    CHECK(comms_send_barcode("C") == COMMS_OK, "text accepted");
    step(20);
    CHECK(fk_publishes == pubs + 1 && last_pub_is("C"), "\"C\" in flight");
    fk_link = PORT_LINK_DOWN;
    step(20);
    fk_link = PORT_LINK_UP;
    broker_up();
    CHECK(fk_publishes == pubs + 2 && last_pub_is("C"),
          "lwIP forgets it when the link drops: sent again after the reconnect");
    broker_ack(ERR_OK); step(20);
    CHECK(bc_sent() == 5, "counted once, when acked");

    /* ---- G: the old connection closes AND a new one is accepted between two
     *      comms_task() calls (lwIP runs in the background), so comms_task()
     *      never sees MQTT down. The text must not wait for an ack forever. ---- */
    pubs = fk_publishes;
    CHECK(comms_send_barcode("D") == COMMS_OK, "text accepted");
    step(20);
    CHECK(fk_publishes == pubs + 1 && last_pub_is("D"), "\"D\" in flight");
    fk_conn_cb(&fk_client, NULL, FK_CONNECT_CLOSED);
    fk_conn_cb(&fk_client, NULL, MQTT_CONNECT_ACCEPTED);
    step(20);
    CHECK(fk_publishes == pubs + 2 && last_pub_is("D"),
          "connection replaced unseen: sent again on the new connection");
    broker_ack(ERR_OK); step(20);
    CHECK(bc_sent() == 6, "and counted when acked");

    /* ---- H: barcode and telemetry acks are kept apart ---- */
    {
        telemetry_sample_t s;
        mqtt_request_cb_t tel_cb, bc_cb;
        void *tel_arg, *bc_arg;

        memset(&s, 0, sizeof s);
        s.state = DL_STATE_LINE_FOLLOWING;

        CHECK(comms_send(&s) == COMMS_OK, "telemetry sample accepted");
        step(10);
        tel_cb = fk_pub_cb; tel_arg = fk_pub_arg;
        CHECK(strcmp(fk_pub_topic, COMMS_TOPIC_TELEMETRY) == 0, "telemetry frame in flight");
        CHECK(comms_send_barcode("E") == COMMS_OK, "text accepted");
        step(10);
        bc_cb = fk_pub_cb; bc_arg = fk_pub_arg;
        CHECK(last_pub_is("E"), "barcode in flight at the same time");
        bc_cb(bc_arg, ERR_OK); step(10);
        CHECK(bc_sent() == 7 && queued() == 1, "barcode ack: the telemetry frame stays queued");
        tel_cb(tel_arg, ERR_OK); step(10);
        CHECK(queued() == 0, "telemetry ack removes the telemetry frame");

        /* the other way round */
        CHECK(comms_send(&s) == COMMS_OK, "telemetry sample accepted");
        step(10);
        tel_cb = fk_pub_cb; tel_arg = fk_pub_arg;
        CHECK(comms_send_barcode("F") == COMMS_OK, "text accepted");
        step(10);
        bc_cb = fk_pub_cb; bc_arg = fk_pub_arg;
        CHECK(last_pub_is("F"), "both in flight");
        pubs = fk_publishes;
        tel_cb(tel_arg, ERR_OK); step(500);
        CHECK(queued() == 0 && bc_sent() == 7 && fk_publishes == pubs,
              "telemetry ack: the barcode stays in flight (not counted, not re-sent)");
        bc_cb(bc_arg, ERR_OK); step(10);
        CHECK(bc_sent() == 8, "barcode ack counts it");
    }

    /* ---- I: COMMS_BARCODE_SLOTS texts can wait; one more is dropped ---- */
    {
        int all_ok = 1;
        int order_ok = 1;

        fk_link = PORT_LINK_DOWN;
        step(20);
        for (unsigned i = 0; i < COMMS_BARCODE_SLOTS; i++) {
            if (comms_send_barcode(num(i)) != COMMS_OK) { all_ok = 0; }
        }
        CHECK(all_ok, "COMMS_BARCODE_SLOTS (7) texts wait while the link is down");
        CHECK(comms_send_barcode("X") == COMMS_ERR_BUSY && bc_dropped() == 1,
              "one more is refused (BUSY) and counted as dropped");
        fk_link = PORT_LINK_UP;
        broker_up();
        for (unsigned i = 0; i < COMMS_BARCODE_SLOTS; i++) {
            if (!last_pub_is(num(i))) { order_ok = 0; }
            broker_ack(ERR_OK); step(20);
        }
        CHECK(order_ok && bc_sent() == 8u + COMMS_BARCODE_SLOTS,
              "after the reconnect all of them go out, oldest first");
    }

    /* ---- J: the longest text goes out whole: 12 characters of code text,
       a space and the longest direction -- exactly COMMS_BARCODE_MAX ---- */
    CHECK(strlen("ABCDEFGHIJKL FORWARD") == COMMS_BARCODE_MAX,
          "COMMS_BARCODE_MAX fits a 12-character code and \" FORWARD\"");
    CHECK(comms_send_barcode("ABCDEFGHIJKL FORWARD") == COMMS_OK,
          "COMMS_BARCODE_MAX characters accepted");
    step(20);
    CHECK(last_pub_is("ABCDEFGHIJKL FORWARD"),
          "and published whole (no '\\0')");
    broker_ack(ERR_OK); step(20);

    /* ---- K: lwIP refuses the publish (output buffer full): tried again ---- */
    fk_publish_ret = ERR_BUF;
    CHECK(comms_send_barcode("K") == COMMS_OK, "text accepted");
    pubs = fk_publishes;
    step(50);
    CHECK(fk_publishes == pubs, "publish refused by lwIP: nothing in flight");
    fk_publish_ret = ERR_OK;
    step(20);
    CHECK(fk_publishes == pubs + 1 && last_pub_is("K"), "published as soon as lwIP takes it");
    broker_ack(ERR_OK); step(20);

    /* ---- L: lwIP lock discipline ---- */
    CHECK(fk_unlocked_calls == 0, "every lwIP call was made with the port lock held");
    CHECK(fk_lock_errors == 0 && fk_lock_depth == 0, "lock and unlock are always paired");

    /* ---- M: online / offline on COMMS_TOPIC_STATUS (Last Will) ---- */
    CHECK(strcmp(FK_STATUS_TOPIC, COMMS_TOPIC_STATUS) == 0,
          "the fake broker watches the car's status topic");
    CHECK(fk_ci != NULL && fk_ci->will_topic != NULL &&
          strcmp(fk_ci->will_topic, COMMS_TOPIC_STATUS) == 0 &&
          fk_ci->will_msg != NULL && strcmp(fk_ci->will_msg, COMMS_STATUS_OFFLINE) == 0 &&
          fk_ci->will_qos == 1 && fk_ci->will_retain == 1,
          "connects with a Last Will: \"offline\" on robocar/car1/status, QoS 1, retained");
    CHECK(fk_status_pubs >= 1 && strcmp(fk_status_payload, COMMS_STATUS_ONLINE) == 0 &&
          fk_status_qos == 1 && fk_status_retain == 1,
          "after connecting it publishes \"online\" there, QoS 1, retained");
    {
        int before = fk_status_pubs;
        step(500);
        CHECK(fk_status_pubs == before, "only once per connection, not continuously");
        fk_conn_cb(&fk_client, NULL, FK_CONNECT_CLOSED);
        step(20);
        CHECK(fk_status_pubs == before, "nothing while the connection is down");
        fk_conn_cb(&fk_client, NULL, MQTT_CONNECT_ACCEPTED);
        step(20);
        CHECK(fk_status_pubs == before + 1 && strcmp(fk_status_payload, COMMS_STATUS_ONLINE) == 0,
              "a new connection says \"online\" again, once");
    }
    CHECK(fk_unlocked_calls == 0 && fk_lock_depth == 0,
          "the status publish is made with the port lock held too");

    /* ---- N: hump heights go out on their own topic ---- */
    {
        comms_stats_t st;
        step(100);
        broker_ack(ERR_OK); step(20);          /* nothing of ours left in flight */
        comms_get_stats(&st);
        uint32_t sent0 = st.humps_sent;
        CHECK(comms_send_hump("h=12 s=R t=5.7") == COMMS_OK, "hump height accepted");
        step(20);
        CHECK(strcmp(fk_pub_topic, COMMS_TOPIC_HUMP) == 0 &&
              strcmp(fk_pub_payload, "h=12 s=R t=5.7") == 0 && fk_pub_qos == 1,
              "published on robocar/car1/hump, whole, QoS 1");
        broker_ack(ERR_OK); step(20);
        comms_get_stats(&st);
        CHECK(st.humps_sent == sent0 + 1u && st.barcodes_sent == bc_sent(),
              "counted as a hump once the broker confirms it, not as a barcode");
    }

    printf("%d checks, %s (%d failed)\n", checks, fails ? "FAILED" : "ALL PASSED", fails);
    return fails != 0;
}
