/*
 * buddy1_mqtt.c  --  Buddy 1 bench test, stage C: WiFi + MQTT telemetry.
 *
 * Joins WiFi (wifi_secrets.h), connects to the MQTT broker, and takes one
 * telemetry sample per second. Every sample goes into the offline queue
 * (trans_queue). When the broker is connected, the oldest queued message is
 * published with QoS 1 and removed only after the broker confirms it
 * (FR-COM-12). If the link drops, samples keep piling up (max 100, oldest
 * dropped) and are sent in order after reconnect (FR-COM-03, 04, 09 to 12).
 *
 * The LED follows the real COM state (FR-COM-05, 06).
 *
 * The lwIP callbacks only set flags. The queue is touched ONLY from the main
 * loop, so no locking is needed around it.
 * The topic name is a placeholder until the team agrees on topics.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"
#include "lwip/ip_addr.h"
#include "lwip/apps/mqtt.h"

#include "com_state.h"
#include "status_led.h"
#include "telemetry.h"
#include "trans_queue.h"
#include "wifi_secrets.h"

#define TOPIC_TELEMETRY  "robocar/buddy1/telemetry"

#define LOOP_MS        10u
#define SAMPLE_MS      1000u
#define RETRY_MS       5000u     /* FR-COM-02: retry every 5 s */
#define INFLIGHT_TO_MS 5000u     /* give up waiting for a broker ack */

/* Set by lwIP callbacks, read by the main loop. */
static volatile int g_mqtt_up = 0;
static volatile int g_mqtt_connecting = 0;
static volatile int g_mqtt_status = -1;
static volatile int g_pub_done = 0;
static volatile int g_pub_err = 0;

static const char *link_name(int s)
{
    switch (s) {
    case CYW43_LINK_DOWN:    return "DOWN";
    case CYW43_LINK_JOIN:    return "JOINING";
    case CYW43_LINK_NOIP:    return "JOINED_NO_IP";
    case CYW43_LINK_UP:      return "UP";
    case CYW43_LINK_FAIL:    return "FAIL";
    case CYW43_LINK_NONET:   return "NETWORK_NOT_FOUND";
    case CYW43_LINK_BADAUTH: return "BAD_PASSWORD";
    default:                 return "UNKNOWN";
    }
}

static const char *state_name(com_state_t s)
{
    switch (s) {
    case COM_DISCONNECTED: return "DISCONNECTED";
    case COM_SYNC:         return "SYNC";
    case COM_CONNECTED:    return "CONNECTED";
    default:               return "UNKNOWN";
    }
}

static void conn_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t status)
{
    (void)c;
    (void)arg;
    g_mqtt_status = (int)status;
    g_mqtt_up = (status == MQTT_CONNECT_ACCEPTED);
    g_mqtt_connecting = 0;
}

static void pub_cb(void *arg, err_t err)
{
    (void)arg;
    g_pub_err = (int)err;
    g_pub_done = 1;
}

static void start_wifi_join(void)
{
    int rc = cyw43_arch_wifi_connect_async(WIFI_SSID, WIFI_PASSWORD,
                                           CYW43_AUTH_WPA2_AES_PSK);
    printf("  WiFi join started (rc=%d)\n", rc);
}

static void take_sample(uint32_t now)
{
    telemetry_sample_t s;
    char buf[TELEM_MSG_MAX];
    int n;

    memset(&s, 0, sizeof s);
    s.ts_ms = now;
    strcpy(s.state, "BENCH");
    s.speed_mm_s = 250;
    s.distance_mm = (int32_t)(now / 4u);
    s.heading_deg_x10 = 905;
    s.battery_mv = 7420;

    n = telemetry_format(&s, buf, sizeof buf);
    if (n > 0) {
        (void)tq_push((const uint8_t *)buf, (uint16_t)n);
    }
}

int main(void)
{
    stdio_init_all();
    sleep_ms(2000);

    if (cyw43_arch_init() != 0) {
        printf("cyw43_arch_init failed\n");
        while (true) { sleep_ms(1000); }
    }
    cyw43_arch_enable_sta_mode();

    tq_init();
    com_init();

    static struct mqtt_connect_client_info_t ci;
    memset(&ci, 0, sizeof ci);
    ci.client_id = MQTT_CLIENT_ID;
    ci.keep_alive = 10;               /* seconds; tuned in stage E */

    mqtt_client_t *client = mqtt_client_new();
    if (client == NULL) {
        printf("mqtt_client_new failed\n");
        while (true) { sleep_ms(1000); }
    }

    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    printf("\nBuddy 1 MQTT test: WiFi \"%s\", broker %s:%d, topic %s\n",
           WIFI_SSID, MQTT_BROKER_IP, MQTT_BROKER_PORT, TOPIC_TELEMETRY);
    start_wifi_join();

    int last_link = -99;
    int last_mqtt_status = -99;
    bool was_up = false;
    bool last_level = false;
    uint32_t next_wifi_retry = RETRY_MS;
    uint32_t next_mqtt_try = 0;
    uint32_t next_sample = SAMPLE_MS;
    uint32_t acked = 0;
    int inflight = 0;
    uint32_t inflight_seq = 0;
    uint32_t inflight_since = 0;

    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time()) - t0;
        int link = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
        bool up = (link == CYW43_LINK_UP);

        /* ---- WiFi ---- */
        if (link != last_link) {
            printf("[%6lu ms] WiFi link: %s\n", (unsigned long)now, link_name(link));
            last_link = link;
        }
        if (up && !was_up) {
            cyw43_arch_lwip_begin();
            printf("[%6lu ms] WiFi UP, Pico IP = %s\n", (unsigned long)now,
                   ip4addr_ntoa(netif_ip4_addr(&cyw43_state.netif[CYW43_ITF_STA])));
            cyw43_arch_lwip_end();
            next_mqtt_try = now;
        }
        if (!up && was_up) {
            printf("[%6lu ms] WiFi LOST\n", (unsigned long)now);
            next_wifi_retry = now;
            cyw43_arch_lwip_begin();
            mqtt_disconnect(client);
            cyw43_arch_lwip_end();
            g_mqtt_up = 0;
            inflight = 0;
        }
        was_up = up;

        if (!up && link != CYW43_LINK_JOIN && link != CYW43_LINK_NOIP &&
            now >= next_wifi_retry) {
            printf("[%6lu ms] retrying WiFi...\n", (unsigned long)now);
            start_wifi_join();
            next_wifi_retry = now + RETRY_MS;
        }

        /* ---- MQTT connect / retry ---- */
        if (up && !g_mqtt_up && !g_mqtt_connecting && now >= next_mqtt_try) {
            ip_addr_t broker;
            if (!ipaddr_aton(MQTT_BROKER_IP, &broker)) {
                printf("[%6lu ms] MQTT_BROKER_IP is not a valid address\n",
                       (unsigned long)now);
            } else {
                cyw43_arch_lwip_begin();
                err_t e = mqtt_client_connect(client, &broker, MQTT_BROKER_PORT,
                                              conn_cb, NULL, &ci);
                cyw43_arch_lwip_end();
                printf("[%6lu ms] MQTT connect started (err=%d)\n",
                       (unsigned long)now, (int)e);
                if (e == ERR_OK) {
                    g_mqtt_connecting = 1;
                }
            }
            next_mqtt_try = now + RETRY_MS;
        }
        if (g_mqtt_status != last_mqtt_status) {
            printf("[%6lu ms] MQTT status: %d (0 = connected)\n",
                   (unsigned long)now, g_mqtt_status);
            last_mqtt_status = g_mqtt_status;
        }

        /* ---- one sample per second, always through the queue ---- */
        if (now >= next_sample) {
            take_sample(now);
            next_sample += SAMPLE_MS;
        }

        /* ---- publish the oldest queued message, one at a time ---- */
        if (g_pub_done) {
            g_pub_done = 0;
            if (inflight) {
                inflight = 0;
                if (g_pub_err == ERR_OK) {
                    (void)tq_ack_oldest(inflight_seq);   /* broker confirmed */
                    acked++;
                }
            }
        }
        if (inflight && (now - inflight_since) > INFLIGHT_TO_MS) {
            inflight = 0;                                /* no ack: try again */
        }
        if (g_mqtt_up && !inflight) {
            tq_msg_t m;
            if (tq_peek_oldest(&m) == TQ_OK) {
                cyw43_arch_lwip_begin();
                err_t e = mqtt_publish(client, TOPIC_TELEMETRY, m.data, m.len,
                                       1, 0, pub_cb, NULL);
                cyw43_arch_lwip_end();
                if (e == ERR_OK) {
                    inflight = 1;
                    inflight_seq = m.seq;
                    inflight_since = now;
                }
            }
        }

        /* ---- COM state and LED, from the real link ---- */
        com_inputs_t in;
        in.wifi_up = up;
        in.mqtt_up = (g_mqtt_up != 0);
        in.queue_empty = (tq_count() == 0u);
        com_state_t st = com_update(&in);

        bool level = status_led_level(st, now);
        if (level != last_level) {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, level);
            last_level = level;
        }

        /* ---- once-a-second status line ---- */
        static uint32_t next_report = SAMPLE_MS;
        if (now >= next_report) {
            printf("[%6lu ms] wifi=%s mqtt=%s state=%s queue=%u acked=%lu dropped=%lu\n",
                   (unsigned long)now, up ? "UP" : "down",
                   g_mqtt_up ? "UP" : "down", state_name(st),
                   (unsigned)tq_count(), (unsigned long)acked,
                   (unsigned long)tq_discarded());
            next_report += SAMPLE_MS;
        }

        sleep_ms(LOOP_MS);
    }
}