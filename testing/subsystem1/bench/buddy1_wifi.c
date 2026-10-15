/*
 * buddy1_wifi.c  --  Buddy 1 bench test, stage B: join WiFi.
 *
 * Joins the WiFi network named in wifi_secrets.h (git-ignored) and prints
 * the link state and the Pico's IP address on the serial port. If the link
 * fails or drops, it tries again every 5 s (FR-COM-02).
 *
 * The MQTT broker is not used yet (stage C), so mqtt_up is always false and
 * the COM state stays COM_DISCONNECTED: the LED keeps blinking at 2 Hz even
 * when WiFi is up. That is expected at this stage. FR-COM-03 needs both
 * WiFi and MQTT before the state changes.
 *
 * The password is never printed.
 */
#include <stdio.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"

#include "com_state.h"
#include "status_led.h"
#include "wifi_secrets.h"

#define LOOP_MS    10u
#define RETRY_MS   5000u    /* FR-COM-02: retry every 5 s */
#define REPORT_MS  5000u

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

static void start_join(void)
{
    int rc = cyw43_arch_wifi_connect_async(WIFI_SSID, WIFI_PASSWORD,
                                           CYW43_AUTH_WPA2_AES_PSK);
    printf("  join attempt started (rc=%d)\n", rc);
}

int main(void)
{
    stdio_init_all();
    sleep_ms(2000);                      /* time to open the serial monitor */

    if (cyw43_arch_init() != 0) {
        printf("cyw43_arch_init failed\n");
        while (true) { sleep_ms(1000); }
    }
    cyw43_arch_enable_sta_mode();
    com_init();

    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    printf("\nBuddy 1 WiFi test: joining \"%s\"\n", WIFI_SSID);
    start_join();

    int last_link = -99;
    bool was_up = false;
    bool last_level = false;
    uint32_t next_retry = RETRY_MS;
    uint32_t next_report = REPORT_MS;

    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time()) - t0;
        int link = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
        bool up = (link == CYW43_LINK_UP);

        if (link != last_link) {
            printf("[%6lu ms] link: %s\n", (unsigned long)now, link_name(link));
            last_link = link;
        }

        if (up && !was_up) {
            cyw43_arch_lwip_begin();
            printf("[%6lu ms] WiFi UP, Pico IP = %s\n", (unsigned long)now,
                   ip4addr_ntoa(netif_ip4_addr(&cyw43_state.netif[CYW43_ITF_STA])));
            cyw43_arch_lwip_end();
        }
        if (!up && was_up) {
            printf("[%6lu ms] WiFi LOST\n", (unsigned long)now);
            next_retry = now;            /* try again right away */
        }
        was_up = up;

        /* Retry every 5 s while not connected and not already joining. */
        if (!up && link != CYW43_LINK_JOIN && link != CYW43_LINK_NOIP &&
            now >= next_retry) {
            printf("[%6lu ms] retrying WiFi...\n", (unsigned long)now);
            start_join();
            next_retry = now + RETRY_MS;
        }

        /* COM state from the real WiFi status; MQTT is not used yet. */
        com_inputs_t in;
        in.wifi_up = up;
        in.mqtt_up = false;
        in.queue_empty = true;
        com_state_t st = com_update(&in);

        bool level = status_led_level(st, now);
        if (level != last_level) {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, level);
            last_level = level;
        }

        if (now >= next_report) {
            printf("[%6lu ms] link=%s com_state=%d\n", (unsigned long)now,
                   link_name(link), (int)st);
            next_report += REPORT_MS;
        }

        sleep_ms(LOOP_MS);
    }
}