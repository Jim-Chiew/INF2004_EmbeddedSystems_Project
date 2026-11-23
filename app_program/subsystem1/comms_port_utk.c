/*
 * comms_port_utk.c  --  comms_port.h for micro T-Kernel (the car build).
 *
 * The radio and lwIP belong to the kernel's WiFi task (cyw43_utk_task,
 * lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c). That task also calls
 * comms_task() on every poll (TEAM.md section 7), so:
 *   - joining the WiFi network and retrying it is done by that task, with
 *     WIFI_SSID / WIFI_PASSWORD from config/wifi_secrets.h;
 *   - every lwIP / MQTT call made by comms.c already runs on the task that
 *     owns lwIP, so the net lock is empty;
 *   - the link state comes from cyw43_utk_get_status(), safe from any task.
 *
 * No kernel register headers here (cyw43_utk.h is plain C), and no Pico SDK
 * headers: the status LED is set with the board library's gpio_set_val().
 */

#include "comms_port.h"
#include "s1_pins.h"
#include "cyw43_utk.h"
#include <bsp/libbsp.h>

extern uint64_t time_us_64(void);

/* cyw43_tcpip_link_status() values (lib/cyw43-driver/src/cyw43.h) */
#define CYW43_LINK_JOIN		1
#define CYW43_LINK_NOIP		2
#define CYW43_LINK_UP		3

uint32_t port_now_ms(void)
{
    return (uint32_t)(time_us_64() / 1000u);
}

bool port_init(const comms_config_t *cfg)
{
    (void)cfg;
    return true;                        /* the kernel starts the radio itself */
}

int port_link(void)
{
    T_CYW43_UTK_STATUS st;

    cyw43_utk_get_status(&st);
    if (!st.init_complete || !st.ready) {
        return PORT_LINK_DOWN;
    }
    if (st.link_status == CYW43_LINK_UP) {
        return PORT_LINK_UP;
    }
    if (st.link_status == CYW43_LINK_JOIN || st.link_status == CYW43_LINK_NOIP) {
        return PORT_LINK_JOINING;
    }
    return PORT_LINK_DOWN;
}

const char *port_link_text(void)
{
    return "";                          /* the WiFi task logs the link itself */
}

int port_wifi_join(const comms_config_t *cfg)
{
    (void)cfg;
    return 0;                           /* the WiFi task retries the join itself */
}

void port_net_lock(void)   { }          /* comms_task() runs on the lwIP owner */
void port_net_unlock(void) { }

/* Called by comms_led_task(), from telemetry_task, which owns GP19 */
void port_led(int on)
{
    (void)gpio_set_val(S1_STATUS_LED_PIN, on ? 1 : 0);
}
