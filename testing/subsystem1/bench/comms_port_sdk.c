/*
 * comms_port_sdk.c  --  plain Pico SDK port of the comms engine (comms.c).
 *
 * This file is the only place that talks to the Pico W WiFi chip.
 * comms.c calls these port_* functions and does not know about the SDK.
 * CMake must define COMMS_PORT_SDK for the target (see step 3).
 */
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "comms_port.h"

uint32_t port_now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

bool port_init(const comms_config_t *cfg)
{
    (void)cfg;
    if (cyw43_arch_init() != 0) {
        return false;
    }
    cyw43_arch_enable_sta_mode();
    cyw43_wifi_pm(&cyw43_state, CYW43_NONE_PM);
    return true;
}

static int raw_link(void)
{
    return cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
}

int port_link(void)
{
    int raw = raw_link();
    if (raw == CYW43_LINK_UP) {
        return PORT_LINK_UP;
    }
    if (raw == CYW43_LINK_JOIN || raw == CYW43_LINK_NOIP) {
        return PORT_LINK_JOINING;
    }
    return PORT_LINK_DOWN;
}

const char *port_link_text(void)
{
    switch (raw_link()) {
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

int port_wifi_join(const comms_config_t *cfg)
{
    (void)cyw43_arch_wifi_connect_async(cfg->wifi_ssid, cfg->wifi_password,
                                        CYW43_AUTH_WPA2_AES_PSK);
    return 1;
}

void port_net_lock(void)   { cyw43_arch_lwip_begin(); }
void port_net_unlock(void) { cyw43_arch_lwip_end(); }

void port_led(int on)
{
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on ? 1 : 0);
}