/* Fake platform for the PC tests (implements comms_port.h).
 * Include this BEFORE comms.c. The test sets fk_now_ms and fk_link and reads
 * the counters. Compile with -DCOMMS_HOST_TEST. */
#ifndef FAKE_PORT_H
#define FAKE_PORT_H
#include <stdint.h>
#include <stdbool.h>
#include "comms.h"
#include "comms_port.h"

static uint32_t fk_now_ms = 0;              /* fake clock */
static int fk_link = PORT_LINK_DOWN;        /* what the "radio" reports */
static int fk_wifi_joins = 0;               /* join attempts started */
static int fk_led = 0;                      /* last LED level written */
static int fk_led_changes = 0;
static int fk_lock_depth = 0;               /* port_net_lock() nesting */
static int fk_lock_errors = 0;              /* unbalanced lock/unlock */
static int fk_unlocked_calls = 0;           /* lwIP calls made without the lock */

uint32_t port_now_ms(void) { return fk_now_ms; }
bool port_init(const comms_config_t *cfg) { (void)cfg; return true; }
int port_link(void) { return fk_link; }
const char *port_link_text(void)
{
    return (fk_link == PORT_LINK_UP) ? "UP" : (fk_link == PORT_LINK_JOINING) ? "JOINING" : "DOWN";
}
int port_wifi_join(const comms_config_t *cfg) { (void)cfg; fk_wifi_joins++; return 1; }
void port_net_lock(void) { fk_lock_depth++; if (fk_lock_depth > 1) { fk_lock_errors++; } }
void port_net_unlock(void) { fk_lock_depth--; if (fk_lock_depth < 0) { fk_lock_errors++; fk_lock_depth = 0; } }
void port_led(int on) { if (on != fk_led) { fk_led_changes++; } fk_led = on; }

#endif