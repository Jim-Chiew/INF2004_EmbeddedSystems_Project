/*
 * comms_port.h  --  the few things comms.c needs from the platform.
 *
 * comms.c (the engine) contains no Pico SDK, no micro T-Kernel and no radio
 * code. Everything it needs from the platform is in this list. There are
 * three implementations:
 *
 *   comms_port_utk.c              micro T-Kernel base (the team's real build)
 *   bench/comms_port_sdk.c        plain Pico SDK bench build
 *   testing/.../fake_port.h       PC tests (fake clock, fake radio)
 *
 * Which one is used is decided by a compile flag, see below. The micro
 * T-Kernel build needs no flag: it is the default.
 */
#ifndef COMMS_PORT_H
#define COMMS_PORT_H

#include <stdbool.h>
#include <stdint.h>

#include "comms.h"

/* ---- which platform are we compiled for? ---- */
#if defined(COMMS_HOST_TEST)            /* PC test: gcc -DCOMMS_HOST_TEST */
  #define COMMS_ENV_HOST 1
  #define COMMS_HAVE_NET 1
#elif defined(COMMS_PORT_SDK)           /* plain Pico SDK: CMake sets it */
  #define COMMS_ENV_SDK 1
  #define COMMS_HAVE_NET 1
#elif defined(_PICO_RP2040_)            /* micro T-Kernel makefile sets it */
  #define COMMS_ENV_UTK 1
  #if defined(TM_MQTT) && (TM_MQTT == 1)
    #define COMMS_HAVE_NET 1            /* make NET=lwip MQTT=1 */
  #else
    #define COMMS_HAVE_NET 0            /* no lwIP/MQTT in this build: stubs */
  #endif
#else
  #error "comms: define COMMS_HOST_TEST, COMMS_PORT_SDK, or build with the micro T-Kernel makefile"
#endif

/* ---- logging: one line per event, never in a loop ---- */
#if defined(COMMS_ENV_UTK)
  #include <tm/tmonitor.h>
  /* tm_printf supports %s %d %u %x (no %l, no %f). Cast values to int/unsigned. */
  #define PORT_LOG(fmt, ...) ((void)tm_printf((const UB *)(fmt), ##__VA_ARGS__))
#else
  #include <stdio.h>
  #define PORT_LOG(...) ((void)printf(__VA_ARGS__))
#endif

/* ---- memory barrier: makes sure data is written before the index that
 *      announces it (needed when the two tasks run on different cores) ---- */
#if defined(__arm__)
  #define PORT_MB() __asm__ volatile("dmb" ::: "memory")
#else
  #define PORT_MB() __sync_synchronize()
#endif

/* ---- link state as comms.c sees it ---- */
#define PORT_LINK_DOWN     0   /* not joined (or failed) */
#define PORT_LINK_JOINING  1   /* joined the access point, waiting for an IP */
#define PORT_LINK_UP       2   /* joined and has an IP address */

#if COMMS_HAVE_NET || defined(COMMS_ENV_UTK)

/* Milliseconds since power-up. Any task may call it. */
uint32_t port_now_ms(void);

/* Called once from comms_init(). Returns false if the radio cannot start. */
bool port_init(const comms_config_t *cfg);

/* Current WiFi state, PORT_LINK_*. Called by comms_task() only. */
int port_link(void);

/* Text for the raw radio state, for the log. Must be a string literal (comms.c
 * compares the pointer to see if it changed). May return "" . */
const char *port_link_text(void);

/* Ask the radio to (re)join the network. Returns 1 if this port started a
 * join (comms.c then logs it), 0 if something else retries the join itself
 * (the micro T-Kernel WiFi task does). Called by comms_task() every 5 s while
 * the link is down. */
int port_wifi_join(const comms_config_t *cfg);

/* Lock around lwIP calls. Needed when lwIP runs in a background context (SDK
 * bench). Empty on micro T-Kernel, where comms_task() already runs inside the
 * one task that owns lwIP. */
void port_net_lock(void);
void port_net_unlock(void);

/* Status LED (1 = on). Called by comms_led_task(). */
void port_led(int on);

#endif /* COMMS_HAVE_NET */

#endif /* COMMS_PORT_H */