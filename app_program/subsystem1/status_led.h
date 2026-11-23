/*
 * status_led.h
 * Decides whether the status LED is ON or OFF for a given COM state and time.
 * FR-COM-05 (solid ON in COM_CONNECTED and COM_SYNC) and
 * FR-COM-06 (2 Hz blink in COM_DISCONNECTED).
 * Plain C: it only computes the level. A separate driver writes it to the pin.
 */
#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <stdbool.h>
#include <stdint.h>

#include "com_state.h"

#define LED_BLINK_HZ             2u     /* FR-COM-06 */
#define LED_BLINK_PERIOD_MS      (1000u / LED_BLINK_HZ)       /* 500 ms */
#define LED_BLINK_HALF_PERIOD_MS (LED_BLINK_PERIOD_MS / 2u)   /* 250 ms */

/* Returns true when the LED should be ON.
 * now_ms is a free-running millisecond clock. */
bool status_led_level(com_state_t state, uint32_t now_ms);

#endif /* STATUS_LED_H */