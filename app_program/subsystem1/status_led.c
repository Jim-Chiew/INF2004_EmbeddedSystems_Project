/*
 * status_led.c 
 * See status_led.h.
 */
#include "status_led.h"

bool status_led_level(com_state_t state, uint32_t now_ms)
{
    bool on;

    switch (state) {
    case COM_CONNECTED:
    case COM_SYNC:
        on = true;                                  /* FR-COM-05 */
        break;

    case COM_DISCONNECTED:
        /* FR-COM-06: ON for the first half of every 500 ms period */
        on = ((now_ms % LED_BLINK_PERIOD_MS) < LED_BLINK_HALF_PERIOD_MS);
        break;

    default:
        on = false;                                 /* unknown state: LED off */
        break;
    }

    return on;
}