/*
 * com_state.c  
 * COM state machine. See com_state.h and the WiFi statechart (Section 6).
 */
#include "com_state.h"

#include <stddef.h>

static com_state_t s_state = COM_DISCONNECTED;

void com_init(void)
{
    s_state = COM_DISCONNECTED;
}

com_state_t com_update(const com_inputs_t *in)
{
    bool link_up;

    if (in == NULL) {
        return s_state;                 /* no inputs: change nothing */
    }

    link_up = in->wifi_up && in->mqtt_up;

    switch (s_state) {
    case COM_DISCONNECTED:
        if (link_up) {
            /* FR-COM-03 */
            s_state = in->queue_empty ? COM_CONNECTED : COM_SYNC;
        }
        break;

    case COM_SYNC:
        if (!link_up) {
            s_state = COM_DISCONNECTED; /* FR-COM-01 */
        } else if (in->queue_empty) {
            s_state = COM_CONNECTED;    /* FR-COM-04 */
        } else {
            /* stay in COM_SYNC */
        }
        break;

    case COM_CONNECTED:
        if (!link_up) {
            s_state = COM_DISCONNECTED; /* FR-COM-01 */
        }
        break;

    default:
        s_state = COM_DISCONNECTED;     /* unknown value: fail safe */
        break;
    }

    return s_state;
}

com_state_t com_get_state(void)
{
    return s_state;
}
