/*
 * com_state.h  
 * COM state machine: COM_DISCONNECTED, COM_SYNC, COM_CONNECTED.
 * FR-COM-01, FR-COM-03, FR-COM-04. Plain C, no kernel or network calls.
 * Only one task calls com_update(); other tasks read the state through an
 * accessor that holds a mutex (NFR-MNT-02).
 */
#ifndef COM_STATE_H
#define COM_STATE_H

#include <stdbool.h>

typedef enum {
    COM_DISCONNECTED = 0,   /* power-up state */
    COM_SYNC,               /* link is up, sending the stored messages */
    COM_CONNECTED           /* link is up, queue empty, sending live */
} com_state_t;

typedef struct {
    bool wifi_up;           /* WiFi joined and has an IP address */
    bool mqtt_up;           /* MQTT connection to the broker is open */
    bool queue_empty;       /* trans_queue holds no messages */
} com_inputs_t;

/* Go to COM_DISCONNECTED (power-up). */
void        com_init(void);

/* Apply the inputs and return the state afterwards. */
com_state_t com_update(const com_inputs_t *in);

com_state_t com_get_state(void);

#endif /* COM_STATE_H */
