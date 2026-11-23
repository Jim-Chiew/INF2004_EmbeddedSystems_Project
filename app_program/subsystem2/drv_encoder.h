/*
 *	drv_encoder.h
 *	Subsystem 2 -- quadrature wheel encoders, counted by two PIO state
 *	machines (no CPU time).  Called ONLY by motion_task.
 *
 *	Plain C on purpose: drv_encoder.c uses Pico SDK headers, which cannot
 *	be mixed with the kernel headers in one file (see app.h).
 */

#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include <stdint.h>
#include <stdbool.h>

/* Loads the encoder program into whichever PIO block the radio left free
   and starts both state machines.  Call only after the kernel's WiFi task
   has set up the radio (it takes a PIO block too).  Returns true if it
   worked; *pio_index (may be NULL) gets 0 or 1. */
bool		enc_init(int *pio_index);

/* Raw running counts (wrap-around safe when subtracted) */
uint32_t	enc_left_raw(void);
uint32_t	enc_right_raw(void);

/* Encoder input level 0/1, for checking the wiring */
unsigned	enc_pin_level(unsigned pin);

#endif /* DRV_ENCODER_H */
