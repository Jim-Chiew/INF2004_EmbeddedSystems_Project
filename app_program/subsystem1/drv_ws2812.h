/*
 *	drv_ws2812.h
 *	Subsystem 1 -- WS2812 ("NeoPixel") RGB LED driver: the Robo Pico's two
 *	RGB LEDs on GP18.  Called ONLY by telemetry_task.
 *
 *	Uses one PIO state machine and 4 instruction slots, in whichever PIO
 *	block has room (the radio's block, or the 4 slots the wheel encoders
 *	leave free).  Call ws2812_init() only after the encoders are running,
 *	so their program (which must sit at offset 0) has been placed first.
 *
 *	Plain C on purpose: drv_ws2812.c uses Pico SDK headers, which cannot be
 *	mixed with the kernel headers in one file (see app.h).
 */

#ifndef DRV_WS2812_H
#define DRV_WS2812_H

#include <stdint.h>
#include <stdbool.h>

/* Returns true if it worked; *pio_index (may be NULL) gets 0 or 1 */
bool	ws2812_init(unsigned pin, int *pio_index);

/* Sets all n LEDs to one colour (0..255 each).  Does nothing before init. */
void	ws2812_fill(unsigned n, uint8_t r, uint8_t g, uint8_t b);

#endif /* DRV_WS2812_H */
