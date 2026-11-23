/*
 *	drv_ws2812.c
 *	Subsystem 1 -- WS2812 RGB LED driver on PIO.  See drv_ws2812.h.
 *
 *	Pico SDK side: this file must NOT include app.h or any tk/ header.
 *
 *	The PIO program is the standard ws2812.pio from the Pico SDK examples,
 *	pre-assembled here (this build has no pioasm):
 *
 *	    .program ws2812
 *	    .side_set 1
 *	    .wrap_target
 *	    bitloop:
 *	        out x, 1       side 0 [3]   ; T3 = 4 cycles low
 *	        jmp !x do_zero side 1 [2]   ; T1 = 3 cycles high
 *	    do_one:
 *	        jmp  bitloop   side 1 [2]   ; T2 = 3 more cycles high: a "1"
 *	    do_zero:
 *	        nop            side 0 [2]   ; or low: a "0"
 *	    .wrap
 *
 *	10 PIO cycles per bit at 800 kHz -> the state machine runs at 8 MHz.
 *	Each LED takes 24 bits, green-red-blue, most significant bit first.
 */

#include "drv_ws2812.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"

#define CLK_SYS_HZ	125000000u	/* set by the kernel (cpu_clock.c) */
#define WS2812_HZ	800000u
#define CYCLES_PER_BIT	10u

static const uint16_t ws2812_instructions[] = {
	0x6321,		/* 0: out    x, 1           side 0 [3] */
	0x1223,		/* 1: jmp    !x, 3          side 1 [2] */
	0x1200,		/* 2: jmp    0              side 1 [2] */
	0xa242,		/* 3: nop                   side 0 [2] */
};

static const pio_program_t ws2812_program = {
	.instructions = ws2812_instructions,
	.length = 4,
	.origin = -1,			/* relocatable: any free 4 slots */
	.pio_version = 0,
};

static PIO	led_pio;
static uint	led_sm;
static bool	ready = false;

bool ws2812_init(unsigned pin, int *pio_index)
{
	PIO	pios[2] = { pio0, pio1 };
	int	i, sm, offset;

	for (i = 0; i < 2; i++) {
		PIO p = pios[i];
		pio_sm_config c;

		if (!pio_can_add_program(p, &ws2812_program)) {
			continue;
		}
		sm = pio_claim_unused_sm(p, false);
		if (sm < 0) {
			continue;
		}
		offset = pio_add_program(p, &ws2812_program);
		if (offset < 0) {
			pio_sm_unclaim(p, (uint)sm);
			continue;
		}

		pio_gpio_init(p, pin);
		pio_sm_set_consecutive_pindirs(p, (uint)sm, pin, 1, true);

		c = pio_get_default_sm_config();
		sm_config_set_wrap(&c, (uint)offset, (uint)offset + 3u);
		sm_config_set_sideset(&c, 1, false, false);
		sm_config_set_sideset_pins(&c, pin);
		sm_config_set_out_shift(&c, false, true, 24);	/* MSB first, autopull 24 */
		sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
		sm_config_set_clkdiv(&c, (float)CLK_SYS_HZ / (float)(WS2812_HZ * CYCLES_PER_BIT));
		pio_sm_init(p, (uint)sm, (uint)offset, &c);
		pio_sm_set_enabled(p, (uint)sm, true);

		led_pio = p;
		led_sm = (uint)sm;
		ready = true;
		if (pio_index != NULL) {
			*pio_index = i;
		}
		return true;
	}
	return false;
}

void ws2812_fill(unsigned n, uint8_t r, uint8_t g, uint8_t b)
{
	uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b;
	unsigned i;

	if (!ready) {
		return;
	}
	for (i = 0; i < n; i++) {
		/* 24 bits, left-aligned (autopull takes the top 24); the 8-deep
		   joined FIFO never fills for 2 LEDs, so this never waits */
		pio_sm_put_blocking(led_pio, led_sm, grb << 8u);
	}
}
