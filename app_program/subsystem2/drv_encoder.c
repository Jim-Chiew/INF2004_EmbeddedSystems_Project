/*
 *	drv_encoder.c
 *	Subsystem 2 -- quadrature wheel encoders on PIO.  See drv_encoder.h.
 *
 *	Pico SDK side: this file must NOT include app.h or any tk/ header.
 *	The program (quadrature_encoder.pio, pre-built into
 *	quadrature_encoder.pio.h with the Pico SDK's pioasm) uses computed
 *	jumps, so it must sit at offset 0 of a PIO block and takes 28 of its 32
 *	instruction slots.  The Pico W radio driver (kernel WiFi task) also
 *	loads a program into one PIO block, so the encoders take the other.
 *	The Pico SDK PIO and GPIO functions used here are the ones the
 *	template already builds for the radio (build_make/pico_rp2040.mk).
 */

#include "drv_encoder.h"
#include "s2_pins.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "quadrature_encoder.pio.h"

static PIO	enc_pio;
static uint	left_sm;
static uint	right_sm;
static bool	ready = false;

bool enc_init(int *pio_index)
{
	PIO	pios[2] = { pio0, pio1 };
	int	i, sl, sr;

	for (i = 0; i < 2; i++) {
		PIO p = pios[i];

		if (!pio_can_add_program_at_offset(p, &quadrature_encoder_program, 0)) {
			continue;		/* the radio is using this one */
		}
		sl = pio_claim_unused_sm(p, false);
		sr = pio_claim_unused_sm(p, false);
		if (sl < 0 || sr < 0) {
			if (sl >= 0) pio_sm_unclaim(p, (uint)sl);
			if (sr >= 0) pio_sm_unclaim(p, (uint)sr);
			continue;
		}
		if (pio_add_program_at_offset(p, &quadrature_encoder_program, 0) < 0) {
			pio_sm_unclaim(p, (uint)sl);
			pio_sm_unclaim(p, (uint)sr);
			continue;
		}

		enc_pio  = p;
		left_sm  = (uint)sl;
		right_sm = (uint)sr;
		quadrature_encoder_program_init(p, left_sm, S2_LEFT_ENC_A, S2_LEFT_ENC_B);
		quadrature_encoder_program_init(p, right_sm, S2_RIGHT_ENC_A, S2_RIGHT_ENC_B);
		ready = true;
		if (pio_index != NULL) {
			*pio_index = i;
		}
		return true;
	}
	return false;
}

uint32_t enc_left_raw(void)
{
	return ready ? (uint32_t)quadrature_encoder_get_count(enc_pio, left_sm) : 0u;
}

uint32_t enc_right_raw(void)
{
	return ready ? (uint32_t)quadrature_encoder_get_count(enc_pio, right_sm) : 0u;
}

unsigned enc_pin_level(unsigned pin)
{
	return gpio_get(pin) ? 1u : 0u;
}
