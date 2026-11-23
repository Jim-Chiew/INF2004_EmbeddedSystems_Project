/*
 * drv_buzzer.c
 * Subsystem 1 -- buzzer hardware. See drv_buzzer.h.
 *
 * GP22 is PWM slice 3, channel A. 125 MHz / 125 = 1 MHz counter, and TOP is
 * set so one counter wrap is one period of the tone.
 *
 * Kernel and BSP headers only, no Pico SDK headers (the "header rule" in
 * TEAM.md): the kernel's sysdef.h and the SDK's headers define the same
 * register names differently.
 */

#include <tk/tkernel.h>
#include <bsp/libbsp.h>
#include "drv_buzzer.h"
#include "s1_pins.h"

/* GPn uses slice n/2 (and n/2 - 8 for GP16 and up), channel A if n is even.
   GP22 -> slice 3 channel A.  NOTE: GP6 is the same slice and channel, but
   subsystem 2 drives it as a PIO encoder input, so the PWM output never
   reaches it (a pin only shows PWM if its function select says PWM). */
#define BUZZER_SLICE	(((S1_BUZZER_PIN) >> 1) & 7)
#define BUZZER_DIV_REG	(PWM_BASE + PWM_CHx_DIV + (BUZZER_SLICE) * 0x14)

/* One counter tick is 1 us, so this many ticks make one period of the tone */
#define BUZZER_TOP	((1000000u / (S1_BUZZER_HZ)) - 1u)

EXPORT void buzzer_init(void)
{
	/* The kernel releases the PWM block from reset only when its own
	   physical timer uses it (USE_PTMR, off here), and a block in reset
	   ignores every write.  Subsystem 2's drv_motor.c and subsystem 6's
	   drv_ultrasonic.c release it too; clearing a bit that is already
	   clear changes nothing.  See the TEAM.md "reset rule". */
	clr_w(RESETS_RESET, RESETS_RESET_PWM);
	while (!(in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM)) { }

	pwm_set_enabled(S1_BUZZER_PIN, FALSE);
	pwm_set_pin(S1_BUZZER_PIN);
	out_w(BUZZER_DIV_REG, (UW)(CLK_PERI_FREQ / 1000000UL) << 4);  /* 8.4 fixed point */
	pwm_set_wrap(S1_BUZZER_PIN, BUZZER_TOP);
	pwm_set_cc(S1_BUZZER_PIN, 0);			/* silent */
	pwm_set_enabled(S1_BUZZER_PIN, TRUE);
}

EXPORT void buzzer_on(void)
{
	buzzer_on_hz(S1_BUZZER_HZ);
}

/* A piezo is loudest near its own resonance (a few kHz) and gets quieter the
   further from it you go, so a second tone is for telling beeps apart, not
   for playing tunes.  The counter runs at 1 MHz, so one period is
   1000000 / hz ticks. */
EXPORT void buzzer_on_hz(unsigned hz)
{
	UW top;

	if (hz < 100u) {
		hz = 100u;			/* keep TOP inside 16 bits */
	}
	top = (1000000u / hz) - 1u;
	pwm_set_wrap(S1_BUZZER_PIN, top);
	pwm_set_cc(S1_BUZZER_PIN, (top + 1u) / 2u);		/* 50 % duty */
}

EXPORT void buzzer_off(void)
{
	pwm_set_cc(S1_BUZZER_PIN, 0);
}
