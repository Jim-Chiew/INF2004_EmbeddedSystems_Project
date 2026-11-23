/*
 *	drv_motor.c
 *	Subsystem 2 -- motor PWM driver.  See drv_motor.h.
 *
 *	Pico SDK side: this file must NOT include app.h or any tk/ header.
 *	The kernel's board library (libbsp) has no PWM clock divider, so the
 *	PWM slice registers are written directly.  Each motor pin is one PWM
 *	channel: GPn uses slice n/2, channel A if n is even, B if odd.
 */

#include "drv_motor.h"
#include "s2_pins.h"
#include "hardware/structs/pwm.h"
#include "hardware/structs/resets.h"
#include "hardware/gpio.h"
#include <stdint.h>
#include <stdlib.h>

#define PWM_FREQ_HZ	20000u		/* above hearing range, fine for small DC motors */
#define PWM_TOP		1000u		/* duty levels 0..1000 (wrap = PWM_TOP - 1) */
#define CLK_SYS_HZ	125000000u	/* set by the kernel (kernel/sysdepend/
					   pico_rp2040/cpu_clock.c) */

static unsigned slice_of(unsigned pin) { return (pin >> 1) & 7u; }

/* level > wrap keeps the output high all the time (used for coast) */
static void set_level(unsigned pin, uint32_t level)
{
	io_rw_32 *cc = &pwm_hw->slice[slice_of(pin)].cc;

	if (pin & 1u) {		/* channel B: upper 16 bits */
		*cc = (*cc & 0x0000FFFFu) | (level << 16);
	} else {		/* channel A: lower 16 bits */
		*cc = (*cc & 0xFFFF0000u) | (level & 0xFFFFu);
	}
}

static void pwm_pin_init(unsigned pin)
{
	unsigned s = slice_of(pin);
	/* Divider, 8.4 fixed point: 125 MHz / (20 kHz x 1000) = 6.25 */
	uint32_t div16 = (CLK_SYS_HZ * 16u) / (PWM_FREQ_HZ * PWM_TOP);

	pwm_hw->slice[s].div = ((div16 / 16u) << PWM_CH0_DIV_INT_LSB) |
			       ((div16 % 16u) << PWM_CH0_DIV_FRAC_LSB);
	pwm_hw->slice[s].top = PWM_TOP - 1u;
	set_level(pin, 0);
	pwm_hw->slice[s].csr |= PWM_CH0_CSR_EN_BITS;
	gpio_set_function(pin, GPIO_FUNC_PWM);
}

bool motor_init(void)
{
	/* The kernel releases the PWM block from reset only when its own
	   physical timer uses it (USE_PTMR, off in this project: see
	   kernel/sysdepend/pico_rp2040/hw_setting.c).  In reset, every write
	   below is ignored and the motor pins stay low (= brake).  So release
	   it here, like drv_i2c.c does for I2C1.  Nothing else uses PWM. */
	hw_clear_bits(&resets_hw->reset, RESETS_RESET_PWM_BITS);
	while (!(resets_hw->reset_done & RESETS_RESET_PWM_BITS)) {
	}

	pwm_pin_init(S2_LEFT_MOTOR_A);
	pwm_pin_init(S2_LEFT_MOTOR_B);
	pwm_pin_init(S2_RIGHT_MOTOR_A);
	pwm_pin_init(S2_RIGHT_MOTOR_B);
	motor_brake(S2_LEFT_MOTOR_A, S2_LEFT_MOTOR_B);
	motor_brake(S2_RIGHT_MOTOR_A, S2_RIGHT_MOTOR_B);

	/* Read back: a block still in reset reads 0 */
	return pwm_hw->slice[slice_of(S2_LEFT_MOTOR_A)].top == PWM_TOP - 1u &&
	       pwm_hw->slice[slice_of(S2_RIGHT_MOTOR_A)].top == PWM_TOP - 1u;
}

void motor_set_pm(unsigned pin_a, unsigned pin_b, int speed_pm)
{
	uint32_t level;

	if (speed_pm > 1000)  speed_pm = 1000;
	if (speed_pm < -1000) speed_pm = -1000;
	level = (uint32_t)abs(speed_pm) * PWM_TOP / 1000u;

	if (speed_pm > 0) {
		set_level(pin_a, level);
		set_level(pin_b, 0);
	} else if (speed_pm < 0) {
		set_level(pin_a, 0);
		set_level(pin_b, level);
	} else {
		set_level(pin_a, 0);
		set_level(pin_b, 0);
	}
}

void motor_brake(unsigned pin_a, unsigned pin_b)
{
	set_level(pin_a, 0);
	set_level(pin_b, 0);
}

void motor_coast(unsigned pin_a, unsigned pin_b)
{
	set_level(pin_a, PWM_TOP);
	set_level(pin_b, PWM_TOP);
}
