/*
 *	drv_line.c
 *	Subsystem 3 -- IR line sensors and buttons.  See drv_line.h.
 *
 *	Pico SDK side: this file must NOT include app.h or any tk/ header.
 */

#include "drv_line.h"
#include "s3_pins.h"
#include "hardware/gpio.h"

static void input_init(unsigned pin, bool pull_up)
{
	gpio_init(pin);
	gpio_set_dir(pin, GPIO_IN);
	if (pull_up) {
		gpio_pull_up(pin);
	} else {
		gpio_disable_pulls(pin);	/* the MH-sensor board drives DO itself */
	}
}

void line_init(void)
{
	input_init(S3_IR_LEFT_PIN, false);
	input_init(S3_IR_RIGHT_PIN, false);
	input_init(S3_START_BUTTON_PIN, true);
	input_init(S3_CAL_BUTTON_PIN, true);
}

static bool level_is_line(int level)
{
	return S3_IR_LINE_IS_HIGH ? (level != 0) : (level == 0);
}

int line_left_raw(void)  { return gpio_get(S3_IR_LEFT_PIN) ? 1 : 0; }
int line_right_raw(void) { return gpio_get(S3_IR_RIGHT_PIN) ? 1 : 0; }

bool line_left_on(void)  { return level_is_line(line_left_raw()); }
bool line_right_on(void) { return level_is_line(line_right_raw()); }

bool button_down(unsigned pin)
{
	return !gpio_get(pin);
}
