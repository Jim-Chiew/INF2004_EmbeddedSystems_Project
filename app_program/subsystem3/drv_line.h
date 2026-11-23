/*
 *	drv_line.h
 *	Subsystem 3 -- IR line sensors (GP13/GP14) and the two buttons
 *	(GP20/GP21).  Called ONLY by nav_task.
 *
 *	Plain C on purpose: drv_line.c uses the Pico SDK GPIO functions (for the
 *	pull-ups, which the kernel's gpio_set_pin does not set), and those
 *	headers cannot be mixed with the kernel headers in one file (see app.h).
 */

#ifndef DRV_LINE_H
#define DRV_LINE_H

#include <stdbool.h>

void	line_init(void);		/* IR inputs (no pulls) and buttons (pull-ups) */
bool	line_left_on(void);		/* left sensor over the black line */
bool	line_right_on(void);
int	line_left_raw(void);		/* raw DO level 0/1, for checking polarity */
int	line_right_raw(void);
bool	button_down(unsigned pin);	/* S3_START_BUTTON_PIN or S3_CAL_BUTTON_PIN */

#endif /* DRV_LINE_H */
