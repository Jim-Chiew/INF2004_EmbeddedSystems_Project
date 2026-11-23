/*
 *	drv_motor.h
 *	Subsystem 2 -- motor PWM driver (Robo Pico truth table):
 *	  forward: A = PWM, B = 0     reverse: A = 0, B = PWM
 *	  brake:   A = 0,   B = 0     coast:   A = 1, B = 1
 *	Called ONLY by motion_task (it owns the PWM slices, TEAM.md section 3).
 *
 *	Plain C on purpose: drv_motor.c uses Pico SDK register headers, which
 *	cannot be mixed with the kernel headers in one file (see app.h).
 */

#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include <stdbool.h>

/* Takes the PWM block out of reset, 20 kHz PWM on the four motor pins, all
   motors braked.  Returns false if the PWM block does not respond. */
bool	motor_init(void);

/* speed_pm in 0.1 % steps: -1000 (full reverse) .. 0 (brake) .. +1000 */
void	motor_set_pm(unsigned pin_a, unsigned pin_b, int speed_pm);

void	motor_brake(unsigned pin_a, unsigned pin_b);	/* stops quickly */
void	motor_coast(unsigned pin_a, unsigned pin_b);	/* wheel turns freely */

#endif /* DRV_MOTOR_H */
