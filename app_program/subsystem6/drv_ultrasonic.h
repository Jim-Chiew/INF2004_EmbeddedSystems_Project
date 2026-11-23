/*
 *	drv_ultrasonic.h
 *	Subsystem 6 -- the hardware under the distance sensor: the HC-SR04's
 *	TRIG and ECHO pins (GP0 / GP1, with a GPIO edge interrupt and the
 *	RP2040's 1 MHz timer) and the SG90 servo's PWM (GP15, slice 7 B).
 *	Called ONLY by subsystem 6, which owns all three (TEAM.md section 3).
 *
 *	Keeping the hardware behind these four calls is what lets the logic in
 *	ultrasonic.c be read and tested on its own.
 */

#ifndef DRV_ULTRASONIC_H
#define DRV_ULTRASONIC_H

#include <stdint.h>

/* What hal_us_ping_us() returns instead of a pulse width */
#define HAL_ECHO_TIMEOUT  (-1)   /* ECHO rose but did not fall in time: open space (valid) */
#define HAL_ECHO_STUCK    (-2)   /* ECHO was already high before the trigger and never fell */
#define HAL_ECHO_NONE     (-3)   /* no rising edge at all: unplugged / unpowered / dead */

/* Set up TRIG, ECHO and the ECHO edge interrupt. Call it from the task that
   owns the sensor: the interrupt is enabled on the core that calls this
   (see the comment in drv_ultrasonic.c). 0 = ok, < 0 = kernel error code. */
int     hal_us_init(void);

/* Fire one 10 us TRIG pulse and return the echo pulse width in microseconds,
   or one of the HAL_ECHO_* codes above. Waits on an event flag, so the task
   sleeps (the CPU is free) until the echo interrupt arrives or time runs out. */
int32_t hal_us_ping_us(uint32_t timeout_us);

int     hal_servo_init(void);               /* 50 Hz PWM on the servo pin */
void    hal_servo_pulse_us(uint32_t pulse_us);  /* 500..2400 us = -90..+90 deg */

#endif /* DRV_ULTRASONIC_H */
