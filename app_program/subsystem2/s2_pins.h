/*
 *	s2_pins.h
 *	Subsystem 2 -- pins (Robo Pico).  Plain #defines only, so both the
 *	kernel-side files and the Pico SDK driver files can include it.
 *
 *	If a wheel spins the wrong way, swap its _A and _B motor pins.
 *	If an encoder counts backwards, swap its _A and _B encoder pins.
 */

#ifndef S2_PINS_H
#define S2_PINS_H

#define S2_LEFT_MOTOR_A		11
#define S2_LEFT_MOTOR_B		10
#define S2_RIGHT_MOTOR_A	8
#define S2_RIGHT_MOTOR_B	9

#define S2_LEFT_ENC_A		16
#define S2_LEFT_ENC_B		17
#define S2_RIGHT_ENC_A		26
#define S2_RIGHT_ENC_B		6

#endif /* S2_PINS_H */
