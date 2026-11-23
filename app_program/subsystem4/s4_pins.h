/*
 *	s4_pins.h
 *	Subsystem 4 -- pins.  Plain #defines only, so both the kernel-side
 *	files and the Pico SDK driver file can include it.
 */

#ifndef S4_PINS_H
#define S4_PINS_H

#define S4_I2C_SDA_PIN	2	/* GP2 = I2C1 SDA -> GY-511 SDA */
#define S4_I2C_SCL_PIN	3	/* GP3 = I2C1 SCL -> GY-511 SCL */

#endif /* S4_PINS_H */
