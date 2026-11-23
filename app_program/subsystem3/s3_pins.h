/*
 *	s3_pins.h
 *	Subsystem 3 -- pins.  Plain #defines only, so both the kernel-side
 *	files and the Pico SDK driver file can include it.
 */

#ifndef S3_PINS_H
#define S3_PINS_H

/* Left and right MH-sensors that straddle the guide line (digital DO) */
#define S3_IR_LEFT_PIN		13	/* MH-sensor #1 DO (Grove 1) */
#define S3_IR_RIGHT_PIN		14	/* MH-sensor #2 DO (Grove 6) */

/* 1 = DO reads HIGH over the black line (usual for MH-sensors).
   Set to 0 if the status line shows the opposite. */
#define S3_IR_LINE_IS_HIGH	1

/* Buttons: read LOW when pressed (internal pull-ups) */
#define S3_START_BUTTON_PIN	20	/* start / stop line following */
#define S3_CAL_BUTTON_PIN	21	/* IMU calibration */

#endif /* S3_PINS_H */
