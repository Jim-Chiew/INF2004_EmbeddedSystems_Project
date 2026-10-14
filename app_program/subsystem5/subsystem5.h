/*
 *	subsystem5.h
 *	Subsystem 5 -- Adaptive ultrasonic scanning and obstacle profiling
 *	(HC-SR04 + SG90 servo).  Student ID: <student ID>.
 *
 *	Public to other subsystems (Nav / vehicle controller): the event flag
 *	flg_obstacle and its bits, plus the profile result.  Everything else
 *	(HC-SR04 and servo driver, filtering, scan) stays local to this folder.
 *
 *	Owns: GP2 (TRIG), GP3 (ECHO + GPIO edge interrupt), GP15 (servo PWM,
 *	slice 7 channel B).  Only the s5_obstacle task touches them.
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"
#include "us_scan.h"
#include "us_config.h"

#define S5_PRI_MAIN	20		/* priority band 20-21 (TEAM.md) */
#define S5_PRI_DEMO	21

/* IF-06: Nav <-> S5.  Owned by S5, created in subsystem5_init(). */
#define S5_FLG_CALIBRATED	0x01	/* in : Nav finished calibration          */
#define S5_FLG_FRONT_OBSTACLE	0x02	/* out: FRONT_OBSTACLE_DETECTED (latched) */
#define S5_FLG_PROFILE_REQ	0x04	/* in : Nav asks for a left/right profile */
#define S5_FLG_PROFILE_DONE	0x08	/* out: s5_profile is valid               */
#define S5_FLG_AVOID_DONE	0x10	/* in : Nav left OBSTACLE_AVOIDANCE       */

IMPORT ID		flg_obstacle;
IMPORT us_profile_t	s5_profile;	/* read after S5_FLG_PROFILE_DONE */
IMPORT volatile INT	s5_last_mm;	/* latest filtered distance, for telemetry */

#endif /* SUBSYSTEM5_H */
