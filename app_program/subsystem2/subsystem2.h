/*
 *	subsystem2.h
 *	Subsystem 2 -- Motion: the two motors (PWM) and wheel encoders (PIO).
 *	Student ID: <student ID>.
 *
 *	This header is subsystem 2's PUBLIC interface: only what other
 *	subsystems are allowed to use.  Everything else stays LOCAL in the .c
 *	files.  Other subsystems include it as "../subsystem2/subsystem2.h".
 *
 *	motion_task is the ONLY task that touches the motor PWM and the encoder
 *	PIO state machines (TEAM.md section 3).  Everyone else:
 *	  - sends a MOTION_CMD to mbf_motion          (IF-01, IF-02)
 *	  - reads the ODOMETRY copy under mtx_odom    (IF-03)
 *
 *	State machine (motion_task):
 *	  STARTING --encoders ready--> STOPPED
 *	  STOPPED  --DRIVE--> DRIVE    --WHEELS--> WHEELS   --STOP--> STOPPED
 *	  any      --ESTOP--> ESTOP    --ENABLE--> STOPPED (or STARTING)
 *	In ESTOP the motors stay braked and DRIVE / WHEELS are ignored, so a
 *	safety stop from subsystem 4 holds even if another task keeps sending
 *	drive commands.  Only ENABLE (sent at the start of a new run) clears it.
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"

#define S2_PRI_MOTION	10		/* priority band 10-11 (TEAM.md) */

/* Diagnostic trace: 1 = while the wheels are driven, print one line every
   20 ms on the USB console:
     m <ms> <state> L<asked>/<given> f<floor> d<counts> | R<asked>/<given> f<floor> d<counts>
   (power in 0.1 % steps; counts = encoder counts in those 20 ms).
   Set back to 0 when done: it is a lot of output. */
#define S2_TRACE	0
#define S2_PERIOD_MS	5		/* odometry refresh period */
#define S2_SYNC_MS	10		/* wheel speed correction period */

/* ---- Pins: motors GP8-11, encoders GP6/16/17/26 (see s2_pins.h) ---- */
#include "s2_pins.h"

/* ---- Wheel geometry, measured with the encoders ---- */
#define ENC_COUNTS_PER_REV	1440	/* counts for ONE wheel turn (placeholder!) */
#define WHEEL_DIAMETER_MM	65	/* (placeholder!) */
#define COUNTS_PER_MM	((float)ENC_COUNTS_PER_REV / (3.14159f * (float)WHEEL_DIAMETER_MM))
#define MM_TO_COUNTS(mm) ((W)((float)(mm) * COUNTS_PER_MM + 0.5f))

/* ---------------------------------------------------------------------
 *  IF-01 (S3 -> S2) and IF-02 (S4 -> S2): motion commands
 *  Message buffer mbf_motion, owned by S2.  One MOTION_CMD per message.
 * --------------------------------------------------------------------- */
typedef enum {
	MOTION_CMD_STOP = 1,	/* brake both wheels */
	MOTION_CMD_DRIVE,	/* left/right: % power 0..100 forwards, or both
				   0..-100 backwards; kept in step by the
				   encoders either way */
	MOTION_CMD_WHEELS,	/* left/right: % power -100..100, set directly (steer, spin) */
	MOTION_CMD_ESTOP,	/* safety stop: brake, ignore DRIVE/WHEELS until ENABLE */
	MOTION_CMD_ENABLE	/* start of a new run: leave ESTOP, and retry the
				   encoders if they failed to start */
} MOTION_CMD_TYPE;

typedef struct {
	UB	type;		/* MOTION_CMD_TYPE */
	B	left;		/* % power, see the type */
	B	right;
	UB	from;		/* sending subsystem (3 or 4), for the log */
} MOTION_CMD;

IMPORT ID	mbf_motion;

/* Send one command to motion_task.  Never blocks for more than a few ms.
   Returns E_OK or a kernel error code. */
IMPORT ER	motion_command(UB type, INT left, INT right, UB from);

/* ---------------------------------------------------------------------
 *  IF-03 (S2 -> anyone): odometry, refreshed every S2_PERIOD_MS
 *  Shared data under mutex mtx_odom, owned by S2.  Read it with
 *  motion_get_odometry() (copies it while holding the mutex).
 * --------------------------------------------------------------------- */
typedef enum {
	MOTION_ST_STARTING,	/* encoders not ready yet: motors braked */
	MOTION_ST_STOPPED,
	MOTION_ST_DRIVE,	/* straight / gentle curve, encoder synced */
	MOTION_ST_WHEELS,	/* each wheel set directly */
	MOTION_ST_ESTOP		/* safety stop latched */
} MOTION_STATE;

typedef struct {
	W	left;		/* encoder counts since power-up (wrap-safe difference) */
	W	right;
	float	speed_cms;	/* forward speed, cm/s, over the last 250 ms */
	UB	state;		/* MOTION_STATE */
	BOOL	encoders_ok;
	H	left_pm;	/* power now applied, 0.1 % steps, + = forward */
	H	right_pm;
	H	left_floor_pm;	/* learned stall floor of each wheel */
	H	right_floor_pm;
	UW	updated_ms;	/* app_now_ms() of this copy */
} ODOMETRY;

IMPORT ID	mtx_odom;
IMPORT void	motion_get_odometry(ODOMETRY *out);
IMPORT const char *motion_state_name(UB state);

/* The motion task (for tests and TEAM.md) */
IMPORT ID	tid_motion;

/* Private to subsystem 2, shared between its own .c files */
IMPORT void	motion_task(INT stacd, void *exinf);
IMPORT void	motion_publish(const ODOMETRY *o);

#endif /* SUBSYSTEM2_H */
