/*
 *	subsystem3.h
 *	Subsystem 3 -- Navigation: IR line following, the GP20/GP21 buttons,
 *	and the car's run state machine.  Student ID: <student ID>.
 *
 *	This header is subsystem 3's PUBLIC interface.  nav_task is the ONLY
 *	task that reads the IR sensors and buttons (TEAM.md section 3).  It
 *	drives through subsystem 2 (IF-01), asks subsystem 4 for calibration
 *	(IF-04), reacts to its safety events (IF-05), and takes remote
 *	commands from subsystem 1 (IF-07).  Everyone may read NAV_STATUS (IF-08).
 *	It also watches subsystem 6 for an obstacle ahead (IF-11, IF-12).
 *
 *	Run state machine (nav_task):
 *	  IDLE --GP20 / START--> FOLLOWING --GP20 / STOP / RESET / lifted /
 *	        overturned--> IDLE
 *	  IDLE --GP21--> CALIBRATING (spin, settle, zero) --done / STOP--> IDLE
 *	  FOLLOWING --obstacle within S6_STOP_MM--> AVOIDING (motors stopped)
 *	  AVOIDING --GP20 / START--> FOLLOWING   (clears the obstacle latch)
 *	  AVOIDING --STOP / RESET / lifted / overturned--> IDLE
 *	  FOLLOWING --a code failed to decode (IF-15)--> BARCODE_RESCAN
 *	         (back along the line, forward again on the recorded heading,
 *	         then the line search) --> FOLLOWING, or IDLE if the line is
 *	         not found again
 *	  RESET clears the run counters (junctions, humps, barcode).
 *	Follow mode, inside FOLLOWING:
 *	  FOLLOW --one sensor on the line--> CONFIRM (drive CONFIRM_MM) -->
 *	         FOLLOW, steering back towards the line
 *	  any    --both sensors on the line--> CROSSING (drive straight
 *	         CROSS_MM .. CROSS_MAX_MM, sensors ignored) --> FOLLOW
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"
#include "s3_pins.h"

#define S3_PRI_NAV	14		/* priority band 14-15 (TEAM.md) */

/* ---------------------------------------------------------------------
 *  IF-07 (S1 -> S3): remote commands.  Message buffer mbf_nav_cmd,
 *  owned by S3.  One UB per message.
 * --------------------------------------------------------------------- */
typedef enum {
	NAV_CMD_START = 1,	/* start line following (from IDLE) */
	NAV_CMD_STOP,		/* stop and wait in IDLE */
	NAV_CMD_RESET		/* stop and clear the run counters */
} NAV_CMD;

IMPORT ID	mbf_nav_cmd;
IMPORT ER	nav_command(UB cmd);	/* never waits */

/* ---------------------------------------------------------------------
 *  IF-08 (S3 -> anyone): run status.  Shared data under mutex mtx_nav,
 *  owned by S3; copy it with nav_get_status().
 * --------------------------------------------------------------------- */
/* NAV_AVOIDING: stopped in front of an obstacle subsystem 6 found.  The
   avoidance manoeuvre itself is not written yet (TEAM.md section 1): the car
   waits there, with the scan on the console, until GP20 or START. */
/* NAV_BARCODE_RESCAN: a code the car drove over would not decode (IF-15), so
   it backs up along the line and drives over it again.  Added at the END of
   the list on purpose: the numbers already published for the other states do
   not move. */
typedef enum { NAV_IDLE, NAV_CALIBRATING, NAV_FOLLOWING, NAV_AVOIDING,
	       NAV_BARCODE_RESCAN, NAV_TURNING } NAV_STATE;

typedef enum {
	ACT_STOP, ACT_STRAIGHT, ACT_STEER_LEFT, ACT_SPIN_LEFT,
	ACT_STEER_RIGHT, ACT_SPIN_RIGHT
} NAV_ACTION;

typedef enum { MODE_FOLLOW, MODE_CONFIRM, MODE_CROSSING } NAV_MODE;

typedef struct {
	UB	state;		/* NAV_STATE */
	UB	action;		/* NAV_ACTION: what the motors were told */
	UB	mode;		/* NAV_MODE (while FOLLOWING) */
	BOOL	left;		/* sensor on the line (debounced while following) */
	BOOL	right;
	UB	avoid_step;	/* AVOID_FINISHED, or the step of the manoeuvre
				   round an object that is running (avoid.h) */
	UW	junctions;	/* junctions crossed in this run */
	UW	run_count;	/* +1 at every new run and every RESET */
} NAV_STATUS;

IMPORT ID	mtx_nav;
IMPORT void	nav_get_status(NAV_STATUS *out);

IMPORT const char *nav_state_name(UB state);
IMPORT const char *nav_action_name(UB action);
IMPORT const char *nav_mode_name(UB mode);

IMPORT ID	tid_nav;

/* Private to subsystem 3, shared between its own .c files */
IMPORT void	nav_task(INT stacd, void *exinf);
IMPORT void	nav_publish(const NAV_STATUS *s);

#endif /* SUBSYSTEM3_H */
