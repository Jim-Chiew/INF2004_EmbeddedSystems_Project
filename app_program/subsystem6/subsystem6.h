/*
 *	subsystem6.h
 *	Subsystem 6 -- Obstacle: the HC-SR04 distance sensor on GP0/GP1 and the
 *	SG90 servo on GP15 that aims it.  Student ID: <student ID>.
 *
 *	This header is subsystem 6's PUBLIC interface.  obstacle_task is the
 *	ONLY task that touches the sensor, the ECHO interrupt and the servo
 *	(TEAM.md section 3).  Everyone else:
 *	  - waits for / polls events on flg_obs                  (IF-11)
 *	  - asks for things through mbf_obs_req                  (IF-12)
 *	  - reads the OBS_STATUS copy with obstacle_get_status() (IF-13)
 *
 *	State machine (obstacle_task):
 *	  MONITOR --S6_CONFIRM_CNT readings at or under S6_STOP_MM--> BLOCKED
 *	  BLOCKED --CLEAR--> MONITOR        (SCAN while blocked: servo scan)
 *	  any     --WATCH_AHEAD--> WATCHING --CLEAR--> MONITOR
 *	  MONITOR --US_FAULT_INVALID_CNT invalid pings--> FAULT
 *	  FAULT   --a ping answers--> MONITOR     --recovery fails--> FAIL
 *	  FAIL    --CLEAR--> MONITOR
 *
 *	What it is for (the requirement): the car must stop before it runs into
 *	anything.  In BLOCKED the obstacle is LATCHED: the flag stays set, and
 *	is set again every pass, until navigation asks for OBS_REQ_CLEAR.  So a
 *	car stopped by an obstacle cannot quietly start again because one
 *	reading came back clear.  Navigation (subsystem 3) stops the motors and
 *	goes to its NAV_AVOIDING state; GP20 or START clears the latch and
 *	carries on.
 */

#ifndef SUBSYSTEM6_H
#define SUBSYSTEM6_H

#include "../app.h"
#include "ultrasonic.h"

/* The sensor is wired to GP0/GP1 (TEAM.md section 2), which UART0 takes when
   the console is on UART or the build asks for a crash report.  Those builds
   would drive TRIG from the UART and read the console's output as echoes, so
   stop here instead: build with a plain "make" (CONSOLE=usb_cdc,
   CRASH_REPORT=0), or move the sensor and US_TRIG_PIN / US_ECHO_PIN. */
#if !defined(BOARD_UART0_FREE) || !BOARD_UART0_FREE
#error "S6: the HC-SR04 is on GP0/GP1, which UART0 takes with CRASH_REPORT=1 or CONSOLE=uart.  Use a plain 'make', or move the sensor (ultrasonic.h)."
#endif

#define S6_PRI_OBSTACLE	20		/* priority band 20-21 (TEAM.md) */
#define S6_PING_MS	20		/* MONITOR pings this often (50 Hz) */
					/* the ECHO interrupt's priority is
					   US_ECHO_INT_PRI in ultrasonic.h */

/* ---- What counts as an obstacle ----
   The car stops when something is this close.  Two filtered readings in a
   row must agree (40 ms), so one stray echo cannot stop the run. */
#define S6_STOP_MM	150		/* 15 cm */
#define S6_CONFIRM_CNT	2

/* ---------------------------------------------------------------------
 *  IF-11 (S6 -> S3): events.  Event flag flg_obs, owned by S6.
 *
 *  S6 both SETS and CLEARS these bits: unlike subsystem 4's safety events,
 *  OBSTACLE is a latch that only OBS_REQ_CLEAR takes off, so the owner of
 *  the latch is the only one who may drop it.  Read them with tk_ref_flg
 *  (or wait on them), never with TWF_BITCLR.
 * --------------------------------------------------------------------- */
#define S6_FLG_OBSTACLE		0x01	/* something within S6_STOP_MM: stop the car.
					   Set again every pass while it lasts. */
#define S6_FLG_SCAN_DONE	0x02	/* the servo scan asked for is in OBS_STATUS */
#define S6_FLG_SENSOR_FAULT	0x04	/* the sensor stopped answering; recovering */
#define S6_FLG_SENSOR_FAIL	0x08	/* recovery gave up: the car is blind */

IMPORT ID	flg_obs;

/* ---------------------------------------------------------------------
 *  IF-12 (S3 -> S6): requests.  Message buffer mbf_obs_req, owned by S6.
 *  One UB per message.
 * --------------------------------------------------------------------- */
typedef enum {
	OBS_REQ_CLEAR = 1,	/* forget the latched obstacle and any sensor
				   fault, and monitor again (sent by every
				   GP20 / START / RESET, and when the button
				   carries on from NAV_AVOIDING).  An object
				   that is STILL there is reported again within
				   S6_CONFIRM_CNT pings. */
	OBS_REQ_WATCH_AHEAD,	/* keep the servo centred and keep pinging, but
				   latch NOTHING: navigation is driving round an
				   object and judges the live distance itself.
				   The latch is no use to it -- the object it is
				   avoiding is still in front when it starts,
				   so the latch would come straight back and
				   stop the manoeuvre before it began. */
	OBS_REQ_SCAN		/* sweep the servo now and publish the result:
				   S6_FLG_SCAN_DONE, and in OBS_STATUS the fan
				   (coarse), the object's length (object) and the
				   clearance either side (profile).  About 3 s,
				   so the car must be stopped. */
} OBS_REQ;

IMPORT ID	mbf_obs_req;
IMPORT ER	obstacle_request(UB req);	/* never waits */

/* ---------------------------------------------------------------------
 *  IF-13 (S6 -> anyone): the latest reading.  Shared data under mutex
 *  mtx_obs, owned by S6; copy it with obstacle_get_status().
 * --------------------------------------------------------------------- */
typedef enum {
	OBS_ST_MONITOR,		/* pinging ahead, path clear so far */
	OBS_ST_BLOCKED,		/* obstacle latched, waiting for OBS_REQ_CLEAR */
	OBS_ST_WATCHING,	/* servo centred, reporting the distance ahead
				   but latching nothing: navigation is driving
				   round something and judges every reading
				   itself (it turns the whole car to look) */
	OBS_ST_FAULT,		/* no answer from the sensor; trying to recover */
	OBS_ST_FAIL		/* recovery gave up: no distance is known */
} OBS_STATE;

typedef struct {
	UB		state;		/* OBS_STATE */
	BOOL		ok;		/* the sensor is answering */
	BOOL		blocked;	/* the obstacle latch is set */
	W		dist_mm;	/* filtered distance: mm, or US_NO_ECHO
					   (nothing in range) / US_INVALID */
	W		stop_mm;	/* the distance that set the latch */
	UW		pings;		/* pings since power-up */
	UW		scans;		/* servo scans finished since power-up.  It
				   changing is how another subsystem spots a
				   new measurement (S1 publishes it). */
	UW		faults;		/* times the sensor stopped answering */
	UW		updated_ms;	/* app_now_ms() of this copy */
	BOOL		scan_valid;	/* the three below hold a finished scan */
	us_coarse_t	coarse;		/* stage 1: the five-angle fan */
	us_object_t	object;		/* stage 2: how long the object is */
	us_profile_t	profile;	/* clearance left and right, and a side */
} OBS_STATUS;

IMPORT ID	mtx_obs;
IMPORT void	obstacle_get_status(OBS_STATUS *out);

IMPORT const char *obs_state_name(UB state);

IMPORT ID	tid_obstacle;

/* Private to subsystem 6, shared between its own .c files */
IMPORT void	obstacle_task(INT stacd, void *exinf);
IMPORT void	obstacle_publish(const OBS_STATUS *s);

#endif /* SUBSYSTEM6_H */
