/*
 *	subsystem4.h
 *	Subsystem 4 -- IMU (LSM303D / GY-511 on I2C1): tilt, heading, humps,
 *	and the lift / overturn safety stop.  Student ID: <student ID>.
 *
 *	This header is subsystem 4's PUBLIC interface.  imu_task is the ONLY
 *	task that touches I2C1 (TEAM.md section 3).  Everyone else:
 *	  - asks for things through mbf_imu_req              (IF-04)
 *	  - waits for / polls events on flg_imu               (IF-05)
 *	  - reads the IMU_STATUS copy with imu_get_status()   (IF-06)
 *
 *	State machine (imu_task):
 *	  SEARCH --sensor answers--> RUN --CAL_START--> CAL_SPIN --8 s-->
 *	  CAL_SETTLE --1 s--> CAL_ZERO --2 s--> RUN
 *	  CAL_ABORT (or the sensor lost) leaves any CAL_ state and keeps the
 *	  old calibration.  RUN --sensor lost--> SEARCH.
 *
 *	Safety stop (the requirement): in RUN, if the car is tilted more than
 *	90 degrees either way, or held off the floor for 0.6 s (see imu.h),
 *	imu_task sends MOTION_CMD_ESTOP to subsystem 2 at once (IF-02) and sets
 *	IMU_FLG_OVERTURNED / IMU_FLG_LIFTED, so navigation ends the run.
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"
#include "imu.h"

#define S4_PRI_IMU	12		/* priority band 12-13 (TEAM.md) */
#define S4_SAMPLE_MS	10		/* IMU read period (hump detection needs it fast) */
#define S4_RETRY_MS	2000		/* IMU not found: look for it again this often */
#define S4_MAX_FAILS	20		/* failed reads in a row (200 ms): restart it */

/* GP21 calibration (navigation spins the car during CAL_SPIN) */
#define CAL_SPIN_SPEED	30		/* % power per wheel while spinning */
#define CAL_SPIN_MS	8000		/* spin time (aim for at least 2 full circles) */
#define CAL_SETTLE_MS	1000		/* motors off, wait for the car to stop rocking */
#define CAL_ZERO_MS	2000		/* car still: average gravity for the tilt zero */

/* ---------------------------------------------------------------------
 *  IF-04 (S3 -> S4): requests.  Message buffer mbf_imu_req, owned by S4.
 *  One UB per message.
 * --------------------------------------------------------------------- */
typedef enum {
	IMU_REQ_CAL_START = 1,	/* start the calibration (only from RUN) */
	IMU_REQ_CAL_ABORT,	/* stop it, keep the old calibration */
	IMU_REQ_RESET_LIFT,	/* forget a lift: the car was put down for a new run */
	IMU_REQ_RESET_RUN,	/* clear the hump count and highest hump */
	IMU_REQ_CLEAR_ERRORS	/* every GP20 / START: forget the lift, clear
				   CAL_FAILED and the restart count, look for a
				   missing sensor now, reconfigure a failing one.
				   A fault that is still true (upside down NOW) is
				   reported again at the next sample. */
} IMU_REQ;

IMPORT ID	mbf_imu_req;
IMPORT ER	imu_request(UB req);

/* ---------------------------------------------------------------------
 *  IF-05 (S4 -> S3): events.  Event flag flg_imu, owned by S4.  S4 only
 *  SETS bits; the receiver clears the ones it has dealt with.
 * --------------------------------------------------------------------- */
#define IMU_FLG_OVERTURNED	0x01	/* tilted past 90 deg (set again while it lasts) */
#define IMU_FLG_LIFTED		0x02	/* held off the floor 0.6 s (set again while latched) */
#define IMU_FLG_CAL_SPIN_DONE	0x04	/* compass part done: stop spinning now */
#define IMU_FLG_CAL_DONE	0x08	/* calibration finished and stored */
#define IMU_FLG_CAL_FAILED	0x10	/* calibration aborted or impossible */
#define IMU_FLG_SAFETY		(IMU_FLG_OVERTURNED | IMU_FLG_LIFTED)
#define IMU_FLG_CAL_ANY		(IMU_FLG_CAL_SPIN_DONE | IMU_FLG_CAL_DONE | IMU_FLG_CAL_FAILED)

IMPORT ID	flg_imu;

/* ---------------------------------------------------------------------
 *  IF-06 (S4 -> anyone): the latest reading.  Shared data under mutex
 *  mtx_imu, owned by S4; copy it with imu_get_status().
 * --------------------------------------------------------------------- */
typedef enum {
	IMU_ST_SEARCH,		/* sensor not found: retried every S4_RETRY_MS */
	IMU_ST_RUN,
	IMU_ST_CAL_SPIN,	/* compass: car spinning */
	IMU_ST_CAL_SETTLE,	/* motors off, waiting */
	IMU_ST_CAL_ZERO		/* car still: averaging gravity */
} IMU_STATE;

typedef struct {
	UB		state;		/* IMU_STATE */
	BOOL		ok;		/* sensor found */
	BOOL		valid;		/* last read worked */
	UB		who;		/* WHO_AM_I at the last start (0x49 = LSM303D) */
	BOOL		calibrated;	/* GP21 calibration completed */
	BOOL		cal_failed;	/* the last calibration failed (until the next
					   one succeeds, or CLEAR_ERRORS) */
	UW		restarts;	/* times it stopped and was restarted */
	W		cal_left_ms;	/* time left in the current CAL_ state */
	float		pitch_off;	/* tilt zero offsets from the calibration */
	float		roll_off;
	imu_data_t	d;		/* the reading itself (see imu.h) */
} IMU_STATUS;

IMPORT ID	mtx_imu;
IMPORT void	imu_get_status(IMU_STATUS *out);

/* Telemetry pulse: TRUE if a hump ended since the last call, and the
   biggest hump peak (g) since then.  Clears both.  One caller (S1). */
IMPORT BOOL	imu_take_hump(float *jolt_g);

IMPORT const char *imu_state_name(UB state);

IMPORT ID	tid_imu;

/* Private to subsystem 4, shared between its own .c files */
IMPORT void	imu_task(INT stacd, void *exinf);
IMPORT void	imu_publish(const IMU_STATUS *s, BOOL hump, float jolt_g);

#endif /* SUBSYSTEM4_H */
