/*
 *	subsystem2.c
 *	Subsystem 2 -- Motion: creates its objects, starts its task, and the
 *	small public helpers other subsystems call (they only touch kernel
 *	objects, never the hardware).
 *
 *	APIs shown: tk_cre_mbf, tk_snd_mbf, tk_cre_mtx, tk_loc_mtx, tk_unl_mtx
 */

#include "subsystem2.h"

EXPORT ID	mbf_motion;	/* IF-01 / IF-02, see subsystem2.h */
EXPORT ID	mtx_odom;	/* IF-03 */
EXPORT ID	tid_motion;

LOCAL ODOMETRY	odom;		/* the shared copy, only touched under mtx_odom */

EXPORT ER subsystem2_init(void)
{
	T_CMBF	cmbf = {0};
	T_CMTX	cmtx = {0};

	/* Motion commands.  Room for 16; motion_task (high priority) normally
	   takes each one the moment it is sent. */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = 16 * (sizeof(MOTION_CMD) + 8);
	cmbf.maxmsz = sizeof(MOTION_CMD);
	mbf_motion = tk_cre_mbf(&cmbf);
	if (mbf_motion < E_OK) {
		return mbf_motion;
	}

	/* Odometry copy.  TA_INHERIT: a low-priority reader holding it is
	   raised to the waiting motion_task's priority (no priority inversion). */
	cmtx.mtxatr = TA_INHERIT;
	mtx_odom = tk_cre_mtx(&cmtx);
	if (mtx_odom < E_OK) {
		return mtx_odom;
	}

	odom.state = MOTION_ST_STARTING;
	return E_OK;
}

EXPORT ER subsystem2_start(void)
{
	tid_motion = app_start_task(motion_task, S2_PRI_MOTION,
				    APP_CORE_CONTROL, "motion_task");
	return (tid_motion < E_OK) ? tid_motion : E_OK;
}

EXPORT ER motion_command(UB type, INT left, INT right, UB from)
{
	MOTION_CMD c;

	c.type  = type;
	c.left  = (B)((left  > 100) ? 100 : (left  < -100) ? -100 : left);
	c.right = (B)((right > 100) ? 100 : (right < -100) ? -100 : right);
	c.from  = from;

	/* motion_task drains the buffer at once; 5 ms only matters if it
	   is somehow stuck, and then the caller must not hang forever. */
	return tk_snd_mbf(mbf_motion, &c, sizeof c, 5);
}

EXPORT void motion_get_odometry(ODOMETRY *out)
{
	if (tk_loc_mtx(mtx_odom, TMO_FEVR) == E_OK) {
		*out = odom;
		tk_unl_mtx(mtx_odom);
	}
}

/* Called by motion_task only */
EXPORT void motion_publish(const ODOMETRY *o)
{
	if (tk_loc_mtx(mtx_odom, TMO_FEVR) == E_OK) {
		odom = *o;
		tk_unl_mtx(mtx_odom);
	}
}

EXPORT const char *motion_state_name(UB state)
{
	switch (state) {
	case MOTION_ST_STARTING:	return "STARTING";
	case MOTION_ST_STOPPED:		return "STOPPED";
	case MOTION_ST_DRIVE:		return "DRIVE";
	case MOTION_ST_WHEELS:		return "WHEELS";
	case MOTION_ST_ESTOP:		return "ESTOP";
	default:			return "?";
	}
}
