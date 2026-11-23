/*
 *	subsystem6.c
 *	Subsystem 6 -- Obstacle: creates its kernel objects, starts its task,
 *	and the small public helpers the other subsystems call.
 *
 *	The sensor and the servo are NOT set up here: obstacle_task does that
 *	itself, because the ECHO interrupt is enabled on the core that calls
 *	us_init() and the task runs on core 1 (see task_obstacle.c).
 */

#include "subsystem6.h"

EXPORT ID	flg_obs;	/* IF-11, see subsystem6.h */
EXPORT ID	mbf_obs_req;	/* IF-12 */
EXPORT ID	mtx_obs;	/* IF-13 */
EXPORT ID	tid_obstacle;

LOCAL OBS_STATUS	shared;	/* only touched under mtx_obs */

EXPORT ER subsystem6_init(void)
{
	T_CFLG	cflg = {0};
	T_CMBF	cmbf = {0};
	T_CMTX	cmtx = {0};

	/* TA_WMUL: navigation may wait for more than one of these bits */
	cflg.flgatr = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0;
	flg_obs = tk_cre_flg(&cflg);
	if (flg_obs < E_OK) {
		return flg_obs;
	}

	/* Room for a few requests; obstacle_task takes them within S6_PING_MS */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = 8 * (sizeof(UB) + 8);
	cmbf.maxmsz = sizeof(UB);
	mbf_obs_req = tk_cre_mbf(&cmbf);
	if (mbf_obs_req < E_OK) {
		return mbf_obs_req;
	}

	cmtx.mtxatr = TA_INHERIT;
	mtx_obs = tk_cre_mtx(&cmtx);
	if (mtx_obs < E_OK) {
		return mtx_obs;
	}

	shared.state = OBS_ST_MONITOR;
	shared.dist_mm = US_INVALID;	/* nothing measured yet */
	return E_OK;
}

EXPORT ER subsystem6_start(void)
{
	/* Core 1, with the other tasks that steer the car: the ECHO interrupt
	   is enabled from this task, and an interrupt is enabled on the core
	   that calls EnableInt(). */
	tid_obstacle = app_start_task(obstacle_task, S6_PRI_OBSTACLE,
				      APP_CORE_CONTROL, "obstacle_task");
	return (tid_obstacle < E_OK) ? tid_obstacle : E_OK;
}

EXPORT ER obstacle_request(UB req)
{
	return tk_snd_mbf(mbf_obs_req, &req, sizeof req, TMO_POL);
}

EXPORT void obstacle_get_status(OBS_STATUS *out)
{
	if (tk_loc_mtx(mtx_obs, TMO_FEVR) == E_OK) {
		*out = shared;
		tk_unl_mtx(mtx_obs);
	}
}

EXPORT void obstacle_publish(const OBS_STATUS *s)
{
	if (tk_loc_mtx(mtx_obs, TMO_FEVR) == E_OK) {
		shared = *s;
		tk_unl_mtx(mtx_obs);
	}
}

EXPORT const char *obs_state_name(UB state)
{
	switch (state) {
	case OBS_ST_BLOCKED:	return "BLOCKED";
	case OBS_ST_WATCHING:	return "WATCHING";
	case OBS_ST_FAULT:	return "FAULT";
	case OBS_ST_FAIL:	return "FAIL";
	default:		return "MONITOR";
	}
}
