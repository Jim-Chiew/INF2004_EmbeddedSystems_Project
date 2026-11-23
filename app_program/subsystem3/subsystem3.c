/*
 *	subsystem3.c
 *	Subsystem 3 -- Navigation: creates its objects, starts its task, and
 *	the small public helpers other subsystems call (kernel objects only).
 */

#include "subsystem3.h"

EXPORT ID	mbf_nav_cmd;	/* IF-07, see subsystem3.h */
EXPORT ID	mtx_nav;	/* IF-08 */
EXPORT ID	tid_nav;

LOCAL NAV_STATUS	shared;	/* only touched under mtx_nav */

EXPORT ER subsystem3_init(void)
{
	T_CMBF	cmbf = {0};
	T_CMTX	cmtx = {0};

	/* Room for a few commands; nav_task takes them within 10 ms */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = 8 * (sizeof(UB) + 8);
	cmbf.maxmsz = sizeof(UB);
	mbf_nav_cmd = tk_cre_mbf(&cmbf);
	if (mbf_nav_cmd < E_OK) {
		return mbf_nav_cmd;
	}

	cmtx.mtxatr = TA_INHERIT;
	mtx_nav = tk_cre_mtx(&cmtx);
	if (mtx_nav < E_OK) {
		return mtx_nav;
	}
	shared.state = NAV_IDLE;
	return E_OK;
}

EXPORT ER subsystem3_start(void)
{
	tid_nav = app_start_task(nav_task, S3_PRI_NAV, APP_CORE_CONTROL, "nav_task");
	return (tid_nav < E_OK) ? tid_nav : E_OK;
}

EXPORT ER nav_command(UB cmd)
{
	return tk_snd_mbf(mbf_nav_cmd, &cmd, sizeof cmd, TMO_POL);
}

EXPORT void nav_get_status(NAV_STATUS *out)
{
	if (tk_loc_mtx(mtx_nav, TMO_FEVR) == E_OK) {
		*out = shared;
		tk_unl_mtx(mtx_nav);
	}
}

EXPORT void nav_publish(const NAV_STATUS *s)
{
	if (tk_loc_mtx(mtx_nav, TMO_FEVR) == E_OK) {
		shared = *s;
		tk_unl_mtx(mtx_nav);
	}
}

EXPORT const char *nav_state_name(UB state)
{
	switch (state) {
	case NAV_IDLE:		return "IDLE";
	case NAV_CALIBRATING:	return "CALIBRATING";
	case NAV_FOLLOWING:	return "FOLLOW";
	case NAV_AVOIDING:	return "AVOIDING";
	case NAV_BARCODE_RESCAN: return "BC-RESCAN";
	case NAV_TURNING:	return "TURNING";
	default:		return "?";
	}
}

EXPORT const char *nav_action_name(UB action)
{
	switch (action) {
	case ACT_STRAIGHT:	return "straight";
	case ACT_STEER_LEFT:	return "steer-L";
	case ACT_SPIN_LEFT:	return "SPIN-L";
	case ACT_STEER_RIGHT:	return "steer-R";
	case ACT_SPIN_RIGHT:	return "SPIN-R";
	default:		return "stop";
	}
}

EXPORT const char *nav_mode_name(UB mode)
{
	switch (mode) {
	case MODE_CONFIRM:	return "CONFIRM";
	case MODE_CROSSING:	return "CROSSING";
	default:		return "FOLLOW";
	}
}
