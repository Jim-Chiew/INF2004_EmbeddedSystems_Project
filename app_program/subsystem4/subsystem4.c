/*
 *	subsystem4.c
 *	Subsystem 4 -- IMU: creates its objects, starts its task, and the
 *	small public helpers other subsystems call (kernel objects only).
 *
 *	APIs shown: tk_cre_mbf, tk_cre_flg, tk_cre_mtx
 */

#include "subsystem4.h"

EXPORT ID	mbf_imu_req;	/* IF-04, see subsystem4.h */
EXPORT ID	flg_imu;	/* IF-05 */
EXPORT ID	mtx_imu;	/* IF-06 */
EXPORT ID	tid_imu;

/* Shared copy and the telemetry pulse, only touched under mtx_imu */
LOCAL IMU_STATUS	shared;
LOCAL BOOL		hump_pending;
LOCAL float		jolt_pending_g;

EXPORT ER subsystem4_init(void)
{
	T_CMBF	cmbf = {0};
	T_CFLG	cflg = {0};
	T_CMTX	cmtx = {0};

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = 8 * (sizeof(UB) + 8);
	cmbf.maxmsz = sizeof(UB);
	mbf_imu_req = tk_cre_mbf(&cmbf);
	if (mbf_imu_req < E_OK) {
		return mbf_imu_req;
	}

	/* TA_WMUL: navigation and the tests may both wait on it */
	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0;
	flg_imu = tk_cre_flg(&cflg);
	if (flg_imu < E_OK) {
		return flg_imu;
	}

	cmtx.mtxatr = TA_INHERIT;
	mtx_imu = tk_cre_mtx(&cmtx);
	if (mtx_imu < E_OK) {
		return mtx_imu;
	}

	shared.state = IMU_ST_SEARCH;
	return E_OK;
}

EXPORT ER subsystem4_start(void)
{
	tid_imu = app_start_task(imu_task, S4_PRI_IMU, APP_CORE_CONTROL, "imu_task");
	return (tid_imu < E_OK) ? tid_imu : E_OK;
}

EXPORT ER imu_request(UB req)
{
	return tk_snd_mbf(mbf_imu_req, &req, sizeof req, 5);
}

EXPORT void imu_get_status(IMU_STATUS *out)
{
	if (tk_loc_mtx(mtx_imu, TMO_FEVR) == E_OK) {
		*out = shared;
		tk_unl_mtx(mtx_imu);
	}
}

EXPORT BOOL imu_take_hump(float *jolt_g)
{
	BOOL hump = FALSE;

	if (tk_loc_mtx(mtx_imu, TMO_FEVR) == E_OK) {
		hump = hump_pending;
		*jolt_g = jolt_pending_g;
		hump_pending = FALSE;
		jolt_pending_g = 0.0f;
		tk_unl_mtx(mtx_imu);
	}
	return hump;
}

/* Called by imu_task only: new copy, plus any hump that just ended */
EXPORT void imu_publish(const IMU_STATUS *s, BOOL hump, float jolt_g)
{
	if (tk_loc_mtx(mtx_imu, TMO_FEVR) == E_OK) {
		shared = *s;
		if (hump) {
			hump_pending = TRUE;
			if (jolt_g > jolt_pending_g) {
				jolt_pending_g = jolt_g;
			}
		}
		tk_unl_mtx(mtx_imu);
	}
}

EXPORT const char *imu_state_name(UB state)
{
	switch (state) {
	case IMU_ST_SEARCH:	return "SEARCH";
	case IMU_ST_RUN:	return "RUN";
	case IMU_ST_CAL_SPIN:	return "CAL-SPIN";
	case IMU_ST_CAL_SETTLE:	return "CAL-SETTLE";
	case IMU_ST_CAL_ZERO:	return "CAL-ZERO";
	default:		return "?";
	}
}
