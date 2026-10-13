/*
 *	s5_demo.c
 *	Bench-test stand-in for Nav (US_DEMO=1 in us_config.h): sets CALIBRATED,
 *	prints the distance twice a second and, on an obstacle, requests a
 *	left/right profile and prints it.  Remove when the real Nav task owns
 *	the S5_FLG_* bits.
 */

#include "subsystem5.h"

#if US_DEMO

LOCAL void s5_demo_task(INT stacd, void *exinf)
{
	UINT ptn;
	(void)stacd; (void)exinf;

	tk_dly_tsk(500);
	tk_set_flg(flg_obstacle, S5_FLG_CALIBRATED);
	tm_printf((UB*)"[s5] started: threshold=%d mm\n", OBSTACLE_THRESHOLD_MM);

	for (;;) {
		ER er = tk_wai_flg(flg_obstacle, S5_FLG_FRONT_OBSTACLE, TWF_ORW, &ptn, 500);
		if (er == E_TMOUT) {
			INT d = s5_last_mm;
			if (d > 0)	tm_printf((UB*)"[s5] dist=%d mm\n", d);
			else		tm_printf((UB*)"[s5] dist=%s\n",
						  d == US_NO_ECHO ? "clear" : "invalid");
			continue;
		}
		tm_printf((UB*)"[s5] OBSTACLE at %d mm -> profiling\n", s5_last_mm);
		tk_set_flg(flg_obstacle, S5_FLG_PROFILE_REQ);
		tk_wai_flg(flg_obstacle, S5_FLG_PROFILE_DONE, TWF_ORW | TWF_CLR, &ptn, 5000);
		tm_printf((UB*)"[s5] clear_L=%d mm  clear_R=%d mm  -> avoid %s\n",
			  s5_profile.clear_L_mm, s5_profile.clear_R_mm,
			  s5_profile.side == AVOID_LEFT ? "LEFT" : "RIGHT");
		tk_dly_tsk(1000);
		tk_set_flg(flg_obstacle, S5_FLG_AVOID_DONE);	/* release the latch */
		tk_dly_tsk(500);
	}
}

EXPORT ER s5_demo_start(void)
{
	ID tid = app_start_task(s5_demo_task, S5_PRI_DEMO, TP_PRC1, "s5_demo");
	return (tid < E_OK) ? tid : E_OK;
}

#else
EXPORT ER s5_demo_start(void) { return E_OK; }
#endif
