/*
 *	subsystem5.c
 *	Subsystem 5 -- Adaptive ultrasonic scanning and obstacle profiling.
 *
 *	One task, s5_obstacle (statechart "Obstacle_Detect_Task"):
 *	  FRONT_MONITOR   ping every 20 ms, rolling median-of-3; two filtered
 *	                  readings <= 15 cm in a row set S5_FLG_FRONT_OBSTACLE.
 *	  AVOID_STANDBY   wait for Nav: PROFILE_REQ -> sweep servo left/right,
 *	                  publish s5_profile, set PROFILE_DONE;
 *	                  AVOID_DONE -> clear the obstacle flag, back to monitoring.
 *	Not yet implemented: CLEARANCE_CHECK, SENSOR_FAULT, coarse/fine scan.
 */

#include "subsystem5.h"
#include "drv_ultrasonic.h"

IMPORT ER s5_demo_start(void);		/* s5_demo.c */

EXPORT ID		flg_obstacle;
EXPORT us_profile_t	s5_profile;
EXPORT volatile INT	s5_last_mm;

LOCAL void s5_obstacle_task(INT stacd, void *exinf)
{
	UINT	ptn;
	INT	hits = 0, idx = 0;
	int32_t	ring[3] = { US_NO_ECHO, US_NO_ECHO, US_NO_ECHO };

	(void)stacd; (void)exinf;

	/* INIT: wait for Nav's calibration */
	tk_wai_flg(flg_obstacle, S5_FLG_CALIBRATED, TWF_ORW, &ptn, TMO_FEVR);

	for (;;) {
		/* FRONT_MONITOR */
		ring[idx] = us_ping_mm();
		idx = (idx + 1) % 3;
		int32_t d = us_filter3(ring);
		s5_last_mm = d;

		hits = (d > 0 && d <= OBSTACLE_THRESHOLD_MM) ? hits + 1 : 0;
		if (hits >= OBSTACLE_CONFIRM_CNT) {
			tk_set_flg(flg_obstacle, S5_FLG_FRONT_OBSTACLE);

			/* AVOID_STANDBY */
			for (;;) {
				tk_wai_flg(flg_obstacle,
					   S5_FLG_PROFILE_REQ | S5_FLG_AVOID_DONE,
					   TWF_ORW | TWF_BITCLR, &ptn, TMO_FEVR);
				if (ptn & S5_FLG_PROFILE_REQ) {
					us_scan_left_right(&s5_profile);   /* ~1 s, car stopped */
					tk_set_flg(flg_obstacle, S5_FLG_PROFILE_DONE);
				}
				if (ptn & S5_FLG_AVOID_DONE) break;
			}
			tk_clr_flg(flg_obstacle, ~(UINT)S5_FLG_FRONT_OBSTACLE);
			hits = 0;
		}
		tk_dly_tsk(US_PING_PERIOD_MS);
	}
}

EXPORT ER subsystem5_init(void)
{
	T_CFLG cflg = { .flgatr = TA_TFIFO | TA_WMUL, .iflgptn = 0 };

	flg_obstacle = tk_cre_flg(&cflg);
	if (flg_obstacle < E_OK) return flg_obstacle;

	/* Sensor + servo + ECHO interrupt are set up here, in usermain, so the
	   GPIO interrupt is owned by core 0 (PORT_RP2040.md rule 2). */
	return (ER)us_init();
}

EXPORT ER subsystem5_start(void)
{
	ID tid = app_start_task(s5_obstacle_task, S5_PRI_MAIN, TP_PRC1, "s5_obstacle");
	if (tid < E_OK) return tid;
	return s5_demo_start();
}
