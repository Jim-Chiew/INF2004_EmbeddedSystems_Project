/*
 *	subsystem2_test.c
 *	On-target tests for subsystem 2.  Built only with: make TEST=1
 *	Record results in testing/subsystem2/TEST_LOG.md.  IDs: S2-U01, ...
 *
 *	Put the car on a stand with the wheels off the floor: S2-U04 sends a
 *	drive command (the ESTOP latch must keep the wheels still).
 */

#include "subsystem2.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem2_test(void)
{
	T_RMBF		rmbf;
	T_RMTX		rmtx;
	ODOMETRY	o1, o2;
	INT		i;

	TEST_INIT();

	/* S2-U01: its kernel objects exist */
	TEST_CHECK("S2-U01", tk_ref_mbf(mbf_motion, &rmbf) == E_OK &&
			     tk_ref_mtx(mtx_odom, &rmtx) == E_OK,
		   "mbf_motion and mtx_odom exist");

	/* S2-U02: the encoders start (once the radio has its PIO), within 10 s */
	for (i = 0; i < 100; i++) {
		motion_get_odometry(&o1);
		if (o1.encoders_ok) break;
		tk_dly_tsk(100);
	}
	TEST_CHECK("S2-U02", o1.encoders_ok && o1.state != MOTION_ST_STARTING,
		   "encoders running (PIO claimed)");

	/* S2-U03: the odometry copy is refreshed every S2_PERIOD_MS */
	motion_get_odometry(&o1);
	tk_dly_tsk(20);
	motion_get_odometry(&o2);
	TEST_CHECK("S2-U03", o2.updated_ms != o1.updated_ms && o2.updated_ms - o1.updated_ms <= 20,
		   "odometry refreshed within 20 ms");

	/* S2-U04: ESTOP latches: DRIVE is ignored, the motors stay braked */
	motion_command(MOTION_CMD_ESTOP, 0, 0, 0);
	motion_command(MOTION_CMD_DRIVE, 10, 10, 0);
	tk_dly_tsk(20);
	motion_get_odometry(&o1);
	TEST_CHECK("S2-U04", o1.state == MOTION_ST_ESTOP && o1.left_pm == 0 && o1.right_pm == 0,
		   "ESTOP holds against DRIVE");

	/* S2-U05: ENABLE leaves ESTOP, back to STOPPED */
	motion_command(MOTION_CMD_STOP, 0, 0, 0);
	motion_command(MOTION_CMD_ENABLE, 0, 0, 0);
	tk_dly_tsk(20);
	motion_get_odometry(&o2);
	TEST_CHECK("S2-U05", o2.state == MOTION_ST_STOPPED, "ENABLE returns to STOPPED");

	TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */
