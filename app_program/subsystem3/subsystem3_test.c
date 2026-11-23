/*
 *	subsystem3_test.c
 *	On-target tests for subsystem 3.  Built only with: make TEST=1
 *	Record results in testing/subsystem3/TEST_LOG.md.  IDs: S3-U01, ...
 *	Do not press GP20/GP21 while the tests run.
 */

#include "subsystem3.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem3_test(void)
{
	T_RMBF		rmbf;
	T_RMTX		rmtx;
	T_RTSK		rtsk;
	NAV_STATUS	a, b;
	INT		i, waiting = 0;

	TEST_INIT();

	/* S3-U01: its kernel objects exist */
	TEST_CHECK("S3-U01", tk_ref_mbf(mbf_nav_cmd, &rmbf) == E_OK &&
			     tk_ref_mtx(mtx_nav, &rmtx) == E_OK,
		   "mbf_nav_cmd and mtx_nav exist");

	/* S3-U02: the car waits in IDLE after power-up */
	nav_get_status(&a);
	TEST_CHECK("S3-U02", a.state == NAV_IDLE && a.action == ACT_STOP, "IDLE, motors stopped");

	/* S3-U03: nav_task sleeps between polls while IDLE */
	for (i = 0; i < 10; i++) {
		tk_dly_tsk(7);
		if (tk_ref_tsk(tid_nav, &rtsk) == E_OK && (rtsk.tskstat & TTS_WAI)) {
			waiting++;
		}
	}
	TEST_CHECK("S3-U03", waiting >= 8, "nav_task sleeps while idle");

	/* S3-U04: a RESET command (IF-07) clears the run: run_count goes up */
	nav_get_status(&a);
	nav_command(NAV_CMD_RESET);
	tk_dly_tsk(50);
	nav_get_status(&b);
	TEST_CHECK("S3-U04", b.run_count == a.run_count + 1 && b.junctions == 0 &&
			     b.state == NAV_IDLE,
		   "RESET clears the run and stays IDLE");

	TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */
