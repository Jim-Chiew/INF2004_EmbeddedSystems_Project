/*
 *	subsystem4_test.c
 *	On-target tests for subsystem 4.  Built only with: make TEST=1
 *	Record results in testing/subsystem4/TEST_LOG.md.  IDs: S4-U01, ...
 *	The car must be upright and still on the bench (S4-U04).
 */

#include "subsystem4.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem4_test(void)
{
	T_RMBF		rmbf;
	T_RFLG		rflg;
	T_RMTX		rmtx;
	IMU_STATUS	a, b;
	INT		i;

	TEST_INIT();

	/* S4-U01: its kernel objects exist */
	TEST_CHECK("S4-U01", tk_ref_mbf(mbf_imu_req, &rmbf) == E_OK &&
			     tk_ref_flg(flg_imu, &rflg) == E_OK &&
			     tk_ref_mtx(mtx_imu, &rmtx) == E_OK,
		   "mbf_imu_req, flg_imu and mtx_imu exist");

	/* S4-U02: the LSM303D answers on I2C1 (GP2/GP3) within 3 s */
	for (i = 0; i < 30; i++) {
		imu_get_status(&a);
		if (a.ok) break;
		tk_dly_tsk(100);
	}
	TEST_CHECK("S4-U02", a.ok && a.who == 0x49, "LSM303D found (WHO_AM_I 0x49)");

	/* S4-U03: readings are valid and alive (a live sensor jitters) */
	imu_get_status(&a);
	tk_dly_tsk(60);
	imu_get_status(&b);
	TEST_CHECK("S4-U03", b.valid && (a.d.acc.x != b.d.acc.x || a.d.acc.y != b.d.acc.y ||
					 a.d.acc.z != b.d.acc.z),
		   "readings valid and changing");

	/* S4-U04: upright on the bench: not overturned, about 1 g */
	TEST_CHECK("S4-U04", !b.d.overturned &&
			     b.d.acc.z * b.d.acc.z + b.d.acc.x * b.d.acc.x + b.d.acc.y * b.d.acc.y > 0.7f,
		   "upright: no overturn, gravity seen");

	/* S4-U05: RESET_LIFT clears the lift latch (IF-04) */
	imu_request(IMU_REQ_RESET_LIFT);
	tk_dly_tsk(30);
	imu_get_status(&a);
	TEST_CHECK("S4-U05", !a.d.lifted, "RESET_LIFT clears lifted");

	TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
