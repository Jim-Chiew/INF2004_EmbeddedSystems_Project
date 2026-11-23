/*
 *	subsystem5_test.c
 *	On-target tests for subsystem 5.  Built only with: make TEST=1
 *	Record results in testing/subsystem5/TEST_LOG.md.  IDs: S5-U01, ...
 *	The barcode sensor should be over plain white floor.
 */

#include "subsystem5.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem5_test(void)
{
	T_RTSK		rtsk;
	UW		s0, s1, t0, t1, rate;
	uint16_t	raw, white, black;
	INT		i, waiting = 0;

	TEST_INIT();

	/* S5-U01: the sampler interrupt runs at 2 kHz (+-10 %) */
	s0 = s5_samples;
	t0 = app_now_ms();
	tk_dly_tsk(500);
	s1 = s5_samples;
	t1 = app_now_ms();
	rate = (s1 - s0) * 1000u / (t1 - t0);
	TEST_CHECK("S5-U01", rate >= 1800 && rate <= 2200, "sampler at 2 kHz");
	tm_printf((UB*)"[TEST] S5-U01 info: %d samples/s\n", (INT)rate);

	/* S5-U02: barcode_task sleeps between polls */
	for (i = 0; i < 10; i++) {
		tk_dly_tsk(3);
		if (tk_ref_tsk(tid_barcode, &rtsk) == E_OK && (rtsk.tskstat & TTS_WAI)) {
			waiting++;
		}
	}
	TEST_CHECK("S5-U02", waiting >= 8, "barcode_task sleeps between polls");

	/* S5-U03: ADC reading in range and a floor level learned */
	barcode_get_levels(&raw, &white, &black);
	TEST_CHECK("S5-U03", raw <= 4095 && white < black && white > 0,
		   "ADC in range, floor (white) level learned");
	tm_printf((UB*)"[TEST] S5-U03 info: raw %d white %d black %d\n",
		  (INT)raw, (INT)white, (INT)black);

	TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */
