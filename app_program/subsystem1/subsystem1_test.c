/*
 *	subsystem1_test.c
 *	On-target tests for subsystem 1.  Built only with: make TEST=1
 *	Record results in testing/subsystem1/TEST_LOG.md.
 *	IDs continue after the PC tests (S1-U01..S1-U61): S1-U62, ...
 */

#include "subsystem1.h"
#include "comms.h"
#include "beeper.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem1_test(void)
{
	T_RMBF		rmbf;
	T_RTSK		rtsk;
	comms_stats_t	a, b;
	INT		i, waiting = 0;

	TEST_INIT();

	/* S1-U62: the barcode message buffer (IF-09) exists */
	TEST_CHECK("S1-U62", tk_ref_mbf(mbf_barcode, &rmbf) == E_OK,
		   "mbf_barcode exists");

	/* S1-U63: telemetry frames are produced, 5 a second.  With no broker
	   they wait in the queue; with one they are acknowledged. */
	comms_get_stats(&a);
	tk_dly_tsk(1100);
	comms_get_stats(&b);
#if S1_PUBLISH_TELEMETRY
	TEST_CHECK("S1-U63",
		   (b.queued + b.sent_acked + b.discarded) >= (a.queued + a.sent_acked + a.discarded) + 4,
		   "telemetry frames produced (>= 4 in 1.1 s)");
#else
	/* Switched off in subsystem1.h: the broker gets barcode letters only,
	   so not one frame may be produced. */
	TEST_CHECK("S1-U63",
		   (b.queued + b.sent_acked + b.discarded) == (a.queued + a.sent_acked + a.discarded),
		   "no telemetry frames (S1_PUBLISH_TELEMETRY 0)");
#endif

	/* S1-U64: telemetry_task sleeps between periods (not busy) */
	for (i = 0; i < 10; i++) {
		tk_dly_tsk(7);
		if (tk_ref_tsk(tid_telemetry, &rtsk) == E_OK && (rtsk.tskstat & TTS_WAI)) {
			waiting++;
		}
	}
	TEST_CHECK("S1-U64", waiting >= 8, "telemetry_task sleeps between periods");

	/* S1-U65: the buzzer pattern runs and ends by itself.  telemetry_task is
	   what advances it (beeper_step), so this also proves the two are wired
	   together -- and you can hear the two beeps while it runs. */
	{
		UW	total = 2u * BEEP_SHORT_MS + BEEP_GAP_MS;
		BOOL	busy_during, idle_after;

		beeper_play(2, BEEP_SHORT_MS, app_now_ms());
		tk_dly_tsk((INT)(total / 2u));
		busy_during = beeper_busy();
		tk_dly_tsk((INT)(total / 2u) + 4 * S1_PERIOD_MS);
		idle_after = !beeper_busy();
		TEST_CHECK("S1-U65", busy_during && idle_after,
			   "two beeps sound and the pattern ends by itself");
	}

	TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
