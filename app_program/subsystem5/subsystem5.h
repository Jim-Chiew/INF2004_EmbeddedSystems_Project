/*
 *	subsystem5.h
 *	Subsystem 5 -- Barcode: Code 39 barcodes read by MH-sensor #3's
 *	analog output on GP28 (ADC input 2).  Student ID: <student ID>.
 *
 *	This header is subsystem 5's PUBLIC interface.  Subsystem 5 owns the
 *	ADC and timer alarm 3 (TEAM.md section 3).  It sends every finished
 *	pass to subsystem 1 (IF-09, mbf_barcode in subsystem1.h).
 *
 *	State machine: the pass state in barcode.c, WAIT_BAR / IN_PASS (see
 *	barcode.h).  The 2 kHz sampler is an interrupt handler (timer alarm 3,
 *	on core 1); barcode_task polls the edges it collects every S5_POLL_MS.
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"
#include "barcode.h"

#define S5_PRI_BARCODE	16		/* priority band 16-17 (TEAM.md) */
#define S5_POLL_MS	5		/* edge queue drained this often */
#define S5_INT_PRI	2		/* sampler interrupt priority (like the
					   kernel's own drivers) */

IMPORT ID	tid_barcode;

/* ---------------------------------------------------------------------
 *  IF-14 (S5 -> S3): is the sensor crossing something black RIGHT NOW?
 *
 *  Subsystem 3 steers by pivoting (STEER_INNER = 0), which swings this
 *  sensor sideways at about twice the car's forward speed.  A pass has to BE
 *  the code -- 29, 39, 49 ... bars/gaps -- so one stray bar from that swing
 *  loses the read.  Navigation therefore softens its steering while a bar is
 *  actually going past, and only then, so curves are still taken with the
 *  full pivot (see STEER_OUTER_SOFT in task_nav.c).
 *
 *  "Going past" means an edge arrived less than S5_CROSSING_US ago.  The
 *  pass state is no good for this: a pass only ENDS BARCODE_END_GAP_MS
 *  (1 s) after the last edge, long after the code has gone.
 *
 *  Raise S5_CROSSING_US if the car is slow enough that one wide element
 *  takes longer than this (a 5 mm bar at 5 cm/s is 100 ms); the cost of too
 *  long is only that the car steers gently for that much longer after the
 *  code.
 * --------------------------------------------------------------------- */
#define S5_CROSSING_US	200000u		/* 200 ms since the last edge */

static inline BOOL barcode_crossing(void)
{
	/* On a bar right now, or a bar edge went past recently.  Both are needed:
	   one wide bar can take longer to cross than the window (a 5 mm bar at
	   5 cm/s is 100 ms), and between two bars there is no black to see.
	   barcode_edges guards the first moments after power-up, when no edge has
	   ever arrived and the timer is still near zero. */
	if (barcode_black != 0u) {
		return TRUE;
	}
	return (barcode_edges != 0u &&
		(app_now_us() - barcode_edge_us) < S5_CROSSING_US) ? TRUE : FALSE;
}

/* Passes read since power-up (any task may read it; one writer) */
IMPORT volatile UW s5_passes;
IMPORT volatile UW s5_samples;	/* sampler interrupts so far (for tests) */

/* ---------------------------------------------------------------------
 *  IF-15 (S5 -> S3): how each pass turned out.
 *
 *  Two counters rather than two flags, and that is the point: a flag needs
 *  clearing, and whoever clears it races the task that sets it.  Subsystem 5
 *  only ever counts UP; a reader remembers the value it last acted on and
 *  compares.  One 32-bit store each, one writer (barcode_task), so any task
 *  on either core may read them without a lock -- the same arrangement as
 *  s5_passes and the IF-14 words above.
 *
 *    s5_reads   passes that DECODED (the text is in the IF-09 message to
 *               subsystem 1)
 *    s5_cmd     what the last decoded code COMMANDS: a barcode_cmd_t, so
 *               BARCODE_CMD_LEFT / RIGHT / FORWARD / NONE (barcode.h).
 *               Written BEFORE s5_reads is stepped, so a reader that notices
 *               the new count is certain to see the command that goes with
 *               it.  Navigation picks it up and holds it until the next
 *               junction, which is where it is obeyed.
 *    s5_fails   passes that got at least BARCODE_FAIL_MIN_PCT of the way
 *               through a code and then would not decode.  This is the
 *               "barcode decoding failed" signal: it counts a code the car
 *               really did drive over, never a black line, a bump or the
 *               clipped edge of a code, which barcode.h filters out first.
 *
 *  Navigation watches s5_fails while FOLLOWING and goes to NAV_BARCODE_RESCAN
 *  to drive back over the code and try again (barcode_rescan in task_nav.c).
 *  It re-syncs its remembered value afterwards, because the rescan's own
 *  passes land in these counters too.
 * --------------------------------------------------------------------- */
IMPORT volatile UW s5_reads;
IMPORT volatile UW s5_fails;
IMPORT volatile UB s5_cmd;	/* barcode_cmd_t of the last decoded code */

/* Private to subsystem 5 */
IMPORT void	barcode_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM5_H */
