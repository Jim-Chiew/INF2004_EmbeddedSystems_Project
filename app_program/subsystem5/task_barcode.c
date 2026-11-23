/*
 *	task_barcode.c
 *	Subsystem 5 -- barcode_task and the 2 kHz sampler interrupt.
 *
 *	APIs shown: tk_def_int, EnableInt (an interrupt handler), tk_snd_mbf
 *
 *	The sampler must run every 500 us, faster than any task can be paced
 *	(the kernel tick is 1 ms), so it is an interrupt handler on timer
 *	alarm 3.  Like a cyclic handler it runs in interrupt context: short,
 *	never waits.  It only reads the ADC and queues black/white edges;
 *	barcode_task does the decoding.
 */

#include "subsystem5.h"
#include "drv_bc_adc.h"
#include "../subsystem1/subsystem1.h"
#include <string.h>

EXPORT volatile UW	s5_passes = 0;
EXPORT volatile UW	s5_samples = 0;
EXPORT volatile UW	s5_reads = 0;	/* IF-15: passes that decoded */
EXPORT volatile UW	s5_fails = 0;	/* IF-15: passes that failed AS A CODE */
EXPORT volatile UB	s5_cmd = (UB)BARCODE_CMD_NONE;	/* IF-15: what it commands */

/* Interrupt handler: timer alarm 3, every BARCODE_SAMPLE_US */
LOCAL void bc_sample_handler(UINT intno)
{
	(void)intno;
	bc_alarm_next();		/* acknowledge and re-arm first */
	barcode_sample();
	s5_samples++;
}

LOCAL ER sampler_start(void)
{
	T_DINT	dint = {0};
	ER	er;

	dint.intatr = TA_HLNG;
	dint.inthdr = (FP)bc_sample_handler;
	er = tk_def_int(BC_ALARM_IRQ, &dint);
	if (er < E_OK) {
		return er;
	}
	bc_alarm_start(BARCODE_SAMPLE_US);
	EnableInt(BC_ALARM_IRQ, S5_INT_PRI);	/* on this task's core (core 1) */
	return E_OK;
}

/* 1 = when a pass does not decode, print the width of every bar and gap it
   collected, in microseconds.  That is enough to work out exactly why it was
   rejected (a merged bar, a clipped edge, or wide and narrow too close
   together), which the error message alone cannot say. */
#define S5_WIDTH_DUMP	1

/* The last pass is remembered so the trace line below can repeat it.  Without
   that, its report line has to be caught as it goes past: the USB console
   drops output when nothing is reading the port, and the interesting line is
   usually the one that was lost. */
LOCAL INT		last_elements = 0;
LOCAL BOOL		last_valid = FALSE;
LOCAL const char	*last_error = NULL;
LOCAL char		last_text[S1_BC_TEXT_MAX + 1] = "";
LOCAL UW		last_lo = 0, last_hi = 0, last_mid = 0;	/* element widths, us */

/* How many bars/gaps the last few passes had.  One number tells you more than
   any error message: 29 is a whole code, 1 is a single stripe (the line), 7
   is the sensor clipping the edge of a code. */
#define PASS_HIST	8
LOCAL UB		hist[PASS_HIST];
LOCAL UB		hist_n = 0;

/* Narrowest, widest and middling element of a pass, in microseconds.  A
   printed code gives a clear two-level split (say 4000 and 10000); if the
   widest is less than about 1.5x the narrowest the print is too marginal to
   read, and if one element is far wider than the rest, two bars have merged
   into one. */
LOCAL void width_summary(const barcode_result_t *r)
{
	INT i, n = r->num_elements;
	UW  lo = 0xFFFFFFFFu, hi = 0, sum = 0;

	if (n <= 0) {
		last_lo = last_hi = last_mid = 0;
		return;
	}
	for (i = 0; i < n; i++) {
		UW w = r->width_us[i];

		if (w < lo) lo = w;
		if (w > hi) hi = w;
		sum += w;
	}
	last_lo = lo;
	last_hi = hi;
	last_mid = sum / (UW)n;			/* mean: no sort needed */
}

LOCAL void dump_widths(const barcode_result_t *r)
{
#if S5_WIDTH_DUMP
	INT i;

	if (r->num_elements <= 0) {
		return;
	}
	/* Ten to a line, so one pass is three or four readable lines.  Bars are
	   the even positions (0, 2, 4, ...), gaps the odd ones. */
	for (i = 0; i < r->num_elements; i++) {
		if ((i % 10) == 0) {
			tm_printf((UB*)"\nbarcode: us[%2d]:", i);
		}
		tm_printf((UB*)" %6d", (INT)r->width_us[i]);
	}
	tm_printf((UB*)"\n");
#else
	(void)r;
#endif
}

/* Hand one finished pass to subsystem 1 (IF-09) */
LOCAL void report(const barcode_result_t *r)
{
	BARCODE_MSG m;

	/* A failed pass counts as a barcode read error only if it got through
	   at least BARCODE_FAIL_MIN_PCT % of a full barcode (barcode.h).  A
	   single black line or a bump under the sensor also makes a short
	   "pass": that is not a barcode, so it is ignored (no cyan error
	   light, no bc_err in telemetry). */
	/* Every pass goes in the history, however short. */
	if (hist_n < PASS_HIST) {
		hist[hist_n++] = (UB)((r->num_elements > 255) ? 255 : r->num_elements);
	} else {
		INT k;
		for (k = 1; k < PASS_HIST; k++) {
			hist[k - 1] = hist[k];
		}
		hist[PASS_HIST - 1] = (UB)((r->num_elements > 255) ? 255 : r->num_elements);
	}

	/* ...but only a pass with something in it replaces the remembered one, so
	   a stray single stripe afterwards does not wipe out the diagnosis. */
	if (r->valid || r->num_elements >= 3) {
		last_elements = r->num_elements;
		last_valid = r->valid ? TRUE : FALSE;
		last_error = r->error;
		last_text[0] = '\0';
		if (r->valid) {
			strncpy(last_text, r->text, S1_BC_TEXT_MAX);
			last_text[S1_BC_TEXT_MAX] = '\0';
		}
		width_summary(r);
	}

	if (!r->valid && r->num_elements < BARCODE_FAIL_MIN_ELEMENTS) {
		if (r->num_elements >= 3) {
			tm_printf((UB*)"barcode: ignored a %d bars/gaps pass "
				  "(under %d%% of a code)\n", r->num_elements,
				  BARCODE_FAIL_MIN_PCT);
		}
		return;
	}

	memset(&m, 0, sizeof m);
	m.valid = r->valid ? 1 : 0;
	m.command = (r->command == BARCODE_CMD_LEFT)    ? S1_BC_LEFT :
		    (r->command == BARCODE_CMD_RIGHT)   ? S1_BC_RIGHT :
		    (r->command == BARCODE_CMD_FORWARD) ? S1_BC_FORWARD :
		    (r->command == BARCODE_CMD_UTURN)   ? S1_BC_UTURN : S1_BC_NONE;
	m.reversed = r->reversed ? 1 : 0;
	if (r->valid) {
		strncpy(m.text, r->text, S1_BC_TEXT_MAX);
		m.text[S1_BC_TEXT_MAX] = '\0';
		/* IF-15: the command goes out FIRST, then the count that tells
		   navigation to look at it.  The other order would let it read
		   the new count beside the previous code's command. */
		s5_cmd = (UB)r->command;
		s5_reads++;			/* IF-15, counted with the line below */
		tm_printf((UB*)"barcode: \"%s\" %s%s (%d bars/gaps)\n", m.text,
			  barcode_cmd_name(r->command), r->reversed ? " (backwards)" : "",
			  r->num_elements);
	} else {
		/* IF-15: the "decoding failed" signal navigation acts on.  Counted
		   here, not earlier, so it means exactly what the console line
		   below says: the passes too short to be a code have already
		   returned above. */
		s5_fails++;
		tm_printf((UB*)"barcode: pass FAILED, %d bars/gaps: %s\n",
			  r->num_elements, r->error ? r->error : "?");
		dump_widths(r);
	}
	/* Never wait: if subsystem 1 is behind, this pass is dropped */
	(void)tk_snd_mbf(mbf_barcode, &m, sizeof m, TMO_POL);
	s5_passes++;
}

/* 1 = print what the sensor is reading every S5_LEVEL_MS.  This is the line
   to look at when nothing decodes: it says whether the sensor sees any
   contrast at all, and whether the floor level it learned at power-up is
   sane.  Set back to 0 when the sensor is behaving.
     raw    the reading right now (0..4095, higher = blacker)
     swing  the lowest and highest reading since the last line: a bar going
            past should show a wide swing, plain floor almost none
     white  the floor level learned from 64 samples at power-up
     black  the bar level: the highest reading lately, decaying slowly
     edges  black <-> white changes since power-up (a *L* code is 29 of them)
   A code reads only if "swing" straddles the midpoint between white and
   black by a clear margin, and if "edges" climbs by about 29 per pass. */
#define S5_LEVEL_TRACE	1
#define S5_LEVEL_MS	500

LOCAL void trace_levels(void)
{
#if S5_LEVEL_TRACE
	uint16_t raw, white, black, lo, hi;
	INT	 elements, glitch_count;

	barcode_get_levels(&raw, &white, &black);
	barcode_take_swing(&lo, &hi);
	barcode_progress(&elements, &glitch_count);
	/* "mid" is the real threshold the sampler uses: the span is never taken
	   as less than BARCODE_MIN_CONTRAST, so on plain floor it sits a fixed
	   distance above white rather than half way to black. */
	{
		INT span = (INT)black - (INT)white;
		INT mid;

		if (span < BARCODE_MIN_CONTRAST) {
			span = BARCODE_MIN_CONTRAST;
		}
		mid = (INT)white + span / 2;

		tm_printf((UB*)"barcode: raw %d swing %d..%d | white %d black %d "
			  "mid %d | %s | %s %d bars/gaps, %d noise | edges %d "
			  "passes %d | last: %d %s%s\n",
			  (INT)raw, (INT)lo, (INT)hi, (INT)white, (INT)black, mid,
			  (barcode_black != 0u) ? "on BLACK" : "on white",
			  (barcode_state() == BARCODE_IN_PASS) ? "IN PASS:" : "waiting,",
			  elements, glitch_count,
			  (INT)barcode_edges, (INT)s5_passes,
			  last_elements,
			  last_valid ? "OK " : (last_elements ? "FAILED " : "none"),
			  last_valid ? last_text : (last_error ? last_error : ""));

		if (last_elements > 0) {
			tm_printf((UB*)"barcode:   last widths us: %d..%d (mean %d, "
				  "widest/narrowest %d.%d)\n",
				  (INT)last_lo, (INT)last_hi, (INT)last_mid,
				  (INT)(last_lo ? (last_hi / last_lo) : 0),
				  (INT)(last_lo ? ((last_hi * 10u / last_lo) % 10u) : 0));
		}
		if (hist_n > 0) {
			INT k;
			tm_printf((UB*)"barcode:   bars/gaps per pass:");
			for (k = 0; k < (INT)hist_n; k++) {
				tm_printf((UB*)" %d", (INT)hist[k]);
			}
			tm_printf((UB*)" (29 = a whole *X* code)\n");
		}
	}
#endif
}

EXPORT void barcode_task(INT stacd, void *exinf)
{
	static barcode_result_t result;		/* static: ~700 B, too big for the stack */
	ER er;
	UW next_level;

	(void)stacd; (void)exinf;

	barcode_init();			/* sensor over white floor: learns the floor */
	er = sampler_start();
	if (er < E_OK) {
		tm_printf((UB*)"barcode: ERROR could not start the sampler (%d)\n", er);
		tk_slp_tsk(TMO_FEVR);
	}
	tm_printf((UB*)"barcode: sampling GP28 at %d Hz\n", 1000000 / BARCODE_SAMPLE_US);

	next_level = app_now_ms();
	while (1) {
		if (barcode_poll(&result)) {
			report(&result);
		}
		if ((W)(app_now_ms() - next_level) >= 0) {
			next_level = app_now_ms() + S5_LEVEL_MS;
			trace_levels();
		}
		tk_dly_tsk(S5_POLL_MS);
	}
}
