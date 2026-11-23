/*
 *	task_obstacle.c
 *	Subsystem 6 -- obstacle_task: the state machine that watches the road
 *	ahead and latches an obstacle so the car stops before it hits it.
 *
 *	APIs shown: tk_rcv_mbf (with a timeout, so a request wakes the task at
 *	            once), tk_set_flg / tk_clr_flg, tk_dly_tsk
 *
 *	  MONITOR   ping every S6_PING_MS, rolling median of the last three.
 *	            S6_CONFIRM_CNT readings at or under S6_STOP_MM in a row
 *	            set S6_FLG_OBSTACLE -> BLOCKED.
 *	  BLOCKED   keeps pinging (so the distance stays live) and keeps the
 *	            flag set.  Only OBS_REQ_CLEAR goes back to MONITOR.
 *	            OBS_REQ_SCAN sweeps the servo and publishes the result.
 *	  FAULT     US_FAULT_INVALID_CNT pings in a row with no answer: the
 *	            distance becomes US_INVALID (never "clear"), and up to
 *	            US_REINIT_MAX recovery rounds are tried.
 *	  FAIL      recovery gave up.  The car is blind until OBS_REQ_CLEAR
 *	            (which every GP20 / START / RESET sends).
 *
 *	Why a latch: an obstacle that stopped the car must not be forgotten
 *	because one reading came back clear.  The flag is set again on every
 *	pass while BLOCKED, so it cannot be lost either.
 */

#include "subsystem6.h"

/* 1 = print the front distance on the USB console every S6_TRACE_MS.  Useful
   when aiming the sensor or checking the wiring; set back to 0 afterwards,
   the status line (subsystem 1) shows the distance anyway. */
#define S6_TRACE	0
#define S6_TRACE_MS	1000

LOCAL OBS_STATUS	obs;		/* this task's copy; published to mtx_obs */
LOCAL int32_t		ring[3];	/* the last three raw pings */
LOCAL INT		ring_idx;
LOCAL INT		bad;		/* invalid pings in a row (us_fault_update) */
LOCAL INT		hits;		/* readings in a row at or under S6_STOP_MM */

LOCAL void publish(void)
{
	obstacle_publish(&obs);
}

/* ---------- Console diagnostics ---------- */

/* Why every ping since power-up ended the way it did.  Printed when the
   sensor stops answering, because the counters say WHICH fault it is: all
   "no-echo" means no power or the wrong pins, "stuck" a shorted ECHO. */
LOCAL void print_health(void)
{
	us_diag_t d;

	us_diag_get(&d);
	tm_printf((UB*)"obstacle: pings ok=%u open=%u | bad: no-echo=%u "
		  "stuck=%u too-close=%u\n",
		  (UINT)d.valid_cnt, (UINT)d.clear_cnt, (UINT)d.none_cnt,
		  (UINT)d.stuck_cnt, (UINT)d.close_cnt);
}

/* The scan report.  Built as one string and printed with ONE tm_printf,
   because tm_printf holds its lock for a whole call: that keeps the block
   together instead of letting other tasks' lines through the middle of it.
   Only this task uses the buffer, and it is static because the task stack is
   2 KB and tm_printf itself needs about 1 KB of it.

   Example:
     [s6] =============== OBSTACLE SCAN ===============
     [s6]  trigger   : object at 70 mm (stop at 150 mm)
     [s6]  angle deg :    -60    -30      0     30     60
     [s6]  dist mm   :    143    133    159    238    243
     [s6]  status    :    BLK    BLK    BLK    BLK    BLK
     [s6]  closest   : -30 deg, 133 mm    scan time: 580 ms
     [s6]  object    : 184 mm long, -25..10 deg (8 points, nearest 133 mm)
     [s6]              lower bound, beam allowance off: 105 mm
     [s6]  sides     : left 365 mm, right 172 mm -> more room LEFT
     [s6] =============================================
   Angles: negative = right of centre, positive = left.  BLK = something
   within SCAN_OBSTACLE_RANGE_MM, clr = clear, INV = unknown (never "clear"). */
#define S6_REPORT_BUF	1024
LOCAL char		report_buf[S6_REPORT_BUF];

LOCAL void print_report(void)
{
	const us_coarse_t	*c = &obs.coarse;
	const us_object_t	*o = &obs.object;
	const us_profile_t	*p = &obs.profile;
	char			*b = report_buf;
	INT			len = 0, i;
	UINT			bit;

	len += tm_sprintf((UB*)&b[len], (const UB*)
		"\n[s6] =============== OBSTACLE SCAN ===============\n"
		"[s6]  trigger   : object at %d mm (stop at %d mm)\n",
		(INT)obs.stop_mm, S6_STOP_MM);

	len += tm_sprintf((UB*)&b[len], (const UB*)"[s6]  angle deg :");
	for (i = 0; i < US_COARSE_N; i++) {
		len += tm_sprintf((UB*)&b[len], (const UB*)"%7d", (INT)c->deg[i]);
	}

	len += tm_sprintf((UB*)&b[len], (const UB*)"\n[s6]  dist mm   :");
	for (i = 0; i < US_COARSE_N; i++) {
		if (c->mm[i] == US_NO_ECHO) {
			len += tm_sprintf((UB*)&b[len], (const UB*)"   open");
		} else if (c->mm[i] == US_INVALID) {
			len += tm_sprintf((UB*)&b[len], (const UB*)"    ???");
		} else {
			len += tm_sprintf((UB*)&b[len], (const UB*)"%7d", (INT)c->mm[i]);
		}
	}

	len += tm_sprintf((UB*)&b[len], (const UB*)"\n[s6]  status    :");
	for (i = 0; i < US_COARSE_N; i++) {
		bit = 1u << (UINT)i;
		if ((UINT)c->invalid_mask & bit) {
			len += tm_sprintf((UB*)&b[len], (const UB*)"    INV");
		} else if ((UINT)c->blocked_mask & bit) {
			len += tm_sprintf((UB*)&b[len], (const UB*)"    BLK");
		} else {
			len += tm_sprintf((UB*)&b[len], (const UB*)"    clr");
		}
	}

	if (c->closest_idx >= 0) {
		len += tm_sprintf((UB*)&b[len], (const UB*)
			"\n[s6]  closest   : %d deg, %d mm    scan time: %u ms\n",
			(INT)c->deg[c->closest_idx], (INT)c->mm[c->closest_idx],
			(UINT)c->duration_ms);
	} else {
		len += tm_sprintf((UB*)&b[len], (const UB*)
			"\n[s6]  closest   : none in range    scan time: %u ms\n",
			(UINT)c->duration_ms);
	}

	if (o->found) {
		len += tm_sprintf((UB*)&b[len], (const UB*)
			"[s6]  object    : %d mm long, %d..%d deg (%d points, nearest %d mm)\n"
			"[s6]              lower bound, beam allowance off: %d mm\n",
			(INT)o->length_mm, (INT)o->right_deg, (INT)o->left_deg,
			(INT)o->hits, (INT)o->near_mm, (INT)o->length_corr_mm);
	} else {
		len += tm_sprintf((UB*)&b[len], (const UB*)
			"[s6]  object    : nothing inside %d mm left to measure\n",
			SCAN_OBSTACLE_RANGE_MM);
	}

	len += tm_sprintf((UB*)&b[len], (const UB*)
		"[s6]  sides     : left %d mm, right %d mm -> more room %s\n"
		"[s6] =============================================\n\n",
		(INT)p->clear_L_mm, (INT)p->clear_R_mm,
		(p->side == AVOID_LEFT) ? "LEFT" : "RIGHT");

	(void)len;
	tm_printf((UB*)"%s", report_buf);
}

LOCAL void trace_step(UW now)
{
#if S6_TRACE
	static UW next;

	if ((W)(now - next) < 0) {
		return;
	}
	next = now + S6_TRACE_MS;
	if (obs.dist_mm > 0) {
		tm_printf((UB*)"obstacle: %d mm (%s)\n", (INT)obs.dist_mm,
			  obs_state_name(obs.state));
	} else {
		tm_printf((UB*)"obstacle: %s (%s)\n",
			  (obs.dist_mm == US_NO_ECHO) ? "nothing in range" : "no answer",
			  obs_state_name(obs.state));
	}
#else
	(void)now;
#endif
}

/* ---------- The states ---------- */

LOCAL void ring_reset(void)
{
	ring[0] = ring[1] = ring[2] = US_NO_ECHO;
	ring_idx = 0;
	hits = 0;
	bad = 0;
}

/* MONITOR.  Also the way back from every other state, so it drops all four
   flag bits: subsystem 6 owns them (IF-11) and is the only one that may. */
LOCAL void enter_monitor(void)
{
	if (obs.state == OBS_ST_WATCHING) {
		servo_set_angle(0);		/* look ahead again */
	}
	obs.state = OBS_ST_MONITOR;
	obs.ok = TRUE;
	obs.blocked = FALSE;
	obs.stop_mm = 0;
	obs.scan_valid = FALSE;
	ring_reset();
	tk_clr_flg(flg_obs, ~(UINT)(S6_FLG_OBSTACLE | S6_FLG_SCAN_DONE |
				    S6_FLG_SENSOR_FAULT | S6_FLG_SENSOR_FAIL));
}

LOCAL void enter_blocked(void)
{
	obs.state = OBS_ST_BLOCKED;
	obs.blocked = TRUE;
	obs.stop_mm = obs.dist_mm;
	obs.scan_valid = FALSE;
	tk_set_flg(flg_obs, S6_FLG_OBSTACLE);
	tm_printf((UB*)"obstacle: object %d mm ahead, under the %d mm limit\n",
		  (INT)obs.dist_mm, S6_STOP_MM);
}

LOCAL void enter_fault(void)
{
	obs.state = OBS_ST_FAULT;
	obs.ok = FALSE;
	obs.faults++;
	obs.dist_mm = US_INVALID;	/* never report "clear" while blind */
	tk_set_flg(flg_obs, S6_FLG_SENSOR_FAULT);
	tm_printf((UB*)"obstacle: no answer from the sensor (%d pings): check "
		  "TRIG GP%d, ECHO GP%d, 3V3 and GND\n",
		  US_FAULT_INVALID_CNT, US_TRIG_PIN, US_ECHO_PIN);
	print_health();
}

/* FAULT: park the servo and try again.  The ECHO interrupt and the PWM were
   set up once in us_init(); running that again would leak kernel objects, so
   a "re-init" here means: recentre, wait, ping. */
LOCAL BOOL try_recover(void)
{
	INT a, i;

	for (a = 0; a < US_REINIT_MAX; a++) {
		servo_set_angle(0);
		tk_dly_tsk(US_REINIT_DELAY_MS);
		for (i = 0; i < US_REINIT_GOOD_PINGS; i++) {
			/* NO_ECHO is a working sensor looking at open space */
			if (us_ping_mm() != US_INVALID) {
				return TRUE;
			}
			tk_dly_tsk(US_PING_GAP_MS);
		}
	}
	return FALSE;
}

LOCAL void fault_step(void)
{
	if (try_recover()) {
		tm_printf((UB*)"obstacle: the sensor answers again\n");
		enter_monitor();
		return;
	}
	obs.state = OBS_ST_FAIL;
	tk_set_flg(flg_obs, S6_FLG_SENSOR_FAIL);
	tm_printf((UB*)"obstacle: sensor dead after %d tries -- the car cannot "
		  "see ahead.  Fix the wiring, then press GP20 (or send START)\n",
		  US_REINIT_MAX);
}

/* WATCHING: navigation only wants the distance, so an object in view must NOT
   latch -- it is the thing being driven round, and latching would stop the car
   in the middle of the manoeuvre.

   The servo stays centred.  It did swivel to watch the object slide past, but
   its horn has limited travel and a manoeuvre takes seconds: asked for an
   angle it cannot quite reach it sits there buzzing against its stop, warm and
   drawing stall current.  Navigation turns the whole car to look instead, so
   deg is only ever 0 today -- the parameter is kept because aiming it for a
   moment is still a reasonable thing for a future caller to do. */
LOCAL void enter_watching(INT deg)
{
	obs.state = OBS_ST_WATCHING;
	obs.ok = TRUE;
	obs.blocked = FALSE;
	ring_reset();
	servo_set_angle(deg);
	tk_dly_tsk(SERVO_SETTLE_LEFT_MS);	/* let it get there before pinging */
	tk_clr_flg(flg_obs, ~(UINT)(S6_FLG_OBSTACLE | S6_FLG_SCAN_DONE));
	tm_printf((UB*)"obstacle: watching %d deg, reporting but not latching\n",
		  deg);
}

/* One ping, filtered and published.  MONITOR, BLOCKED and WATCHING all come
   here; only MONITOR may latch. */
LOCAL void ping_step(void)
{
	int32_t raw = us_ping_mm();

	obs.pings++;
	ring[ring_idx] = raw;
	ring_idx = (ring_idx + 1) % 3;
	obs.dist_mm = us_filter3(ring);
	obs.updated_ms = app_now_ms();

	/* A sensor that stopped answering must never look like an empty road */
	if (us_fault_update(&bad, raw)) {
		enter_fault();
		return;
	}

	if (obs.state == OBS_ST_WATCHING) {
		return;			/* report the distance, latch nothing */
	}

	if (obs.state == OBS_ST_BLOCKED) {
		/* Latched until navigation asks for OBS_REQ_CLEAR.  Setting a
		   bit that is already set costs nothing, and means the latch
		   cannot be lost if someone else clears the flag. */
		tk_set_flg(flg_obs, S6_FLG_OBSTACLE);
		return;
	}

	/* MONITOR: a real distance (not "nothing in range", not unknown) at or
	   under the limit, S6_CONFIRM_CNT times in a row */
	hits = (obs.dist_mm > 0 && obs.dist_mm <= S6_STOP_MM) ? hits + 1 : 0;
	if (hits >= S6_CONFIRM_CNT) {
		enter_blocked();
	}
}

/* OBS_REQ_SCAN: the servo sweeps, in three stages.  About 3 s in all, so the
   car must be stopped; navigation only asks for it from NAV_AVOIDING, with
   the motors off.
     1. coarse fan   five angles: roughly WHERE the object is
     2. fine sweep   small steps around it: HOW LONG it is
     3. left/right   clearance either side: which way is open
   Stage 2 sweeps only a window around the angle stage 1 found, so it costs
   about as much as stage 1 instead of covering everything twice. */
LOCAL void do_scan(void)
{
	INT centre;

	us_coarse_scan(&obs.coarse);		/* stage 1: five angles, right to left */

	if (obs.coarse.closest_idx >= 0) {
		centre = (INT)obs.coarse.deg[obs.coarse.closest_idx];
		us_object_scan(&obs.object,	/* stage 2: measure it */
			       centre - SCAN_FINE_WINDOW_DEG,
			       centre + SCAN_FINE_WINDOW_DEG);
	} else {
		/* The fan found nothing in range (the object was taken away while
		   the car was stopping), so there is nothing to measure.  With no
		   samples the arrays are never read. */
		us_object_from_samples(&obs.object, NULL, NULL, 0,
				       SCAN_OBSTACLE_RANGE_MM);
	}

	us_scan_left_right(&obs.profile);	/* stage 3: clearance either side */
	obs.scan_valid = TRUE;
	obs.scans++;
	obs.updated_ms = app_now_ms();
	publish();
	print_report();
	tk_set_flg(flg_obs, S6_FLG_SCAN_DONE);
}

LOCAL void handle_request(UB req)
{
	switch (req) {
	case OBS_REQ_CLEAR:
		if (obs.state != OBS_ST_MONITOR) {
			tm_printf((UB*)"obstacle: %s cleared, watching the road again\n",
				  obs_state_name(obs.state));
		}
		enter_monitor();
		break;

	case OBS_REQ_WATCH_AHEAD:
		enter_watching(0);
		break;

	case OBS_REQ_SCAN:
		if (obs.state == OBS_ST_FAIL) {
			tm_printf((UB*)"obstacle: cannot scan, the sensor is dead\n");
			break;
		}
		do_scan();
		break;

	default:
		break;
	}
}

/* ---------- The task ---------- */
EXPORT void obstacle_task(INT stacd, void *exinf)
{
	UW	now, next_ping;
	UB	req;
	INT	sz, tmo, er;

	(void)stacd; (void)exinf;

	/* The sensor, the servo and the ECHO interrupt are set up HERE and not
	   in subsystem6_init(): EnableInt() enables an interrupt on the core
	   that calls it, and this task runs on core 1 (TEAM.md section 4). */
	er = us_init();
	if (er < 0) {
		obs.state = OBS_ST_FAIL;
		obs.dist_mm = US_INVALID;
		publish();
		tk_set_flg(flg_obs, S6_FLG_SENSOR_FAIL);
		tm_printf((UB*)"obstacle: ERROR could not set up the sensor (%d)\n", er);
		tk_slp_tsk(TMO_FEVR);
	}

	obs.dist_mm = US_INVALID;	/* nothing measured yet */
	enter_monitor();
	publish();
	tm_printf((UB*)"obstacle: HC-SR04 on GP%d/GP%d, servo on GP%d -- the car "
		  "stops %d cm from anything ahead\n",
		  US_TRIG_PIN, US_ECHO_PIN, US_SERVO_PIN, S6_STOP_MM / 10);

	next_ping = app_now_ms();
	while (1) {
		now = app_now_ms();
		tmo = (INT)(next_ping - now);
		if (tmo < 0) {
			tmo = 0;
		}

		/* Wait for a request until the next ping is due, so a request is
		   answered at once.  Blind (FAIL): nothing to do but wait. */
		sz = tk_rcv_mbf(mbf_obs_req, &req,
				(obs.state == OBS_ST_FAIL) ? TMO_FEVR : tmo);
		if (sz == (INT)sizeof req) {
			handle_request(req);
			publish();
		}

		now = app_now_ms();
		if ((W)(now - next_ping) < 0) {
			continue;		/* woken by a request, not due yet */
		}
		next_ping += S6_PING_MS;
		if ((W)(now - next_ping) >= 0) {
			next_ping = now + S6_PING_MS;	/* fell behind: start again */
		}

		switch (obs.state) {
		case OBS_ST_FAULT:
			fault_step();
			break;
		case OBS_ST_FAIL:
			break;			/* only a request gets us out */
		default:
			ping_step();		/* MONITOR and BLOCKED */
			break;
		}

		publish();
		trace_step(now);
	}
}
