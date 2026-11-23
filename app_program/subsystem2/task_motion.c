/*
 *	task_motion.c
 *	Subsystem 2 -- motion_task: the motion state machine.
 *
 *	APIs shown: tk_rcv_mbf with a timeout (wait for a command OR the next
 *	            control tick, whichever comes first)
 *
 *	The task wakes the moment a command arrives (so steering reacts at
 *	once) and otherwise every S2_PERIOD_MS to refresh the odometry.  Every
 *	S2_SYNC_MS it corrects the wheel speeds:
 *
 *	  DRIVE:  both encoders are compared and the wheel that is ahead is
 *	          slowed, the one behind sped up, so both cover the distance
 *	          the command asked for (equal = straight).  Forwards (powers
 *	          0..100) or backwards (both 0..-100): the same correction.
 *	  Stall floor (DRIVE and WHEELS): a small DC motor needs a certain
 *	          power before it turns at all.  Each wheel learns its own
 *	          "floor": told to move but not turning -> +5 %; turning freely
 *	          while the floor drives it -> -2 %.  A wheel that is told to
 *	          move never gets less than its floor, so 10 % still moves.
 */

#include "subsystem2.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "cyw43_utk.h"		/* only to see when the radio has its PIO */
#include <stdlib.h>

/* ---- Speed correction (DRIVE) ---- */
#define SYNC_KP		0.5f	/* correction per count of difference, 0.1 % steps
				   robot weaves side to side -> lower it
				   still drifts to one side  -> raise it */
#define SYNC_MAX_CORR	150	/* biggest correction allowed: 15 % */
#define SYNC_MIN_FRACTION 0.5f	/* a wheel keeps at least half its commanded power */

/* ---- Stall floor (about 7 counts per mm: 1440 counts/rev, 65 mm wheel) ---- */
#define STALL_CHECK_MS		50	/* how often each wheel's progress is checked */
#define STALL_MIN_COUNTS	3	/* fewer counts than this per check = stalled */
#define STALL_FREE_COUNTS	15	/* more than this per check = turning freely */
#define STALL_STEP_PM		50	/* +5 % power each time a wheel is found stalled */
#define STALL_DECAY_PM		20	/* -2 % each check while it turns freely */
#define STALL_MAX_PM		500	/* the floor never goes above 50 % */

#define SPEED_WINDOW_MS		250	/* speed is measured over this time */
#define RADIO_WAIT_MS		10000	/* give up waiting for the radio after this */

typedef struct {
	UW	check_raw;	/* encoder count at the start of this check */
	INT	ticks;		/* sync ticks since then */
	INT	last_sign;	/* direction it was told to turn at the last tick */
	INT	floor_pm;	/* learned lowest power that keeps it turning */
} STALL;

LOCAL STALL	stall[2];	/* [0] left, [1] right */

/* Everything the state machine remembers */
LOCAL struct {
	UB	state;		/* MOTION_STATE */
	BOOL	enc_ok;
	BOOL	enc_failed;	/* tried and could not get a PIO block */
	INT	drive_left_pm;	/* DRIVE: commanded power 0..1000 (size only) */
	INT	drive_right_pm;
	INT	drive_dir;	/* DRIVE: +1 forward, -1 backward */
	UW	drive_left_start;	/* raw counts when this DRIVE started */
	UW	drive_right_start;
	INT	direct_left_pm;	/* WHEELS: -1000..1000 */
	INT	direct_right_pm;
	INT	applied_left_pm;	/* what the motors were last given */
	INT	applied_right_pm;
} m;

LOCAL INT clamp(INT v, INT lo, INT hi)
{
	return (v < lo) ? lo : (v > hi) ? hi : v;
}

/* ------------------------------------------------------------------ */
/* Stall floor                                                          */
/* ------------------------------------------------------------------ */
LOCAL void stall_restart(STALL *s, UW raw, INT sign)
{
	s->check_raw = raw;
	s->ticks = 0;
	s->last_sign = sign;
}

/* Every S2_SYNC_MS with the power the wheel is being given (before the floor) */
LOCAL void stall_update(STALL *s, UW raw, INT pm)
{
	INT sign = (pm > 0) - (pm < 0);
	W   moved;
	INT power;

	if (sign == 0 || sign != s->last_sign) {
		stall_restart(s, raw, sign);	/* stopped, or changed direction */
		return;
	}
	if (++s->ticks < STALL_CHECK_MS / S2_SYNC_MS) {
		return;
	}
	moved = abs((W)(raw - s->check_raw));
	power = abs(pm);
	if (moved < STALL_MIN_COUNTS) {
		INT from = (power > s->floor_pm) ? power : s->floor_pm;
		s->floor_pm = clamp(from + STALL_STEP_PM, 0, STALL_MAX_PM);
	} else if (moved > STALL_FREE_COUNTS && s->floor_pm > power) {
		s->floor_pm = clamp(s->floor_pm - STALL_DECAY_PM, 0, STALL_MAX_PM);
	}
	stall_restart(s, raw, sign);
}

/* The power to apply: never less than the wheel's floor while it is told to move */
LOCAL INT with_floor(const STALL *s, INT pm)
{
	if (pm == 0 || abs(pm) >= s->floor_pm) {
		return pm;
	}
	return (pm > 0) ? s->floor_pm : -s->floor_pm;
}

LOCAL void set_motors(INT left_pm, INT right_pm)
{
	m.applied_left_pm  = with_floor(&stall[0], left_pm);
	m.applied_right_pm = with_floor(&stall[1], right_pm);
	motor_set_pm(S2_LEFT_MOTOR_A, S2_LEFT_MOTOR_B, m.applied_left_pm);
	motor_set_pm(S2_RIGHT_MOTOR_A, S2_RIGHT_MOTOR_B, m.applied_right_pm);
}

LOCAL void brake(void)
{
	motor_brake(S2_LEFT_MOTOR_A, S2_LEFT_MOTOR_B);
	motor_brake(S2_RIGHT_MOTOR_A, S2_RIGHT_MOTOR_B);
	m.applied_left_pm = 0;
	m.applied_right_pm = 0;
}

/* ------------------------------------------------------------------ */
/* Commands (the state machine's transitions)                           */
/* ------------------------------------------------------------------ */
LOCAL BOOL can_move(const MOTION_CMD *c)
{
	if (m.state == MOTION_ST_ESTOP) {
		return FALSE;			/* latched: only ENABLE leaves */
	}
	if (!m.enc_ok) {
		/* Without encoders the stall floor would see "not moving"
		   and keep raising the power, so do not move at all. */
		static UW last_warn = 0;
		UW now = app_now_ms();
		if (now - last_warn > 2000u) {
			last_warn = now;
			tm_printf((UB*)"motion: S%d asked to move, but the encoders "
				  "are not ready -- staying stopped\n", (INT)c->from);
		}
		return FALSE;
	}
	return TRUE;
}

LOCAL void start_drive(INT left_pct, INT right_pct)
{
	/* BACKWARDS when both are zero or below and one is below: the same
	   encoder sync, with the motors turned the other way.  Anything else
	   is forwards and is clamped to 0..100 exactly as it always was, so no
	   forward drive changes. */
	INT dir = (left_pct <= 0 && right_pct <= 0 &&
		   (left_pct < 0 || right_pct < 0)) ? -1 : +1;
	INT lpm = clamp(dir * left_pct, 0, 100) * 10;
	INT rpm = clamp(dir * right_pct, 0, 100) * 10;

	/* Already driving like this: keep going without restarting the count */
	if (m.state == MOTION_ST_DRIVE && lpm == m.drive_left_pm &&
	    rpm == m.drive_right_pm && dir == m.drive_dir) {
		return;
	}
	if (lpm == 0 && rpm == 0) {
		brake();
		m.state = MOTION_ST_STOPPED;
		return;
	}
	m.drive_left_pm = lpm;
	m.drive_right_pm = rpm;
	m.drive_dir = dir;
	m.drive_left_start = enc_left_raw();
	m.drive_right_start = enc_right_raw();
	set_motors(dir * lpm, dir * rpm);
	m.state = MOTION_ST_DRIVE;
}

LOCAL void drive_wheels(INT left_pct, INT right_pct)
{
	INT lpm = clamp(left_pct, -100, 100) * 10;
	INT rpm = clamp(right_pct, -100, 100) * 10;

	if (m.state == MOTION_ST_WHEELS && lpm == m.direct_left_pm && rpm == m.direct_right_pm) {
		return;				/* already driving like this */
	}
	m.direct_left_pm = lpm;
	m.direct_right_pm = rpm;
	set_motors(lpm, rpm);
	m.state = (lpm != 0 || rpm != 0) ? MOTION_ST_WHEELS : MOTION_ST_STOPPED;
}

LOCAL void handle_command(const MOTION_CMD *c)
{
	switch (c->type) {
	case MOTION_CMD_ESTOP:
		brake();			/* first, before anything else */
		if (m.state != MOTION_ST_ESTOP) {
			tm_printf((UB*)"motion: EMERGENCY STOP (from S%d)\n", (INT)c->from);
		}
		m.state = MOTION_ST_ESTOP;
		break;

	case MOTION_CMD_ENABLE:
		/* Start of a new run (GP20 / START): clear every latched error */
		if (m.enc_failed) {
			m.enc_failed = FALSE;		/* try_start_encoders() tries again */
			tm_printf((UB*)"motion: retrying the encoders\n");
		}
		if (m.state == MOTION_ST_ESTOP) {
			m.state = m.enc_ok ? MOTION_ST_STOPPED : MOTION_ST_STARTING;
		}
		break;

	case MOTION_CMD_STOP:
		brake();
		if (m.state == MOTION_ST_DRIVE || m.state == MOTION_ST_WHEELS) {
			m.state = MOTION_ST_STOPPED;
		}
		break;

	case MOTION_CMD_DRIVE:
		if (can_move(c)) {
			start_drive(c->left, c->right);
		}
		break;

	case MOTION_CMD_WHEELS:
		if (can_move(c)) {
			drive_wheels(c->left, c->right);
		}
		break;

	default:
		break;
	}
}

/* ------------------------------------------------------------------ */
/* Diagnostic trace (S2_TRACE in subsystem2.h)                          */
/* ------------------------------------------------------------------ */
#if S2_TRACE
LOCAL void trace_step(UW lraw, UW rraw, INT lask, INT rask)
{
	static UW	last_l, last_r, quiet_until, n;
	UW		now = app_now_ms();
	BOOL		moving = (m.state == MOTION_ST_DRIVE || m.state == MOTION_ST_WHEELS);

	if (moving) {
		quiet_until = now + 300;	/* and 300 ms after it stops */
	} else if ((W)(now - quiet_until) > 0) {
		last_l = lraw;
		last_r = rraw;
		return;
	}
	if (++n & 1u) {
		return;				/* every 2nd step: 20 ms */
	}
	tm_printf((UB*)"m %5lu %-7s L%4d/%4d f%3d d%4d | R%4d/%4d f%3d d%4d\n",
		  (unsigned long)(now % 100000u), motion_state_name(m.state),
		  lask, m.applied_left_pm, stall[0].floor_pm, (INT)(W)(lraw - last_l),
		  rask, m.applied_right_pm, stall[1].floor_pm, (INT)(W)(rraw - last_r));
	last_l = lraw;
	last_r = rraw;
}
#else
#define trace_step(l, r, la, ra)	((void)0)
#endif

/* ------------------------------------------------------------------ */
/* Every S2_SYNC_MS: speed correction and stall floor                   */
/* ------------------------------------------------------------------ */
LOCAL void control_step(void)
{
	UW	lraw = enc_left_raw();
	UW	rraw = enc_right_raw();
	INT	left_pm, right_pm;

	if (m.state == MOTION_ST_DRIVE) {
		/* Distance each wheel has covered since the drive started */
		float l  = (float)abs((W)(lraw - m.drive_left_start));
		float r  = (float)abs((W)(rraw - m.drive_right_start));
		float sl = (float)m.drive_left_pm;
		float sr = (float)m.drive_right_pm;

		/* How many counts the left wheel is ahead of where it should be.
		   With equal speeds that's just left minus right. */
		float ahead = (l * sr - r * sl) / ((sl + sr) * 0.5f);
		float corr = SYNC_KP * ahead;

		if (corr > SYNC_MAX_CORR)  corr = SYNC_MAX_CORR;
		if (corr < -SYNC_MAX_CORR) corr = -SYNC_MAX_CORR;

		/* Slow the wheel that's ahead, speed up the one behind -- but
		   never below SYNC_MIN_FRACTION of what it was asked for.  At 10 %
		   power the full 15 % correction would otherwise stop a wheel
		   dead, then the other one, in turn ("one wheel at a time"),
		   e.g. when the wheels spin freely off the floor. */
		left_pm  = clamp((INT)(sl - corr), (INT)(sl * SYNC_MIN_FRACTION), 1000);
		right_pm = clamp((INT)(sr + corr), (INT)(sr * SYNC_MIN_FRACTION), 1000);

		/* Everything above is in sizes -- the distances are abs() and the
		   powers are the commanded magnitudes -- so it holds the wheels in
		   step the same way in either direction.  Only now is the
		   direction put back on. */
		left_pm  *= m.drive_dir;
		right_pm *= m.drive_dir;
	} else if (m.state == MOTION_ST_WHEELS) {
		left_pm  = m.direct_left_pm;
		right_pm = m.direct_right_pm;
	} else {
		/* Stopped: leave the motors alone, keep the checks fresh */
		stall_restart(&stall[0], lraw, 0);
		stall_restart(&stall[1], rraw, 0);
		trace_step(lraw, rraw, 0, 0);
		return;
	}

	stall_update(&stall[0], lraw, left_pm);
	stall_update(&stall[1], rraw, right_pm);
	set_motors(left_pm, right_pm);
	trace_step(lraw, rraw, left_pm, right_pm);
}

/* ------------------------------------------------------------------ */
/* Encoders: started once the radio has taken its PIO block             */
/* ------------------------------------------------------------------ */
LOCAL void try_start_encoders(UW now, UW t0)
{
	T_CYW43_UTK_STATUS radio;
	INT pio = -1;

	if (m.enc_ok || m.enc_failed) {
		return;
	}
	cyw43_utk_get_status(&radio);
	if (!radio.init_complete && (now - t0) < RADIO_WAIT_MS) {
		return;				/* radio still loading its firmware */
	}

	if (enc_init(&pio)) {
		m.enc_ok = TRUE;
		tm_printf((UB*)"motion: encoders running on PIO%d "
			  "(L GP%d/GP%d, R GP%d/GP%d)\n", pio,
			  S2_LEFT_ENC_A, S2_LEFT_ENC_B, S2_RIGHT_ENC_A, S2_RIGHT_ENC_B);
		if (m.state == MOTION_ST_STARTING) {
			m.state = MOTION_ST_STOPPED;
		}
	} else {
		m.enc_failed = TRUE;
		tm_printf((UB*)"motion: ERROR no free PIO block for the encoders -- "
			  "the car will not drive\n");
	}
}

/* ------------------------------------------------------------------ */
/* Odometry copy for the other subsystems (IF-03)                       */
/* ------------------------------------------------------------------ */
LOCAL void publish(UW now, float speed_cms)
{
	ODOMETRY o;

	o.left = (W)enc_left_raw();
	o.right = (W)enc_right_raw();
	o.speed_cms = speed_cms;
	o.state = m.state;
	o.encoders_ok = m.enc_ok;
	o.left_pm = (H)m.applied_left_pm;
	o.right_pm = (H)m.applied_right_pm;
	o.left_floor_pm = (H)stall[0].floor_pm;
	o.right_floor_pm = (H)stall[1].floor_pm;
	o.updated_ms = now;
	motion_publish(&o);
}

/* ------------------------------------------------------------------ */
/* The task                                                             */
/* ------------------------------------------------------------------ */
EXPORT void motion_task(INT stacd, void *exinf)
{
	UW		t0, now, next_tick, next_sync, speed_t0 = 0;
	W		speed_l0 = 0, speed_r0 = 0;
	float		speed_cms = 0.0f;
	MOTION_CMD	cmd;
	INT		sz, tmo;

	(void)stacd; (void)exinf;

	if (motor_init()) {			/* motors braked from the start */
		tm_printf((UB*)"motion: motor PWM ready (GP%d-GP%d, 20 kHz)\n",
			  S2_RIGHT_MOTOR_A, S2_LEFT_MOTOR_A);
	} else {
		tm_printf((UB*)"motion: ERROR the PWM block does not respond -- "
			  "the motors cannot run\n");
	}
	m.state = MOTION_ST_STARTING;
	t0 = next_tick = next_sync = app_now_ms();

	while (1) {
		/* Wait for a command, but no longer than the next tick */
		now = app_now_ms();
		tmo = (INT)(next_tick - now);
		if (tmo < 0) tmo = 0;		/* 0 = TMO_POL: just look */

		sz = tk_rcv_mbf(mbf_motion, &cmd, tmo);
		if (sz == (INT)sizeof cmd) {
			handle_command(&cmd);
		}

		now = app_now_ms();
		if ((W)(now - next_tick) < 0) {
			continue;			/* woken by a command only */
		}
		next_tick += S2_PERIOD_MS;
		if ((W)(now - next_tick) >= 0) {
			next_tick = now + S2_PERIOD_MS;	/* fell behind: no burst */
		}

		try_start_encoders(now, t0);

		if ((W)(now - next_sync) >= 0) {
			next_sync += S2_SYNC_MS;
			if ((W)(now - next_sync) >= 0) next_sync = now + S2_SYNC_MS;
			control_step();
		}

		/* Speed: signed average of both wheels, so spinning reads ~0 */
		if (now - speed_t0 >= SPEED_WINDOW_MS) {
			W l = (W)enc_left_raw(), r = (W)enc_right_raw();
			if (speed_t0 != 0) {
				float counts = ((float)(l - speed_l0) + (float)(r - speed_r0)) / 2.0f;
				if (counts < 0) counts = -counts;
				speed_cms = counts / COUNTS_PER_MM * 1000.0f
					    / (float)(now - speed_t0) / 10.0f;
			}
			speed_t0 = now;
			speed_l0 = l;
			speed_r0 = r;
		}

		publish(now, speed_cms);
	}
}
