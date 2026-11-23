/*
 *	task_imu.c
 *	Subsystem 4 -- imu_task: the IMU state machine and the safety stop.
 *
 *	APIs shown: tk_rcv_mbf with a timeout, tk_set_flg, tk_clr_flg
 *
 *	Every S4_SAMPLE_MS the task reads the sensor (in every state except
 *	SEARCH), publishes the result, and checks for a lift or an overturn.
 *	Between samples it waits on mbf_imu_req, so a request (for example
 *	"forget the lift, a new run is starting") is handled at once.
 */

#include "subsystem4.h"
#include "hump_height.h"
#include "../subsystem2/subsystem2.h"

LOCAL IMU_STATUS	st;		/* this task's working copy */
LOCAL UW		phase_t0;	/* when the current CAL_ state began */
LOCAL UW		next_retry;	/* SEARCH: next attempt */
LOCAL INT		fails;		/* failed reads in a row */
LOCAL INT		same;		/* identical readings in a row */
LOCAL vec3		last_acc;
LOCAL BOOL		search_reported;	/* "not found" printed once per search */

LOCAL BOOL in_calibration(void)
{
	return st.state == IMU_ST_CAL_SPIN || st.state == IMU_ST_CAL_SETTLE ||
	       st.state == IMU_ST_CAL_ZERO;
}

/* Integer + 2 decimals, since tm_printf has no %f */
LOCAL void print_cdeg(const char *name, float v)
{
	INT c = (INT)(v * 100.0f + (v < 0 ? -0.5f : 0.5f));
	INT a = (c < 0) ? -c : c;
	tm_printf((UB*)" %s %s%d.%02d", name, (c < 0) ? "-" : "", a / 100, a % 100);
}

/* ------------------------------------------------------------------ */
/* State changes                                                        */
/* ------------------------------------------------------------------ */

/* (Re)starts the IMU.  A restart clears the calibration, so GP21 is needed again. */
LOCAL BOOL imu_start(void)
{
	uint8_t who = 0;

	st.ok = imu_init(&who);
	st.valid = st.ok;
	st.who = who;
	st.calibrated = FALSE;
	st.pitch_off = st.roll_off = 0.0f;
	fails = 0;
	same = 0;
	return st.ok;
}

LOCAL void enter_search(UW now)
{
	st.state = IMU_ST_SEARCH;
	st.ok = FALSE;
	next_retry = now;
}

LOCAL void cal_fail(const char *why)
{
	imu_compass_cal_abort();
	imu_zero_abort();
	tm_printf((UB*)"imu: calibration aborted (%s), old calibration kept\n", why);
	st.cal_failed = TRUE;
	tk_set_flg(flg_imu, IMU_FLG_CAL_FAILED);
}

LOCAL void handle_request(UB req, UW now)
{
	switch (req) {
	case IMU_REQ_CAL_START:
		if (st.state != IMU_ST_RUN) {
			tm_printf((UB*)"imu: cannot calibrate now (%s)\n",
				  imu_state_name(st.state));
			st.cal_failed = TRUE;
			tk_set_flg(flg_imu, IMU_FLG_CAL_FAILED);
			break;
		}
		tk_clr_flg(flg_imu, ~(UINT)IMU_FLG_CAL_ANY);
		imu_compass_cal_begin();
		st.state = IMU_ST_CAL_SPIN;
		phase_t0 = now;
		tm_printf((UB*)"imu: calibrating -- compass (car spins %d s)\n",
			  CAL_SPIN_MS / 1000);
		break;

	case IMU_REQ_CAL_ABORT:
		if (in_calibration()) {
			cal_fail("asked to stop");
			st.state = IMU_ST_RUN;
		}
		break;

	case IMU_REQ_RESET_LIFT:
		if (st.ok) {
			imu_reset_lift();
		}
		st.d.lifted = FALSE;
		break;

	case IMU_REQ_RESET_RUN:
		if (st.ok) {
			imu_reset_hump();
		}
		st.d.hump_count = 0;
		st.d.hump_max = 0.0f;
		break;

	case IMU_REQ_CLEAR_ERRORS:
		/* Every GP20 / START: clear what is latched.  A fault that is
		   still true (upside down NOW) comes back at the next sample. */
		tk_clr_flg(flg_imu, ~(UINT)IMU_FLG_CAL_FAILED);
		st.cal_failed = FALSE;
		st.restarts = 0;
		fails = 0;
		same = 0;
		if (st.state == IMU_ST_SEARCH) {
			next_retry = now;		/* look for the sensor at once */
			search_reported = FALSE;
		} else {
			if (st.ok) {
				imu_reset_lift();
			}
			st.d.lifted = FALSE;
			if (!st.valid && !imu_reconfigure(&st.who)) {
				enter_search(now);	/* failing: start it from scratch */
			}
		}
		break;

	default:
		break;
	}
}

/* Timed CAL_ phases: move on when the current one has run its time */
LOCAL void calibration_step(UW now)
{
	UW elapsed = now - phase_t0;
	INT n;

	switch (st.state) {
	case IMU_ST_CAL_SPIN:
		st.cal_left_ms = CAL_SPIN_MS - (W)elapsed;
		if (elapsed >= CAL_SPIN_MS) {
			imu_compass_cal_end();
			tk_set_flg(flg_imu, IMU_FLG_CAL_SPIN_DONE);	/* stop spinning */
			st.state = IMU_ST_CAL_SETTLE;
			phase_t0 = now;
		}
		break;

	case IMU_ST_CAL_SETTLE:
		st.cal_left_ms = CAL_SETTLE_MS - (W)elapsed;
		if (elapsed >= CAL_SETTLE_MS) {
			imu_zero_begin();
			st.state = IMU_ST_CAL_ZERO;
			phase_t0 = now;
		}
		break;

	case IMU_ST_CAL_ZERO:
		st.cal_left_ms = CAL_ZERO_MS - (W)elapsed;
		if (elapsed >= CAL_ZERO_MS) {
			n = imu_zero_end();
			st.state = IMU_ST_RUN;
			st.cal_left_ms = 0;
			if (n > 0) {
				st.calibrated = TRUE;
				st.cal_failed = FALSE;
				imu_get_tilt_offsets(&st.pitch_off, &st.roll_off);
				tm_printf((UB*)"imu: calibrated from %d samples (offsets:", n);
				print_cdeg("pitch", st.pitch_off);
				print_cdeg("roll", st.roll_off);
				tm_printf((UB*)")\n");
				tk_set_flg(flg_imu, IMU_FLG_CAL_DONE);
			} else {
				cal_fail("no readings");
			}
		}
		break;

	default:
		st.cal_left_ms = 0;
		break;
	}
}

/* ------------------------------------------------------------------ */
/* Hump height (hump_height.h)                                          */
/* ------------------------------------------------------------------ */

/* An accelerometer cannot tell tilt from acceleration: a pivot turn pushes
   the car sideways by ~0.06 g, which reads exactly like a 3-4 degree roll --
   a 7 mm "hump" -- and starting or braking reads as the nose tipping.  So a
   reading only counts as tilt while the motors have been driving straight
   (MOTION_ST_DRIVE, both wheels in step) or stopped, without a change, for
   HH_STEADY_MS.  Subsystem 2's state says which, at no cost: navigation
   steers and spins through MOTION_ST_WHEELS. */
#define HH_STEADY_MS	200

LOCAL UB		hh_mstate = 0xFFu;	/* motion state last seen */
LOCAL UW		hh_mstate_t;		/* ...since when */
LOCAL hh_event_t	hh_last;		/* the most recent hump measured */
LOCAL UW		hh_count;		/* humps measured since power-up */

LOCAL BOOL hh_usable(UW now)
{
	ODOMETRY o;

	motion_get_odometry(&o);
	if (o.state != hh_mstate) {
		hh_mstate = o.state;
		hh_mstate_t = now;
	}
	if (in_calibration()) {
		return FALSE;			/* the GP21 spin */
	}
	return (o.state == MOTION_ST_DRIVE || o.state == MOTION_ST_STOPPED ||
		o.state == MOTION_ST_STARTING) &&
	       (now - hh_mstate_t) >= HH_STEADY_MS;
}

/* Feed one reading; fill d's hump-height fields either way */
LOCAL void hh_step(imu_data_t *d, UW now)
{
	hh_event_t ev;

	if (hh_update(d->acc.x, d->acc.y, d->acc.z, imu_get_g_ref(),
		      hh_usable(now) ? true : false, (uint32_t)now, &ev)) {
		INT mm = (INT)(ev.height_mm + 0.5f);
		INT t10 = (INT)(ev.tilt_deg * 10.0f + 0.5f);

		hh_last = ev;
		hh_count++;
		/* For calibrating S4_HUMP_LEVER_MM with a block (hump_height.h) */
		if (ev.side == 'C') {
			tm_printf((UB*)"imu: hump %d mm (car pitched %d.%d deg, not "
				  "one wheel; %d ms)\n", mm, t10 / 10, t10 % 10,
				  (INT)ev.dur_ms);
		} else {
			tm_printf((UB*)"imu: hump %d mm under the %s wheel (peak tilt "
				  "%d.%d deg, %d ms; D = %d mm)\n", mm,
				  (ev.side == 'R') ? "RIGHT" : "LEFT", t10 / 10,
				  t10 % 10, (INT)ev.dur_ms, (INT)S4_HUMP_LEVER_MM);
		}
	}
	d->hh_height_mm = hh_last.height_mm;
	d->hh_tilt_deg = hh_last.tilt_deg;
	d->hh_side = (hh_count > 0u) ? hh_last.side : '-';
	d->hh_count = hh_count;
	d->hh_tilt_now = hh_tilt_now();
}

/* ------------------------------------------------------------------ */
/* One reading                                                          */
/* ------------------------------------------------------------------ */

/* Lift / overturn: stop the motors at once and tell navigation */
LOCAL void safety_check(void)
{
	BOOL	over = st.d.overturned;
	BOOL	lifted = st.d.lifted;
	UINT	bits;

	if (in_calibration()) {
		lifted = FALSE;			/* the spin is not a lift */
		if (!st.calibrated) {
			over = FALSE;		/* "up" not known before the first calibration */
		}
	}
	bits = (over ? IMU_FLG_OVERTURNED : 0) | (lifted ? IMU_FLG_LIFTED : 0);
	if (bits == 0) {
		return;
	}

	/* Motors first, then the event.  Re-sent while the condition lasts
	   whenever motion is not already latched in ESTOP (e.g. a new run
	   was enabled while the car was still upside down). */
	{
		ODOMETRY o;
		motion_get_odometry(&o);
		if (o.state != MOTION_ST_ESTOP) {
			motion_command(MOTION_CMD_ESTOP, 0, 0, 4);
		}
	}
	tk_set_flg(flg_imu, bits);
}

LOCAL void sample(UW now)
{
	imu_data_t	d;
	BOOL		hump = FALSE;

	st.valid = imu_read(&d);
	if (!st.valid) {
		if (++fails >= S4_MAX_FAILS) {
			/* Write the settings again (keeps the calibration). If it
			   does not answer, look for it from scratch. */
			fails = 0;
			st.restarts++;
			if (!imu_reconfigure(&st.who)) {
				if (in_calibration()) {
					cal_fail("IMU lost");
				}
				tm_printf((UB*)"imu: sensor lost, searching again\n");
				enter_search(now);
			}
		}
		return;
	}
	fails = 0;

	/* A live sensor always jitters by a few counts.  Exactly the same
	   reading IMU_FROZEN_READS times in a row means it has stopped (a
	   power dip resets the LSM303D to power-down). */
	if (d.acc.x == last_acc.x && d.acc.y == last_acc.y && d.acc.z == last_acc.z) {
		if (++same >= IMU_FROZEN_READS) {
			same = 0;
			st.restarts++;
			if (!imu_reconfigure(&st.who)) {
				if (in_calibration()) {
					cal_fail("IMU lost");
				}
				enter_search(now);
				return;
			}
		}
	} else {
		same = 0;
		last_acc = d.acc;
	}

	hh_step(&d, now);		/* hump height (imu_read() leaves it out) */
	st.d = d;
	hump = d.hump_detected;
	safety_check();
	imu_publish(&st, hump, d.hump_last);
}

/* ------------------------------------------------------------------ */
/* The task                                                             */
/* ------------------------------------------------------------------ */
EXPORT void imu_task(INT stacd, void *exinf)
{
	UW	now, next_sample;
	UB	req;
	INT	sz, tmo;

	(void)stacd; (void)exinf;

	st.state = IMU_ST_SEARCH;
	next_retry = next_sample = app_now_ms();

	while (1) {
		now = app_now_ms();
		tmo = (INT)(next_sample - now);
		if (tmo < 0) tmo = 0;

		sz = tk_rcv_mbf(mbf_imu_req, &req, tmo);
		now = app_now_ms();
		if (sz == (INT)sizeof req) {
			handle_request(req, now);
			imu_publish(&st, FALSE, 0.0f);
		}
		if ((W)(now - next_sample) < 0) {
			continue;
		}
		next_sample += S4_SAMPLE_MS;
		if ((W)(now - next_sample) >= 0) next_sample = now + S4_SAMPLE_MS;

		if (st.state == IMU_ST_SEARCH) {
			if ((W)(now - next_retry) >= 0) {
				next_retry = now + S4_RETRY_MS;
				if (imu_start()) {
					st.state = IMU_ST_RUN;
					search_reported = FALSE;
					tm_printf((UB*)"imu: LSM303D found (WHO_AM_I 0x%02x). "
						  "Press GP21 to calibrate.\n", st.who);
				} else {
					if (!search_reported) {
						search_reported = TRUE;
						tm_printf((UB*)"imu: LSM303D not found (WHO_AM_I 0x%02x, "
							  "want 0x49): check SDA GP2 / SCL GP3. "
							  "Retrying every %d s\n", st.who, S4_RETRY_MS / 1000);
					}
				}
				imu_publish(&st, FALSE, 0.0f);
			}
			continue;
		}

		sample(now);
		calibration_step(now);
		imu_publish(&st, FALSE, 0.0f);
	}
}
