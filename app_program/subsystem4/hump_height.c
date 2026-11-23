/*
 *	hump_height.c
 *	Subsystem 4 -- the height of a hump one wheel rolls over.  See
 *	hump_height.h for the method; this is the arithmetic.
 *
 *	Every reading:
 *	  1. If the car is not turning or changing speed (usable), and its size
 *	     is within HH_QUIET_G of 1 g, it goes into a window of the last
 *	     HH_MEDIAN_N such readings.  Anything else is the car being pushed
 *	     about -- a pivot, a start or stop, the wheel hitting the hump -- and
 *	     would show up as a tilt that is not there.
 *	  2. The window's per-axis MEDIAN is the gravity direction now.  A
 *	     median, not a mean: one spike cannot move it.
 *	  3. The tilt is the angle between that and the BASELINE, which follows
 *	     the gravity direction slowly (HH_BASE_TC_MS) while no hump is going
 *	     on, and is frozen during one.
 *	  4. A hump starts when the tilt passes HH_START_DEG, its peak is kept,
 *	     and it ends once the tilt has stayed under HH_END_DEG for
 *	     HH_END_HOLD_MS.  Then height = S4_HUMP_LEVER_MM * sin(peak).
 */

#include "hump_height.h"

#include <math.h>

#define HH_PI		3.14159265f
#define HH_RAD2DEG	(180.0f / HH_PI)

/* The window of quiet readings */
static float	 s_wx[HH_MEDIAN_N], s_wy[HH_MEDIAN_N], s_wz[HH_MEDIAN_N];
static int	 s_wn;			/* readings in it (up to HH_MEDIAN_N) */
static int	 s_wi;			/* where the next one goes */

/* The baseline: a unit vector, valid once s_have_base */
static bool	 s_have_base;
static float	 s_bx, s_by, s_bz;

static bool	 s_have_last;
static uint32_t	 s_last_ms;

/* The hump in progress */
static bool	 s_in;
static uint32_t	 s_t0;			/* when the tilt passed HH_START_DEG */
static bool	 s_below;		/* under HH_END_DEG since s_below_t */
static uint32_t	 s_below_t;
static float	 s_peak;		/* peak tilt, degrees */
static float	 s_peak_roll;		/* roll change at the peak, degrees */
static bool	 s_steep;		/* went past HH_MAX_TILT_DEG */
static bool	 s_long;		/* went past HH_MAX_MS */

static float	 s_tilt_now;

void hh_reset(void)
{
	s_wn = 0;
	s_wi = 0;
	s_have_base = false;
	s_have_last = false;
	s_in = false;
	s_below = false;
	s_peak = 0.0f;
	s_peak_roll = 0.0f;
	s_steep = false;
	s_long = false;
	s_tilt_now = 0.0f;
}

float hh_tilt_now(void)
{
	return s_tilt_now;
}

/* Median of n values (n <= HH_MEDIAN_N), without disturbing the window */
static float median(const float *v, int n)
{
	float	a[HH_MEDIAN_N], t;
	int	i, j;

	for (i = 0; i < n; i++) {
		a[i] = v[i];
	}
	for (i = 1; i < n; i++) {		/* insertion sort: n is tiny */
		t = a[i];
		for (j = i; j > 0 && a[j - 1] > t; j--) {
			a[j] = a[j - 1];
		}
		a[j] = t;
	}
	return a[n / 2];
}

static void set_base(float x, float y, float z)
{
	float n = sqrtf(x * x + y * y + z * z);

	if (n > 1e-6f) {
		s_bx = x / n;
		s_by = y / n;
		s_bz = z / n;
		s_have_base = true;
	}
}

bool hh_update(float ax, float ay, float az, float g_ref, bool usable,
	       uint32_t now_ms, hh_event_t *out)
{
	float	 mag = sqrtf(ax * ax + ay * ay + az * az);
	float	 mx, my, mz, n, cx, cy, cz, dot, tilt, droll, a;
	uint32_t dt = s_have_last ? (uint32_t)(now_ms - s_last_ms) : 0u;
	bool	 done = false;

	s_last_ms = now_ms;
	s_have_last = true;

	/* (1) only steady, quiet readings are evidence of tilt */
	if (usable && fabsf(mag - g_ref) <= HH_QUIET_G) {
		s_wx[s_wi] = ax;
		s_wy[s_wi] = ay;
		s_wz[s_wi] = az;
		s_wi = (s_wi + 1) % HH_MEDIAN_N;
		if (s_wn < HH_MEDIAN_N) {
			s_wn++;
		}
	}
	if (s_wn < HH_MEDIAN_N) {
		return false;			/* not enough yet */
	}

	/* (2) gravity direction now */
	mx = median(s_wx, s_wn);
	my = median(s_wy, s_wn);
	mz = median(s_wz, s_wn);
	n = sqrtf(mx * mx + my * my + mz * mz);
	if (n < 1e-6f) {
		return false;
	}
	mx /= n;
	my /= n;
	mz /= n;

	if (!s_have_base) {
		set_base(mx, my, mz);
		return false;
	}

	/* (3) the angle between now and the baseline: atan2(|a x b|, a . b),
	   which stays accurate for small angles where acos() does not */
	cx = my * s_bz - mz * s_by;
	cy = mz * s_bx - mx * s_bz;
	cz = mx * s_by - my * s_bx;
	dot = mx * s_bx + my * s_by + mz * s_bz;
	tilt = atan2f(sqrtf(cx * cx + cy * cy + cz * cz), dot) * HH_RAD2DEG;
	s_tilt_now = tilt;

	/* Roll change, for which side rose (imu.h: roll = atan2(y, z), right
	   side down = positive) */
	droll = (atan2f(my, mz) - atan2f(s_by, s_bz)) * HH_RAD2DEG;
	if (droll > 180.0f) {
		droll -= 360.0f;
	} else if (droll < -180.0f) {
		droll += 360.0f;
	}

	if (!s_in) {
		/* The baseline follows slowly while nothing is going on -- and
		   only on fresh evidence (a reading left out above must not drag
		   it towards the last good one), and only while the car is near
		   level: once a wheel has started up a hump the baseline must
		   stay where it was, or it eats into the height being measured.
		   A real slope rises slowly enough to stay under HH_END_DEG
		   while the baseline catches up with it. */
		if (usable && tilt < HH_END_DEG) {
			a = (float)dt / (float)HH_BASE_TC_MS;
			if (a > 1.0f) {
				a = 1.0f;
			}
			set_base(s_bx + a * (mx - s_bx), s_by + a * (my - s_by),
				 s_bz + a * (mz - s_bz));
		}

		if (tilt >= HH_START_DEG) {
			s_in = true;
			s_t0 = now_ms;
			s_below = false;
			s_peak = tilt;
			s_peak_roll = droll;
			s_steep = (tilt > HH_MAX_TILT_DEG);
			s_long = false;
		}
		return false;
	}

	/* (4) a hump is going on */
	if (tilt > s_peak) {
		s_peak = tilt;
		s_peak_roll = droll;
	}
	if (tilt > HH_MAX_TILT_DEG) {
		s_steep = true;
	}

	if ((uint32_t)(now_ms - s_t0) > HH_MAX_MS) {
		/* Tilted far too long to be a hump driven over: the car was put
		   down on something, or is being held.  It will NOT be reported
		   -- but the event stays open, so that taking the car off again
		   (back to the old level) closes it quietly instead of looking
		   like a hump the other way. */
		s_long = true;
		if ((uint32_t)(now_ms - s_t0) > HH_RELEARN_MS) {
			/* Still there: this is the floor now (a slope), not a
			   hump.  Start again from here. */
			s_in = false;
			set_base(mx, my, mz);
			return false;
		}
	}

	if (tilt < HH_END_DEG) {
		if (!s_below) {
			s_below = true;
			s_below_t = now_ms;
		} else if ((uint32_t)(now_ms - s_below_t) >= HH_END_HOLD_MS) {
			uint32_t dur = (uint32_t)(s_below_t - s_t0);

			s_in = false;
			if (dur >= HH_MIN_MS && !s_steep && !s_long) {
				float roll_part = fabsf(s_peak_roll);

				out->tilt_deg = s_peak;
				out->height_mm = S4_HUMP_LEVER_MM *
						 sinf(s_peak / HH_RAD2DEG);
				out->dur_ms = dur;
				if (roll_part < 0.5f * s_peak) {
					out->side = 'C';	/* mostly nose/tail */
				} else {
					/* right side UP = roll negative */
					bool right = (s_peak_roll < 0.0f);
					if (S4_HUMP_SIDE_FLIP) {
						right = !right;
					}
					out->side = right ? 'R' : 'L';
				}
				done = true;
			}
		}
	} else {
		s_below = false;
	}
	return done;
}
