/*
 *	hump_height.h
 *	Subsystem 4 -- the HEIGHT of a hump that one wheel rolls over.
 *
 *	Plain C: no sensor, no kernel.  imu.c feeds it every accelerometer
 *	reading; testing/subsystem4/test_hump_height.c feeds it made-up ones.
 *
 *	HOW: when only ONE wheel rolls onto a hump, the other wheel and the caster
 *	stay on the floor, so the whole car tilts about the line joining those two
 *	contact points.  The accelerometer sees gravity lean over by that tilt
 *	angle, and the height follows from the geometry:
 *
 *	        h = S4_HUMP_LEVER_MM * sin(peak tilt)
 *
 *	S4_HUMP_LEVER_MM (D) is the distance from the climbing wheel's contact
 *	point to that pivot line: about the track width if the caster is far from
 *	the axle, a bit less if it is close.  At D = 120 mm, 1 degree is ~2 mm.
 *
 *	The tilt is the angle between the gravity direction now and a BASELINE --
 *	a slow average of it while nothing is happening -- so the sensor's mounting
 *	angle and any slope in the floor cancel out, and no GP21 calibration is
 *	needed.  Using the full angle between the two vectors, rather than roll
 *	alone, makes it right whatever direction the pivot line runs in.
 *
 *	The existing hump detector in imu.c measures the JOLT (g), which depends
 *	on speed and the hump's shape, not its height.  It is left as it is.
 */

#ifndef HUMP_HEIGHT_H
#define HUMP_HEIGHT_H

#include <stdbool.h>
#include <stdint.h>

/* ---------------------------------------------------------------------
 *  D, in mm.  CALIBRATE IT ONCE, with something of known height:
 *  slide e.g. a 10 mm stack of coins under the RIGHT wheel, leave it 2 s,
 *  take it away.  The console prints "imu: hump N mm ...".  Then
 *      new S4_HUMP_LEVER_MM = old * (real height / N)
 *  e.g. it says 12 for a 10 mm block: 130 * 10 / 12 = 108.
 *  130 is the track width (AVOID_TRACK_MM), right if the caster is far away.
 * --------------------------------------------------------------------- */
#ifndef S4_HUMP_LEVER_MM
#define S4_HUMP_LEVER_MM	130.0f
#endif

/* Which side rose comes from which way the car leans (roll, with imu.h's
   convention: +Y left, right side down = positive roll).  If the block test
   under the RIGHT wheel reports s=L, the board is mounted the other way
   round: set this to 1. */
#ifndef S4_HUMP_SIDE_FLIP
#define S4_HUMP_SIDE_FLIP	0
#endif

/* Tuning (the reasoning is in hump_height.c) */
#define HH_MEDIAN_N		9	/* readings in the median: imu_task reads
					   every 10 ms, so 90 ms of them */
#define HH_QUIET_G		0.25f	/* a reading counts as tilt only within
					   this of 1 g; further off it is a jolt.
					   Not tighter: driving shakes this car by
					   up to ~0.25 g RMS (lift_gate.h), and the
					   median deals with ordinary shaking */
#define HH_BASE_TC_MS		1000	/* baseline time constant */
#define HH_START_DEG		1.5f	/* tilt that starts a hump (~3 mm) */
#define HH_END_DEG		0.7f	/* ...and ends it, once below for: */
#define HH_END_HOLD_MS		150
#define HH_MIN_MS		60	/* shorter than this is a knock, not a hump */
#define HH_MAX_MS		10000	/* tilted longer than this is not a hump the
					   car drove over: not reported */
#define HH_RELEARN_MS		30000	/* ...and still tilted after this: take it
					   as the new level (a slope, not a hump) */
#define HH_MAX_TILT_DEG		30.0f	/* steeper than this is the car being
					   picked up or tipped, not a hump */

typedef struct {
	float		height_mm;	/* S4_HUMP_LEVER_MM * sin(tilt_deg) */
	float		tilt_deg;	/* the peak tilt from the baseline */
	char		side;		/* 'R' / 'L': the wheel that rose.
					   'C': the car tipped mostly nose/tail,
					   so it was not one wheel (the height is
					   then only a rough figure) */
	uint32_t	dur_ms;		/* how long the car was tilted */
} hh_event_t;

/* Forget everything, baseline included (the next HH_MEDIAN_N quiet readings
   set a new one). */
void hh_reset(void);

/* One accelerometer reading, in g, in the sensor's axes.  g_ref is the 1 g
   the sensor reads at rest.  now_ms is any millisecond clock (it may wrap).
 *
 * usable: FALSE while the car is turning, spinning, speeding up or braking.
 * An accelerometer cannot tell tilt from acceleration -- a pivot turn pushes
 * the car sideways by ~0.06 g, which reads as a 3-4 degree roll -- so those
 * readings are left out altogether (imu_task decides, from subsystem 2's
 * motion state).  The tilt then holds its last value.
 *
 * Returns true on the reading where a hump has just ENDED, with *out filled. */
bool hh_update(float ax, float ay, float az, float g_ref, bool usable,
	       uint32_t now_ms, hh_event_t *out);

/* The tilt from the baseline right now, degrees (0 before there is one). */
float hh_tilt_now(void);

#endif /* HUMP_HEIGHT_H */
