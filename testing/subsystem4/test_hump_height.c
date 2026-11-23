/* test_hump_height.c -- PC test for the hump HEIGHT (S4-H01..).
 *
 * Feeds hump_height.c accelerometer readings made up the way the car would
 * produce them: one every 10 ms (imu_task's rate), a new value only every
 * 20 ms (the LSM303D runs at 50 Hz, so each one is read twice), the gravity
 * direction tilted by a known angle, plus the things that go wrong on the
 * floor -- a sensor mounted crooked, driving vibration, the jolt of the wheel
 * hitting the hump, steering, knocks, the car being picked up.
 *
 * A hump of h mm under one wheel tilts the car by asin(h / D), D being
 * S4_HUMP_LEVER_MM, so each case states the height and checks what comes back.
 *
 * Build and run from the repo root:
 *   sh testing/subsystem4/run_pc_tests.sh
 * or by hand:
 *   gcc -std=c11 -Wall -Wextra -I app_program/subsystem4 \
 *       testing/subsystem4/test_hump_height.c app_program/subsystem4/hump_height.c \
 *       -lm -o /tmp/t && /tmp/t
 */

#include <math.h>
#include <stdio.h>

#include "hump_height.h"

static int g_fail = 0;
#define CHECK(id, cond, desc) do { if (cond) printf("[TEST] %s PASS  %s\n", id, desc); \
    else { printf("[TEST] %s FAIL  %s (line %d)\n", id, desc, __LINE__); g_fail++; } } while (0)

#define PI 3.14159265f
#define D2R (PI / 180.0f)

/* ---- the simulated car ---- */
static float    b0[3] = {0.0f, 0.0f, 1.0f};	/* "up" as the sensor sees it, level */
static uint32_t t_ms = 1000u;
static int      n_events = 0;
static hh_event_t last_ev;

static float    sd_xy = 0.0f, sd_z = 0.0f;	/* vibration, g */
static float    spike_z = 0.0f;			/* a one-off jolt on the next reading */
static unsigned rng = 12345u;
static float    nx_, ny_, nz_;			/* noise held for two reads (50 Hz) */
static unsigned tick = 0u;

static float urand(void)		/* 0..1 */
{
	rng = rng * 1103515245u + 12345u;
	return (float)((rng >> 8) & 0xFFFF) / 65535.0f;
}

static float nrand(void)		/* roughly N(0,1) */
{
	return (urand() + urand() + urand() + urand() - 2.0f) * 1.732f;
}

static void set_mount(float pitch_deg, float roll_deg)
{
	/* the board fitted slightly crooked: the car is level, the sensor not */
	float x = sinf(pitch_deg * D2R), y = sinf(roll_deg * D2R);
	float z = sqrtf(1.0f - x * x - y * y);
	b0[0] = x; b0[1] = y; b0[2] = z;
}

/* One read.  tilt_deg towards the horizontal direction (ux, uy). */
static void read1(float tilt_deg, float ux, float uy, int usable)
{
	float w[3] = {ux, uy, 0.0f}, d, n, acc[3];
	hh_event_t ev;
	int i;

	/* direction to tilt towards, made square to b0 */
	d = w[0] * b0[0] + w[1] * b0[1] + w[2] * b0[2];
	for (i = 0; i < 3; i++) {
		w[i] -= d * b0[i];
	}
	n = sqrtf(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
	for (i = 0; i < 3; i++) {
		w[i] /= n;
		acc[i] = cosf(tilt_deg * D2R) * b0[i] + sinf(tilt_deg * D2R) * w[i];
	}
	if ((tick++ & 1u) == 0u) {		/* a new sample every 20 ms */
		nx_ = sd_xy * nrand();
		ny_ = sd_xy * nrand();
		nz_ = sd_z * nrand();
	}
	acc[0] += nx_;
	acc[1] += ny_;
	acc[2] += nz_ + spike_z;
	spike_z = 0.0f;

	if (hh_update(acc[0], acc[1], acc[2], 1.0f, usable != 0, t_ms, &ev)) {
		n_events++;
		last_ev = ev;
	}
	t_ms += 10u;
}

static void hold(int ms, float tilt, float ux, float uy, int usable)
{
	for (int i = 0; i < ms / 10; i++) {
		read1(tilt, ux, uy, usable);
	}
}

static void ramp(int ms, float from, float to, float ux, float uy)
{
	int n = ms / 10;
	for (int i = 1; i <= n; i++) {
		read1(from + (to - from) * (float)i / (float)n, ux, uy, 1);
	}
}

static float tilt_for(float h_mm)
{
	return asinf(h_mm / S4_HUMP_LEVER_MM) / D2R;
}

/* The wheel climbs (150 ms), rides the top (200 ms), comes down (150 ms) */
static void hump(float h_mm, float ux, float uy)
{
	float th = tilt_for(h_mm);
	ramp(150, 0.0f, th, ux, uy);
	hold(200, th, ux, uy, 1);
	ramp(150, th, 0.0f, ux, uy);
	hold(400, 0.0f, ux, uy, 1);
}

/* A fresh, level car standing still long enough for the baseline */
static void fresh(void)
{
	hh_reset();
	set_mount(0.0f, 0.0f);
	sd_xy = sd_z = 0.0f;
	n_events = 0;
	hold(1500, 0.0f, 0.0f, 1.0f, 1);
}

/* Directions, sensor axes (imu.h: +X forward, +Y left).  The RIGHT wheel
   going up leans the car to the left... and gravity, as the sensor sees it,
   towards -Y (roll negative, imu.h's "right side down = positive"). */
#define RIGHT_UP	0.0f, -1.0f
#define LEFT_UP		0.0f,  1.0f
#define NOSE_UP		-1.0f, 0.0f

int main(void)
{
	char msg[96];
	float g;

	/* S4-H01: the formula itself, no noise */
	fresh();
	hump(10.0f, RIGHT_UP);
	CHECK("S4-H01", n_events == 1 && fabsf(last_ev.height_mm - 10.0f) < 0.2f,
	      "10 mm under the right wheel, clean: reads 10 mm");
	CHECK("S4-H02", n_events == 1 && last_ev.side == 'R', "...and says RIGHT");
	snprintf(msg, sizeof msg, "...peak tilt %.2f deg (asin(10/%.0f) = %.2f)",
		 last_ev.tilt_deg, S4_HUMP_LEVER_MM, tilt_for(10.0f));
	CHECK("S4-H03", fabsf(last_ev.tilt_deg - tilt_for(10.0f)) < 0.1f, msg);

	/* S4-H04: the other wheel */
	fresh();
	hump(25.0f, LEFT_UP);
	CHECK("S4-H04", n_events == 1 && fabsf(last_ev.height_mm - 25.0f) < 0.3f &&
	      last_ev.side == 'L', "25 mm under the left wheel: 25 mm, LEFT");

	/* S4-H05: the pivot line runs diagonally (the caster is behind the
	   axle), so the car rolls AND pitches -- the full angle is still right */
	fresh();
	hump(15.0f, -sinf(35.0f * D2R), -cosf(35.0f * D2R));
	CHECK("S4-H05", n_events == 1 && fabsf(last_ev.height_mm - 15.0f) < 0.3f &&
	      last_ev.side == 'R', "diagonal pivot line (35 deg): still 15 mm, RIGHT");

	/* S4-H06: the board is fitted crooked; the baseline takes it out */
	hh_reset();
	set_mount(4.0f, -3.0f);
	sd_xy = sd_z = 0.0f;
	n_events = 0;
	hold(1500, 0.0f, 0.0f, 1.0f, 1);
	hump(12.0f, RIGHT_UP);
	CHECK("S4-H06", n_events == 1 && fabsf(last_ev.height_mm - 12.0f) < 0.3f &&
	      last_ev.side == 'R', "sensor mounted 4 deg / -3 deg off: still 12 mm");
	set_mount(0.0f, 0.0f);

	/* S4-H07: driving -- vibration, and the jolt as the wheel hits the hump */
	fresh();
	sd_xy = 0.01f;
	sd_z = 0.15f;
	hold(1500, 0.0f, 0.0f, 1.0f, 1);
	spike_z = 0.6f;
	ramp(150, 0.0f, tilt_for(10.0f), RIGHT_UP);
	spike_z = -0.5f;
	hold(200, tilt_for(10.0f), RIGHT_UP, 1);
	spike_z = 0.7f;
	ramp(150, tilt_for(10.0f), 0.0f, RIGHT_UP);
	hold(600, 0.0f, RIGHT_UP, 1);
	snprintf(msg, sizeof msg, "driving vibration + jolts: 10 mm reads %.1f mm",
		 n_events ? last_ev.height_mm : -1.0f);
	CHECK("S4-H07", n_events == 1 && fabsf(last_ev.height_mm - 10.0f) < 1.5f &&
	      last_ev.side == 'R', msg);
	sd_xy = sd_z = 0.0f;

	/* S4-H08: a knock -- tilted for 30 ms -- is not a hump */
	fresh();
	hold(30, 5.0f, RIGHT_UP, 1);
	hold(600, 0.0f, RIGHT_UP, 1);
	CHECK("S4-H08", n_events == 0, "a 30 ms knock is not a hump");

	/* S4-H09: small rocking below the start threshold */
	fresh();
	for (int i = 0; i < 10; i++) {
		hold(200, 0.8f, RIGHT_UP, 1);
		hold(200, 0.0f, RIGHT_UP, 1);
	}
	CHECK("S4-H09", n_events == 0, "rocking by 0.8 deg is not a hump");

	/* S4-H10: steering.  A pivot turn reads as a 3.5 deg roll -- but those
	   readings are marked unusable, so nothing happens; and a real hump
	   straight after is still measured */
	fresh();
	hold(300, 3.5f, RIGHT_UP, 0);
	hold(300, 0.0f, RIGHT_UP, 1);
	CHECK("S4-H10", n_events == 0, "a pivot turn (readings not usable) is not a hump");
	hump(10.0f, RIGHT_UP);
	CHECK("S4-H11", n_events == 1 && fabsf(last_ev.height_mm - 10.0f) < 0.3f,
	      "...and a real hump straight after it still reads 10 mm");

	/* S4-H12: picked up / tipped past HH_MAX_TILT_DEG */
	fresh();
	ramp(200, 0.0f, 40.0f, RIGHT_UP);
	hold(500, 40.0f, RIGHT_UP, 1);
	ramp(200, 40.0f, 0.0f, RIGHT_UP);
	hold(600, 0.0f, RIGHT_UP, 1);
	CHECK("S4-H12", n_events == 0, "tipped to 40 deg is not a hump");

	/* S4-H13: left standing on a block for 12 s, then taken off */
	fresh();
	ramp(100, 0.0f, tilt_for(10.0f), RIGHT_UP);
	hold(12000, tilt_for(10.0f), RIGHT_UP, 1);
	ramp(100, tilt_for(10.0f), 0.0f, RIGHT_UP);
	hold(1000, 0.0f, RIGHT_UP, 1);
	CHECK("S4-H13", n_events == 0,
	      "standing on a block 12 s, then off: no hump either way");

	/* S4-H14: the block test, as done to calibrate D: 2 s, then off */
	fresh();
	ramp(100, 0.0f, tilt_for(10.0f), RIGHT_UP);
	hold(2000, tilt_for(10.0f), RIGHT_UP, 1);
	ramp(100, tilt_for(10.0f), 0.0f, RIGHT_UP);
	hold(600, 0.0f, RIGHT_UP, 1);
	CHECK("S4-H14", n_events == 1 && fabsf(last_ev.height_mm - 10.0f) < 0.3f &&
	      last_ev.side == 'R', "block test: 10 mm block for 2 s reads 10 mm, RIGHT");

	/* S4-H15: two humps in a row */
	fresh();
	hump(8.0f, RIGHT_UP);
	g = last_ev.height_mm;
	hump(20.0f, RIGHT_UP);
	CHECK("S4-H15", n_events == 2 && fabsf(g - 8.0f) < 0.3f &&
	      fabsf(last_ev.height_mm - 20.0f) < 0.3f, "two humps: 8 mm then 20 mm");

	/* S4-H16: nose / tail only -- not one wheel */
	fresh();
	hump(10.0f, NOSE_UP);
	CHECK("S4-H16", n_events == 1 && last_ev.side == 'C',
	      "the car pitching (not one wheel) is marked C");

	/* S4-H17: a gentle floor slope (2 deg over 10 s) is followed, not a hump */
	fresh();
	for (int i = 1; i <= 1000; i++) {
		read1(2.0f * (float)i / 1000.0f, RIGHT_UP, 1);
	}
	hold(2000, 2.0f, RIGHT_UP, 1);
	CHECK("S4-H17", n_events == 0, "a gentle slope is not a hump");

	/* S4-H18: the start threshold is ~3 mm, so a 2 mm bump is ignored */
	fresh();
	hump(2.0f, RIGHT_UP);
	CHECK("S4-H18", n_events == 0, "2 mm (under HH_START_DEG) is not reported");

	printf("%s\n", g_fail ? "SOME TESTS FAILED" : "ALL PASSED");
	return g_fail ? 1 : 0;
}
