/*
 *	subsystem6_test.c
 *	On-target tests for subsystem 6.  Built only with: make TEST=1
 *	Record results in testing/subsystem6/TEST_LOG.md.  IDs: S6-U01, ...
 *
 *	S6-U01..U19 are pure-logic checks: they never touch the sensor or the
 *	servo, because obstacle_task owns them (TEAM.md section 3).
 *	S6-U20..U22 watch that task through its published status.
 *	Accuracy and real detection are manual tests (TEST_LOG.md), with the
 *	road ahead of the car clear when the board is powered up.
 */

#include "subsystem6.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem6_test(void)
{
	OBS_STATUS	os;
	T_RTSK		rtsk;
	UW		p0, p1, t0, t1, rate;
	INT		i, waiting = 0;

	TEST_INIT();

	/* ---- the median-of-3 filter ---- */
	{ int32_t s[3] = { 100, 90, 5000 };
	  TEST_CHECK("S6-U01", us_filter3(s) == 100, "median rejects a single spike"); }
	{ int32_t s[3] = { US_INVALID, US_INVALID, US_INVALID };
	  TEST_CHECK("S6-U02", us_filter3(s) == US_INVALID, "all-invalid samples give US_INVALID"); }
	{ int32_t s[3] = { US_NO_ECHO, 120, US_NO_ECHO };
	  TEST_CHECK("S6-U03", us_filter3(s) == US_NO_ECHO, "no-echo counts as clear"); }
	{ int32_t s[3] = { 120, US_INVALID, 200 };
	  TEST_CHECK("S6-U04", us_filter3(s) == 120, "invalid dropped, lower middle value used"); }

	/* ---- servo angle -> pulse width ---- */
	TEST_CHECK("S6-U05",
		   servo_angle_to_pulse_us(-90) == SERVO_PULSE_MIN_US &&
		   servo_angle_to_pulse_us(90)  == SERVO_PULSE_MAX_US &&
		   servo_angle_to_pulse_us(0)   == (SERVO_PULSE_MIN_US + SERVO_PULSE_MAX_US) / 2,
		   "servo pulse: -90 = min, 0 = centre, +90 = max");
	TEST_CHECK("S6-U06",
		   servo_angle_to_pulse_us(200) == SERVO_PULSE_MAX_US &&
		   servo_angle_to_pulse_us(-200) == SERVO_PULSE_MIN_US,
		   "servo angle is clamped to +-90");

	/* ---- which side has more room ---- */
	TEST_CHECK("S6-U07", us_pick_side(300, 800) == AVOID_RIGHT, "more room on the right -> RIGHT");
	TEST_CHECK("S6-U08", us_pick_side(800, 300) == AVOID_LEFT,  "more room on the left -> LEFT");
	TEST_CHECK("S6-U09", us_pick_side(300, 320) == AVOID_LEFT,  "tie within 3 cm -> LEFT");

	/* ---- the fault counter: an unplugged sensor must not read as "clear" ---- */
	{ int bad = 0; bool f = false;
	  for (i = 0; i < US_FAULT_INVALID_CNT - 1; i++) f = us_fault_update(&bad, US_INVALID);
	  TEST_CHECK("S6-U10", !f, "9 invalid pings: no fault yet");
	  f = us_fault_update(&bad, US_INVALID);
	  TEST_CHECK("S6-U11", f, "the 10th invalid ping in a row raises SENSOR_FAULT"); }
	{ int bad = 0;
	  for (i = 0; i < 9; i++) (void)us_fault_update(&bad, US_INVALID);
	  (void)us_fault_update(&bad, 300);
	  TEST_CHECK("S6-U12", bad == 0, "a valid ping resets the invalid counter"); }
	{ int bad = 0;
	  for (i = 0; i < 9; i++) (void)us_fault_update(&bad, US_INVALID);
	  (void)us_fault_update(&bad, US_NO_ECHO);
	  TEST_CHECK("S6-U13", bad == 0, "open-space NO_ECHO is valid and resets the counter"); }

	/* ---- the coarse scan: pure classification and config sanity ---- */
	{ static const int8_t d[] = SCAN_COARSE_DEG;
	  bool ok = (sizeof(d) / sizeof(d[0]) == US_COARSE_N);
	  for (i = 0; ok && i < US_COARSE_N; i++) {
		if (d[i] < -90 || d[i] > 90) ok = false;
		if (i > 0 && d[i] <= d[i-1]) ok = false;
	  }
	  TEST_CHECK("S6-U14", ok, "coarse angles: 5 entries, ascending, within +-90"); }
	{ us_coarse_t c = { .mm = { US_NO_ECHO, US_NO_ECHO, US_NO_ECHO, US_NO_ECHO, US_NO_ECHO } };
	  us_coarse_classify(&c, SCAN_OBSTACLE_RANGE_MM);
	  TEST_CHECK("S6-U15", c.blocked_mask == 0 && c.invalid_mask == 0 && c.closest_idx == -1,
		     "all clear: nothing blocked, no closest"); }
	{ us_coarse_t c = { .mm = { US_NO_ECHO, 200, 150, US_NO_ECHO, US_NO_ECHO } };
	  us_coarse_classify(&c, SCAN_OBSTACLE_RANGE_MM);
	  TEST_CHECK("S6-U16", c.blocked_mask == 0x06 && c.closest_idx == 2,
		     "blocked mask bits 1,2; closest is index 2 (150 mm)"); }
	{ us_coarse_t c = { .mm = { US_NO_ECHO, US_NO_ECHO, US_INVALID, US_NO_ECHO, US_NO_ECHO } };
	  us_coarse_classify(&c, SCAN_OBSTACLE_RANGE_MM);
	  TEST_CHECK("S6-U17", c.invalid_mask == 0x04 && c.blocked_mask == 0 && c.closest_idx == -1,
		     "an invalid sample is flagged, never counted as clear or blocked"); }
	{ us_coarse_t c = { .mm = { SCAN_OBSTACLE_RANGE_MM, SCAN_OBSTACLE_RANGE_MM + 1,
				    US_NO_ECHO, US_NO_ECHO, US_NO_ECHO } };
	  us_coarse_classify(&c, SCAN_OBSTACLE_RANGE_MM);
	  TEST_CHECK("S6-U18", c.blocked_mask == 0x01, "the range limit is inclusive (<=)"); }

	/* The settle waits plus one ping per angle must fit the scan budget.
	   Pure arithmetic on the config, so it runs without the servo; the real
	   scan time is measured in S6-M05 (and printed by every scan report). */
	TEST_CHECK("S6-U19",
		   SCAN_COARSE_SETTLE_TOTAL_MS +
		   (US_COARSE_N * SCAN_COARSE_PING_ALLOW_MS) <= SCAN_COARSE_BUDGET_MS,
		   "coarse scan settle + ping time fits the 800 ms budget");

	/* ---- stage 2: measuring the object (pure geometry) ---- */

	/* The settle waits, one ping per point and the recentre must fit the fine
	   sweep's budget.  Arithmetic on the config, like S6-U19. */
	TEST_CHECK("S6-U23",
		   SCAN_FINE_SETTLE_TOTAL_MS +
		   (SCAN_FINE_POINTS * SCAN_COARSE_PING_ALLOW_MS) <= SCAN_FINE_BUDGET_MS,
		   "fine sweep settle + ping time fits its budget");

	/* A flat board straight ahead at 200 mm, seen from -10 to +10 deg. The
	   chord between the edge points is 2 * 200 * sin(10 deg) = 69 mm. */
	{
		static const int8_t  d[5] = { -20, -10, 0, 10, 20 };
		static const int32_t m[5] = { US_NO_ECHO, 203, 200, 203, US_NO_ECHO };
		us_object_t o;

		us_object_from_samples(&o, d, m, 5, SCAN_OBSTACLE_RANGE_MM);
		TEST_CHECK("S6-U24",
			   o.found && o.hits == 3 && o.near_mm == 200 &&
			   o.left_deg == 10 && o.right_deg == -10 && o.centre_deg == 0 &&
			   o.length_mm >= 60 && o.length_mm <= 80,
			   "a board ahead: 3 points, -10..+10 deg, about 70 mm long");
		tm_printf((UB*)"[TEST] S6-U24 info: %d mm long, %d mm after the beam "
			  "allowance\n", (INT)o.length_mm, (INT)o.length_corr_mm);
	}

	/* Nothing in range anywhere in the window */
	{
		static const int8_t  d[3] = { -10, 0, 10 };
		static const int32_t m[3] = { US_NO_ECHO, US_NO_ECHO, US_INVALID };
		us_object_t o;

		us_object_from_samples(&o, d, m, 3, SCAN_OBSTACLE_RANGE_MM);
		TEST_CHECK("S6-U25", !o.found && o.length_mm == 0,
			   "an empty window measures nothing, not a zero-length object");
	}

	/* One hit only: the object is narrower than a single step, so its length
	   is not measurable (0) -- but it IS found, so it cannot be ignored. */
	{
		static const int8_t  d[3] = { -5, 0, 5 };
		static const int32_t m[3] = { US_NO_ECHO, 150, US_NO_ECHO };
		us_object_t o;

		us_object_from_samples(&o, d, m, 3, SCAN_OBSTACLE_RANGE_MM);
		TEST_CHECK("S6-U26", o.found && o.hits == 1 && o.length_mm == 0 &&
				     o.span_deg == 0 && o.centre_deg == 0,
			   "a single hit is found but too narrow to measure");
	}

	/* A box in front of a wall: the step out to the wall is where the object
	   ends, so the two must not be measured as one. */
	{
		static const int8_t  d[5] = { -20, -10, 0, 10, 20 };
		static const int32_t m[5] = { 300, 300, 150, 155, 300 };
		us_object_t o;

		us_object_from_samples(&o, d, m, 5, SCAN_OBSTACLE_RANGE_MM);
		TEST_CHECK("S6-U27",
			   o.found && o.hits == 2 && o.near_mm == 150 &&
			   o.right_deg == 0 && o.left_deg == 10,
			   "the step out to the wall behind ends the object");
	}

	/* An object off to one side is measured at its own angles */
	{
		static const int8_t  d[5] = { -60, -55, -50, -45, -40 };
		static const int32_t m[5] = { US_NO_ECHO, 220, 210, 215, US_NO_ECHO };
		us_object_t o;

		us_object_from_samples(&o, d, m, 5, SCAN_OBSTACLE_RANGE_MM);
		TEST_CHECK("S6-U28",
			   o.found && o.hits == 3 && o.right_deg == -55 &&
			   o.left_deg == -45 && o.centre_deg == -50 && o.length_mm > 0,
			   "an object off to the right is measured at its own angles");
	}

	/* ---- the running task ---- */

	/* S6-U20: it really is pinging, at about 1000 / S6_PING_MS Hz */
	obstacle_get_status(&os);
	p0 = os.pings;
	t0 = app_now_ms();
	tk_dly_tsk(500);
	obstacle_get_status(&os);
	p1 = os.pings;
	t1 = app_now_ms();
	rate = (t1 > t0) ? (p1 - p0) * 1000u / (t1 - t0) : 0u;
	/* The upper bound allows one extra tick inside the 500 ms window, and
	   the lower one allows a ping that waits out the full echo timeout. */
	TEST_CHECK("S6-U20", rate >= (1000u / S6_PING_MS) / 2u &&
			     rate <= (1000u / S6_PING_MS) + 5u,
		   "obstacle_task pings at about the asked rate");
	tm_printf((UB*)"[TEST] S6-U20 info: %d pings/s (asked %d), state %s, %d mm\n",
		  (INT)rate, (INT)(1000u / S6_PING_MS), obs_state_name(os.state),
		  (INT)os.dist_mm);

	/* S6-U21: it sleeps between pings instead of spinning */
	for (i = 0; i < 10; i++) {
		tk_dly_tsk(3);
		if (tk_ref_tsk(tid_obstacle, &rtsk) == E_OK && (rtsk.tskstat & TTS_WAI)) {
			waiting++;
		}
	}
	TEST_CHECK("S6-U21", waiting >= 8, "obstacle_task sleeps between pings");

	/* S6-U22: the sensor answers, and a distance is either a real reading
	   or "nothing in range" -- never the unknown value */
	obstacle_get_status(&os);
	TEST_CHECK("S6-U22", os.ok && os.dist_mm != US_INVALID,
		   "the sensor answers (check the wiring if this fails)");

	TEST_SUMMARY("subsystem6");
}

#endif /* APP_TEST */
