/*
 *	subsystem5_test.c
 *	On-target tests for subsystem 5.  Built only with: make TEST=1
 *	Record results in testing/subsystem5/TEST_LOG.md.  IDs: S5-U01, ...
 *
 *	These are pure-logic checks only.  They never touch the sensor or the
 *	servo, because the s5_obstacle task owns them (TEAM.md section 3).
 *	Accuracy, detection and scan behaviour are manual tests (TEST_LOG.md).
 */

#include "subsystem5.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem5_test(void)
{
	TEST_INIT();

	/* median-of-3 */
	{ int32_t s[3] = { 100, 90, 5000 };
	  TEST_CHECK("S5-U01", us_filter3(s) == 100, "median rejects a single spike"); }
	{ int32_t s[3] = { US_INVALID, US_INVALID, US_INVALID };
	  TEST_CHECK("S5-U02", us_filter3(s) == US_INVALID, "all-invalid samples give US_INVALID"); }
	{ int32_t s[3] = { US_NO_ECHO, 120, US_NO_ECHO };
	  TEST_CHECK("S5-U03", us_filter3(s) == US_NO_ECHO, "no-echo counts as clear"); }
	{ int32_t s[3] = { 120, US_INVALID, 200 };
	  TEST_CHECK("S5-U04", us_filter3(s) == 120, "invalid dropped, lower middle value used"); }

	/* servo angle -> pulse width */
	TEST_CHECK("S5-U05",
		   servo_angle_to_pulse_us(-90) == SERVO_PULSE_MIN_US &&
		   servo_angle_to_pulse_us(90)  == SERVO_PULSE_MAX_US &&
		   servo_angle_to_pulse_us(0)   == (SERVO_PULSE_MIN_US + SERVO_PULSE_MAX_US) / 2,
		   "servo pulse: -90 = min, 0 = centre, +90 = max");
	TEST_CHECK("S5-U06",
		   servo_angle_to_pulse_us(200) == SERVO_PULSE_MAX_US &&
		   servo_angle_to_pulse_us(-200) == SERVO_PULSE_MIN_US,
		   "servo angle is clamped to +-90");

	/* side selection */
	TEST_CHECK("S5-U07", us_pick_side(300, 800) == AVOID_RIGHT, "more room on the right -> RIGHT");
	TEST_CHECK("S5-U08", us_pick_side(800, 300) == AVOID_LEFT,  "more room on the left -> LEFT");
	TEST_CHECK("S5-U09", us_pick_side(300, 320) == AVOID_LEFT,  "tie within 3 cm -> LEFT");

	TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */
