/*
 * avoid.c
 * Subsystem 3 -- the arithmetic behind driving round an object. See avoid.h.
 * Plain C, no kernel and no hardware, so it can be checked on a PC.
 */

#include "avoid.h"

/* 3.14159265 as a fraction, so the arc comes out in whole mm without floats:
 * 355/113 is pi to seven digits, which is far closer than the track width is
 * ever going to be measured. */
#define PI_NUM	355
#define PI_DEN	113

int32_t avoid_turn_arc_track(int degrees, int32_t track_mm)
{
	int32_t a = (degrees < 0) ? -degrees : degrees;

	/* arc = degrees/360 * pi * track */
	return (int32_t)(((int64_t)a * PI_NUM * track_mm) /
			 ((int64_t)PI_DEN * 360));
}

int32_t avoid_turn_arc_mm(int degrees)
{
	return avoid_turn_arc_track(degrees, AVOID_TRACK_MM);
}

int32_t avoid_track_update(int32_t track_now, int went_deg)
{
	int32_t a = (went_deg < 0) ? -went_deg : went_deg;
	int32_t suggest;

	if (a < AVOID_TURN_TRUST_MIN_DEG || a > AVOID_TURN_TRUST_MAX_DEG) {
		return track_now;	/* the compass is being disturbed */
	}

	/* It turned 'a' degrees on an arc meant for 90, so the real track is
	   bigger (or smaller) in that proportion. */
	suggest = (int32_t)(((int64_t)track_now * 90) / a);
	if (suggest < AVOID_TRACK_MIN_MM) {
		suggest = AVOID_TRACK_MIN_MM;
	}
	if (suggest > AVOID_TRACK_MAX_MM) {
		suggest = AVOID_TRACK_MAX_MM;
	}
	return (track_now * 3 + suggest) / 4;
}

void avoid_plan(bool scan_found, int32_t scan_width_mm, avoid_side_t scan_side,
		avoid_plan_t *out)
{
	bool believable = scan_found &&
			  (scan_width_mm >= AVOID_WIDTH_MIN_MM) &&
			  (scan_width_mm <= AVOID_WIDTH_MAX_MM);

	out->side = scan_side;
	out->assumed = !believable;
	out->width_mm = believable ? scan_width_mm : AVOID_ASSUMED_WIDTH_MM;

	/* Step far enough aside that the widest part of the car clears the
	 * widest part of the object, plus a margin.  The object is taken as
	 * sitting across the line the car was following, so half of it lies on
	 * the side being passed. */
	out->lateral_mm = out->width_mm / 2 + AVOID_CAR_HALF_MM +
			  AVOID_CLEARANCE_MM;

	out->turn_arc_mm = avoid_turn_arc_mm(90);
}

const char *avoid_step_name(avoid_step_t step)
{
	switch (step) {
	case AVOID_TURN_OUT:	return "turn away from it";
	case AVOID_SIDESTEP:	return "step aside";
	case AVOID_TURN_ALONG:	return "turn along the line";
	case AVOID_CHECK_AHEAD:	return "look: is it still in the way?";
	case AVOID_PASS:	return "drive past it";
	case AVOID_TURN_IN:	return "turn in towards the line";
	case AVOID_CHECK_PAST:	return "look: are we past it?";
	case AVOID_RETURN:	return "step back to the line";
	case AVOID_TURN_AHEAD:	return "face the way we were going";
	default:		return "finished";
	}
}
