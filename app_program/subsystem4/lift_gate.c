/*
 *	lift_gate.c
 *	Subsystem 4 -- see lift_gate.h for what these are for and why.
 *	No sensor, no kernel, no floating-point surprises: three max()s.
 */

#include "lift_gate.h"

/* Negative or silly inputs cannot be allowed to LOWER a gate, or a bad
   vibration estimate would make the detector more trigger-happy instead of
   less.  So every gate is at least its fixed value. */

float lift_gate_trigger_g(float vib_g, float fixed_g)
{
	float scaled;

	if (vib_g <= 0.0f) {
		return fixed_g;
	}
	scaled = LIFT_VIB_TRIGGER * vib_g;
	return (scaled > fixed_g) ? scaled : fixed_g;
}

float lift_gate_min_speed(float vib_g, float rise_s, float fixed_speed)
{
	float drift;

	if (vib_g <= 0.0f || rise_s <= 0.0f) {
		return fixed_speed;
	}
	drift = LIFT_VIB_SPEED * vib_g * LIFT_GATE_G * rise_s;
	return (drift > fixed_speed) ? drift : fixed_speed;
}

float lift_gate_coast_g(float vib_g, float fixed_g)
{
	if (vib_g <= 0.0f) {
		return fixed_g;
	}
	return (vib_g > fixed_g) ? vib_g : fixed_g;
}
