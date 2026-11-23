/*
 *	lift_gate.h
 *	Subsystem 4 -- the two thresholds the lift detector has to clear, worked
 *	out from how hard the car is shaking.
 *
 *	Why this is its own file: it is the arithmetic that decides whether the
 *	car e-stops in the middle of a run, it has no sensor and no kernel in
 *	it, and it was wrong once.  Kept plain C it can be checked on a PC --
 *	testing/subsystem4/test_lift_gate.c -- instead of by picking the car up
 *	and hoping.  imu.c does the integrating and calls these.
 *
 *	THE PROBLEM THEY SOLVE
 *
 *	The lift detector watches vertical acceleration, integrates it, and
 *	calls a rise of IMU_LIFT_HEIGHT_CM at IMU_LIFT_MIN_SPEED a lift.  Those
 *	numbers describe a car AT REST.  Driving this car measures 0.25 g RMS of
 *	vertical shake -- twice IMU_LIFT_TRIGGER_G -- so against fixed numbers
 *	the detector is working below its own noise floor.  It then integrates
 *	the noise and reports a lift.  A real run logged
 *
 *	    vib 0.254g lift - 0.0cm (last 19.9cm 0.43m/s) LIFTED ... ESTOP
 *
 *	with the car flat on the floor from start to finish.  That e-stopped the
 *	motors half way round an obstacle and ended the run.
 *
 *	THE FIX: measure the lift against the shaking, not against a constant.
 *	imu.c already measures the shaking and published it as vib_g; it was
 *	only ever displayed.  Now it decides.
 *
 *	Both gates fall back to the fixed numbers when the car is quiet, so
 *	picking up a stationary car behaves exactly as it did before.
 *
 *	WHAT THIS GIVES UP, stated plainly: a car shaking at 0.25 g cannot have
 *	an 8 cm rise measured out of it, so a SLOW lift off a car that is
 *	driving will no longer be caught.  A brisk one still is (see the test),
 *	and a lift off a standing car -- which is how anyone actually picks it
 *	up -- is unaffected.  The alternative is a car that stops by itself
 *	several times a run, which is worse and was the bug.
 */

#ifndef LIFT_GATE_H
#define LIFT_GATE_H

/* The upward push must beat this many times the RMS shake before a rise is
   followed at all.  Three RMS is a push, not a bump. */
#define LIFT_VIB_TRIGGER	3.0f

/* Over a rise lasting t seconds, shaking of one RMS could by itself integrate
   to rms * g * t of bogus speed, so a real lift has to be faster than that.
   One is the whole of that bound; there is no margin to add, because the bound
   IS the margin. */
#define LIFT_VIB_SPEED		1.0f

#define LIFT_GATE_G		9.81f	/* m/s per s per g */

/* The upward acceleration (g) needed to start following a rise.
   vib_g is the RMS vertical shake now; fixed_g is IMU_LIFT_TRIGGER_G. */
float lift_gate_trigger_g(float vib_g, float fixed_g);

/* The top speed (m/s) a rise must have reached to count as a lift.
   rise_s is how long the rise has been going; fixed_speed is
   IMU_LIFT_MIN_SPEED.  Returns the larger of that and the drift bound. */
float lift_gate_min_speed(float vib_g, float rise_s, float fixed_speed);

/* "Nothing is pushing any more" cannot be read below the shake itself, so the
   coast test scales with it too: without this, |accel| never dips under a
   fixed 0.05 g on a driving car and that escape never fires. */
float lift_gate_coast_g(float vib_g, float fixed_g);

#endif /* LIFT_GATE_H */
