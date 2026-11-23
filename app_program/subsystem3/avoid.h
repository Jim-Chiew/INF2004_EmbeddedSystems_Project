/*
 * avoid.h
 * Subsystem 3 -- the plan for driving around an object, and the numbers it
 * needs. The arithmetic is here and in avoid.c, kept free of the kernel so it
 * can be checked on a PC (testing/subsystem3/test_avoid.c); task_nav.c does
 * the driving.
 *
 * The manoeuvre is a box round the object, and it ends where it began: on the
 * original line, pointing the way the car was already going.
 *
 *                  (5) drive along past it
 *        +-------------------------------->
 *        |                                |
 *   (3) turn along                    (6) turn in
 *   (4) LOOK: still in the way?       (7) LOOK: are we past it?
 *        |                                |
 *        ^         [ OBJECT ]             v
 *        |                                |
 *   (2) step aside                   (8) step back until the sensors
 *        |                               see the line, then on until
 *        |                               the AXLE is over it
 *        |                                |
 *        +--- (1) turn away          (9) turn to face the way we were going
 *             ^
 *         the car, stopped at the object
 *
 * THE SONAR NEVER SWIVELS DURING THIS.  It sits on a servo with limited
 * travel, and holding it hard over for the seconds a manoeuvre takes presses
 * it against its stop.  So the car points ITSELF at whatever it wants to look
 * at: both checks happen with the sonar facing straight ahead, after a turn
 * that aimed the whole car in that direction.
 *
 * The two checks are what make the DEPTH of the object unnecessary to know --
 * and it cannot be known, because the scan only ever measures the width:
 *
 *   (4) after stepping aside and turning back along the line: is the object
 *       still in the way?  If it is, the sidestep was not enough -- turn out,
 *       step AVOID_EXTRA_STEP_MM further, and ask again.
 *   (7) after driving along and turning in towards the line: is the object
 *       still there?  If it is, the car has not passed it yet -- turn back
 *       along, drive AVOID_EXTRA_ALONG_MM further, and ask again.
 *
 * So nothing is assumed about how deep the object is.  The car finds out by
 * looking, and AVOID_PASS_MM only decides how often it has to look twice.
 *
 * The turns aim at ABSOLUTE compass headings worked out from the one the car
 * came in on, never at "90 degrees from wherever I am now".  That matters:
 * dead-reckoned turns add their errors together, so by the fourth one the car
 * is pointing somewhere of its own invention, whereas an absolute target
 * cannot drift -- each turn corrects whatever the last one got wrong.  The
 * encoders only decide how far one attempt spins; the compass decides when to
 * stop.  Without the GP21 calibration there is no usable compass, and the
 * turns fall back to pure encoder dead reckoning, which is what
 * AVOID_TRACK_MM is for.
 *
 * Finding the line again: step (8) does not count out the distance stepped
 * aside.  It drives until the line sensors cross the line and then on until
 * the AXLE is over it, so the turn in (9) pivots on the line itself and the
 * line search after it only has to sweep a few degrees.  Turning anywhere
 * else leaves the car pointing ACROSS the line once a sensor finds it.  The
 * reason, and the fallback when the line is never seen, are in
 * avoid_return_to_line() in task_nav.c.
 */

#ifndef AVOID_H
#define AVOID_H

#include <stdbool.h>
#include <stdint.h>

#include "../subsystem6/ultrasonic.h"	/* avoid_side_t: AVOID_LEFT / AVOID_RIGHT */

/* ---------------------------------------------------------------------
 *  The car's own measurements.  THESE TWO MUST BE MEASURED on the car,
 *  with a ruler, or every turn and every sidestep will be wrong.
 *
 *  AVOID_TRACK_MM is the distance between the two wheels' contact
 *  patches, centre to centre.  A turn on the spot rolls each wheel along
 *  a circle of that diameter, so turning by A degrees moves each wheel
 *      arc = A / 360 * pi * AVOID_TRACK_MM
 *  which is what the encoders count out.
 *
 *  To calibrate: build with S3_AVOID_TURN_TEST = 1 (task_nav.c), which
 *  turns 90 degrees once and stops instead of running the manoeuvre.
 *  Mark the floor, run it, measure the angle actually turned, then
 *      new AVOID_TRACK_MM = old * (90 / angle you measured)
 *  Repeat until it lands within a couple of degrees.
 * --------------------------------------------------------------------- */
#define AVOID_TRACK_MM		130	/* wheel centre to wheel centre (MEASURE IT) */
#define AVOID_CAR_HALF_MM	60	/* centre line to the widest part (MEASURE IT) */

/* ---------------------------------------------------------------------
 *  SLIP.  The arc above is pure geometry: it says how far the wheels must
 *  roll for the car to turn by a given angle, assuming the tyres go
 *  exactly where they point.  In a turn on the spot they do not.  Both
 *  tyres are being dragged sideways across the floor the whole time, so
 *  the wheels turn further than the car does, and the encoders -- which
 *  only ever see the wheels -- think the turn is finished early.
 *
 *  Measured on this car: a spin asked for 90 degrees comes out about 40
 *  degrees short.  That is far too much to be a mis-measured track (the
 *  track would have to be 234 mm, which it plainly is not), so it is not
 *  something the geometry or the track learning can fix -- it is friction,
 *  and it gets its own allowance.
 *
 *  So the encoders are asked for this much extra on every spin.  The angle
 *  the turn is JUDGED by is still the real one, so the track learning is
 *  not fooled by the allowance and keeps working on top of it.
 *
 *  To tune: build with S3_AVOID_TURN_TEST = 1 (task_nav.c), which turns 90
 *  degrees once and stops.  Mark the floor, run it, measure the angle:
 *      new extra = old extra + (90 - the angle you measured)
 *  Carpet slips more than lino, so it is worth redoing on a new surface.
 * --------------------------------------------------------------------- */
#define AVOID_SPIN_EXTRA_DEG	40

/* ---------------------------------------------------------------------
 *  Encoders and compass together, neither trusted on its own.
 *
 *  The encoders turn the car: they are repeatable and they work with the
 *  motors running, but they only know the angle through AVOID_TRACK_MM,
 *  which is a measurement of the car and is never exactly right.
 *
 *  The compass knows the real angle, but only with the motors stopped
 *  (they are magnets a few centimetres away), only after the GP21
 *  calibration, and it can be pulled about by anything steel on the
 *  floor.
 *
 *  So each turn does three things: turn by the encoders, check with the
 *  compass, and then use the difference to improve the track width the
 *  encoders are working from.  After two or three turns the encoders are
 *  right by themselves, and if the compass then goes quiet -- or starts
 *  talking nonsense -- the turns stay good.
 *
 *  A compass reading is only believed if the turn it reports is in the
 *  band below: an encoder turn aimed at 90 degrees cannot really have
 *  come out at 20 or 170, so such a reading is the magnetometer being
 *  disturbed, not the car being lost.
 * --------------------------------------------------------------------- */
#define AVOID_TURN_TRUST_MIN_DEG	60
#define AVOID_TURN_TRUST_MAX_DEG	120
#define AVOID_TRACK_MIN_MM		70	/* a track estimate outside */
#define AVOID_TRACK_MAX_MM		300	/* these two is not credible */

/* ---------------------------------------------------------------------
 *  How much room to leave, and what to assume when the scan is no good.
 * --------------------------------------------------------------------- */

/* Used when subsystem 6's scan found nothing, or found something whose width
   is not believable.  The brief says to take the object as 20 cm wide. */
#define AVOID_ASSUMED_WIDTH_MM	200

/* A measured width outside these is not believed, and the assumption above is
   used instead.  The low end rejects a single ping that clipped a corner; the
   high end rejects a wall, which the car cannot go round anyway. */
#define AVOID_WIDTH_MIN_MM	30
#define AVOID_WIDTH_MAX_MM	400

/* Added to the half-width before stepping aside.  It covers what the scan
   gets wrong (it reads up to one 5 degree step NARROW, which at 15 cm is
   about 13 mm), the turns being imperfect, and simply not wanting to brush
   the thing.  Raised 70 -> 120 -> 170 after watching it go round, 5 cm at a
   time.  Only the FIRST sidestep uses this; the extra ones a failed clearance
   check asks for are AVOID_EXTRA_STEP_MM.  The way back drives the total of
   whatever was stepped, so out and back match however many steps it took. */
#define AVOID_CLEARANCE_MM	170

/* A clearance check passes when the nearest thing ahead is further off than
   this, or there is nothing in range at all.  Comfortably more than the
   S6_STOP_MM the car stopped at, so a box just cleared does not read as still
   being in the way, and comfortably less than the width of a room, so a far
   wall does not either. */
#define AVOID_CHECK_CLEAR_MM	300

/* Readings taken for one check, and the gap between them.  The CLOSEST of
   them decides: one ping can slip past a corner and come back with the far
   wall, and being over-cautious here only ever costs another sidestep. */
#define AVOID_LOOK_PINGS	5
#define AVOID_LOOK_GAP_MS	40

/* How much further to step aside when the check says the object is still in
   the way, and how much further along to drive when it says the car has not
   passed it yet.  ALONG raised 150 -> 220 -> 270 after watching the car turn
   in, still see the object, and have to go round the loop again: a bigger
   step costs nothing when it is not needed, because the clearance check is
   what decides when to stop and not the distance, whereas every wasted loop
   costs a turn in and a turn back out.  This is the only number that grows
   when the car keeps finding the object still beside it. */
#define AVOID_EXTRA_STEP_MM	100
#define AVOID_EXTRA_ALONG_MM	270

/* Tries allowed for each check before the car gives up and waits for a
   human.  Three means up to 300 mm of extra sidestep, or 450 mm of extra
   travel along the object, before it decides it is looking at a wall. */
#define AVOID_MAX_TRIES		3

/* The first go at driving along past the object, square to the heading the
   car came in on.  Tuned on the floor: 450 was too short -- the car nearly
   always turned in, still found the object beside it, and went round the
   retry loop -- then 1125 was further than needed, so 750.

   Getting this wrong cannot break the manoeuvre, only make it slow or wide:
   too short and check (7) sends the car round the loop, too long and it
   drives past more floor than it needed.  The clearance check decides when
   the car has passed the object, never this distance -- which is the whole
   reason the depth of the object never has to be known.

   It is still three quarters of a metre with nothing watching the sides, so
   the front guard (AVOID_FRONT_STOP_MM, task_nav.c) is what keeps it honest:
   anything that appears ahead stops the car and ends the manoeuvre safely. */
#define AVOID_PASS_MM		750

/* ---------------------------------------------------------------------
 *  Speeds, % of full power.  Gentler than line following: these moves are
 *  measured by the encoders, and a slow wheel overshoots less.
 * --------------------------------------------------------------------- */
#define AVOID_TURN_SPEED	30	/* each wheel, opposite directions */
#define AVOID_DRIVE_SPEED	20	/* both wheels, going forward */

/* ---------------------------------------------------------------------
 *  The plan, worked out once when the manoeuvre starts.
 * --------------------------------------------------------------------- */
typedef struct {
	avoid_side_t	side;		/* which way round the object */
	int32_t		width_mm;	/* the width used: measured or assumed */
	bool		assumed;	/* true = the scan was no good */
	int32_t		lateral_mm;	/* how far to step aside, and back again */
	int32_t		turn_arc_mm;	/* per-wheel arc of one 90 degree turn */
} avoid_plan_t;

/* The steps, in order.  nav publishes the current one so the status line and
   the console can say where the car has got to. */
typedef enum {
	AVOID_TURN_OUT = 0,	/* 1: turn 90 away from the object */
	AVOID_SIDESTEP,		/* 2: drive aside */
	AVOID_TURN_ALONG,	/* 3: turn back to the heading we came in on */
	AVOID_CHECK_AHEAD,	/* 4: sonar forward -- still in the way? */
	AVOID_PASS,		/* 5: drive along past it */
	AVOID_TURN_IN,		/* 6: turn 90 towards the line */
	AVOID_CHECK_PAST,	/* 7: sonar forward -- are we past it? */
	AVOID_RETURN,		/* 8: drive back until the axle is on the line */
	AVOID_TURN_AHEAD,	/* 9: turn to face the way we were going */
	AVOID_FINISHED
} avoid_step_t;

/* Work out the plan.  Pure: give it what the scan found (or did not).
 *   scan_found   subsystem 6 measured an object
 *   scan_width   its length across the path, mm
 *   scan_side    the side the scan said has more room
 * Everything the manoeuvre needs comes out in *out. */
void avoid_plan(bool scan_found, int32_t scan_width_mm, avoid_side_t scan_side,
		avoid_plan_t *out);

/* Per-wheel arc, in mm, for turning this many degrees on the spot, using the
   track width given.  avoid_turn_arc_mm() uses the one from this header; nav
   passes its own running estimate to the other. */
int32_t avoid_turn_arc_mm(int degrees);
int32_t avoid_turn_arc_track(int degrees, int32_t track_mm);

/* Improve the track width from a turn that asked for 90 degrees and came out
   as went_deg on the compass.  Returns the new estimate: a quarter of the way
   towards what this turn suggests, so one odd reading cannot wreck it, and
   track_now unchanged if the reading is not believable. */
int32_t avoid_track_update(int32_t track_now, int went_deg);

const char *avoid_step_name(avoid_step_t step);

#endif /* AVOID_H */
