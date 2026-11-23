/*
 *	task_nav.c
 *	Subsystem 3 -- nav_task: the run state machine and the line follower.
 *
 *	APIs shown: tk_rcv_mbf(TMO_POL), tk_wai_flg(TMO_POL), tk_ref_flg,
 *	            tk_clr_flg, tk_dly_tsk
 *
 *	Control (buttons read LOW when pressed, remote commands from S1):
 *	  GP20            ONE BUTTON, TWO MEANINGS, AND NOTHING ELSE:
 *	                    stopped -> clear every latched flag, whatever it
 *	                               is, and go straight into line following
 *	                    running -> stop the car, from ANY state --
 *	                               following, avoiding, turning at a
 *	                               junction, rescanning a barcode, or
 *	                               part-way round an object
 *	                  So it never resumes, never confirms and never asks a
 *	                  question.  Whatever the car is doing, GP20 stops it;
 *	                  whatever is latched, the next GP20 starts it again.
 *	  START           start line following (broker)
 *	  STOP            stop line following (broker)
 *	  GP21            IMU calibration (only while stopped): the car spins
 *	                  for CAL_SPIN_MS, then stops; it must stay still and
 *	                  level for CAL_SETTLE_MS + CAL_ZERO_MS.  Put it on flat,
 *	                  open floor first.  STOP aborts.
 *	  RESET           stop and clear the run counters (junctions, humps, barcode)
 *	The run also ends at once if subsystem 4 reports the car lifted or
 *	overturned (it has already stopped the motors through subsystem 2).
 *
 *	Obstacles: while FOLLOWING, subsystem 6 watches the road ahead.  When it
 *	latches something within S6_STOP_MM (15 cm) the car goes to
 *	NAV_AVOIDING and drives round it.  If that cannot be finished the car
 *	waits there.  GP20 ENDS THE RUN from that wait, as it does from
 *	anywhere else; press it again to clear the latch and start following
 *	afresh.  START from the broker still resumes in place.
 *
 *	Barcodes that will not read: when subsystem 5 reports a pass that got
 *	most of the way through a code and then failed to decode (IF-15), the
 *	run goes to NAV_BARCODE_RESCAN -- back along the line, forward again
 *	over the code, then the line search.  See barcode_rescan() below.
 *
 *	Junctions: both line sensors on the line at once.  The car stops, creeps
 *	far enough forward to put its WHEELS on the crossing, and turns the way
 *	the last barcode that decoded told it to (NAV_TURNING).  See
 *	junction_turn() below.
 *
 *	NEITHER OF THOSE TWO WATCHES THE ROAD AHEAD, and that is deliberate.
 *	The obstacle manoeuvre is reachable from line following and nowhere
 *	else, so a sonar reading can never interrupt a rescan or a junction
 *	turn.  Nothing is lost by that: subsystem 6 LATCHES what it sees, so an
 *	object spotted mid-state is still waiting in the flag the instant the
 *	state hands back to follow_line(), which is a fraction of a second
 *	later and is where it gets dealt with.  Safety -- lifted, overturned,
 *	STOP, RESET -- is checked throughout, as everywhere else.
 *
 *	Barcodes: while a bar is going past subsystem 5's sensor (IF-14), the
 *	steering softens and never escalates to a spin, so the sensor is not
 *	whipped sideways across the code.  It goes back to the full pivot as soon
 *	as the code has gone, so curves are unaffected.  The run's "stopped"
 *	line says how long the steering was softened, which is how you tell
 *	whether it engaged at all.
 */

#include "subsystem3.h"
#include "drv_line.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem5/subsystem5.h"
#include "../subsystem6/subsystem6.h"
#include "avoid.h"
#include <stdlib.h>

/* ---------- Speeds, % of full power (0..100) ---------- */
/* Steering is a pivot: the inside wheel stops and the outside one drives, so
   the car turns in its own length and holds a tight curve.  These are the
   values the car followed the line with (they match the pre-kernel version,
   LineFollow/app_program/main/line_follow_imu.c).
   They were softened once (22 % / 8 %, a gentle curve) to stop the right-most
   sensor swinging sideways across a barcode -- but a gentle curve cannot hold
   the curved parts of the track, so the line following came first and the
   barcode side was made tolerant of the swing instead (see barcode.h,
   "Reading a code while the car is steering"). */
#define STRAIGHT_SPEED	10	/* both wheels while the line is between the sensors */
#define STEER_OUTER	35	/* outside wheel while steering back towards the line */
#define STEER_INNER	0	/* inside wheel while steering (0 = pivot around it) */

/* ...and the same correction, made gently, used ONLY while a barcode is going
   past the sensor (IF-14, barcode_crossing()).
   What steers the car is the DIFFERENCE between the wheels (14 here against
   35 above), so this turns at 40 % of the normal rate.  Both wheels keep
   driving on purpose, rather than pivoting around a stopped one:
     - the sensor's sideways drift per millimetre travelled is what smears a
       code, and driving both wheels doubles the distance covered per degree
       turned, which halves that drift again;
     - subsystem 2 raises any non-zero wheel power up to that wheel's learned
       stall floor (with_floor() in task_motion.c, up to 50 %).  A low inner
       value would be quietly floored up -- and if the floor passed the outer
       value, BOTH wheels would end up at the same power and the car would
       drive straight instead of correcting.  21 % and 35 % stay clear of a
       floor up to 21 %; watch "pwr L/R" on the status line to see what the
       wheels were really given.
   It lasts only while a bar is under the sensor (plus S5_CROSSING_US), so
   curves away from a code are taken with the full pivot above. */
#define STEER_OUTER_SOFT	35
#define STEER_INNER_SOFT	21

/* 1 = soften the steering while a bar is under the barcode sensor (IF-14).
   OFF by default: measured on the car, it was engaging for about a quarter of
   a run, which is far more than the codes account for -- the sensor sees the
   LINE as well, and then weaker steering lets the car drift further onto it,
   which feeds itself.  With it off the car steers exactly as it did before
   any of this.  Turn it on only to try it, and read the "steered gently for
   N ms" line at the end of a run: a couple of hundred ms per code is what it
   should look like, seconds is the loop. */
#define S3_BARCODE_SOFT		0

/* 1 = say on the console when the steering softens and when it stops.  Two
   lines per code: the quickest way to see whether it is engaging at all.
   Harmless with S3_BARCODE_SOFT = 0: it then reports what the barcode sensor
   is seeing without changing how the car drives. */
#define S3_BARCODE_TRACE	1

/* Longest the steering may stay soft in one go.  barcode_crossing() is true
   while the sensor reads black, so if it ends up parked over the line (or any
   dark patch) it would stay true and the car would creep round with a weak
   correction for ever.  Crossing a whole code takes well under this, so when
   it is exceeded the softening is given up on until the sensor sees white
   again. */
#define SOFT_MAX_MS	1500
#define SPIN_SPEED	35	/* each wheel (one forward, one back) when spinning
				   (sharp corners only, see SPIN_AFTER_MS) */

/* ---------- Timing ---------- */
#define SPIN_AFTER_MS	300	/* a sensor still on the line after this long -> spin */
#define CLEAR_MS	4	/* a sensor must read floor this long to count as off the line */
#define STEER_EXTRA_MS	50	/* keep turning this long after the sensor comes off the
				   line, so the car ends up along the line (0 = none).
				   Rule of thumb: keep STEER_EXTRA_MS x
				   (STEER_OUTER - STEER_INNER) at about 1750 if you
				   change the steering speeds. */
#define LOOP_MS		1	/* line following: sensors read every 1 ms */
#define IDLE_LOOP_MS	10	/* waiting for a button or a command */

/* ---------- Distances, measured with the wheel encoders (S2) ---------- */
#define CONFIRM_MM	6	/* one sensor sees black: drive straight this far first */
#define CROSS_MM	35	/* junction: drive straight at least this far, sensors ignored */
#define CROSS_MAX_MM	100	/* junction: never drive blind further than this */

typedef enum {
	REQ_NONE,
	REQ_START,	/* GP20 or START */
	REQ_STOP,	/* GP20 or STOP */
	REQ_CALIBRATE,	/* GP21 */
	REQ_RESET,	/* RESET */
	REQ_SAFETY,	/* subsystem 4: lifted or overturned */
	REQ_ABORT	/* the manoeuvre round an object could not finish */
} REQUEST;

LOCAL NAV_STATUS nav;			/* this task's copy; published to mtx_nav */
LOCAL OBS_STATUS obs_copy;		/* static: ~130 bytes, off the 2 KB stack */

LOCAL void publish(void)
{
	nav_publish(&nav);
}

/* ---------- Distance travelled, from subsystem 2's odometry ---------- */
typedef struct { W left, right; } ENC_MARK;

LOCAL ENC_MARK enc_mark(void)
{
	ODOMETRY o;
	ENC_MARK m;

	motion_get_odometry(&o);
	m.left = o.left;
	m.right = o.right;
	return m;
}

LOCAL W dist_since(ENC_MARK m)
{
	ODOMETRY o;

	motion_get_odometry(&o);
	return (abs((W)(o.left - m.left)) + abs((W)(o.right - m.right))) / 2;
}

/* How far the car has actually TRAVELLED since a mark, in the millimetres
   this car's odometry believes in (S3_TRUE_MM below says what that means).
   The two wheels are averaged WITH their signs, unlike dist_since() above:
   through a spin on the spot one wheel goes forward and the other back, so
   the signed average stays near zero while the absolute one grows.  That
   matters wherever the question is "how much floor has gone past" rather than
   "how far have the wheels rolled" -- the barcode rescan stops to straighten
   up mid-leg, and those corrections must not count as progress.
   Used ONLY by the two states added at the end of this file.  Nothing in the
   obstacle manoeuvre calls it; dist_since() above is still what that uses. */
LOCAL W moved_mm_since(ENC_MARK m)
{
	ODOMETRY o;
	W	 d;

	motion_get_odometry(&o);
	d = ((W)(o.left - m.left) + (W)(o.right - m.right)) / 2;
	if (d < 0) {
		d = -d;
	}
	return (W)((float)d / COUNTS_PER_MM);
}

/* ---------------------------------------------------------------------
 *  TRUE MILLIMETRES -> THE MILLIMETRES THIS CAR BELIEVES IN.
 *
 *  THIS DOES NOT TOUCH THE WHEELS.  It is a conversion used by the barcode
 *  rescan and the junction turn and by nothing else, and it exists precisely
 *  so that those two can be added without changing the odometry constants
 *  that every tuned distance in avoid.h depends on.
 *
 *  The problem it solves: V5 runs ENC_COUNTS_PER_REV = 1440, and the figure
 *  measured on the car is 2437 (3545 counts over the 297 mm long edge of a
 *  sheet of A4).  So the car under-drives -- ask for 100 mm and about 59 mm
 *  of floor goes past.
 *
 *  That does NOT make V5 wrong, and it is exactly why the constant is left
 *  alone.  Every distance in avoid.h was tuned by driving the car at a real
 *  box and making the number bigger until the manoeuvre worked, with 1440 in
 *  force -- so those numbers already move the right amount of floor, whatever
 *  units you call them.  Re-scaling the odometry underneath them re-reads all
 *  of them at once, and that is what made the manoeuvre swing wide.
 *
 *  The two new states are the other case.  Their distances were not tuned on
 *  the floor, they were MEASURED WITH A RULER: A4 is 297 mm, the car is about
 *  180 mm long.  Handing a true millimetre to an odometry that under-reads
 *  moves the car 59 % of the way, so a 297 mm reverse leg over a 297 mm code
 *  would stop the sensor in the middle of the barcode.
 *
 *  So ruler-measured lengths go through this and tuned ones do not.  It is a
 *  ratio against ENC_COUNTS_PER_REV rather than a bare 1.69 on purpose: if
 *  the odometry is ever calibrated properly it collapses to a no-op on its
 *  own, and every length below is already right.
 * --------------------------------------------------------------------- */
#define S3_ODOM_TRUE_CPR	2437	/* measured: counts for one wheel turn */
#define S3_TRUE_MM(mm)	((W)(((W)(mm) * S3_ODOM_TRUE_CPR) / ENC_COUNTS_PER_REV))

/* ---------- Motors: everything goes through subsystem 2 (IF-01) ---------- */
LOCAL void apply(UB a, BOOL soft)
{
	INT outer = soft ? STEER_OUTER_SOFT : STEER_OUTER;
	INT inner = soft ? STEER_INNER_SOFT : STEER_INNER;

	switch (a) {
	case ACT_STRAIGHT:
		motion_command(MOTION_CMD_DRIVE, STRAIGHT_SPEED, STRAIGHT_SPEED, 3);
		break;
	case ACT_STEER_LEFT:
		motion_command(MOTION_CMD_WHEELS, inner, outer, 3);
		break;
	case ACT_STEER_RIGHT:
		motion_command(MOTION_CMD_WHEELS, outer, inner, 3);
		break;
	case ACT_SPIN_LEFT:
		motion_command(MOTION_CMD_WHEELS, -SPIN_SPEED, +SPIN_SPEED, 3);
		break;
	case ACT_SPIN_RIGHT:
		motion_command(MOTION_CMD_WHEELS, +SPIN_SPEED, -SPIN_SPEED, 3);
		break;
	default:
		motion_command(MOTION_CMD_STOP, 0, 0, 3);
		break;
	}
	nav.action = a;
}

LOCAL UB steer_action(INT side, UW steering_ms, BOOL soft)
{
	/* A spin is the most violent thing the car does; while a code is going
	   past, keep steering instead and let the next pass escalate. */
	BOOL spin = !soft && (steering_ms >= SPIN_AFTER_MS);

	if (side < 0) {
		return spin ? ACT_SPIN_LEFT : ACT_STEER_LEFT;
	}
	return spin ? ACT_SPIN_RIGHT : ACT_STEER_RIGHT;
}

/* ---------- Inputs ---------- */
LOCAL UB poll_remote(void)
{
	UB c;

	if (tk_rcv_mbf(mbf_nav_cmd, &c, TMO_POL) == (INT)sizeof c) {
		return c;
	}
	return 0;
}

/* Subsystem 4's safety events (IF-05), without waiting */
LOCAL UINT safety_bits(void)
{
	UINT ptn = 0;

	if (tk_wai_flg(flg_imu, IMU_FLG_SAFETY, TWF_ORW, &ptn, TMO_POL) == E_OK) {
		return ptn & IMU_FLG_SAFETY;
	}
	return 0;
}

/* Report a safety stop, with the numbers behind it.  A bare "lifted off the
   floor" is no help when the car is plainly on the floor: what settles it is
   how big a rise the IMU thought it saw against how hard the car was shaking,
   because a rise that is small next to the shaking is the integrator drifting
   and not a hand at all (see the vibration note in imu.c).  Printed as
   integers because tm_printf has no %f. */
LOCAL void print_safety(UINT bits)
{
	IMU_STATUS is;

	if (bits & IMU_FLG_OVERTURNED) {
		tm_printf((UB*)"SAFETY STOP: overturned (tilted more than 90 deg)\n");
		return;
	}
	imu_get_status(&is);
	tm_printf((UB*)"SAFETY STOP: lifted off the floor -- the IMU measured a "
		  "%d.%d cm rise at 0.%02d m/s while the car was shaking "
		  "0.%03d g\n",
		  (INT)(is.d.lift_last_cm),
		  (INT)(is.d.lift_last_cm * 10.0f) % 10,
		  (INT)(is.d.lift_last_speed * 100.0f),
		  (INT)(is.d.vib_g * 1000.0f));
}

LOCAL UINT imu_flags(void)
{
	T_RFLG r;

	return (tk_ref_flg(flg_imu, &r) == E_OK) ? r.flgptn : 0;
}

/* Subsystem 6's obstacle events (IF-11), without waiting.  Subsystem 6 owns
   the obstacle latch and is the only one that drops it (subsystem6.h), so
   these bits are read here, never taken with TWF_BITCLR. */
LOCAL UINT obs_flags(void)
{
	T_RFLG r;

	return (tk_ref_flg(flg_obs, &r) == E_OK) ? r.flgptn : 0;
}

LOCAL void read_idle_sensors(void)
{
	nav.left = line_left_on();
	nav.right = line_right_on();
}

/* A real press: still pressed 20 ms later (a glitch or noise is not) */
LOCAL BOOL button_pressed(UINT pin)
{
	if (!button_down(pin)) {
		return FALSE;
	}
	tk_dly_tsk(20);
	return button_down(pin);
}

/* Waits for the button to be let go (with debounce) */
LOCAL void wait_release(UINT pin)
{
	tk_dly_tsk(30);
	while (button_down(pin)) {
		tk_dly_tsk(IDLE_LOOP_MS);
	}
	tk_dly_tsk(30);
}

/* Clears the per-run counters (start of a run, and the RESET command).
   Subsystem 1 sees run_count change and clears its barcode state. */
LOCAL void reset_run(void)
{
	nav.junctions = 0;
	nav.run_count++;
	imu_request(IMU_REQ_RESET_RUN);
	publish();
}

/* Every GP20 start (and START / RESET from the broker) first clears every
   latched error in the car:
     S2 motion  ESTOP latch; an encoder start that failed is tried again
     S4 IMU     lift latch, failed calibration, restart count; a missing
                sensor is looked for at once, a failing one reconfigured
     S6 obstacle  the latched obstacle and any sensor fault
     S3 / S1    run counters and barcode error state (reset_run)
   A fault that is still TRUE is not overridden: if the car is upside down
   right now, subsystem 4 reports it again within one sample (10 ms) and
   the run stops at once (TEAM.md, safety stop). */
LOCAL void clear_errors(void)
{
	tk_clr_flg(flg_imu, 0);				/* every old event */
	imu_request(IMU_REQ_CLEAR_ERRORS);
	obstacle_request(OBS_REQ_CLEAR);
	motion_command(MOTION_CMD_ENABLE, 0, 0, 3);
	tm_printf((UB*)"nav: errors cleared\n");
}

/* Right after power-up the motors are not ready for a few seconds: subsystem
   2 starts the wheel encoders only once the radio has taken its PIO block
   (at most ~10 s, even with no WiFi network or a failed radio).  A START in
   that time is not lost: wait here until the motors accept commands.
   Returns FALSE if they never do. */
#define MOTION_READY_WAIT_MS	12000

LOCAL BOOL wait_motion_ready(void)
{
	ODOMETRY o;
	UW t0 = app_now_ms();
	BOOL said = FALSE;

	while (1) {
		motion_get_odometry(&o);
		if (o.encoders_ok) {
			return TRUE;
		}
		if (app_now_ms() - t0 >= MOTION_READY_WAIT_MS) {
			tm_printf((UB*)"nav: motors never became ready (see the motion: "
				  "messages) -- not starting\n");
			return FALSE;
		}
		if (!said) {
			said = TRUE;
			tm_printf((UB*)"nav: START -- waiting for the motors to be ready...\n");
		}
		read_idle_sensors();
		publish();
		tk_dly_tsk(50);
	}
}

/* How long to give subsystem 6 to drop the obstacle latch after a clear, on
   the way out of IDLE.  OBSTACLE_CLEAR_WAIT_MS below is the same wait for the
   same reason, but it is declared further down the file than this. */
#define START_CLEAR_WAIT_MS	5000

/* Wait for OBS_REQ_CLEAR to take effect, and for NO OTHER REASON.
 *
 * THIS IS NOT A CHECK ON WHETHER THE OBJECT IS STILL THERE.  GP20 from a
 * standstill means "go", unconditionally, whatever is latched -- so nothing
 * here may refuse to start.  All this does is make the clear that was just
 * asked for actually happen before line following begins to watch the flag.
 *
 * It is needed because the latch lives in subsystem 6 and OBS_REQ_CLEAR is a
 * message, not a function call: the request can sit behind a servo scan for
 * seconds (see OBSTACLE_CLEAR_WAIT_MS).  Without this, follow_line() would
 * read the OLD latch on its very first pass and dive straight back into the
 * manoeuvre, and the button would look broken.
 *
 * If the object really is still in front of the car, subsystem 6 latches it
 * again a few pings later and the manoeuvre runs -- which is the right answer,
 * and is what the car would do if it had driven up to it normally.  Timing out
 * is not an error either: start anyway and let line following deal with what
 * it finds. */
LOCAL void wait_obstacle_cleared(void)
{
	UW t0 = app_now_ms();

	while (obs_flags() & S6_FLG_OBSTACLE) {
		if (app_now_ms() - t0 >= START_CLEAR_WAIT_MS) {
			tm_printf((UB*)"nav: subsystem 6 did not drop the obstacle "
				  "latch in %d ms -- starting anyway\n",
				  (INT)START_CLEAR_WAIT_MS);
			return;
		}
		read_idle_sensors();
		publish();
		tk_dly_tsk(10);
	}
}

/* ---------- IDLE: wait for a start / calibrate request ---------- */
LOCAL REQUEST wait_for_request(void)
{
	UB c;

	nav.state = NAV_IDLE;
	nav.action = ACT_STOP;
	while (1) {
		/* GP20 works on its own: no WiFi or broker is needed to start.
		   IT GOES, WHATEVER IS LATCHED.  Every flag is cleared -- the
		   IMU's errors, subsystem 6's obstacle, the motion safety stop
		   -- and the car enters line following.  There is no question
		   asked about the road ahead and no second press needed: if
		   something really is in the way, subsystem 6 latches it again
		   within a few pings and the manoeuvre handles it from
		   following, exactly as it would have anyway. */
		if (button_pressed(S3_START_BUTTON_PIN)) {
			wait_release(S3_START_BUTTON_PIN);
			clear_errors();
			wait_obstacle_cleared();	/* let the clear land */
			if (wait_motion_ready()) {
				return REQ_START;
			}
			continue;
		}
		if (button_pressed(S3_CAL_BUTTON_PIN)) {
			wait_release(S3_CAL_BUTTON_PIN);
			return REQ_CALIBRATE;
		}

		c = poll_remote();
		if (c == NAV_CMD_START || c == NAV_CMD_RESET) {
			clear_errors();
		}
		if (c == NAV_CMD_START && wait_motion_ready()) {
			return REQ_START;
		}
		if (c == NAV_CMD_RESET) {
			reset_run();		/* STOP while stopped: nothing to do */
		}

		read_idle_sensors();
		publish();
		tk_dly_tsk(IDLE_LOOP_MS);
	}
}

/* ---------- CALIBRATING (GP21) ---------- */

/* Waits for any of 'bits' on flg_imu, or a STOP / RESET, or the deadline.
   Returns the bits seen, or 0 if stopped / timed out. */
LOCAL UINT cal_wait(UINT bits, UW timeout_ms, BOOL *stopped)
{
	UW t0 = app_now_ms();
	UINT f;
	UB c;

	while (app_now_ms() - t0 < timeout_ms) {
		f = imu_flags();
		if (f & (bits | IMU_FLG_CAL_FAILED | IMU_FLG_OVERTURNED)) {
			return f;
		}
		c = poll_remote();
		if (c == NAV_CMD_STOP || c == NAV_CMD_RESET) {
			*stopped = TRUE;
			if (c == NAV_CMD_RESET) {
				reset_run();
			}
			return 0;
		}
		read_idle_sensors();
		publish();
		tk_dly_tsk(IDLE_LOOP_MS);
	}
	return 0;
}

LOCAL void calibrate(void)
{
	IMU_STATUS	is;
	BOOL		stopped = FALSE;
	UINT		f;

	/* GP21 also clears the latched errors first, and gives a missing IMU
	   one more look before giving up */
	clear_errors();
	tk_dly_tsk(50);
	imu_get_status(&is);
	if (!is.ok) {
		tm_printf((UB*)"nav: no IMU found, cannot calibrate\n");
		return;
	}

	nav.state = NAV_CALIBRATING;
	publish();
	/* Order matters: clear old events first (subsystem 4 may answer
	   CAL_START with CAL_FAILED at once), forget a lift from while the
	   car stood idle, put subsystem 4 into calibration (where the spin is
	   not a "lift"), and only then let the motors move again. */
	tk_clr_flg(flg_imu, ~(UINT)(IMU_FLG_CAL_ANY | IMU_FLG_SAFETY));
	imu_request(IMU_REQ_RESET_LIFT);
	imu_request(IMU_REQ_CAL_START);
	motion_command(MOTION_CMD_ENABLE, 0, 0, 3);

	/* 1) spin on the spot while subsystem 4 records the compass */
	motion_command(MOTION_CMD_WHEELS, +CAL_SPIN_SPEED, -CAL_SPIN_SPEED, 3);
	nav.action = ACT_SPIN_RIGHT;
	f = cal_wait(IMU_FLG_CAL_SPIN_DONE, CAL_SPIN_MS + 2000, &stopped);
	motion_command(MOTION_CMD_STOP, 0, 0, 3);
	nav.action = ACT_STOP;

	/* 2) motors off: subsystem 4 settles, then zeroes the tilt */
	if ((f & IMU_FLG_CAL_SPIN_DONE) && !(f & (IMU_FLG_CAL_FAILED | IMU_FLG_OVERTURNED))) {
		f = cal_wait(IMU_FLG_CAL_DONE, CAL_SETTLE_MS + CAL_ZERO_MS + 2000, &stopped);
	}

	if (!(f & IMU_FLG_CAL_DONE)) {
		imu_request(IMU_REQ_CAL_ABORT);
		tm_printf((UB*)"nav: calibration %s\n",
			  stopped ? "stopped (STOP/RESET)" :
			  (f & IMU_FLG_OVERTURNED) ? "stopped: car overturned" :
			  (f & IMU_FLG_CAL_FAILED) ? "failed (see imu: message)" : "timed out");
	}
	tk_clr_flg(flg_imu, ~(UINT)IMU_FLG_CAL_ANY);
	nav.state = NAV_IDLE;
	publish();
}

/* How long to wait for subsystem 6 to drop the latch after OBS_REQ_CLEAR.
   Generous because the request waits behind the servo scan, which measures
   the object in three stages (about 3 s) and cannot be interrupted. */
#define OBSTACLE_CLEAR_WAIT_MS	5000

/* ...and how long to let it look again afterwards before calling the road
   clear.  Subsystem 6 starts from scratch after a clear, so it needs
   S6_CONFIRM_CNT + 1 pings (plus the median-of-3 warm-up) to latch an object
   that is still there; this is comfortably more than that. */
#define OBSTACLE_RECHECK_MS	150

/* One look at everything that can end the wait, without blocking.  REQ_NONE
   means nothing has. */
LOCAL REQUEST wait_in_avoid_poll(void)
{
	UINT	bits;
	UB	c;

	bits = safety_bits();
	if (bits) {
		print_safety(bits);
		return REQ_SAFETY;
	}
	if (button_pressed(S3_START_BUTTON_PIN)) {
		wait_release(S3_START_BUTTON_PIN);
		return REQ_STOP;
	}
	c = poll_remote();
	if (c == NAV_CMD_STOP) {
		return REQ_STOP;
	}
	if (c == NAV_CMD_RESET) {
		return REQ_RESET;
	}
	return REQ_NONE;
}

/* ---------- The manoeuvre: driving round the object ----------
   The plan, the picture and the numbers are in avoid.h / avoid.c; this part
   does the driving.  Every move is measured with the wheel encoders and every
   move can be abandoned: the car is driving blind to the sides, so the safety
   stop, the button, STOP/RESET and anything close in FRONT are all checked
   throughout.

   Two things are worth knowing before reading on:

   The sonar never swivels.  Every clearance check is taken with it facing
   forward, after turning the car to point where it wants to look.

   The turns aim at absolute compass headings derived from h0, the heading the
   car came in on -- face(target, nominal) below.  A turn that lands 6 degrees
   off does not pass that 6 degrees on to the next turn, which is exactly what
   dead reckoning does and why the old version ended up pointing anywhere.
   The encoders do each turn and the compass only trims it, within a bound:
   an unbounded trim let one disturbed reading point the car anywhere, which
   is worse than the error it was correcting.

   Coming back, the car drives towards the line watching the line sensors,
   and stops with its AXLE over the line -- the only place a turn on the spot
   can leave it pointing along the line (avoid_return_to_line()).  Then it
   turns to the original heading and hands over to line_search() below, which
   sweeps the nose until the line is back between the sensors. */

/* 1 = turn 90 degrees once and stop, instead of running the manoeuvre.  For
   measuring AVOID_TRACK_MM: see the calibration note in avoid.h. */
#define S3_AVOID_TURN_TEST	0

/* 1 = drive round the object by itself.  0 = stop and wait there, which is
   what the car did before the manoeuvre existed. */
#define S3_AVOID_AUTO		1

/* A move that cannot reach its target -- a stalled wheel, a slipping tyre --
   must not loop for ever.  Allowed time is the distance at the speed asked,
   times this, plus a second to get going. */
#define AVOID_SLOWEST_CMS	3	/* assume at least this fast, cm/s */
#define AVOID_TIME_MARGIN	3

/* While going round, subsystem 6 reports the distance but latches nothing
   (OBS_REQ_WATCH_AHEAD), and nav stops only if something is really close in
   the direction it is driving.  The latch cannot be used here: the object
   being avoided is still in front when the manoeuvre starts, so it would
   come straight back and stop step 2 before the car had moved. */
#define AVOID_FRONT_STOP_MM	120

/* The ENCODERS turn the car and the COMPASS trims the result, neither trusted
   on its own -- see face() below for how the two are combined and why the
   trim is bounded.  The motors must be stopped before every compass reading:
   they are magnets a few centimetres from the sensor and spoil the heading
   while they run, and even braked windings carry current.

   With no GP21 calibration there is no usable compass, the trim is skipped,
   and the turn is plain encoder dead reckoning.  It says so on the console,
   so the cause of a wonky run is obvious. */
#define AVOID_TURN_TOL_DEG	4	/* close enough; do not chase noise */
#define AVOID_FACE_FIXES	4	/* corrections allowed after the main turn */
#define AVOID_TURN_FIX_SPEED	25	/* % power for the last few degrees */
#define AVOID_COMPASS_SETTLE_MS	250	/* after stopping, before reading */

/* How far the compass is allowed to move the car away from what the encoders
   did, added up over one turn.  THIS BOUND IS THE POINT: the encoders are
   repeatable but only as accurate as AVOID_TRACK_MM, while the compass knows
   the true angle but is pulled about by the motors (magnets a few centimetres
   away, and braked windings still carry current), by steel in the floor, and
   by any tilt.  Letting it correct without a limit means one bad reading
   turns the car to face anywhere -- which is exactly what an unbounded
   version did on the first turn.  So: the encoders do the turn, the compass
   trims it, and a compass that asks for more than this is not believed. */
#define AVOID_FACE_MAX_FIX_DEG	25
#define AVOID_FACE_FIX_MAX_DEG	15	/* ...and no more than this at once */

LOCAL avoid_plan_t	plan;
LOCAL avoid_step_t	step;

/* The track width the encoders are turning by.  Starts at the measured guess
   in avoid.h and is improved by the compass after every turn it can trust, so
   it carries across turns and across runs (see the blend note in avoid.h). */
LOCAL W			track_mm = AVOID_TRACK_MM;

/* FALSE = the next face() turns the car but does NOT let what it measures
   move track_mm.
 *
 * THIS IS THE ONE LINE THE JUNCTION TURN NEEDED INSIDE THE OBSTACLE CODE, and
 * it is here because track_mm is the single piece of state the manoeuvre
 * carries between turns: it is the wheelbase that avoid_turn_arc_track() works
 * out every 90-degree spin from, and face() is the only thing that writes it.
 *
 * A junction turn is 90 degrees, so without this it feeds that learning like
 * any other turn -- and it is a WORSE sample than the manoeuvre's turns.  It
 * is judged on two compass readings taken mid-course, where the line, the
 * floor and whatever the course is sitting on are all free to disturb the
 * magnetometer, and avoid_track_update() moves track_mm a quarter of the way
 * towards whatever those two readings imply.  A few junctions into a run and
 * the manoeuvre is spinning on a wheelbase it was taught by the barcodes,
 * which is how 90-degree turns stop coming out at 90.
 *
 * It is NOT an opinion about whether the learning is a good idea.  V5 learns
 * from its own turns exactly as it always did -- this build changes nothing
 * about that.  It only says that a junction is not entitled to a vote.
 *
 * Default TRUE, so every path that does not deliberately clear it behaves
 * precisely as V5 did.  junction_turn() clears it immediately before its
 * face() call and restores it immediately after, on every path out. */
LOCAL BOOL		face_learn = TRUE;

/* 1 = the obstacle manoeuvre, and the line search at its end, trim their
   turns with the compass once GP21 has calibrated it.  0 = encoders only,
   whether GP21 was pressed or not.
 *
 * 0 BECAUSE THAT IS WHAT WORKED ON THE FLOOR.  Run without a calibration,
 * the manoeuvre brought the car back onto the line; run after one, with the
 * compass trimming every turn, it did not.  Within a few centimetres of two
 * motors, and next to whatever the obstacle is made of, the magnetometer is
 * not a better judge of a quarter turn than the encoders are -- and the
 * trim is allowed to move each turn by up to AVOID_FACE_MAX_FIX_DEG, so a
 * disturbed reading costs more than the encoders' own error.
 *
 * This only switches the compass off for the obstacle manoeuvre.  The
 * junction turn and the barcode rescan read it exactly as before, and GP21
 * still calibrates the IMU for tilt, lift and the telemetry heading. */
#define S3_AVOID_USE_COMPASS	0

/* FALSE while a manoeuvre has the compass switched off: heading_settled()
   then answers "no compass" at once, so every turn and the search behave
   exactly as they do on a car that was never calibrated.  avoid_manoeuvre()
   sets it and puts it back on every way out. */
LOCAL BOOL		compass_allowed = TRUE;

/* Everything that can stop a move, looked at once.  REQ_NONE means carry on.
   The motors are stopped before returning anything else, so every caller can
   just hand the answer back up. */
LOCAL REQUEST avoid_checks(BOOL front_guard)
{
	UINT	bits;
	UB	c;

	bits = safety_bits();
	if (bits) {
		motion_command(MOTION_CMD_STOP, 0, 0, 3);
		print_safety(bits);
		return REQ_SAFETY;
	}
	if (button_pressed(S3_START_BUTTON_PIN)) {
		motion_command(MOTION_CMD_STOP, 0, 0, 3);
		wait_release(S3_START_BUTTON_PIN);
		return REQ_STOP;
	}
	c = poll_remote();
	if (c == NAV_CMD_STOP || c == NAV_CMD_RESET) {
		motion_command(MOTION_CMD_STOP, 0, 0, 3);
		return (c == NAV_CMD_RESET) ? REQ_RESET : REQ_STOP;
	}
	if (front_guard) {
		obstacle_get_status(&obs_copy);
		if (obs_copy.dist_mm > 0 &&
		    obs_copy.dist_mm < AVOID_FRONT_STOP_MM) {
			motion_command(MOTION_CMD_STOP, 0, 0, 3);
			tm_printf((UB*)"nav: %d mm ahead while %s -- giving up on "
				  "going round\n", (INT)obs_copy.dist_mm,
				  avoid_step_name(step));
			return REQ_ABORT;
		}
	}
	read_idle_sensors();
	publish();
	return REQ_NONE;
}

/* Start a move.  Going straight goes through MOTION_CMD_DRIVE, which is
   subsystem 2's encoder-synced mode: it compares how far each wheel has
   actually turned and trims whichever is ahead, so the car holds a straight
   line instead of drifting off with the stronger wheel.  Turning has to be
   MOTION_CMD_WHEELS, because the wheels go opposite ways and there is nothing
   to keep in step. */
LOCAL void avoid_drive(INT lpct, INT rpct)
{
	if (lpct == rpct && lpct > 0) {
		motion_command(MOTION_CMD_DRIVE, lpct, rpct, 3);
	} else {
		motion_command(MOTION_CMD_WHEELS, lpct, rpct, 3);
	}
}

/* Drive until the wheels have rolled mm, or something stops us.
   lpct/rpct are the two wheel powers: equal to go straight, opposite to turn.
   Returns REQ_NONE when the distance was covered. */
LOCAL REQUEST avoid_move(INT lpct, INT rpct, W mm, BOOL front_guard)
{
	ENC_MARK mark = enc_mark();
	const W  target = MM_TO_COUNTS(mm);
	UW	 t0 = app_now_ms();
	UW	 limit_ms = (UW)((mm * 1000 / (AVOID_SLOWEST_CMS * 10)) *
				 AVOID_TIME_MARGIN) + 1000u;
	REQUEST	 r;

	if (mm <= 0) {
		return REQ_NONE;
	}
	avoid_drive(lpct, rpct);

	while (dist_since(mark) < target) {
		r = avoid_checks(front_guard);
		if (r != REQ_NONE) {
			return r;
		}
		if (app_now_ms() - t0 > limit_ms) {
			motion_command(MOTION_CMD_STOP, 0, 0, 3);
			tm_printf((UB*)"nav: %s did not finish in %d ms (wheels "
				  "slipping or stalled) -- giving up\n",
				  avoid_step_name(step), (INT)limit_ms);
			return REQ_ABORT;
		}
		tk_dly_tsk(LOOP_MS);
	}

	motion_command(MOTION_CMD_STOP, 0, 0, 3);
	nav.action = ACT_STOP;
	return REQ_NONE;
}

/* Difference between two compass headings, as -180..+180 degrees.  Positive
   means a is reached from b by turning RIGHT: the compass grows clockwise. */
LOCAL float heading_diff(float a, float b)
{
	float d = a - b;

	while (d > 180.0f) {
		d -= 360.0f;
	}
	while (d < -180.0f) {
		d += 360.0f;
	}
	return d;
}

/* The compass heading now, or -1 if it cannot be trusted.  It needs the GP21
   calibration: without that the IMU still reports a heading, but not one with
   a known zero, so it cannot be compared with a target.  Call it with the
   motors stopped -- see AVOID_COMPASS_SETTLE_MS. */
LOCAL float turn_heading(void)
{
	IMU_STATUS is;

	imu_get_status(&is);
	if (!is.ok || !is.valid || !is.calibrated) {
		return -1.0f;
	}
	return is.d.heading;
}

/* One spin on the spot: the wheels roll arc_mm in opposite directions. */
LOCAL REQUEST spin_arc(BOOL left, W arc_mm, INT speed)
{
	if (left) {
		nav.action = ACT_SPIN_LEFT;
		return avoid_move(-speed, +speed, arc_mm, FALSE);
	}
	nav.action = ACT_SPIN_RIGHT;
	return avoid_move(+speed, -speed, arc_mm, FALSE);
}

/* Turn until the car is facing target, an ABSOLUTE compass heading.
 *
 * Aiming at absolute headings is the whole point: every turn in the manoeuvre
 * targets an angle worked out from the heading the car came in on, so no turn
 * inherits the error of the one before it.  Four turns later the car is still
 * aiming at a known direction rather than at the sum of four guesses.
 *
 * Each attempt spins by what the compass says is left to do, using the
 * encoders to measure it out, then stops and reads again.  Big attempts run
 * at AVOID_TURN_SPEED, the last few degrees at AVOID_TURN_FIX_SPEED, because
 * a slow wheel overshoots less.  A turn big enough to be measured honestly
 * also teaches the encoders a better track width, which is what keeps the
 * fallback below usable.
 *
 * fallback_deg is used when there is no compass at all (no GP21 calibration,
 * or the IMU has gone): one encoder turn of that many degrees, negative for
 * left.  That is dead reckoning, and it says so on the console.
 */
/* ---------------------------------------------------------------------
 *  MEASURING WHERE THE CAR IS POINTING.
 *
 *  Everything about a 90-degree turn comes back to one question: can the
 *  heading be trusted at the moment it is read?  There is no gyroscope on
 *  this car -- the LSM303D is an accelerometer and a magnetometer, nothing
 *  else -- so a turn cannot be integrated as it happens.  It has to be
 *  spun by the encoders and then CHECKED against the compass, which means
 *  the check is only as good as that one reading.
 *
 *  It was not good.  The reading was taken a fixed 250 ms after the motors
 *  braked, and a captured run says the field is nowhere near dead by then:
 *  250 ms after a drive the compass read 111, at 500 ms 115, at 750 ms 124,
 *  still climbing -- while the same sensor on a car that has stood still for
 *  a few seconds reads 146, 146, 146, 147.  A trim aimed at a heading that
 *  wrong turns the car the wrong way by that much.
 *
 *  A LONGER FIXED WAIT IS NOT THE ANSWER, because the settling time is not
 *  fixed: it depends on how long the motors have just been running and how
 *  hard.  After a full 90-degree spin the field takes a while; after a 5
 *  degree trim it is quiet almost at once.  Any single number is either too
 *  short after the big moves or wasted after the small ones.
 *
 *  So this waits for the reading to BE settled rather than for the clock:
 *  it samples until the last AVOID_HEADING_SAMPLES readings agree within
 *  AVOID_HEADING_STABLE_DEG of each other, and then returns their MEDIAN.
 *  The median rather than the mean because one spike from a motor current
 *  moves a mean and cannot move a median.  AVOID_COMPASS_SETTLE_MS is still
 *  honoured as a floor, so nothing reads sooner than V5 did, and there is a
 *  ceiling so a disturbed compass cannot stall the run.
 * --------------------------------------------------------------------- */
#define AVOID_HEADING_SAMPLES	 5	/* readings that have to agree */
#define AVOID_HEADING_SAMPLE_MS	 40	/* between them */
#define AVOID_HEADING_STABLE_DEG 3	/* spread across the window that counts
					   as settled.  Tighter than the 4 deg
					   AVOID_TURN_TOL_DEG the turn is judged
					   by, so the measurement is never the
					   loosest thing in the loop. */
#define AVOID_HEADING_MAX_MS	 1500	/* give up waiting, use what we have */

/* Returns a settled heading in degrees, or -1 if there is no compass (same
   contract as turn_heading(), so every caller's "no compass" path is
   unchanged). */
LOCAL float heading_settled(void)
{
	float	win[AVOID_HEADING_SAMPLES];	/* the last few raw readings */
	float	d[AVOID_HEADING_SAMPLES];	/* ...as offsets from win[0] */
	float	h, lo, hi, m, t;
	INT	i, j, have = 0;
	UW	t0;

	tk_dly_tsk(AVOID_COMPASS_SETTLE_MS);	/* the floor: never read sooner */
	if (!compass_allowed) {
		/* Switched off for this manoeuvre (S3_AVOID_USE_COMPASS).  The
		   wait above still happens, so the car has come to rest before
		   the next move starts, just as it did on an uncalibrated car;
		   only the 1.5 s of asking an IMU that will not answer is
		   skipped. */
		return -1.0f;
	}
	t0 = app_now_ms();

	for (;;) {
		h = turn_heading();
		if (h < 0.0f) {
			/* ONE BAD SAMPLE IS NOT "THERE IS NO COMPASS", and
			   treating it as one was quietly costing turns their
			   check.  turn_heading() reports -1 when the IMU is
			   missing, uncalibrated OR when its LAST READ simply
			   failed -- and a single failed read on an I2C bus is
			   a thing that happens.  Returning -1 here handed that
			   momentary hiccup to face(), which has no way to tell
			   it from a car with no compass at all, so it skipped
			   the whole trim and turned by dead reckoning -- on a
			   car whose compass was working perfectly a few
			   milliseconds later.
			   So: wait and ask again.  The window is dropped, not
			   kept, because readings either side of a dropout are
			   not consecutive and must not be judged as if they
			   were.  Only running out of time means there really
			   is no compass. */
			have = 0;
			if (app_now_ms() - t0 >= AVOID_HEADING_MAX_MS) {
				return -1.0f;		/* genuinely no compass */
			}
			tk_dly_tsk(AVOID_HEADING_SAMPLE_MS);
			continue;
		}
		if (have < AVOID_HEADING_SAMPLES) {
			win[have++] = h;
		} else {
			for (i = 1; i < AVOID_HEADING_SAMPLES; i++) {
				win[i - 1] = win[i];
			}
			win[AVOID_HEADING_SAMPLES - 1] = h;
		}

		if (have == AVOID_HEADING_SAMPLES) {
			/* Measured ABOUT THE FIRST SAMPLE, never as plain
			   subtraction: a window sitting across north reads
			   359, 0, 1, which is three readings two degrees apart
			   and not a 359-degree spread.  heading_diff() is what
			   knows that. */
			lo = 0.0f;
			hi = 0.0f;
			for (i = 0; i < AVOID_HEADING_SAMPLES; i++) {
				d[i] = heading_diff(win[i], win[0]);
				if (d[i] < lo) {
					lo = d[i];
				}
				if (d[i] > hi) {
					hi = d[i];
				}
			}
			if (hi - lo <= (float)AVOID_HEADING_STABLE_DEG) {
				break;			/* settled */
			}
			if (app_now_ms() - t0 >= AVOID_HEADING_MAX_MS) {
				tm_printf((UB*)"nav:     compass still moving "
					  "(%d deg across %d reads) after %d ms "
					  "-- taking the middle one\n",
					  (INT)(hi - lo), AVOID_HEADING_SAMPLES,
					  (INT)AVOID_HEADING_MAX_MS);
				break;
			}
		}
		tk_dly_tsk(AVOID_HEADING_SAMPLE_MS);
	}

	/* Median of the window (insertion sort, five items). */
	for (i = 1; i < AVOID_HEADING_SAMPLES; i++) {
		t = d[i];
		for (j = i; j > 0 && d[j - 1] > t; j--) {
			d[j] = d[j - 1];
		}
		d[j] = t;
	}
	m = win[0] + d[AVOID_HEADING_SAMPLES / 2];
	while (m < 0.0f) {
		m += 360.0f;
	}
	while (m >= 360.0f) {
		m -= 360.0f;
	}
	return m;
}

/* ---------------------------------------------------------------------
 *  "Degrees I want the car to turn" -> "arc to ask the encoders for".
 *
 *  ONE RULE, in one place, for every spin in this file.  It used to be
 *  three rules, which is most of why a 90-degree turn would not settle:
 *
 *    the main spin in face()   added AVOID_SPIN_EXTRA_DEG   (deg + 40)
 *    the TRIM in face()        ignored it entirely          (deg)
 *    the line search's ls_arc  scaled by it                 (deg * 130/90)
 *
 *  The allowance corrects a PROPORTIONAL error -- the wheels roll a fixed
 *  fraction of the arc they are asked for -- so scaling is the form that is
 *  right, and ls_arc() already had it.  A trim that asks for a bare 13
 *  degrees of arc therefore delivered about 8, so EVERY correction fell
 *  short by the same fraction and the turn settled short however many tries
 *  it was given.  That is the "always a bit off, never the same amount"
 *  the turns were showing.
 *
 *  Nothing else moves: every face() in this file turns +/-90 (the manoeuvre
 *  at avoid_manoeuvre(), the junction at junction_turn()), and at 90 the two
 *  forms are the same number -- 90 + 40 = 90 * (90 + 40) / 90 = 130 -- so
 *  the main spin is unchanged to the count, and ls_arc() below is this
 *  function already.  Only the trim gets anything it did not have. */
LOCAL W turn_arc(INT deg)
{
	return avoid_turn_arc_track((deg * (90 + AVOID_SPIN_EXTRA_DEG)) / 90,
				    track_mm);
}

/* Turn until the car is facing target, an ABSOLUTE compass heading.
 *
 * nominal_deg is how far the turn is MEANT to be from where the car stands
 * now (negative for left).  Every call site has the car a nominal quarter
 * turn from its target, so this is always +/-90 in practice.
 *
 * How the two sensors share the work:
 *
 *   The encoders do the turn.  One spin of the nominal arc, every time,
 *   whatever the compass thinks.  This is the part that is repeatable, works
 *   with the motors running, and cannot be thrown off by a magnet.
 *
 *   The compass then trims it, by at most AVOID_FACE_MAX_FIX_DEG in total.
 *   That bound is what makes this safe to use: the compass knows the true
 *   angle, but it is read next to two motors and over whatever is in the
 *   floor, and without a limit one disturbed reading turns the car to face
 *   anywhere.  Past the bound the encoders' answer stands and the console
 *   says the compass is being ignored.
 *
 * Aiming at an absolute heading is still the point: target is worked out from
 * h0, the heading the car came in on, so a turn that lands 5 degrees off does
 * not hand those 5 degrees to the next turn.  Four turns later the car is
 * still aiming at a known direction and not at the sum of four guesses.
 *
 * With no compass at all (no GP21 calibration, or the IMU has gone) the first
 * spin is the whole of it: plain dead reckoning, and it says so.
 */
LOCAL REQUEST face(float target, INT nominal_deg)
{
	float	now, after, err, went;
	INT	fixes, deg, fixed_deg = 0;
	REQUEST	r;
	W	learnt;

	deg = (nominal_deg < 0) ? -nominal_deg : nominal_deg;
	/* SETTLED, not a bare sample.  This reading is the "before" half of the
	   track-width learning below, and track_mm scales every spin the car
	   makes afterwards -- so a reading taken through a field that has not
	   died away does not merely waste itself, it walks the geometry and
	   makes the NEXT turn wrong too.  It used to be taken the instant
	   face() was called, with no wait at all, while the reading after the
	   spin got the full AVOID_COMPASS_SETTLE_MS. */
	now = heading_settled();		/* before, for the learning */

	/* ---- the encoders turn the car ----
	   Asked for AVOID_SPIN_EXTRA_DEG more than wanted, because the tyres
	   are dragged sideways through a spin on the spot and the wheels turn
	   further than the car does (see the slip note in avoid.h).  deg stays
	   the angle actually wanted: that is what the trim aims at and what the
	   track learning is judged by, so the allowance cannot feed back into
	   the geometry. */
	r = spin_arc(nominal_deg < 0, turn_arc(deg), AVOID_TURN_SPEED);
	if (r != REQ_NONE) {
		return r;
	}

	/* Motors off, then read -- and wait for the reading to settle rather
	   than for a fixed quarter second.  This is the measurement the whole
	   turn is judged by. */
	after = heading_settled();
	if (after < 0.0f || now < 0.0f) {
		tm_printf((UB*)"nav:     turned %d deg by the encoders alone, "
			  "+%d for slip (%s)\n", nominal_deg,
			  AVOID_SPIN_EXTRA_DEG,
			  compass_allowed ? "no compass -- press GP21 once on "
					    "a flat floor"
					  : "compass off for this manoeuvre, "
					    "S3_AVOID_USE_COMPASS");
		return REQ_NONE;
	}

	/* What the turn actually achieved improves the track width the encoders
	   work from, so the spin above gets better every time.  Only a big turn
	   is worth learning from, and avoid_track_update() throws out anything
	   that cannot be a 90-degree turn gone slightly wrong. */
	if (deg >= AVOID_TURN_TRUST_MIN_DEG && face_learn) {
		went = heading_diff(after, now);
		if (went < 0.0f) {
			went = -went;
		}
		learnt = avoid_track_update(track_mm,
					    (INT)(went * 90.0f / (float)deg));
		if (learnt != track_mm) {
			tm_printf((UB*)"nav:     asked %d deg, turned %d: "
				  "track %d -> %d mm\n", deg, (INT)went,
				  (INT)track_mm, (INT)learnt);
			track_mm = learnt;
		}
	}
	now = after;

	/* ---- the compass trims it, within the bound ---- */
	for (fixes = 0; fixes < AVOID_FACE_FIXES; fixes++) {
		err = heading_diff(target, now);
		if ((err > -(float)AVOID_TURN_TOL_DEG) &&
		    (err < (float)AVOID_TURN_TOL_DEG)) {
			/* ALWAYS reported, not only when it needed trimming.
			   How far each turn lands from its target is the one
			   number that says whether the turns are repeatable,
			   and it is no use only being printed on the runs that
			   went badly. */
			tm_printf((UB*)"nav:     facing %d deg, wanted %d "
				  "(off by %d) after %d deg of trim in %d\n",
				  (INT)now, (INT)target, (INT)err, fixed_deg,
				  fixes);
			return REQ_NONE;		/* there */
		}

		deg = (INT)((err < 0.0f) ? -err : err);
		if (deg > AVOID_FACE_FIX_MAX_DEG) {
			deg = AVOID_FACE_FIX_MAX_DEG;
		}
		if (fixed_deg + deg > AVOID_FACE_MAX_FIX_DEG) {
			tm_printf((UB*)"nav:     compass wants %d deg more "
				  "after %d deg of trim -- not believing it, "
				  "the encoders' turn stands\n",
				  (INT)err, fixed_deg);
			return REQ_NONE;
		}

		/* turn_arc(), the same rule the main spin uses.  This line is
		   the fix for a turn that would not close: it used to ask for a
		   bare arc, so every correction under-turned by the same
		   fraction and the error never ran out. */
		r = spin_arc((err < 0.0f), turn_arc(deg), AVOID_TURN_FIX_SPEED);
		if (r != REQ_NONE) {
			return r;
		}
		fixed_deg += deg;

		after = heading_settled();
		if (after < 0.0f) {
			tm_printf((UB*)"nav:     lost the compass mid-trim, "
				  "carrying on\n");
			return REQ_NONE;
		}
		now = after;
	}

	tm_printf((UB*)"nav:     still %d deg off after %d deg of trim, "
		  "carrying on\n", (INT)heading_diff(target, now), fixed_deg);
	return REQ_NONE;
}

LOCAL REQUEST avoid_straight(W mm, BOOL front_guard)
{
	nav.action = ACT_STRAIGHT;
	return avoid_move(AVOID_DRIVE_SPEED, AVOID_DRIVE_SPEED, mm, front_guard);
}

/* The nearest thing straight ahead, in mm, or 0 for "nothing in range".
   Several pings and the closest wins: a single ping can slip past a corner
   and bring back the far wall, and being over-cautious here only ever costs
   another sidestep.  Subsystem 6 is in WATCHING, so this latches nothing. */
LOCAL W look_ahead(void)
{
	W   best = 0;
	INT i;

	for (i = 0; i < AVOID_LOOK_PINGS; i++) {
		tk_dly_tsk(AVOID_LOOK_GAP_MS);
		obstacle_get_status(&obs_copy);
		if (obs_copy.dist_mm > 0 &&
		    (best == 0 || obs_copy.dist_mm < best)) {
			best = obs_copy.dist_mm;
		}
	}
	return best;
}

/* TRUE when the way straight ahead is clear enough to drive.  The car must
   already be pointing where it wants to look, and standing still. */
LOCAL BOOL ahead_is_clear(const char *what)
{
	W d = look_ahead();

	if (d == 0) {
		tm_printf((UB*)"nav:     %s: nothing in range -- clear\n", what);
		return TRUE;
	}
	if (d > AVOID_CHECK_CLEAR_MM) {
		tm_printf((UB*)"nav:     %s: nearest thing %d mm -- clear\n",
			  what, (INT)d);
		return TRUE;
	}
	tm_printf((UB*)"nav:     %s: %d mm ahead -- still in the way\n",
		  what, (INT)d);
	return FALSE;
}

/* Say where the car has got to, on the status line and on the console. */
LOCAL void avoid_at(avoid_step_t s)
{
	step = s;
	nav.avoid_step = (UB)s;
	publish();
	tm_printf((UB*)"nav:   (%d) %s\n", (INT)s + 1, avoid_step_name(s));
}

/* ---------- Finding the line again: the line search ----------
 *
 * Where the car stands when this starts: back at the line it left, pointing
 * the way it was going -- but "back at the line" is only as good as nine
 * encoder moves and four compass turns, so the line is NEAR the sensors and
 * not necessarily under them.  Finding it is a sweep of the nose, and nothing
 * else: the car does not drive anywhere.  A car that hunts for a line by
 * driving about ends up somewhere it cannot explain, whereas a spin on the
 * spot can only ever be wrong about its angle.
 *
 * The sweep goes LEFT first, then right through the middle to the other side,
 * then back to the middle if there was nothing either way.  It is bounded at
 * LS_MAX_DEG each side of the heading the search began at, and no further.
 *
 * WHICH SENSOR SEES THE LINE SAYS WHERE THE LINE IS.  The two sensors
 * straddle the line while the car follows it, so "found" means getting the
 * line BETWEEN them, not merely touching it.  A spin drags the pair sideways
 * across the floor: turning left, the line travels rightwards across the
 * pair, so it reaches the LEFT sensor first and the RIGHT sensor last.  That
 * gives the sweep its two answers:
 *
 *   turning left, the LEFT sensor has it    the pair has only just reached
 *                                           the line, so it is still OUTSIDE
 *                                           them -- keep turning the same way
 *                                           until it comes in
 *   ...and then the LEFT sensor lets go     the line has slid into the gap
 *                                           between the sensors, in the
 *                                           middle, where following wants it.
 *                                           Stop there: found.
 *   turning left, the RIGHT sensor has it   the line is at the far edge of
 *                                           the pair, about to leave it.
 *                                           Only reached first when the line
 *                                           started between the sensors; stop
 *                                           there too, it is the nearest the
 *                                           sweep gets to the middle.
 *
 * ...and the mirror image turning right.  It used to stop ONLY at the second
 * sensor, on the reasoning that the line had "crossed the pair" by then --
 * but by then it is leaving the pair, and the few ms the motors take to stop
 * were enough to carry it out of the far side.
 *
 * A SPIN CAN ONLY REACH A LINE WITHIN ITS REACH.  The sensors ride on a
 * circle round the axle, of radius the distance they are ahead of it, so a
 * line further from the axle than that is never touched however far the car
 * sweeps -- and a line the axle is NOT on can only be met by pointing the car
 * across it.  That is why whoever calls this must first put the axle on the
 * line: JN_ADVANCE_MM, i.e. S3_SENSOR_TO_AXLE_TRUE_MM, is what does it.
 *
 * The angle comes from the IMU where there is one.  The compass cannot be
 * read with the motors running (they are magnets a few centimetres from it,
 * see avoid.h), so the sweep is taken in bites: spin, stop, read, spin on.
 * The SENSORS, though, are read every LOOP_MS right through a bite, so the
 * bite size is not how accurately the line is found -- the car stops the
 * millisecond it sees it -- only how often the heading is checked.  With no
 * GP21 calibration there is no usable compass and the encoders count the
 * sweep out by themselves, which is exactly what the manoeuvre's own turns
 * fall back to (face()); it says so on the console, so a wonky sweep has an
 * obvious cause.
 */
#define LS_MAX_DEG	90	/* never sweep further than this either side */
#define LS_STEP_DEG	15	/* one bite of the sweep, between compass reads */

/* Allowed PAST LS_MAX_DEG once a sensor has actually seen the line, to bring
   it between the pair.  Giving up at exactly 90 degrees with the line sitting
   under the outer sensor would throw away a line the car had already found. */
#define LS_EXTRA_DEG	30

/* Each wheel, opposite directions.  The sweep is hunting for a line a
   centimetre or two wide, so it spins at the manoeuvre's trimming speed
   rather than its travelling one: a slow spin carries the sensor less far
   past the line in the millisecond before it is noticed.  Not lower than this
   on purpose -- subsystem 2 raises any non-zero wheel power up to that
   wheel's learned stall floor, so a smaller number here may not mean a slower
   car (with_floor() in task_motion.c). */
#define LS_SPIN_SPEED	25

/* A sensor must read black this many polls running (LOOP_MS apart) before it
   is believed.  One stray read is a speck of dirt or a bit of noise, and
   acting on it would end the sweep pointing at nothing. */
#define LS_SEEN_READS	3

/* A sweep that gets nowhere must still end.  The compass is what decides the
   limit has been reached, so a car whose wheels turn without turning the car
   -- held in a hand, or on a very slippery floor -- would otherwise bite away
   for ever.  Twice the bites a clean sweep of the widest span needs. */
#define LS_MAX_BITES	(2 * (2 * LS_MAX_DEG + LS_EXTRA_DEG) / LS_STEP_DEG)

/* Per-wheel arc for sweeping deg degrees, with the spin slip allowed for.
   avoid.h measures that slip as "a spin asked for 90 degrees comes out about
   AVOID_SPIN_EXTRA_DEG short", and the manoeuvre adds it flat because it only
   ever turns 90 at a time.  The sweep's bites are much smaller, so the same
   measurement is applied as a RATIO instead -- the angle wanted times
   (90 + extra) / 90, which comes to the same thing at 90 degrees.  With a
   compass it only has to be close, because the next reading says where the
   car really got to; without one it is the difference between sweeping 90
   degrees and sweeping 62. */
LOCAL W ls_arc(INT deg)
{
	return turn_arc(deg);		/* the same rule as every other spin */
}

/* One bite of the sweep: spin on the spot, watching the sensors every
   LOOP_MS, until the line is between them, the wheels have rolled arc_mm, or
   something ends the run.
     left      TRUE = anticlockwise
     want_deg  the angle arc_mm stands for, for the console
     *inside   set TRUE if it stopped because the line is between the sensors
               (or at the trailing one, see below)
     *outside  set TRUE if the LEADING sensor saw it: the line is there, but
               not between the pair yet.  Never cleared here -- it belongs to
               the whole sweep, not to one bite of it.
     *rolled   wheel counts rolled before the motors were told to stop, so the
               caller can tell how far a bite that stopped early really got
   Every bite ends with ONE console line saying what the two sensors saw on
   the way round -- how many polls each read black and the longest unbroken
   run -- and the longest gap between polls.  That line is the difference
   between "the sensors never passed over the line" (the axle is not where it
   should be) and "they passed over it and it was not believed" (the polling
   or the debounce), which look exactly the same from the floor. */
LOCAL REQUEST ls_bite(BOOL left, W arc_mm, INT want_deg, BOOL *inside,
		      BOOL *outside, W *rolled)
{
	ENC_MARK mark = enc_mark();
	const W	 target = MM_TO_COUNTS(arc_mm);
	UW	 t0 = app_now_ms();
	UW	 limit_ms = (UW)((arc_mm * 1000 / (AVOID_SLOWEST_CMS * 10)) *
				 AVOID_TIME_MARGIN) + 1000u;
	INT	 lead_run = 0, trail_run = 0;
	INT	 lead_clear = 0;	/* polls running with the leading sensor
					   white AFTER it had the line */
	/* What the sensors saw, for the console line at the end */
	INT	 polls = 0, l_black = 0, r_black = 0;
	INT	 l_run = 0, r_run = 0, l_best = 0, r_best = 0;
	UW	 last = t0, gap_max = 0, now;
	const char *why = "no line";
	REQUEST	 r = REQ_NONE;

	*inside = FALSE;
	*rolled = 0;
	if (arc_mm <= 0) {
		return REQ_NONE;
	}

	nav.action = left ? ACT_SPIN_LEFT : ACT_SPIN_RIGHT;
	avoid_drive(left ? -LS_SPIN_SPEED : +LS_SPIN_SPEED,
		    left ? +LS_SPIN_SPEED : -LS_SPIN_SPEED);

	while (dist_since(mark) < target) {
		BOOL lon = line_left_on();
		BOOL ron = line_right_on();
		/* The LEADING sensor is the one the line reaches first: the left
		   one turning left, the right one turning right. */
		BOOL lead  = left ? lon : ron;
		BOOL trail = left ? ron : lon;

		now = app_now_ms();
		if (polls > 0 && now - last > gap_max) {
			gap_max = now - last;
		}
		last = now;
		polls++;
		if (lon) {
			l_black++;
			if (++l_run > l_best) {
				l_best = l_run;
			}
		} else {
			l_run = 0;
		}
		if (ron) {
			r_black++;
			if (++r_run > r_best) {
				r_best = r_run;
			}
		} else {
			r_run = 0;
		}

		lead_run  = lead  ? (lead_run  + 1) : 0;
		trail_run = trail ? (trail_run + 1) : 0;
		/* Counted only once the leading sensor HAS had the line (in
		   this bite or an earlier one of the same sweep): white after
		   that means the line has slid off it into the gap. */
		lead_clear = (*outside && !lead) ? (lead_clear + 1) : 0;

		if (lead_clear >= LS_SEEN_READS) {
			/* The leading sensor had the line and has let go of it,
			   and the trailing one has not reached it: the line is
			   BETWEEN the pair, which is where following wants it.
			   Stopping here rather than at the trailing sensor is
			   the difference between handing over a centred line
			   and one that is about to leave the pair -- the motors
			   take a few ms to stop, and from the trailing sensor
			   that coast carries the line out of the pair, both
			   sensors read white, and following drives straight off
			   beside it. */
			*inside = TRUE;
			why = "line BETWEEN the sensors -- stopped";
			break;			/* centred: found */
		}
		if (trail_run >= LS_SEEN_READS) {
			/* The trailing sensor reached it with no sighting on the
			   leading one first: the line started between the pair
			   (the car was nearly lined up already), so this is as
			   close to the middle as this sweep gets. */
			*inside = TRUE;
			why = "line at the trailing sensor -- stopped";
			break;			/* at the trailing sensor: found */
		}
		if (lead_run >= LS_SEEN_READS && !*outside) {
			*outside = TRUE;
			tm_printf((UB*)"nav:   the %s sensor has the line, so it "
				  "is still outside the pair -- turning on to "
				  "bring it in\n", left ? "left" : "right");
		}

		r = avoid_checks(FALSE);	/* safety, GP20, STOP / RESET */
		if (r != REQ_NONE) {
			why = "stopped by safety, GP20 or a command";
			break;			/* motors already stopped */
		}
		if (app_now_ms() - t0 > limit_ms) {
			tm_printf((UB*)"nav: a search sweep did not finish in %d "
				  "ms (wheels slipping or stalled) -- giving "
				  "up\n", (INT)limit_ms);
			why = "timed out";
			r = REQ_ABORT;
			break;
		}
		tk_dly_tsk(LOOP_MS);
	}

	motion_command(MOTION_CMD_STOP, 0, 0, 3);
	nav.action = ACT_STOP;
	*rolled = dist_since(mark);

	tm_printf((UB*)"nav:   sweep %s %d deg: %s.  Left sensor black %d "
		  "polls (longest run %d), right %d (%d); %d polls, longest "
		  "gap %d ms; wheels rolled %d of %d counts\n",
		  left ? "LEFT" : "RIGHT", want_deg, why,
		  l_black, l_best, r_black, r_best, polls, (INT)gap_max,
		  (INT)*rolled, (INT)target);
	return r;
}

/* Sweep one way, in bites, until the line is between the sensors, the
   allowance runs out, or something ends the run.
     left     TRUE = anticlockwise
     h0       the heading the search began at, or -1 with no usable compass
     to_deg   where to stop, in degrees RIGHT of h0.  The compass grows
              clockwise, so right is positive and a left sweep aims at a
              negative number.
     *at_deg  in: where the nose is now, same convention.  out: where it got to
     *inside  set TRUE when the line ends up between the sensors */
LOCAL REQUEST ls_sweep(BOOL left, float h0, INT to_deg, INT *at_deg,
		       BOOL *inside)
{
	BOOL	outside = FALSE;	/* THIS sweep's leading sensor has it */
	INT	want, limit, done, bites = 0;
	W	rolled, full;
	float	now;
	REQUEST	r;

	while (1) {
		limit = to_deg;
		if (outside) {
			limit += left ? -LS_EXTRA_DEG : +LS_EXTRA_DEG;
		}
		want = left ? (*at_deg - limit) : (limit - *at_deg);
		if (want <= 0) {
			return REQ_NONE;	/* at the limit, and no line */
		}
		/* Counted BEFORE want is capped, so the message can say how far
		   the sweep still had to go and not just the size of a bite. */
		if (++bites > LS_MAX_BITES) {
			tm_printf((UB*)"nav: %d sweeps and the car is still %d "
				  "deg from the end of its scan -- the wheels "
				  "are turning but the car is not.  Giving "
				  "up\n", bites - 1, want);
			return REQ_ABORT;
		}
		/* BITES ONLY WHEN THERE IS A COMPASS TO READ BETWEEN THEM.  That
		   is the only thing a bite is for.  With no compass the sweep is
		   ONE smooth spin to the limit, the sensors watched every ms of
		   it, because every stop-and-start costs something: the coast
		   after each stop is a stretch of floor nobody is watching, and
		   each restart from standstill turns the car by a different
		   amount for the same wheel count. */
		if (want > LS_STEP_DEG && h0 >= 0.0f) {
			want = LS_STEP_DEG;
		}

		r = ls_bite(left, ls_arc(want), want, inside, &outside,
			    &rolled);
		if (r != REQ_NONE) {
			return r;
		}

		/* Where the nose really is: the compass, read with the motors
		   stopped and SETTLED, or else the angle that was asked for.
		   The compass is worth reading even when the bite found the
		   line, because such a bite stopped part way through and the
		   angle asked for is then too big.
		   Settled, because this read comes straight off the end of a
		   bite -- motors braked a moment ago -- which is precisely when
		   a fixed quarter second is not enough.  A bite is a small spin
		   though, so the field is quiet almost at once and the adaptive
		   wait usually returns at its floor.
		   Without a compass, the share of the bite's wheel counts that
		   was actually rolled: a bite that found the line stopped part
		   way, and a whole sweep is one bite now. */
		full = MM_TO_COUNTS(ls_arc(want));
		done = (full > 0) ? (INT)((rolled * want + full / 2) / full)
				  : want;
		if (done > want) {
			done = want;
		}
		*at_deg += left ? -done : +done;
		if (h0 >= 0.0f) {
			now = heading_settled();
			if (now >= 0.0f) {
				*at_deg = (INT)heading_diff(now, h0);
			}
		}
		if (*inside) {
			return REQ_NONE;
		}
	}
}

/* Find the line again.  Called with the car back at the line it left,
   pointing the way it was going.  REQ_NONE means the line is between the
   sensors again and following can carry straight on; REQ_ABORT means it was
   not found, and the car waits in NAV_AVOIDING exactly as it does
   for any other manoeuvre that could not finish. */
/* ---------------------------------------------------------------------
 *  THE LINE SEARCH MUST NEVER START A BARCODE RESCAN.
 *
 *  A sweep pivots the car on the spot, which drags the barcode sensor
 *  sideways across the black line it is hunting for.  Subsystem 5 cannot
 *  tell that from a car driving over a printed code: it sees bars and gaps,
 *  starts a pass, and when the pass will not decode -- which it will not,
 *  because a line is not a code -- it steps s5_fails.  follow_line() reads
 *  that as "a code failed", and the car reverses back up the line to look
 *  again at a barcode that was never there.  Worse, the rescan ends in
 *  another line search, which shakes out another failure, which starts
 *  another rescan.
 *
 *  Syncing the counters when the search returns is NOT enough, and that is
 *  the trap.  A pass does not finish when the bars stop going past: it
 *  finishes BARCODE_END_GAP_MS (a whole second) after its LAST edge
 *  (barcode.h).  So the sweep ends, the counters are taken as read, the car
 *  goes back to following -- and a second later the pass the sweep started
 *  lands in s5_fails and trips the rescan anyway.  Only a window that
 *  outlasts the pass closes that door.
 *
 *  So the search mutes the rescan until any pass it may have started has
 *  had time to finish.  Failures inside the window are logged and dropped.
 *  READS ARE NOT MUTED: a sweep cannot fake a Code 39 decode -- that needs
 *  the start and stop characters and credible bar ratios -- so a read
 *  arriving here is a real code and still sets the command for the next
 *  junction.
 *
 *  One mute, inside line_search(), covers every caller: the junction turn,
 *  the rescan's own tail, and the end of the obstacle manoeuvre all reach
 *  the line through this one function.
 * --------------------------------------------------------------------- */
#define BC_MUTE_MS	(BARCODE_END_GAP_MS + 500)

LOCAL UW bc_mute_until = 0;	/* no rescan from a pass finishing before this */

LOCAL void bc_mute_rescan(void)
{
	bc_mute_until = app_now_ms() + BC_MUTE_MS;
}

LOCAL BOOL bc_rescan_muted(void)
{
	return (W)(app_now_ms() - bc_mute_until) < 0;
}

/* "I have no heading to give you -- take your own." */
#define NO_HEADING	(-1.0f)

/* ref_h is what "straight" MEANS for this search: the heading the sweep's
   angles are measured from, and the middle of the arc it sweeps.
 *
 * PASS ONE IN WHENEVER THE CALLER KNOWS IT, and the obstacle manoeuvre always
 * does.  It read the heading the car came in on before it turned a wheel, and
 * every turn since has aimed at an absolute heading worked out from that one
 * reading -- so the manoeuvre's h0 IS straight, by definition, for the whole
 * manoeuvre.
 *
 * Letting the search read its own instead was the car quietly redefining
 * straight at the very last moment.  The reading it takes is of wherever the
 * car actually ended up, which is the original heading PLUS whatever the last
 * turn failed to correct -- so the sweep would centre on the error, hunt for
 * the line around the wrong middle, and hand back a car that believed a
 * crooked heading was straight.  Each manoeuvre would then start from the
 * previous one's leftovers.  Taking the reference from the caller is what
 * stops that compounding: there is ONE reading of what straight is, at the
 * start, and everything after is worked out from it.
 *
 * NO_HEADING means the caller genuinely has no reference to give -- the
 * barcode rescan, which is steered by the line sensors and never reads the
 * compass at all -- and then the search reads one of its own as before.
 * Passing an h0 that is itself NO_HEADING (no compass on the car) lands in
 * the same place, which is right: there is nothing to be true to. */
LOCAL REQUEST line_search(float ref_h)
{
	float	h0;
	INT	at = 0;			/* degrees right of h0 (negative = left) */
	BOOL	inside = FALSE;
	REQUEST	r;

	motion_command(MOTION_CMD_STOP, 0, 0, 3);
	nav.action = ACT_STOP;
	publish();

	/* Subsystem 6 keeps measuring but latches nothing while the car sweeps:
	   a spin on the spot points the sonar at everything in the room, and
	   something a few centimetres off to one side is not in the car's way.
	   The latch goes back on at the end, which is how the manoeuvre left
	   it. */
	obstacle_request(OBS_REQ_WATCH_AHEAD);

	/* The caller's reference if there is one -- see the note above, this is
	   the whole point of the parameter.  Only when there is none is a fresh
	   one read, and then it gets the same settled treatment as any other
	   heading: one bad sample here would bias the whole sweep's idea of
	   where it is pointing, not just one bite of it. */
	h0 = (ref_h >= 0.0f) ? ref_h : heading_settled();
	tm_printf((UB*)"nav: looking for the line (sensors now: left %s, right "
		  "%s) -- sweeping up to %d deg each side, %s\n",
		  line_left_on() ? "BLACK" : "white",
		  line_right_on() ? "BLACK" : "white", LS_MAX_DEG,
		  (h0 < 0.0f) ? (compass_allowed
				 ? "by the encoders alone (no compass -- press "
				   "GP21 once on a flat floor)"
				 : "by the encoders alone (compass off for this "
				   "manoeuvre, S3_AVOID_USE_COMPASS)")
		  : (ref_h >= 0.0f) ? "straight is the heading the run came in "
				      "on, not a fresh reading"
				    : "checking the heading on the IMU");

	/* Left to the limit, then right through the middle to the other limit,
	   and if there is still nothing, back to the middle so the car is left
	   pointing the way it came in.  That last leg is a third look and not
	   just a tidy-up: the same rule decides it. */
	r = ls_sweep(TRUE, h0, -LS_MAX_DEG, &at, &inside);
	if (r == REQ_NONE && !inside) {
		r = ls_sweep(FALSE, h0, +LS_MAX_DEG, &at, &inside);
	}
	if (r == REQ_NONE && !inside) {
		r = ls_sweep(TRUE, h0, 0, &at, &inside);
	}

	motion_command(MOTION_CMD_STOP, 0, 0, 3);
	nav.action = ACT_STOP;
	obstacle_request(OBS_REQ_CLEAR);
	publish();

	/* Whatever the sweep found, it has just rubbed the barcode sensor along
	   the line.  Shut the rescan out until anything that started can have
	   finished -- see the note above.  Set here, after the last bite and
	   before every return, so no way out of the search misses it. */
	bc_mute_rescan();

	if (r != REQ_NONE) {
		return r;			/* safety, STOP, RESET, abort */
	}
	if (inside) {
		tm_printf((UB*)"nav: line found %s%d deg %s of where the search "
			  "started -- following it again\n",
			  (h0 < 0.0f) ? "about " : "",
			  (INT)((at < 0) ? -at : at),
			  (at < 0) ? "left" : "right");
		return REQ_NONE;
	}
	tm_printf((UB*)"nav: no line within %d deg either side -- giving up on "
		  "the search\n", LS_MAX_DEG);
	return REQ_ABORT;
}

/* ---------- BARCODE_RESCAN: a code that would not decode gets one more look ----------
 *
 * Entered from FOLLOWING when subsystem 5 counts a failed pass (IF-15,
 * s5_fails).  That counter means a code the car really did drive over: a pass
 * has to get through at least BARCODE_FAIL_MIN_PCT of a whole code before it
 * counts, so a black line, a bump or the clipped edge of a code never brings
 * the car in here (barcode.h).
 *
 * What it does, in order:
 *   1. stop
 *   2. BACK UP straight along the line: both wheels held in step by the
 *      encoders (MOTION_CMD_DRIVE backwards), no steering -- see bc_reverse()
 *   3. stand still long enough for the pass to finish, and see what it said
 *   4. come FORWARD again over the code, steering by the line sensors, by
 *      the distance actually reversed plus a margin
 *   5. wait for that pass too
 *   6. hand over to line_search(), whatever either pass said
 *
 * Why the car does not need telling to "read it again": subsystem 5 samples
 * at 2 kHz the whole time and decodes anything that goes under the sensor, in
 * EITHER direction -- Code 39 is symmetrical and barcode.c reports a backwards
 * read as one (barcode_result_t.reversed).  So both legs are decode attempts
 * on their own, and all nav has to do is drive, then wait for the answer.
 *
 * Waiting is the part that is easy to get wrong.  A pass does not finish when
 * the code has gone past: it finishes BARCODE_END_GAP_MS (1 s) after the LAST
 * edge (barcode.h).  Checking the counters the moment a leg ends therefore
 * reads the answer to the previous question, so each leg's counters are
 * sampled before it starts and waited on afterwards.
 *
 * REVERSING DOES NOT STEER AT ALL.  Two versions tried to steer the reverse
 * leg by the line sensors -- one slowing the wheel on the side the sensor saw
 * the line, one swapping it -- and both walked the car off the line.  Neither
 * could work: the sensors are at the front, the TRAILING end going backwards,
 * and they say only where that end sits across the line, not which way the
 * car points.  A correction from them alone moves the trailing end the wrong
 * way before the right one, so the error grows whichever wheel is slowed.
 * So the reverse leg holds the heading the car stopped on -- which is along
 * the line, because it was following it -- with the encoders keeping both
 * wheels at the same speed, and the line sensors only report.  Going
 * FORWARD the sensors lead and steering by them is stable, so the forward
 * leg still does, with the follower's own pivot wound on by a ramp (see the
 * note on BC_STEER_OUTER below).
 */

/* Times round the loop.  One back-and-forward, as asked; it is a loop, so
   raising this does more of them, and the second one starts from wherever the
   first left the car. */
#define BC_RESCAN_TRIES		1

/* THE LENGTH OF THE PRINTED CODE, end to end.  Measure it with a ruler -- the
   course codes are a whole A4 sheet the long way, which is this.  It is the
   one number here that describes the world rather than the car, and the
   reverse leg is built out of it, so a different course means changing the
   TRUE figure and nothing else.
   It came off a ruler, so it goes through S3_TRUE_MM to become the distance
   this car has to be ASKED for to move it.  Everything past the pair works in
   the car's own scale, which is what the odometry reports and what the
   console prints. */
#define BC_CODE_TRUE_MM		297	/* ISO 216: A4 long edge, with a ruler */
#define BC_CODE_MM		S3_TRUE_MM(BC_CODE_TRUE_MM)

/* ...and how far short of the first bar to stop, so the sensor is on plain
   floor before the code when the car sets off forwards again.

   THIS IS MEASURED AT THE SENSOR, not at the wheels, and the difference is
   worth being clear about because the barcode sensor is mounted well ahead of
   the wheel axis.  The reverse leg starts from a mark dropped when the SENSOR
   last saw a bar (IF-14) and subtracts a length the SENSOR measured, so the
   distance between the sensor and the axle is on both sides of the sum and
   cancels: the sensor ends exactly this far behind the first bar, whatever
   that mounting distance is.  The WHEELS end up further back still -- by the
   mounting distance -- so seeing them well behind the paper while the sensor
   sits just off its edge is the arithmetic working, not a fault.

   So: to change where the SENSOR stops, change the TRUE figure.  Only if the
   distance wanted is measured at the wheels does the mounting distance have
   to be added to it. */
#define BC_BACK_MARGIN_TRUE_MM	15
#define BC_BACK_MARGIN_MM	S3_TRUE_MM(BC_BACK_MARGIN_TRUE_MM)

/* The whole sum above, times this (in per cent): a plain margin on top of the
   arithmetic, because being long costs nothing here -- the sensor simply
   crosses more plain floor before the code -- while being short means the leg
   ends on the paper.

   IT USED TO BE 145, AND THAT WAS NOT A MARGIN, IT WAS A SYMPTOM.  The note
   that came with it said the arithmetic kept leaving the sensor short of the
   first bar, so the leg was asked for half again as much.  It was: the sum is
   built out of ruler-measured lengths, those were being handed straight to an
   odometry that under-reads by 1.69x, and 1.45 was a floor-tuned guess at
   that 1.69 -- which is why it was STILL short even with the fudge in.

   S3_TRUE_MM does that conversion properly now, so the fudge has come out and
   this is back to being what it says it is.  Raising it is not the way to fix
   a leg that stops on the code: if that happens the scale is wrong, and
   S3_ODOM_TRUE_CPR is where it lives. */
#define BC_BACK_SCALE_PCT	110

/* A sanity cap on the reverse leg, however the arithmetic below comes out: a
   wild odometry reading must not send the car reversing across the room.
   Comfortably above what the scaled sum asks for, so it is a backstop and not
   a silent clip.  A metre of real floor, hence the conversion. */
#define BC_BACK_MAX_TRUE_MM	1000
#define BC_BACK_MAX_MM		S3_TRUE_MM(BC_BACK_MAX_TRUE_MM)

/* After a successful read on the way BACK, ignore this many finished passes
   before acting on one again.
   The car is behind the code when that happens, so following drives straight
   over it a second time -- and that second pass says nothing new.  Worse, if
   it fails (a code read cleanly going backwards can still fail going
   forwards) it would start another rescan, and the car would go round and
   round the same barcode.  So the next pass is swallowed, decoded or not; the
   command from the read that worked is already in hand. */
#define BC_IGNORE_AFTER_BACK_READ	1

/* Clear floor that proves the sensor really is off the code.
 *
 * THE DISTANCE ABOVE IS ONLY AS GOOD AS THE ODOMETRY, and this is the rule
 * that stops that mattering.  S3_TRUE_MM gets the arithmetic into the right
 * scale, but it is still arithmetic, resting on one measured constant and on
 * tyres that slip.
 *
 * So the distance is treated as a MINIMUM, and the sensor gets the last word:
 * the leg does not end while the barcode sensor has seen black within the
 * last BC_CLEAR_MM of travel.  Reversing off the end of the code, that means
 * the car keeps going until there is real white floor behind it, however
 * wrong the scale is.
 *
 * It has to be comfortably more than the widest white gap INSIDE a code, or
 * the leg could end in the middle of one: 29 elements across 297 mm makes the
 * widest about 19 mm, so 40.  Which also means that when this rule is what
 * stops the car, the clearance is 40 mm rather than BC_BACK_MARGIN_MM -- and
 * stopping 40 mm clear beats stopping on the paper.
 *
 * If the console says it fired on every run, the scale is out and
 * S3_ODOM_TRUE_CPR is the thing to re-measure, not this number. */
#define BC_CLEAR_TRUE_MM	40
#define BC_CLEAR_MM		S3_TRUE_MM(BC_CLEAR_TRUE_MM)

/* ...and how much further than the reverse leg to come forward, so the sensor
   is clear of the FAR end of the code when the car stops.  The forward leg
   measures itself off the distance actually reversed, so a reverse leg cut
   short still comes back the right amount. */
#define BC_FWD_EXTRA_TRUE_MM	60
#define BC_FWD_EXTRA_MM		S3_TRUE_MM(BC_FWD_EXTRA_TRUE_MM)

/* Both legs run at the speed the car follows the line at, not the faster one
   the obstacle manoeuvre uses.  The decoder is tuned around that speed:
   BARCODE_GLITCH_US throws away anything under 1 ms, so crossing a code twice
   as fast halves every element's time and brings the thin ones towards that
   floor. */
#define BC_SCAN_SPEED		STRAIGHT_SPEED

/* ---------------------------------------------------------------------
 *  Correcting during a rescan: THE SAME SHAPE AS LINE FOLLOWING, ramped.
 *  FORWARD LEG ONLY -- the reverse leg does not steer (bc_reverse()).
 *
 *  These are STEER_OUTER and STEER_INNER themselves -- the line follower's
 *  own numbers, not copies -- so the rescan steers the way the car steers
 *  everywhere else, and tuning the follower tunes this with it.  STEER_INNER
 *  is 0: the inside wheel STOPS and the car pivots around it while the
 *  outside wheel speeds up from STRAIGHT_SPEED to STEER_OUTER.
 *
 *  It used to be the soft pair (21/35, both wheels driven) on the grounds
 *  that reversing with the sensors at the trailing end is the trailer-backing
 *  problem, where the correction also swings the car further off line, so a
 *  hard pivot could jackknife rather than settle.
 *
 *  THE RAMP IS WHAT MAKES THE PIVOT SAFE TO USE HERE.  The jackknife comes
 *  from the suddenness, not from the inner wheel being stopped: a step change
 *  straight to 0/35 kicks the tail out faster than the sensors can report it.
 *  So the correction is not switched on, it is wound on -- bc_steer() walks
 *  the wheels from straight to full pivot over BC_RAMP_MS a step, and back
 *  again the same way, and a correction that flips from one side to the other
 *  passes through straight on its way rather than slamming across.
 *
 *  And the rescan is the one place that can afford it, because the path is
 *  KNOWN STRAIGHT -- one code on one straight stretch of line.  There is no
 *  real curve to follow, so every correction is a small one against drift,
 *  and the ramp has time to do its work before the error grows.
 *
 *  If it ever does jackknife, raise BC_STEER_INNER off zero -- that alone
 *  turns the pivot back into a curve, with the ramp still in place.  (And
 *  note subsystem 2 raises any non-zero wheel power to that wheel's learned
 *  stall floor, so a small non-zero inner value may not be as gentle as it
 *  looks -- with_floor() in task_motion.c.)
 * --------------------------------------------------------------------- */
#define BC_STEER_OUTER		STEER_OUTER	/* the follower's own 35 */
#define BC_STEER_INNER		STEER_INNER	/* ...and its 0: a pivot */

/* How the correction is wound on and off.  BC_STEER_FULL is just the scale
   the level is counted in; the pair below is the real setting -- a step of
   BC_RAMP_STEP every BC_RAMP_MS, so straight to full pivot takes
   (FULL / STEP) * MS = 250 ms, and so does coming back.

   FINER AND SLOWER THAN IT WAS (was 10 every 12 ms, 120 ms in ten steps).
   Ten steps across a 25 % power range is 2-3 % a step, which is coarse enough
   to feel on a car crawling at STRAIGHT_SPEED; at 4 every 10 ms the outer
   wheel moves one per cent at a time, 26 distinct powers on the way up, and
   the change stops being something you can see happen.
   Slower is also safer here for a reason beyond comfort: the correction has
   longer to show up in the sensors before it is fully wound on, so the loop
   is less able to overshoot and start hunting. */
#define BC_STEER_FULL		100
#define BC_RAMP_STEP		4
#define BC_RAMP_MS		10

/* Longer than one pass takes to finish, plus the slack for subsystem 5 to
   poll its edge queue (S5_POLL_MS) and decode. */
#define BC_PASS_WAIT_MS		(BARCODE_END_GAP_MS + 400)

/* The IF-15 counters as they were when the current leg began. */
LOCAL UW bc_reads0, bc_fails0;

LOCAL void bc_leg_start(void)
{
	bc_reads0 = s5_reads;
	bc_fails0 = s5_fails;
}

/* ---- The run's barcode bookkeeping ----
   Kept here rather than inside follow_line() because barcode_rescan() has to
   reach it too: the rescan is where a code is read on the way back, and the
   command it carries, and the pass that has to be ignored afterwards, all
   belong to the run and not to one leg. */
LOCAL UW bc_reads_seen, bc_fails_seen;	/* IF-15 counters already acted on */
LOCAL UB bc_pending_cmd;		/* what the next junction will obey */
LOCAL UB bc_ignore_passes;		/* finished passes still to be swallowed */

/* Take the counters as read: whatever subsystem 5 has finished up to now has
   been dealt with, and must not come round again as something new. */
LOCAL void bc_sync_seen(void)
{
	bc_reads_seen = s5_reads;
	bc_fails_seen = s5_fails;
}

LOCAL const char *bc_cmd_name(UB cmd)
{
	return (cmd == (UB)BARCODE_CMD_LEFT)    ? "LEFT" :
	       (cmd == (UB)BARCODE_CMD_RIGHT)   ? "RIGHT" :
	       (cmd == (UB)BARCODE_CMD_FORWARD) ? "FORWARD" :
	       (cmd == (UB)BARCODE_CMD_UTURN)   ? "U-TURN" : "nothing";
}

/* Point the wheels for the rescan's FORWARD leg (the reverse leg sends its
   one synced command itself and never steers -- bc_reverse()).
   Everything here goes through avoid_drive(): equal powers above zero go to
   MOTION_CMD_DRIVE, subsystem 2's encoder-synced mode; anything else goes
   straight to MOTION_CMD_WHEELS.  So
     straight               -> DRIVE, synced, as line following always was
     any correction         -> WHEELS, not synced, which is correct: a curve
                               is meant to happen and must not be synced away
   (Reversing used to come through here too, as unsynced WHEELS because DRIVE
   only went forwards, and the stronger motor walked the car off the line.
   DRIVE now runs backwards as well, synced the same way, and that is what
   bc_reverse() uses.) */
/* How much correction is on, and which way: -BC_STEER_FULL is hard left,
   +BC_STEER_FULL hard right, 0 straight.  SIGNED on purpose -- it is what
   makes a correction that changes sides pass through straight instead of
   slamming from one pivot to the other. */
LOCAL INT bc_level;
LOCAL INT bc_sent_l, bc_sent_r;		/* what the motors were last told */
LOCAL UW  bc_ramp_due;			/* when the level may next move */

/* Start a leg from straight, with nothing yet sent. */
LOCAL void bc_steer_reset(void)
{
	bc_level = 0;
	bc_sent_l = 0x7fff;
	bc_sent_r = 0x7fff;
	bc_ramp_due = app_now_ms();
}

/* Called EVERY pass of the leg's loop, not just when the line moves: the ramp
   needs the ticks.  It is cheap -- the level moves at most once per
   BC_RAMP_MS, and the motors are only told anything when the powers actually
   change, so the message buffer sees a few dozen commands per correction
   rather than one every LOOP_MS.
     dir   +1 driving forward, -1 backing up.  It sets the SIGN of the two
           powers and nothing else: which wheel is slowed does not depend on
           it (see the note at the top of the rescan).
     side  which side the line was seen on: -1 left, +1 right, 0 neither
   Returns the NAV_ACTION to publish. */
LOCAL UB bc_steer(INT dir, INT side)
{
	INT want = side * BC_STEER_FULL;
	INT a, slow, fast, l, r;

	/* Wind the level towards what the sensors are asking for. */
	if (bc_level != want && (W)(app_now_ms() - bc_ramp_due) >= 0) {
		bc_ramp_due = app_now_ms() + BC_RAMP_MS;
		if (bc_level < want) {
			bc_level += BC_RAMP_STEP;
			if (bc_level > want) {
				bc_level = want;
			}
		} else {
			bc_level -= BC_RAMP_STEP;
			if (bc_level < want) {
				bc_level = want;
			}
		}
	}

	/* Blend between going straight and the full pivot.  At level 0 both
	   wheels are on BC_SCAN_SPEED; at full the inner one is on
	   BC_STEER_INNER (stopped) and the outer on BC_STEER_OUTER. */
	a = (bc_level < 0) ? -bc_level : bc_level;
	slow = BC_SCAN_SPEED + ((BC_STEER_INNER - BC_SCAN_SPEED) * a) / BC_STEER_FULL;
	fast = BC_SCAN_SPEED + ((BC_STEER_OUTER - BC_SCAN_SPEED) * a) / BC_STEER_FULL;

	/* SLOW THE WHEEL ON THE SIDE THE SENSOR SAW THE LINE -- and note there
	   is no 'dir' in this, deliberately.  The wheel that does the work is
	   the same whichever way the car is travelling; only the sign applied
	   at the bottom changes.  See the note at the top of the rescan for why
	   that is right even though the nose swings the other way in reverse. */
	if (bc_level == 0) {
		l = BC_SCAN_SPEED;
		r = BC_SCAN_SPEED;
	} else if (bc_level < 0) {
		l = slow;		/* line out to the LEFT: slow the left */
		r = fast;
	} else {
		l = fast;
		r = slow;		/* line out to the RIGHT: slow the right */
	}

	if (l != bc_sent_l || r != bc_sent_r) {
		bc_sent_l = l;
		bc_sent_r = r;
		/* Straight goes to avoid_drive() with both powers equal, which
		   routes it to the encoder-synced mode going forward.  Anything
		   mid-ramp is unequal and is NOT synced, which is right: it is
		   a curve that is meant to happen. */
		avoid_drive(dir * l, dir * r);
	}
	/* Reported by where the car is being steered TOWARDS, which is the side
	   the line is on.  Going backwards the nose swings the other way, so
	   this is the car's path and not its nose -- the useful one to watch. */
	return (bc_level == 0) ? ACT_STRAIGHT :
	       (bc_level < 0)  ? ACT_STEER_LEFT : ACT_STEER_RIGHT;
}

/* Which way the line wants the car to yaw, from the two sensors.  Both on the
   line is a crossing or a wide mark: nothing to correct towards, so hold
   whatever the wheels are doing and let it pass. */
LOCAL INT bc_side(BOOL left, BOOL right)
{
	if (left && right) {
		return 0;
	}
	if (left) {
		return -1;		/* line out to the left: yaw left */
	}
	if (right) {
		return +1;
	}
	return 0;			/* straddling it: straight on */
}

/* Back up along the line: STRAIGHT BACK, BOTH WHEELS HELD IN STEP BY THE
   ENCODERS, AND NO STEERING.
 *
 * MOTION_CMD_DRIVE with both powers negative is subsystem 2's synced mode
 * running backwards: every S2_SYNC_MS it compares how far each wheel has
 * really turned since the leg began and slows whichever is ahead, so both
 * cover the same distance and the car keeps the heading it stopped on.  It
 * was not available before -- DRIVE clamped to 0..100, so a straight reverse
 * went out as MOTION_CMD_WHEELS, unsynced, and the stronger motor walked the
 * car off to one side.
 *
 * WHY THE LINE SENSORS DO NOT STEER IT.  They are at the front, which going
 * backwards is the TRAILING end, and they see only where that end sits
 * across the line -- not which way the car is pointing.  Any correction from
 * them alone swings the trailing end the wrong way first, then overshoots:
 * worked through, the loop is unstable whichever wheel is slowed, and on the
 * floor it walked the car off the line and left the forward leg nothing to
 * come back along.  The car was following the line when it stopped, so it
 * is already pointing along it; keeping that heading exactly is what retraces
 * the line, and that is a job for the encoders, not the sensors.  (The code
 * sits on a straight stretch -- see BC_STEER_OUTER above -- which is what
 * makes straight back the same thing as back along the line.)
 *
 * The sensors are still read the whole way and reported at the end -- how
 * much of the leg each one had the line under it -- so a run says whether
 * the car tracked the line or drifted across it.
 *
 * Stops after mm, or when something ends the run.  *done_mm is how far it
 * actually got, which is what the forward leg comes back by. */
LOCAL REQUEST bc_reverse(W mm, W *done_mm)
{
	ENC_MARK mark = enc_mark();
	ENC_MARK black_mark;		/* where the sensor last saw the code */
	UW	 t0 = app_now_ms();
	UW	 limit_ms = (UW)((BC_BACK_MAX_MM * 1000 / (AVOID_SLOWEST_CMS * 10)) *
				 AVOID_TIME_MARGIN) + 1000u;
	UW	 edges_seen = barcode_edges;
	BOOL	 held_on = FALSE;		/* the sensor kept the leg going */
	W	 done;
	INT	 polls = 0, l_on = 0, r_on = 0;	/* for the report at the end */
	ODOMETRY o;
	REQUEST	 r;

	black_mark = mark;
	*done_mm = 0;
	nav.action = ACT_STRAIGHT;
	motion_command(MOTION_CMD_DRIVE, -BC_SCAN_SPEED, -BC_SCAN_SPEED, 3);

	for (;;) {
		BOOL left  = line_left_on();
		BOOL right = line_right_on();

		/* Where the barcode sensor last had the code under it (IF-14):
		   an edge just now, or black right now. */
		if (barcode_edges != edges_seen || barcode_black != 0u) {
			edges_seen = barcode_edges;
			black_mark = enc_mark();
		}

		/* Far enough AND clear floor behind: done.  The distance alone is
		   not trusted to say where the sensor is -- see BC_CLEAR_MM.
		   Measured as TRAVEL, so the straightening spins below do not
		   count towards it. */
		done = moved_mm_since(mark);
		if (done >= BC_BACK_MAX_MM) {
			break;
		}
		if (done >= mm) {
			if (moved_mm_since(black_mark) >= BC_CLEAR_MM) {
				break;
			}
			held_on = TRUE;		/* still over the code: keep going */
		}

		/* Watched, not steered by -- see the note above. */
		nav.left = left;
		nav.right = right;
		polls++;
		if (left) {
			l_on++;
		}
		if (right) {
			r_on++;
		}

		/* No front guard -- twice over.  The car is going the other way
		   and nothing watches behind it (the sonar faces forward), and
		   in this build the obstacle manoeuvre is reachable from line
		   following and nowhere else.  What the car is backing over is
		   floor it has just driven along, which is the best reason
		   there is to believe it is clear.
		   Safety is still checked every pass: lifted, overturned, STOP
		   and RESET all end the leg here. */
		r = avoid_checks(FALSE);
		if (r != REQ_NONE) {
			*done_mm = moved_mm_since(mark);
			return r;
		}
		if (app_now_ms() - t0 > limit_ms) {
			motion_command(MOTION_CMD_STOP, 0, 0, 3);
			nav.action = ACT_STOP;
			tm_printf((UB*)"nav: backing up did not finish in %d ms "
				  "(wheels slipping or stalled) -- giving up\n",
				  (INT)limit_ms);
			*done_mm = moved_mm_since(mark);
			return REQ_ABORT;
		}
		tk_dly_tsk(LOOP_MS);
	}

	motion_command(MOTION_CMD_STOP, 0, 0, 3);
	nav.action = ACT_STOP;
	*done_mm = moved_mm_since(mark);		/* what it really did, not what it was asked */

	/* The two wheels' counts should match to within a few: that is the
	   sync working.  The sensor shares say whether the car stayed on the
	   line -- both near 0 means it straddled the line all the way back,
	   which is right; one climbing towards 100 means the car drifted that
	   way across it. */
	motion_get_odometry(&o);
	tm_printf((UB*)"nav:   backed up %d mm straight, wheels in step: left "
		  "%d counts, right %d.  Line under the left sensor %d%% of the "
		  "way, the right %d%%\n", (INT)*done_mm,
		  (INT)(mark.left - o.left), (INT)(mark.right - o.right),
		  polls ? (l_on * 100) / polls : 0,
		  polls ? (r_on * 100) / polls : 0);
	if (held_on) {
		tm_printf((UB*)"nav:   the sensor was still over the code at %d mm, "
			  "so it backed up %d mm in all -- check the distance "
			  "scale (S3_ODOM_TRUE_CPR, this file)\n",
			  (INT)mm, (INT)*done_mm);
	}
	return REQ_NONE;
}

/* Come forward over the code again: the encoders holding the two wheels in
   step while it is straight, and the line sensors correcting where it sits
   across the line -- which, going forward, they lead and can steer.
 *
 * No compass anywhere in this.  It used to stop every so often and turn the
 * car onto a heading read off the IMU, and that was what made it set off at an
 * angle: a magnetometer reading taken beside two motors, on a car that has
 * just been shuffling about, is worth less than the line under the sensors --
 * and a bad reading does not merely fail to help, it actively turns the car
 * away from a line it was already on.
 *
 * What makes the angle right is the reversing itself.  The reverse leg holds
 * the heading the car was following the line on, wheels in step, so it ends
 * with the car sitting on the line and pointing along it; driving forward
 * from there with the line still in
 * charge cannot wander off, because any drift puts a sensor on the line and is
 * corrected within a millimetre or two of travel.  There is nothing left for
 * an absolute heading to add.
 */
LOCAL REQUEST bc_forward(W mm)
{
	ENC_MARK mark = enc_mark();
	UW	 t0 = app_now_ms();
	UW	 limit_ms = (UW)((mm * 1000 / (AVOID_SLOWEST_CMS * 10)) *
				 AVOID_TIME_MARGIN) + 1000u;
	INT	 side = 0;
	REQUEST	 r;

	bc_steer_reset();			/* this leg starts straight */
	nav.action = bc_steer(+1, 0);

	while (moved_mm_since(mark) < mm) {
		BOOL left  = line_left_on();
		BOOL right = line_right_on();

		side = bc_side(left, right);		/* every pass: see above */
		nav.action = bc_steer(+1, side);
		nav.left = left;
		nav.right = right;

		/* NO FRONT GUARD, although this leg does go where the sonar
		   looks.  The obstacle manoeuvre is reachable from line
		   following and nowhere else, so nothing here reacts to the
		   sonar -- and nothing is lost by that, because subsystem 6
		   LATCHES what it sees.  An object spotted during this leg is
		   still sitting in the flag when follow_line() takes over a
		   fraction of a second later, which is where it gets handled,
		   by the manoeuvre, exactly as it would have been had the car
		   never left the line.
		   Safety -- lifted, overturned, STOP, RESET -- is checked every
		   pass regardless. */
		r = avoid_checks(FALSE);
		if (r != REQ_NONE) {
			return r;
		}
		if (app_now_ms() - t0 > limit_ms) {
			motion_command(MOTION_CMD_STOP, 0, 0, 3);
			nav.action = ACT_STOP;
			tm_printf((UB*)"nav: coming forward did not finish in %d "
				  "ms (wheels slipping or stalled) -- giving "
				  "up\n", (INT)limit_ms);
			return REQ_ABORT;
		}
		tk_dly_tsk(LOOP_MS);
	}

	motion_command(MOTION_CMD_STOP, 0, 0, 3);
	nav.action = ACT_STOP;
	return REQ_NONE;
}

/* Stand still until subsystem 5 has finished the pass for the leg that just
   ended, and say what it decided.  *decoded is TRUE only if a code actually
   read.  The run can still be ended while waiting. */
LOCAL REQUEST bc_wait_pass(const char *leg, BOOL *decoded)
{
	UW	t0 = app_now_ms();
	REQUEST	r;

	*decoded = FALSE;
	while (app_now_ms() - t0 < BC_PASS_WAIT_MS) {
		if (s5_reads != bc_reads0) {
			/* Take the command here, where the read is noticed: the
			   follow loop's own pickup is about to be told to ignore
			   this pass, and the command must not be lost with it. */
			bc_pending_cmd = s5_cmd;
			tm_printf((UB*)"nav:   %s: the code READ this time, and "
				  "says %s\n", leg, bc_cmd_name(bc_pending_cmd));
			*decoded = TRUE;
			return REQ_NONE;
		}
		if (s5_fails != bc_fails0) {
			tm_printf((UB*)"nav:   %s: it failed to decode again\n", leg);
			return REQ_NONE;
		}
		r = avoid_checks(FALSE);
		if (r != REQ_NONE) {
			return r;
		}
		tk_dly_tsk(IDLE_LOOP_MS);
	}
	tm_printf((UB*)"nav:   %s: no code went past the sensor at all\n", leg);
	return REQ_NONE;
}

/* The state.  REQ_NONE means the line is back between the sensors and
   following can carry on; anything else ended the run.
   past_code_mm is how far the car has driven since the LAST bar of the code
   went under the sensor -- measured, not guessed, because the failed pass only
   ended a second after that bar and the car kept following the line for all
   of it. */
LOCAL REQUEST barcode_rescan(W past_code_mm)
{
	W	back, backed = 0;
	INT	go;
	BOOL	decoded = FALSE, any = FALSE;
	REQUEST	r;

	/* Where the sensor has to get back to: behind the first bar.  That is
	   everything the car has covered since the last bar (which includes the
	   whole of BARCODE_END_GAP_MS of driving), plus the length of the code
	   it was leaving, plus a margin to be sure of clearing the first bar. */
	back = (past_code_mm + BC_CODE_MM + BC_BACK_MARGIN_MM) *
	       BC_BACK_SCALE_PCT / 100;
	if (back > BC_BACK_MAX_MM) {
		back = BC_BACK_MAX_MM;
	}

	motion_command(MOTION_CMD_STOP, 0, 0, 3);
	nav.action = ACT_STOP;
	nav.state = NAV_BARCODE_RESCAN;
	nav.mode = MODE_FOLLOW;
	publish();

	/* Distances here are in the car's own scale, not the ruler's: the code
	   is BC_CODE_TRUE_MM of paper, which is BC_CODE_MM as far as the wheels
	   are concerned (see S3_TRUE_MM).  Both are printed so a run can be
	   checked against a tape measure. */
	tm_printf((UB*)"nav: a barcode would not decode.  %d mm driven since its "
		  "last bar, code %d mm (%d on the ruler), so backing up %d mm "
		  "straight, both wheels held in step by the encoders\n",
		  (INT)past_code_mm, (INT)BC_CODE_MM, (INT)BC_CODE_TRUE_MM,
		  (INT)back);

	for (go = 0; go < BC_RESCAN_TRIES; go++) {
		/* (2,3) back along the line, then let the pass finish */
		bc_leg_start();
		r = bc_reverse(back, &backed);
		if (r != REQ_NONE) {
			goto out;
		}
		r = bc_wait_pass("backwards", &decoded);
		if (r != REQ_NONE) {
			goto out;
		}
		if (decoded) {
			/* Read going backwards, so there is nothing left to find
			   out by driving over it again: straight to the line
			   search from here.  The car is further back along the
			   line than it started, which the search does not mind --
			   it sweeps the nose until the line is between the
			   sensors, wherever that is.
			   Following will then take the car over this same code a
			   second time, so that pass is set to be ignored: it can
			   say nothing new, and if it failed it would start the
			   whole rescan again. */
			bc_ignore_passes = BC_IGNORE_AFTER_BACK_READ;
			tm_printf((UB*)"nav: read it going backwards -- no need for "
				  "the forward pass.  Looking for the line, and "
				  "ignoring the next %d pass over this code\n",
				  (INT)bc_ignore_passes);
			return line_search(NO_HEADING);
		}
		any = any || decoded;

		/* (4,5) it did NOT read going backwards, so come forward over it
		   again -- guided by the encoders and the line, the same two
		   things that brought it back.  THE ANGLE COMES FROM THE
		   REVERSING: that leg left the car on the line and pointing
		   along it, and the line sensors hold it there on the way out,
		   so there is no heading to aim at and nothing to turn to.
		   It comes forward by the distance actually reversed, so a
		   reverse leg cut short still returns the right amount. */
		tm_printf((UB*)"nav: coming forward %d mm on the encoders and the "
			  "line sensors\n", (INT)(backed + BC_FWD_EXTRA_MM));
		bc_leg_start();
		r = bc_forward(backed + BC_FWD_EXTRA_MM);
		if (r != REQ_NONE) {
			goto out;
		}
		r = bc_wait_pass("forwards", &decoded);
		if (r != REQ_NONE) {
			goto out;
		}
		any = any || decoded;
	}

	tm_printf((UB*)"nav: rescan done (%s) -- looking for the line again\n",
		  any ? "the code read" : "it still would not decode");

	/* (6) the line search, whatever the passes said */
	return line_search(NO_HEADING);

out:
	motion_command(MOTION_CMD_STOP, 0, 0, 3);
	nav.action = ACT_STOP;
	publish();
	return r;
}

/* ---------- JUNCTION: turn the way the last barcode said ----------
 *
 * A junction is BOTH line sensors over the line AT ONCE -- inside the CLEAR_MS
 * debounce that the two flags already carry, and no looser than that.
 *
 * A looser test was tried and taken out again.  It allowed the two sightings
 * to be up to JN_WINDOW_MM of travel apart, so that a crossing met at an angle
 * -- which puts one sensor on the line well before the other -- would still
 * count.  It did, but it also counted ordinary steering: a correction that
 * carries the car across the line touches one sensor and then the other, and
 * the car started turning off down imaginary junctions in the middle of normal
 * following.  Requiring both at the same instant is the only version that can
 * tell a crossing from a weave, so a diagonal junction has to be caught some
 * other way (a third sensor, or the angle the two sightings imply) rather than
 * by widening this window.
 *
 * What happens at one, in order:
 *   1. stop, and read the heading the car came in on (motors off: the
 *      magnetometer is no use while they turn, see avoid.h)
 *   2. creep forward JN_ADVANCE_MM, so the WHEELS are on the crossing before
 *      the car pivots on it -- the sensors are mounted ahead of the axle, so
 *      stopping where the sensors saw the junction leaves the axle short of it
 *   3. turn a quarter turn (a half turn for U-TURN, JN_UTURN_DEG), aimed at
 *      an ABSOLUTE heading worked out from (1).
 *      face() does it: the encoders turn the car, the compass trims within a
 *      bound, and with no GP21 calibration it falls back to the encoders
 *      alone and says so
 *   4. hand over to line_search(), which sweeps the nose until the new line
 *      is between the sensors again
 *
 * FORWARD, or nothing read yet, drives straight over the crossing and goes
 * back to following with no search at all: there is no turn, so the line still
 * runs out the far side and is already under the car.
 *
 * The command comes from the last code that decoded (IF-15, s5_cmd) and is
 * used ONCE: navigation takes a copy, and the copy is spent here.  Without
 * that, one "*L*" would turn the car left at every junction for the rest of
 * the run.
 */

/* ---------------------------------------------------------------------
 *  FROM THE IR LINE SENSORS BACK TO THE WHEEL AXLE, IN REAL MILLIMETRES.
 *  MEASURE IT: a ruler from the sensors' eyes (GP13 / GP14) back to the
 *  centre of the drive wheels.  Half the line's width on top (about 9 mm for
 *  electrical tape) makes it exact, but the ruler alone is close enough.
 *
 *  It is how far the car creeps after its sensors first see a line it means
 *  to turn onto, so that the AXLE -- the point it spins about -- is over the
 *  line when it turns.  Both the junction turn and the way back from an
 *  obstacle use it.
 *
 *  THIS NUMBER DECIDES WHETHER THE LINE SEARCH CAN WORK AT ALL.  A spin on
 *  the spot carries the sensors round a circle of this radius, so if the
 *  creep leaves the axle further from the line than the sensors are ahead of
 *  it, the sweep cannot touch the line however far it turns: the car sits
 *  "on" the line, sweeps left, sweeps right, and finds nothing.  Too short
 *  is gentler -- the line is found, but at an angle.
 *
 *  It used to be 65 % of a guessed 180 mm car length (117), which is where
 *  the default comes from.  If it is wrong, the obstacle manoeuvre measures
 *  it when its search fails (avoid_rejoin_from_far_side()) and prints the
 *  answer: "set S3_SENSOR_TO_AXLE_TRUE_MM to about N".
 * --------------------------------------------------------------------- */
#define S3_SENSOR_TO_AXLE_TRUE_MM	117

/* ...in the units the odometry counts in, which is what the moves take. */
#define JN_ADVANCE_MM		S3_TRUE_MM(S3_SENSOR_TO_AXLE_TRUE_MM)

/* The reverse of S3_TRUE_MM: a distance the odometry measured, as real mm,
   for the console. */
#define S3_REAL_MM(mm)	((W)(((W)(mm) * ENC_COUNTS_PER_REV) / S3_ODOM_TRUE_CPR))

/* A FORWARD command (or none) drives straight across instead: far enough that
   both sensors are clear of the crossing on the far side. */
#define JN_CROSS_TRUE_MM	70
#define JN_CROSS_MM		S3_TRUE_MM(JN_CROSS_TRUE_MM)

#define JN_TURN_DEG		90	/* a quarter turn, either way */
/* U-TURN: a half turn on the axle, then the same line search as a LEFT or
   RIGHT.  Negative spins left (anticlockwise, the way LEFT does); make it
   +180 to spin right instead. */
#define JN_UTURN_DEG		(-180)

/* There is no JN_COMPASS_SETTLE_MS any more, and its absence is the point.
 *
 * It used to be 1500 ms, waited twice: once before reading the heading the
 * car came in on, and once before face(), because the magnetometer is
 * useless for about a second after the motors stop and a junction turn comes
 * straight off the end of line following.  A turn that starts from a heading
 * 50 degrees wrong ends 50 degrees wrong.
 *
 * heading_settled() now does that properly for every heading in the file --
 * it waits for the reading to stop moving instead of waiting a fixed time,
 * which is both safer after a long drive and quicker after a short one.  So
 * the junction simply reads through it, like the manoeuvre does, and three
 * seconds of fixed delay per junction goes away. */

/* 1 = a junction turns the car the way the last barcode said and then looks
   for the line.  0 = what the car did before: drive straight across with the
   sensors ignored (MODE_CROSSING) and carry on following. */
#define S3_JUNCTION_TURN	1

/* Turn at a junction, then find the line again.  REQ_NONE means the line is
   back between the sensors and following can carry on. */
LOCAL REQUEST junction_turn(UB cmd)
{
	float	h0;
	INT	deg;
	REQUEST	r;

	motion_command(MOTION_CMD_STOP, 0, 0, 3);
	nav.action = ACT_STOP;
	nav.state = NAV_TURNING;
	nav.mode = MODE_FOLLOW;
	publish();

	/* (1) the heading the car came in on, read through heading_settled()
	   so it is a heading and not a picture of the motor field. */
	h0 = heading_settled();

	tm_printf((UB*)"nav: JUNCTION %d -- the barcode said %s.  %s\n",
		  (INT)nav.junctions,
		  (cmd == BARCODE_CMD_LEFT)    ? "LEFT" :
		  (cmd == BARCODE_CMD_RIGHT)   ? "RIGHT" :
		  (cmd == BARCODE_CMD_FORWARD) ? "FORWARD" :
		  (cmd == BARCODE_CMD_UTURN)   ? "U-TURN" : "nothing yet",
		  (h0 < 0.0f) ? "Turning by the encoders alone (no compass -- "
				"press GP21 once on a flat floor)"
			      : "Turning to a heading off the IMU");

	if (cmd == BARCODE_CMD_LEFT || cmd == BARCODE_CMD_RIGHT ||
	    cmd == BARCODE_CMD_UTURN) {
		/* (2) get the axle onto the crossing before pivoting on it:
		   65 % of the car's length forward from where the sensors saw
		   the junction. */
		tm_printf((UB*)"nav:   forward %d mm first (sensors %d mm ahead "
			  "of the axle), so the wheels are on the crossing\n",
			  (INT)JN_ADVANCE_MM, (INT)S3_SENSOR_TO_AXLE_TRUE_MM);
		nav.action = ACT_STRAIGHT;
		/* FALSE: no front guard.  See the note at the top of the file --
		   the obstacle manoeuvre belongs to line following, and what
		   subsystem 6 latched is still there when this returns. */
		r = avoid_move(STRAIGHT_SPEED, STRAIGHT_SPEED, JN_ADVANCE_MM, FALSE);
		if (r != REQ_NONE) {
			goto out;
		}

		/* (3) the quarter turn (half turn for U-TURN).  face() aims at an absolute heading, so
		   the turn does not inherit the error of the one before it; it
		   falls back to the encoders on its own if there is no compass,
		   which is why h0 is passed through even when it is -1.

		   face() settles its own readings now, so there is no delay to
		   add here: the creep forward above has only just finished, and
		   heading_settled() will wait out whatever field that left.

		   THE LEARNING IS OFF ACROSS THE CALL.  That reading is the
		   "before" half of the track-width learning, and track_mm is
		   the wheelbase the OBSTACLE manoeuvre computes every one of
		   its 90-degree spins from.  Letting a junction vote on it is
		   how the manoeuvre stopped turning 90 degrees.  face_learn is
		   put back immediately, before the result is even looked at, so
		   no path out of here can leave it off. */
		deg = (cmd == BARCODE_CMD_UTURN) ? JN_UTURN_DEG :
		      (cmd == BARCODE_CMD_LEFT)  ? -JN_TURN_DEG : +JN_TURN_DEG;
		face_learn = FALSE;
		r = face(h0 + (float)deg, deg);
		face_learn = TRUE;
		if (r != REQ_NONE) {
			goto out;
		}
	} else {
		/* NOTHING COMMANDED (or FORWARD, which is the same motion):
		   carry straight on over the crossing and go back to following
		   the line, with no line search at all.  There is no turn, so
		   the line still runs out the far side and is already between
		   the sensors -- sweeping the nose to find a line that is under
		   the car costs time and risks picking up the arm of the
		   junction instead. */
		nav.action = ACT_STRAIGHT;
		r = avoid_move(STRAIGHT_SPEED, STRAIGHT_SPEED, JN_CROSS_MM, FALSE);
		if (r != REQ_NONE) {
			goto out;
		}
		tm_printf((UB*)"nav:   straight over it -- following on\n");
		return REQ_NONE;
	}

	/* (4) turned: the old line is behind the car and the new one is off at
	   an angle, so it has to be found before following can resume.
	   Straight is the heading the car was just told to turn ONTO -- h0
	   plus the quarter turn -- for the same reason the manoeuvre hands its
	   own reference over: that is the direction the new line runs in, and
	   a fresh reading would only re-centre the sweep on whatever the turn
	   got wrong. */
	return line_search(h0 + (float)deg);

out:
	motion_command(MOTION_CMD_STOP, 0, 0, 3);
	nav.action = ACT_STOP;
	publish();
	return r;
}

/* Drive round the object.  Returns REQ_NONE only if it got all the way back
   to the line; anything else ended the run and the motors are already stopped.

   The three headings, all absolute:
     h0      the way the car was going when it stopped
     away_h  h0 turned 90 deg away from the object (out towards clear floor)
     in_h    h0 turned 90 deg the other way (back towards the line) */
/* ---------------------------------------------------------------------
 *  COMING BACK TO THE LINE: STOP WITH THE AXLE ON IT, NOT NEAR IT.
 *
 *  The car spins on its axle, and the IR sensors sit ahead of the axle.  So
 *  the ONLY place a spin on the spot can leave the car pointing along the
 *  line with the line between its sensors is with the axle on the line.
 *
 *  Anywhere else the sweep can still bring a sensor onto the line, but only
 *  by pointing the car ACROSS it.  With the axle e off the line and the
 *  sensors d ahead of the axle, the nose has to swing asin(e / d) to reach
 *  it: 5 cm off with the sensors about 12 cm ahead is about 25 degrees.  The
 *  search then stops with the line under its TRAILING sensor, the one about
 *  to lose it, and hands follow_line() a car crossing the line at 25
 *  degrees.  follow_line() drives a few mm straight to confirm the sighting,
 *  which carries that sensor off the line; both sensors read white, white on
 *  both sides means "drive straight", and the car drives off at that angle
 *  with the line behind it.  That is the car arriving "in the right place"
 *  and then drifting away beside the line.
 *
 *  The return leg used to drive blind: exactly what was stepped out, plus a
 *  deliberate 5 cm overshoot.  So the axle was 5 cm past the line even when
 *  every turn went perfectly, and further when they did not.
 *
 *  So the leg now WATCHES the line sensors.  The car drives square to the
 *  line, so the pair crosses it side by side.  The moment either sensor has
 *  it (LS_SEEN_READS polls running, the same rule the search believes) the
 *  car carries on JN_ADVANCE_MM further -- the junction turn's figure for
 *  exactly this job, sensors-over-the-line to axle-over-the-line -- counted
 *  from where the line was FIRST seen, so the time it takes to notice is
 *  inside that distance and not added on top.  The quarter turn to h0 then
 *  pivots ON the line, and the search after it is the same short sweep a
 *  junction turn ends with.  Measure S3_SENSOR_TO_AXLE_TRUE_MM and both get
 *  better together.
 *
 *  If that number is too big, the creep carries the axle out of the
 *  sweep's reach and the search finds nothing either side.  The car then
 *  comes at the line again from the far side and measures the right creep
 *  on the way: avoid_rejoin_from_far_side() below.
 *
 *  If the sensors never see it within AVOID_RETURN_SEEK_TRUE_MM past the
 *  distance stepped out, the car stops there and does what it always did:
 *  face the original heading and sweep.  That only happens if a sensor
 *  missed a line it drove straight across, or the line is further off
 *  than a whole allowance -- and the console says which run it was.
 *
 *  A REAL-CENTIMETRE allowance, which is why it goes through S3_TRUE_MM (see
 *  the note on S3_TRUE_MM near the top of this file).  Generous on purpose:
 *  the car stops at the line, not at the allowance, so a long one costs
 *  nothing when the line is there, while one too short throws away a line a
 *  few degrees of error on the long pass leg had moved further off.
 * --------------------------------------------------------------------- */
#define AVOID_RETURN_SEEK_TRUE_MM	150
#define AVOID_RETURN_SEEK_MM		S3_TRUE_MM(AVOID_RETURN_SEEK_TRUE_MM)

/* Step (8): drive forward, square to the line, until the sensors cross it,
   then on advance_mm further.  Front guard on throughout.
     max_mm      give up looking after this far (the car stops there)
     advance_mm  how far past the first sighting to stop: JN_ADVANCE_MM puts
                 the axle over the line; 0 stops as soon as it is seen
     *found      TRUE  = the line was seen
                 FALSE = max_mm went by with no line
     *seen_mm    how far the car had come when the line was first seen (0 if
                 it never was).  Next to the distance stepped out, it says
                 how true the way back was.
     *past_mm    how far the car went from that first sighting to where it
                 came to REST -- the advance plus the coast after the motors
                 were stopped, measured after they had.  0 if never seen.
   All three distances are odometry mm (S3_REAL_MM() turns them into real
   ones).  REQ_NONE either way; anything else ended the run and the motors
   are already stopped. */
LOCAL REQUEST avoid_return_to_line(W max_mm, W advance_mm, BOOL *found,
				   W *seen_mm, W *past_mm)
{
	ENC_MARK mark = enc_mark();
	ENC_MARK seen_at = mark;		/* where this run of black began */
	W	 seen_cnt = 0;			/* ...and how far in that was */
	const W	 target = MM_TO_COUNTS(max_mm);
	const W	 advance = MM_TO_COUNTS(advance_mm);
	UW	 t0 = app_now_ms();
	UW	 limit_ms = (UW)(((max_mm + advance_mm) * 1000 /
				  (AVOID_SLOWEST_CMS * 10)) * AVOID_TIME_MARGIN) +
			    1000u;
	INT	 run = 0;			/* polls running with black seen */
	REQUEST	 r;

	*found = FALSE;
	*seen_mm = 0;
	*past_mm = 0;
	if (max_mm <= 0) {
		return REQ_NONE;
	}

	nav.action = ACT_STRAIGHT;
	avoid_drive(AVOID_DRIVE_SPEED, AVOID_DRIVE_SPEED);

	while (*found ? (dist_since(seen_at) < advance)
		      : (dist_since(mark) < target)) {
		if (!*found) {
			if (line_left_on() || line_right_on()) {
				if (run == 0) {
					seen_at = enc_mark();
					seen_cnt = dist_since(mark);
				}
				if (++run >= LS_SEEN_READS) {
					*found = TRUE;
					*seen_mm = (W)((float)seen_cnt / COUNTS_PER_MM);
				}
			} else {
				run = 0;
			}
		}

		r = avoid_checks(TRUE);		/* safety, GP20, STOP, front */
		if (r != REQ_NONE) {
			return r;		/* motors already stopped */
		}
		if (app_now_ms() - t0 > limit_ms) {
			motion_command(MOTION_CMD_STOP, 0, 0, 3);
			nav.action = ACT_STOP;
			tm_printf((UB*)"nav: %s did not finish in %d ms (wheels "
				  "slipping or stalled) -- giving up\n",
				  avoid_step_name(step), (INT)limit_ms);
			return REQ_ABORT;
		}
		tk_dly_tsk(LOOP_MS);
	}

	motion_command(MOTION_CMD_STOP, 0, 0, 3);
	nav.action = ACT_STOP;
	if (*found) {
		/* Let the coast finish before measuring where the car came to
		   rest: that is the position the next turn pivots on. */
		tk_dly_tsk(AVOID_COMPASS_SETTLE_MS);
		*past_mm = moved_mm_since(seen_at);
	}
	return REQ_NONE;
}

/* ---------------------------------------------------------------------
 *  THE SEARCH FOUND NOTHING, BUT THE WAY BACK DID SEE THE LINE.
 *
 *  Then the creep carried the axle out of the sweep's reach: further past
 *  the line than the sensors are ahead of the axle, which is the only way a
 *  full sweep either side can miss a line that close (see the note on
 *  S3_SENSOR_TO_AXLE_TRUE_MM).  So S3_SENSOR_TO_AXLE_TRUE_MM is too big for
 *  this car, and the line is behind the axle on the side the car came from.
 *
 *  Come at it again from this side -- and MEASURE the creep on the way,
 *  which needs no ruler and no value for the sensor distance at all.  Along
 *  the way back, with the near edge of the line at 0, the line w wide and the
 *  sensors d ahead of the axle:
 *
 *    first time    sensors meet the near edge   axle at -d
 *                  car rolls on p               axle at p - d
 *    turned round  sensors start at p - 2d, meet the far edge (w) after t
 *                  so  p - 2d - t = w
 *    the creep that puts the axle on the middle of the line (w/2) from here:
 *                  (p - d - t) - w/2  =  (p - t) / 2
 *
 *  d and w cancel out.  Both p and t are straight-line encoder distances,
 *  which this car measures well, so (p - t) / 2 is a measured creep, and it
 *  is printed as the value S3_SENSOR_TO_AXLE_TRUE_MM should have.
 *
 *  The turn round is two quarter turns: the one the search ended on (the
 *  car is back at h0 when a search finds nothing) and one more, towards the
 *  side the car came from.  Then the car turns back to h0 and searches once
 *  more.  If that finds nothing too, the manoeuvre gives up and waits, as
 *  before.
 * --------------------------------------------------------------------- */
LOCAL REQUEST avoid_rejoin_from_far_side(float h0, float back_h,
					 INT back_deg, W past_mm)
{
	BOOL	found;
	W	t_mm, rest_mm, creep_mm, real_mm;
	REQUEST	r;

	tm_printf((UB*)"nav: no line within reach -- but the way back DID see "
		  "it, so the axle went %d mm past it, out of the sensors' "
		  "reach.  S3_SENSOR_TO_AXLE_TRUE_MM (%d) is too big for this "
		  "car.  Coming at the line again from this side to measure "
		  "it\n", (INT)S3_REAL_MM(past_mm),
		  (INT)S3_SENSOR_TO_AXLE_TRUE_MM);

	/* Driving again, sideways to the run: watch ahead, latch nothing
	   (line_search() puts it back the way follow_line() wants it). */
	obstacle_request(OBS_REQ_WATCH_AHEAD);

	avoid_at(AVOID_TURN_IN);
	r = face(back_h, back_deg);
	if (r != REQ_NONE) {
		return r;
	}

	/* Stop the moment it is seen (advance 0): the creep is worked out from
	   the distance this leg took, so it can only be done afterwards. */
	avoid_at(AVOID_RETURN);
	r = avoid_return_to_line(past_mm + AVOID_RETURN_SEEK_MM, 0, &found,
				 &t_mm, &rest_mm);
	if (r != REQ_NONE) {
		return r;
	}
	if (!found) {
		tm_printf((UB*)"nav: drove %d mm back towards it and never saw the "
			  "line -- giving up\n",
			  (INT)S3_REAL_MM(past_mm + AVOID_RETURN_SEEK_MM));
		return REQ_ABORT;
	}

	creep_mm = (past_mm - t_mm) / 2;
	real_mm = S3_REAL_MM(creep_mm);
	if (real_mm < 20 || real_mm > 250) {
		/* Not a sensor distance any car of this size could have, so a
		   turn went wrong somewhere along the way; the numbers say
		   nothing about the car.  Creep the configured amount. */
		tm_printf((UB*)"nav:     line seen again after %d mm, which makes "
			  "the creep %d mm -- not believable, using %d\n",
			  (INT)S3_REAL_MM(t_mm), (INT)real_mm,
			  (INT)S3_SENSOR_TO_AXLE_TRUE_MM);
		creep_mm = JN_ADVANCE_MM;
	} else {
		tm_printf((UB*)"nav:     line seen again after %d mm: creep %d mm "
			  "to put the axle on it.  SET S3_SENSOR_TO_AXLE_TRUE_MM "
			  "TO ABOUT %d (task_nav.c), it is %d now\n",
			  (INT)S3_REAL_MM(t_mm), (INT)real_mm, (INT)real_mm,
			  (INT)S3_SENSOR_TO_AXLE_TRUE_MM);
	}

	/* The car has already rolled rest_mm past the sighting while it
	   stopped; creep the rest. */
	if (creep_mm > rest_mm) {
		r = avoid_straight(creep_mm - rest_mm, TRUE);
		if (r != REQ_NONE) {
			return r;
		}
	}

	avoid_at(AVOID_TURN_AHEAD);
	r = face(h0, -back_deg);
	if (r != REQ_NONE) {
		return r;
	}

	nav.avoid_step = (UB)AVOID_FINISHED;
	publish();
	obstacle_request(OBS_REQ_CLEAR);
	return line_search(h0);
}

LOCAL REQUEST avoid_manoeuvre(void)
{
	float	h0, away_h, in_h;
	INT	away_deg, in_deg, tries;
	W	stepped = 0;		/* total distance stepped aside */
	W	seen_mm = 0;		/* way back: where the line showed up */
	W	past_mm = 0;		/* ...and how far past it the car came
					   to rest */
	BOOL	clear;
	BOOL	on_line = FALSE;	/* way back: axle put over the line */
	REQUEST	r;

	obstacle_get_status(&obs_copy);
	avoid_plan(obs_copy.object.found ? TRUE : FALSE,
		   obs_copy.object.length_mm, obs_copy.profile.side, &plan);

	/* Going round the LEFT of the object means turning left first, which is
	   anticlockwise, which the compass counts as -90. */
	away_deg = (plan.side == AVOID_LEFT) ? -90 : +90;
	in_deg = -away_deg;

	/* THE HEADING EVERY TURN IN THIS MANOEUVRE AIMS AT, so it is read the
	   same careful way as the headings the turns are checked against --
	   settled and median-filtered, not one raw sample.
	   It matters more here than anywhere else in the file.  away_h and in_h
	   are both worked out from it ONCE and then used by all four turns, so
	   an error in this single reading is not a wobble on one turn: it is a
	   constant bias on every turn of the manoeuvre, pointing the car the
	   same wrong amount each time.  And no amount of trimming can see it,
	   because the trim is doing exactly what it was told -- aiming at a
	   target that was wrong before the first wheel moved.
	   With S3_AVOID_USE_COMPASS 0 there is no reading at all (-1), and
	   every turn below is the encoders' alone.  compass_allowed goes back
	   to TRUE on every way out of this function, before anything else can
	   read a heading. */
	compass_allowed = S3_AVOID_USE_COMPASS ? TRUE : FALSE;
	h0 = heading_settled();
	away_h = h0 + (float)away_deg;
	in_h = h0 + (float)in_deg;

	tm_printf((UB*)"nav: going round to the %s.  Object %d mm wide (%s), "
		  "stepping %d mm aside, track %d mm, %s\n",
		  (plan.side == AVOID_LEFT) ? "LEFT" : "RIGHT",
		  (INT)plan.width_mm,
		  plan.assumed ? "assumed -- the scan was no good" : "measured",
		  (INT)plan.lateral_mm, (INT)track_mm,
		  (h0 >= 0.0f) ? "turning to compass headings"
		  : compass_allowed ? "NO COMPASS: turning by the encoders alone"
				    : "turning by the encoders alone (compass "
				      "off: S3_AVOID_USE_COMPASS)");

	/* Subsystem 6 keeps reporting the distance ahead but stops latching:
	   the object is right in front of the car now, so a latch would come
	   straight back and stop the manoeuvre before it began.  From here on
	   nav judges every reading itself. */
	obstacle_request(OBS_REQ_WATCH_AHEAD);
	motion_command(MOTION_CMD_ENABLE, 0, 0, 3);
	tk_dly_tsk(OBSTACLE_RECHECK_MS);

#if S3_AVOID_TURN_TEST
	tm_printf((UB*)"nav: TURN TEST -- one 90 deg turn, then stop.  Measure "
		  "the angle and adjust AVOID_TRACK_MM (avoid.h)\n");
	avoid_at(AVOID_TURN_OUT);
	(void)face(away_h, away_deg);
	r = REQ_STOP;
	goto out;
#endif

	/* ---- (1,2,3,4) aside, until the way along the line is clear ---- */
	avoid_at(AVOID_TURN_OUT);
	r = face(away_h, away_deg);
	if (r != REQ_NONE) {
		goto out;
	}

	clear = FALSE;
	for (tries = 0; tries < AVOID_MAX_TRIES; tries++) {
		W this_step = (tries == 0) ? plan.lateral_mm
					   : (W)AVOID_EXTRA_STEP_MM;

		avoid_at(AVOID_SIDESTEP);
		r = avoid_straight(this_step, TRUE);
		if (r != REQ_NONE) {
			goto out;
		}
		stepped += this_step;

		avoid_at(AVOID_TURN_ALONG);
		r = face(h0, in_deg);
		if (r != REQ_NONE) {
			goto out;
		}

		avoid_at(AVOID_CHECK_AHEAD);
		if (ahead_is_clear("stepped aside")) {
			clear = TRUE;
			break;
		}

		/* Not enough room yet: back out and step further. */
		avoid_at(AVOID_TURN_OUT);
		r = face(away_h, away_deg);
		if (r != REQ_NONE) {
			goto out;
		}
	}
	if (!clear) {
		tm_printf((UB*)"nav: %d mm aside and it is still in the way -- "
			  "that is a wall, not an obstacle.  Giving up\n",
			  (INT)stepped);
		r = REQ_ABORT;
		goto out;
	}

	/* ---- (5,6,7) along, until turning in finds nothing in front ---- */
	avoid_at(AVOID_PASS);
	r = avoid_straight(AVOID_PASS_MM, TRUE);
	if (r != REQ_NONE) {
		goto out;
	}

	clear = FALSE;
	for (tries = 0; tries < AVOID_MAX_TRIES; tries++) {
		avoid_at(AVOID_TURN_IN);
		r = face(in_h, in_deg);
		if (r != REQ_NONE) {
			goto out;
		}

		avoid_at(AVOID_CHECK_PAST);
		if (ahead_is_clear("turned in towards the line")) {
			clear = TRUE;
			break;
		}

		/* Still beside it: face along again and drive further. */
		avoid_at(AVOID_TURN_ALONG);
		r = face(h0, away_deg);
		if (r != REQ_NONE) {
			goto out;
		}
		avoid_at(AVOID_PASS);
		r = avoid_straight(AVOID_EXTRA_ALONG_MM, TRUE);
		if (r != REQ_NONE) {
			goto out;
		}
	}
	if (!clear) {
		tm_printf((UB*)"nav: drove %d mm along and still cannot turn in "
			  "-- giving up\n",
			  (INT)(AVOID_PASS_MM + AVOID_MAX_TRIES * AVOID_EXTRA_ALONG_MM));
		r = REQ_ABORT;
		goto out;
	}

	/* ---- (8,9) back until the AXLE is on the line, then face along it ----
	   Watching the line sensors, not counting out a distance: see
	   avoid_return_to_line() above for why the axle has to be ON the line
	   before the car turns, and not merely near it. */
	avoid_at(AVOID_RETURN);
	r = avoid_return_to_line(stepped + AVOID_RETURN_SEEK_MM, JN_ADVANCE_MM,
				 &on_line, &seen_mm, &past_mm);
	if (r != REQ_NONE) {
		goto out;
	}
	if (on_line) {
		tm_printf((UB*)"nav:     line seen %d mm in (%d stepped out) -- "
			  "on %d mm (S3_SENSOR_TO_AXLE_TRUE_MM) to put the axle "
			  "over it; came to rest %d mm past the sighting\n",
			  (INT)seen_mm, (INT)stepped,
			  (INT)S3_SENSOR_TO_AXLE_TRUE_MM,
			  (INT)S3_REAL_MM(past_mm));
	} else {
		tm_printf((UB*)"nav:     %d mm back (%d stepped out) and the "
			  "sensors never saw the line -- turning and sweeping "
			  "for it from here\n",
			  (INT)(stepped + AVOID_RETURN_SEEK_MM), (INT)stepped);
	}

	avoid_at(AVOID_TURN_AHEAD);
	r = face(h0, away_deg);
	if (r != REQ_NONE) {
		goto out;
	}

	tm_printf((UB*)"nav: back %s the line, facing the way we were going\n",
		  on_line ? "ON" : "somewhere near");
	nav.avoid_step = (UB)AVOID_FINISHED;
	publish();
	obstacle_request(OBS_REQ_CLEAR);
	/* STRAIGHT IS h0 -- the one heading read at the top of this function,
	   before the car turned a wheel.  Every turn of the manoeuvre has aimed
	   at an absolute heading worked out from it, and the search gets the
	   same reference rather than a fresh reading of where the car happens
	   to have ended up. */
	r = line_search(h0);

	/* Nothing either side, yet the sensors saw the line on the way back:
	   the creep overshot it out of reach.  Go back for it, measuring the
	   creep this car really needs. */
	if (r == REQ_ABORT && on_line) {
		r = avoid_rejoin_from_far_side(h0, away_h, away_deg, past_mm);
		if (r != REQ_NONE) {
			goto out;
		}
	}
	compass_allowed = TRUE;
	return r;

out:
	motion_command(MOTION_CMD_STOP, 0, 0, 3);
	obstacle_request(OBS_REQ_CLEAR);
	nav.avoid_step = (UB)AVOID_FINISHED;
	publish();
	compass_allowed = TRUE;
	return r;
}

/* ---------- AVOIDING: stopped in front of an obstacle (IF-11) ----------
   Subsystem 6 has latched something within S6_STOP_MM of the car, so the run
   pauses here with the motors off and the obstacle flag set.
     GP20                 END THE RUN.  The button means the same thing here
                          as everywhere else the car is moving or waiting:
                          stop.  To go again, press it a second time from the
                          standstill -- that clears the latch and starts line
                          following afresh.  It used to resume in place, which
                          made one button mean "stop" in five states and
                          "carry on" in this one.
     START                clear every latched flag and carry on following, in
                          place.  The broker keeps the old behaviour, because
                          START and STOP are already two separate commands
                          there and neither has to do double duty.
     STOP / RESET         end the run
     lifted / overturned  end the run (subsystem 4 has braked already)
   The avoidance manoeuvre itself is not written yet (TEAM.md section 1).
   What this state does do is the part any manoeuvre needs first: it asks
   subsystem 6 to sweep the servo and look around (IF-12), which prints the
   scan and leaves the numbers in OBS_STATUS for whoever writes the driving. */

/* Waits in AVOIDING for whatever ends it.  REQ_NONE = carry on (START). */
LOCAL REQUEST wait_in_avoid(void)
{
	BOOL	said = FALSE;	/* the scan result is reported once */
	UINT	bits;
	UB	c;

	while (1) {
		/* Subsystem 6 has finished measuring: say how big the thing in the
		   way is, and which side has more room.  The full table is on the
		   console already (the [s6] block) -- this is the one-line version,
		   and the numbers stay in OBS_STATUS for the manoeuvre to use. */
		if (!said && (obs_flags() & S6_FLG_SCAN_DONE)) {
			said = TRUE;
			obstacle_get_status(&obs_copy);
			if (obs_copy.object.found) {
				tm_printf((UB*)"nav: the object is about %d mm long "
					  "(%d..%d deg, nearest %d mm); more room %s\n",
					  (INT)obs_copy.object.length_mm,
					  (INT)obs_copy.object.right_deg,
					  (INT)obs_copy.object.left_deg,
					  (INT)obs_copy.object.near_mm,
					  (obs_copy.profile.side == AVOID_LEFT) ? "LEFT" : "RIGHT");
			} else {
				tm_printf((UB*)"nav: the scan found nothing to measure "
					  "-- the object may have been taken away\n");
			}
		}

		bits = safety_bits();
		if (bits) {
			print_safety(bits);
			return REQ_SAFETY;
		}
		if (button_pressed(S3_START_BUTTON_PIN)) {
			/* GP20 STOPS, it does not resume.  See the note above. */
			wait_release(S3_START_BUTTON_PIN);
			return REQ_STOP;
		}
		c = poll_remote();
		if (c == NAV_CMD_STOP) {
			return REQ_STOP;
		}
		if (c == NAV_CMD_RESET) {
			return REQ_RESET;
		}
		if (c == NAV_CMD_START) {
			return REQ_NONE;
		}
		read_idle_sensors();
		publish();
		tk_dly_tsk(IDLE_LOOP_MS);
	}
}

LOCAL REQUEST avoid_obstacle(void)
{
	REQUEST	req;
	INT	i;

	motion_command(MOTION_CMD_STOP, 0, 0, 3);
	nav.action = ACT_STOP;
	nav.state = NAV_AVOIDING;
	publish();

	obstacle_get_status(&obs_copy);
	tm_printf((UB*)"nav: OBSTACLE %d mm ahead (limit %d mm) -- stopped.  "
		  "GP20 ends the run (press it again to start afresh); "
		  "START carries on from here\n",
		  (INT)obs_copy.stop_mm, S6_STOP_MM);

	/* Have subsystem 6 sweep the servo: where the object is, how long it is
	   and which side has room.  It takes about 3 s and the car is standing
	   still anyway.  wait_in_avoid() prints the answer when it arrives. */
	obstacle_request(OBS_REQ_SCAN);

#if S3_AVOID_AUTO
	/* Wait for the scan to finish, then go round it.  The scan is what
	   decides the side and the width, so it is worth the ~3 s. */
	for (i = 0; i < 4000 / 10; i++) {
		if (obs_flags() & S6_FLG_SCAN_DONE) {
			break;
		}
		req = wait_in_avoid_poll();	/* safety / STOP / button only */
		if (req != REQ_NONE) {
			return req;
		}
		tk_dly_tsk(10);
	}

	req = avoid_manoeuvre();
	if (req == REQ_NONE) {
		nav.state = NAV_FOLLOWING;
		nav.mode = MODE_FOLLOW;
		publish();
		return REQ_NONE;
	}
	if (req != REQ_ABORT) {
		return req;			/* safety, STOP or RESET */
	}
	/* Could not finish: fall through and wait for a human, as before. */
	nav.state = NAV_AVOIDING;
	publish();
	tm_printf((UB*)"nav: waiting.  START to carry on from here; GP20 or STOP "
		  "to end the run -- GP20 again then starts a fresh one\n");
#endif

	while (1) {
		req = wait_in_avoid();
		if (req != REQ_NONE) {
			return req;		/* the run ends here */
		}

		/* GP20 / START: clear every latched flag in the car, subsystem 6's
		   obstacle among them (clear_errors sends OBS_REQ_CLEAR). */
		clear_errors();
		for (i = 0; i < OBSTACLE_CLEAR_WAIT_MS / 10; i++) {
			if (!(obs_flags() & S6_FLG_OBSTACLE)) {
				break;
			}
			tk_dly_tsk(10);
		}

		/* Let subsystem 6 look again before trusting the road: if the
		   object is still in front of the car it latches once more, and
		   then this press keeps us waiting here instead of driving into
		   it.  (A sensor that has died does NOT trap the car here: an
		   unknown reading is never an obstacle.) */
		tk_dly_tsk(OBSTACLE_RECHECK_MS);
		if (!(obs_flags() & S6_FLG_OBSTACLE)) {
			break;
		}
		tm_printf((UB*)"nav: the object is still there -- move it out of the "
			  "way, then send START again\n");
	}

	nav.state = NAV_FOLLOWING;
	nav.mode = MODE_FOLLOW;
	publish();
	tm_printf((UB*)"nav: road clear, following the line again\n");
	return REQ_NONE;
}

/* ---------- FOLLOWING: line following until GP20, STOP, RESET or safety ---------- */
LOCAL REQUEST follow_line(void)
{
	const W	confirm_counts = MM_TO_COUNTS(CONFIRM_MM);
	const W	cross_counts = MM_TO_COUNTS(CROSS_MM);
	const W	cross_max_counts = MM_TO_COUNTS(CROSS_MAX_MM);

	INT	steer_side = 0;
	UW	steer_start = 0;
	INT	confirm_side = 0;
	ENC_MARK mark;
	UW	start, now, left_seen, right_seen;
	REQUEST	stop_req = REQ_NONE;
	UINT	bits;
	UB	c;
	BOOL	soft = FALSE;		/* a barcode is going past: steer gently */
	BOOL	soft_now = FALSE;	/* what the motors were last told */
	BOOL	soft_seen = FALSE;	/* what the console was last told */
	BOOL	soft_block = FALSE;	/* softening given up on: sensor stuck on black */
	BOOL	crossing;
	UW	soft_ms = 0;		/* how long this run steered gently */
	UW	soft_t0 = 0;		/* when the current soft spell started */
	UW	soft_edges0 = 0;	/* edges counted when it started */
	/* IF-14: where the car was when the last bar went past the barcode
	   sensor, and the edge count that marked it.  A rescan has to put the
	   sensor back BEFORE the code, and the failed pass only ended a whole
	   BARCODE_END_GAP_MS after its last bar -- a second in which the car
	   kept driving -- so that distance is MEASURED here rather than worked
	   out from a speed the car does not really know. */
	UW	bc_edges_seen = barcode_edges;
	ENC_MARK bc_edge_mark;

	/* A new run: forget a lift from before (the car was put down to start
	   it), and let the motors move again after a safety stop. */
	imu_request(IMU_REQ_RESET_LIFT);
	tk_clr_flg(flg_imu, ~(UINT)IMU_FLG_SAFETY);
	/* IF-15: start the run having "dealt with" whatever subsystem 5 has
	   finished so far, so a code that failed before this run -- or while the
	   car sat in IDLE -- does not send it straight into a rescan. */
	bc_sync_seen();
	bc_pending_cmd = (UB)BARCODE_CMD_NONE;
	bc_ignore_passes = 0;
	motion_command(MOTION_CMD_ENABLE, 0, 0, 3);

	nav.state = NAV_FOLLOWING;
	nav.action = ACT_STOP;
	nav.mode = MODE_FOLLOW;
	nav.avoid_step = (UB)AVOID_FINISHED;
	mark = enc_mark();
	bc_edge_mark = mark;
	start = app_now_ms();
	left_seen = start - CLEAR_MS;
	right_seen = start - CLEAR_MS;
	tm_printf((UB*)"nav: line following started\n");

	while (1) {
		/* Safety first: checked before any motor command in this pass */
		bits = safety_bits();
		if (bits) {
			motion_command(MOTION_CMD_STOP, 0, 0, 3);
			print_safety(bits);
			stop_req = REQ_SAFETY;
			break;
		}
		/* Then the road ahead (IF-11): subsystem 6 has latched something
		   within S6_STOP_MM, so stop and wait in NAV_AVOIDING. */
		if (obs_flags() & S6_FLG_OBSTACLE) {
			REQUEST r = avoid_obstacle();

			if (r != REQ_NONE) {
				stop_req = r;
				break;
			}
			/* Carrying on: start the follow state fresh, as at the
			   beginning of a run (the car stood still for a while).
			   The bar mark comes with it: the manoeuvre has driven
			   the car round a box since the last bar, which is not
			   distance a rescan should try to reverse. */
			nav.mode = MODE_FOLLOW;
			steer_side = 0;
			mark = enc_mark();
			bc_edge_mark = mark;
			bc_edges_seen = barcode_edges;
			now = app_now_ms();
			left_seen = now - CLEAR_MS;
			right_seen = now - CLEAR_MS;
			continue;
		}
		/* Keep a mark on the floor at the last bar seen (IF-14), so a
		   rescan knows how far the car has come since the code without
		   having to assume a speed. */
		if (barcode_edges != bc_edges_seen) {
			bc_edges_seen = barcode_edges;
			bc_edge_mark = enc_mark();
		}

		/* One place for "subsystem 5 has finished a pass" (IF-15), so
		   that ignoring a pass means ignoring it whatever it said.  A
		   pass steps exactly one of the two counters. */
		if (s5_reads != bc_reads_seen || s5_fails != bc_fails_seen) {
			BOOL got = (s5_reads != bc_reads_seen);

			bc_sync_seen();
			if (bc_ignore_passes > 0) {
				/* The second look at a code that was already read
				   going backwards.  Swallowed whichever way it
				   went: it can say nothing new, and a failure here
				   would start the rescan over again. */
				bc_ignore_passes--;
				tm_printf((UB*)"nav: ignoring this pass over the code "
					  "already read (%s)%s\n",
					  got ? "it decoded" : "it failed",
					  bc_ignore_passes ? "" : " -- watching again now");
			} else if (got) {
				/* Remember what it commands until a junction turns
				   on it.  A code carrying no command still lands
				   here as BARCODE_CMD_NONE and replaces an older
				   one: the last code read is the one in force. */
				bc_pending_cmd = s5_cmd;
				tm_printf((UB*)"nav: barcode says %s -- holding it "
					  "for the next junction\n",
					  bc_cmd_name(bc_pending_cmd));
			} else if (bc_rescan_muted()) {
				/* A line search has just swept the sensor along
				   the line, and this is what it shook out: bars
				   and gaps that were never a code.  Dropped, so
				   the car does not reverse up the line hunting a
				   barcode that does not exist. */
				tm_printf((UB*)"nav: a pass failed just after the "
					  "line search -- that is the line under "
					  "the sensor, not a code.  Ignoring it\n");
			} else {
				/* It would not decode, so back up and look again.
				   Only a pass that got most of the way through a
				   code counts as a failure, so a black line or a
				   bump never brings the car in here. */
				REQUEST r = barcode_rescan(moved_mm_since(bc_edge_mark));

				/* The rescan's own passes land in the same counters,
				   and so do the ones the line search's spin shakes
				   out of the line, so take them as read now -- the
				   command, if a leg found one, is already held. */
				bc_sync_seen();
				if (r != REQ_NONE) {
					stop_req = r;
					break;
				}
				/* Back on the line: start the follow state fresh,
				   as after an obstacle (the car has been still a
				   while).  The bar mark goes back to here too --
				   the rescan drove the car a long way, and a mark
				   left over from before it would make the NEXT
				   rescan reverse all of that as well. */
				nav.state = NAV_FOLLOWING;
				nav.mode = MODE_FOLLOW;
				steer_side = 0;
				mark = enc_mark();
				bc_edge_mark = mark;
				bc_edges_seen = barcode_edges;
				now = app_now_ms();
				left_seen = now - CLEAR_MS;
				right_seen = now - CLEAR_MS;
				continue;
			}
		}
		if (button_down(S3_START_BUTTON_PIN)) {
			stop_req = REQ_STOP;
			break;
		}
		c = poll_remote();
		if (c == NAV_CMD_STOP || c == NAV_CMD_RESET) {
			stop_req = (c == NAV_CMD_RESET) ? REQ_RESET : REQ_STOP;
			break;
		}

		now = app_now_ms();

		/* IF-14: a bar is going past the barcode sensor, so correct gently
		   and do not spin -- a pivot would whip the sensor across the code
		   and lose the read. */
		crossing = barcode_crossing();
		if (!crossing) {
			soft_block = FALSE;	/* sensor sees white again: allow it */
		} else if (soft_seen && (now - soft_t0) >= SOFT_MAX_MS) {
			soft_block = TRUE;	/* stuck on black: steer properly */
		}
#if S3_BARCODE_SOFT
		soft = (crossing && !soft_block) ? TRUE : FALSE;
#else
		soft = FALSE;			/* steer the same whatever the sensor sees */
#endif
		if (soft) {
			soft_ms++;		/* the loop runs every LOOP_MS = 1 ms */
		}
		if ((crossing && !soft_block) != soft_seen) {
			soft_seen = (crossing && !soft_block) ? TRUE : FALSE;
#if S3_BARCODE_TRACE
			if (soft_seen) {
				soft_t0 = now;
				soft_edges0 = (UW)barcode_edges;
				tm_printf((UB*)"nav: bar under the barcode sensor%s\n",
					  S3_BARCODE_SOFT ? " -- steering gently" : "");
			} else {
				tm_printf((UB*)"nav: barcode gone (%d ms, %d bars/gaps)%s\n",
					  (INT)(now - soft_t0),
					  (INT)((UW)barcode_edges - soft_edges0),
					  soft_block ? " GAVE UP, sensor still black" : "");
			}
#else
			(void)soft_t0; (void)soft_edges0;
#endif
		}

		if (line_left_on())  left_seen = now;
		if (line_right_on()) right_seen = now;
		BOOL left  = (now - left_seen) < CLEAR_MS;
		BOOL right = (now - right_seen) < CLEAR_MS;
		nav.left = left;
		nav.right = right;

		UB want = ACT_STRAIGHT;

		/* A junction: both sensors on the line AT THE SAME TIME, within
		   the CLEAR_MS debounce the two flags already carry.  See the
		   note on junction_turn() for why it is no looser than this. */
		if (nav.mode != MODE_CROSSING && left && right) {
			nav.junctions++;
#if S3_JUNCTION_TURN
			{
				REQUEST r = junction_turn(bc_pending_cmd);

				bc_pending_cmd = (UB)BARCODE_CMD_NONE;	/* spent */
				bc_sync_seen();		/* the turn and the sweep
							   shake bars out of the
							   line; none of them is a
							   code to act on */
				if (r != REQ_NONE) {
					stop_req = r;
					break;
				}
				nav.state = NAV_FOLLOWING;
				nav.mode = MODE_FOLLOW;
				steer_side = 0;
				mark = enc_mark();
				bc_edge_mark = mark;
				bc_edges_seen = barcode_edges;
				now = app_now_ms();
				left_seen = now - CLEAR_MS;
				right_seen = now - CLEAR_MS;
				continue;
			}
#else
			nav.mode = MODE_CROSSING;
			mark = enc_mark();
			steer_side = 0;
#endif
		}

		switch (nav.mode) {
		case MODE_CROSSING: {
			W d = dist_since(mark);
			if ((d >= cross_counts && !(left && right)) || d >= cross_max_counts) {
				nav.mode = MODE_FOLLOW;
			}
			break;
		}

		case MODE_CONFIRM: {
			BOOL first = (confirm_side < 0) ? left : right;
			if (!first) {
				nav.mode = MODE_FOLLOW;
			} else if (dist_since(mark) >= confirm_counts) {
				nav.mode = MODE_FOLLOW;
				steer_side = confirm_side;
				steer_start = now;
				want = steer_action(steer_side, 0, soft);
			}
			break;
		}

		case MODE_FOLLOW:
		default:
			if (left || right) {
				INT side = left ? -1 : +1;
				if (steer_side == side) {
					want = steer_action(side, now - steer_start, soft);
				} else {
					nav.mode = MODE_CONFIRM;
					confirm_side = side;
					mark = enc_mark();
					steer_side = 0;
				}
			} else if (steer_side != 0 &&
				   now - (steer_side < 0 ? left_seen : right_seen) < CLEAR_MS + STEER_EXTRA_MS) {
				/* The sensor has just come off the line: turn a little further
				   so the car ends up pointing along the line, instead of going
				   straight while still angled at it and hitting it again */
				want = steer_action(steer_side, now - steer_start, soft);
			} else {
				steer_side = 0;
			}
			break;
		}

		/* The motors are touched when the plan changes -- and also when the
		   steering strength does while the plan stays STEER_LEFT or
		   STEER_RIGHT, which is what happens when a code comes under the
		   sensor mid-correction.  Nothing else uses those two speeds, so a
		   code appearing during STRAIGHT costs no extra motor command. */
		if (want != nav.action ||
		    ((want == ACT_STEER_LEFT || want == ACT_STEER_RIGHT) &&
		     soft != soft_now)) {
			apply(want, soft);
			soft_now = soft;
		}

		publish();
		tk_dly_tsk(LOOP_MS);
	}

	motion_command(MOTION_CMD_STOP, 0, 0, 3);
	nav.action = ACT_STOP;
	nav.state = NAV_IDLE;
	publish();
	if (soft_ms > 0) {
		tm_printf((UB*)"nav: line following stopped (steered gently for %d ms "
			  "while barcodes went past)\n", (INT)soft_ms);
	} else {
		tm_printf((UB*)"nav: line following stopped\n");
	}
	if (stop_req == REQ_STOP && button_down(S3_START_BUTTON_PIN)) {
		wait_release(S3_START_BUTTON_PIN);
	}
	return stop_req;
}

/* ---------- The task ---------- */
EXPORT void nav_task(INT stacd, void *exinf)
{
	REQUEST req;

	(void)stacd; (void)exinf;

	line_init();
	tk_dly_tsk(50);			/* let the button pull-ups settle first */
	nav.state = NAV_IDLE;
	nav.avoid_step = (UB)AVOID_FINISHED;	/* step 0 is a real step, so say
						   "none" explicitly */
	read_idle_sensors();
	publish();
	tm_printf((UB*)"nav: GP20 = start when stopped / stop when running, "
		  "GP21 = IMU calibration "
		  "(car spins), RESET = clear counters\n");

	while (1) {
		req = wait_for_request();
		if (req == REQ_CALIBRATE) {
			calibrate();
			continue;
		}

		reset_run();			/* count humps / junctions per run */
		if (follow_line() == REQ_RESET) {
			reset_run();
		}
	}
}
