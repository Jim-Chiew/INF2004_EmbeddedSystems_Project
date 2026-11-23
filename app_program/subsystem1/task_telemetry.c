/*
 *	task_telemetry.c
 *	Subsystem 1 -- telemetry_task: feeds the comms engine and prints the
 *	status line.  It only READS the other subsystems (their published
 *	copies under mutex) and never touches the motors.
 *
 *	Every S1_PERIOD_MS (10 ms):
 *	  - status LED from the COM state (comms_led_task)
 *	  - a START / STOP / RESET from the broker goes to navigation (IF-07)
 *	  - each finished obstacle scan from subsystem 6 (IF-13) is published
 *	    once on robocar/car1/obstacle: the object's length, how close it is,
 *	    its edge angles and the side with more room (comms.h)
 *	  - barcode passes from subsystem 5 (IF-09); each decoded text is
 *	    published once on robocar/car1/barcode (comms_send_barcode), and
 *	    every code that decodes is beeped out on the buzzer (GP22): LEFT one
 *	    long beep, RIGHT two short, FORWARD three, any other letter one
 *	    short lower beep (beeper.h)
 *	Every S1_TELEMETRY_MS (200 ms): one 10-byte frame to
 *	robocar/car1/telemetry (layout in data_layout.h) -- built every time,
 *	but only PUBLISHED with S1_PUBLISH_TELEMETRY 1 (subsystem1.h; 0 by
 *	default, so the broker sees the barcode letters only).  Every field comes
 *	from a sensor; fields this car has no sensor for are sent as their
 *	"invalid" code:
 *	  state      CALIBRATE (stopped / calibrating) or LINE_FOLLOWING (S3)
 *	  error      1 if the IMU is missing or its last read failed (S4)
 *	  hump       1 if a hump ended since the previous frame (S4)
 *	  hump_h     hump JOLT: biggest hump peak since the previous frame,
 *	             0.02 g steps (0 = no hump, 128 = 2.56 g or more)
 *	  speed      cm/s, from the wheel encoders (S2)
 *	  bc_dir     last LEFT/RIGHT barcode; bc_err 1 if a pass failed (S5)
 *	  line       centred / drifted right / drifted left / crossing (S3)
 *	  dir        compass heading, degrees (S4, only after GP21 calibration)
 *	  tilt, roll pitch and roll in degrees (S4)
 *	  dist       distance to whatever is ahead, cm (S6; 63 = nothing
 *	             in range or the sensor is not answering)
 *	  obj_len    length of the object the servo last measured, cm (S6;
 *	             63 = none measured yet, and after every RESET / new run)
 *	  low_batt   0: the battery is not measured
 *	  seq, drop, wifi_loss are filled in by the comms engine.
 *	Every S1_STATUS_MS (250 ms): one status line on the USB console.
 *	Every 100 ms: the error indicator, the Robo Pico's two RGB LEDs on GP18:
 *	off = no error, otherwise the colour of the error (several errors take
 *	turns, 0.5 s each, most serious first):
 *	  red      safety stop: lifted / overturned, or the e-stop is latched
 *	  green    an object is within S6_STOP_MM ahead (S6); it stays lit while
 *	           the car waits in AVOIDING, and goes out when GP20 clears it
 *	  magenta  motors cannot move (wheel encoders did not start)
 *	  yellow   IMU missing or not reading
 *	  white    IMU calibration failed
 *	  cyan     the last barcode pass did not decode
 *	  blue     WiFi / MQTT disconnected (telemetry build only)
 *	A GP20 start clears every latched error (subsystem 3, clear_errors()).
 */

#include "subsystem1.h"
#include "comms.h"
#include "cyw43_utk.h"
#include "drv_ws2812.h"
#include "beeper.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem6/subsystem6.h"
#include <string.h>

/* What this task remembers between frames */
LOCAL struct {
	UB	bc_cmd;				/* last LEFT/RIGHT read (S1_BC_*) */
	char	bc_text[S1_BC_TEXT_MAX + 1];
	BOOL	bc_last_failed;			/* most recent pass did not decode */
	BOOL	bc_seen;			/* any pass so far */
	BOOL	bc_err_pending;			/* a pass failed since the last frame */
	BOOL	hump_pending;			/* a hump ended since the last frame */
	float	jolt_pending_g;			/* biggest hump peak since then */
	UW	run_count;			/* S3's run counter we last saw */
	UW	obs_scans_seen;			/* S6's scan counter we last recorded */
	UW	obs_scans_pub;			/* ...and the one we last published as text */
	UW	hh_seen;			/* S4's hump-height counter we last dealt with */
	UB	obj_len_cm;			/* for the frame: cm, or DL_OBJ_NONE */
	UW	errors;				/* ERR_* active now (error indicator) */
} s1;

/* ------------------------------------------------------------------ */
/* Error indicator: RGB LEDs on GP18                                    */
/* ------------------------------------------------------------------ */
/* The bit values are just labels; err_colours[] below sets which error wins
   the LED when several are active. */
#define ERR_SAFETY	0x01
#define ERR_MOTOR	0x02
#define ERR_IMU		0x04
#define ERR_CAL		0x08
#define ERR_BARCODE	0x10
#define ERR_NET		0x20
#define ERR_OBSTACLE	0x40

#define LED_UPDATE_MS	100		/* how often the indicator is refreshed */
#define LED_STEP_MS	500		/* several errors: each colour shown this long */
#define MOTOR_GRACE_MS	12000		/* after power-up the encoders may take this long */
#define NET_GRACE_MS	15000		/* ...and WiFi + broker this long to connect */
#define IMU_FAIL_MS	300		/* IMU reads failing this long = error */

typedef struct {
	UW		bit;
	const char	*name;		/* for the status line */
	UB		r, g, b;	/* full-scale colour, dimmed by S1_RGB_BRIGHTNESS */
} ERR_COLOUR;

/* Most serious first: that is also the order they take turns in */
LOCAL const ERR_COLOUR err_colours[] = {
	{ ERR_SAFETY,  "SAFETY",  255,   0,   0 },	/* red */
	/* Green, not the obvious orange: ERR_IMU's "yellow" below is really
	   amber (255,160,0), so an orange here would be hard to tell from it
	   when the two take turns.  Green is the only colour left that cannot
	   be mistaken for another one on these LEDs. */
	{ ERR_OBSTACLE, "OBSTACLE", 0, 255,   0 },	/* green */
	{ ERR_MOTOR,   "MOTOR",   255,   0, 255 },	/* magenta */
	{ ERR_IMU,     "IMU",     255, 160,   0 },	/* yellow */
	{ ERR_CAL,     "CAL",     255, 255, 255 },	/* white */
	{ ERR_BARCODE, "BARCODE",   0, 255, 255 },	/* cyan */
	{ ERR_NET,     "WIFI",      0,   0, 255 },	/* blue */
};
#define ERR_KINDS	((INT)(sizeof err_colours / sizeof err_colours[0]))

LOCAL struct {
	BOOL	ok;			/* LEDs running */
	BOOL	failed;			/* no PIO room: gave up */
	UW	next_ms;
	UW	shown;			/* colour on the LEDs now (0xRRGGBB), or ~0 */
} led = { FALSE, FALSE, 0, 0xFFFFFFFFu };

/* ------------------------------------------------------------------ */
/* Number formatting (tm_printf has no %f)                              */
/* ------------------------------------------------------------------ */
LOCAL INT round_i(float v)
{
	return (INT)(v + ((v < 0) ? -0.5f : 0.5f));
}

/* v with dec (1..3) decimals, e.g. fx(b, -1.25f, 2, FALSE) -> "-1.25" */
LOCAL const char *fx(char *buf, float v, INT dec, BOOL plus)
{
	INT	scale = (dec >= 3) ? 1000 : (dec == 2) ? 100 : 10;
	BOOL	neg = (v < 0);
	float	a = neg ? -v : v;
	UW	x = (UW)(a * (float)scale + 0.5f);
	const char *sign = neg ? "-" : (plus ? "+" : "");

	if (dec >= 3) {
		tm_sprintf((UB*)buf, (UB*)"%s%u.%03u", sign, x / 1000, x % 1000);
	} else if (dec == 2) {
		tm_sprintf((UB*)buf, (UB*)"%s%u.%02u", sign, x / 100, x % 100);
	} else {
		tm_sprintf((UB*)buf, (UB*)"%s%u.%u", sign, x / 10, x % 10);
	}
	return buf;
}

/* ------------------------------------------------------------------ */
/* Inputs from the other subsystems                                     */
/* ------------------------------------------------------------------ */

/* Broker command -> navigation (IF-07).  FR-COM-08: within 200 ms. */
LOCAL void forward_commands(void)
{
	comms_cmd_t c = comms_get_command();
	UB n;

	if (c == COMMS_CMD_NONE) {
		return;
	}
	n = (c == COMMS_CMD_START) ? NAV_CMD_START :
	    (c == COMMS_CMD_STOP)  ? NAV_CMD_STOP  : NAV_CMD_RESET;
	tm_printf((UB*)"COMMAND RECEIVED: %s\n",
		  (n == NAV_CMD_START) ? "START" : (n == NAV_CMD_STOP) ? "STOP" : "RESET");
	if (nav_command(n) < E_OK) {
		tm_printf((UB*)"comms: navigation busy, command dropped\n");
	}
}

/* What each read sounds like (beeper.h has the whole table and the lengths).
   Every code that DECODES gets a beep, so a successful read is always
   audible; the letters that are not commands share one lower, shorter beep
   of their own. */
LOCAL void beep_command(UB cmd, UW now)
{
	switch (cmd) {
	case S1_BC_LEFT:	beeper_play(1, BEEP_LONG_MS, now);  break;
	case S1_BC_RIGHT:	beeper_play(2, BEEP_SHORT_MS, now); break;
	case S1_BC_FORWARD:	beeper_play(3, BEEP_SHORT_MS, now); break;
	case S1_BC_UTURN:	beeper_play(2, BEEP_LONG_MS, now);  break;
	default:
		/* Read, but the letter is not one of BARCODE_LETTERS_* */
		beeper_play_tone(1, BEEP_SHORT_MS, BEEP_HZ_OTHER, now);
		break;
	}
}

/* The direction a code commands, as it goes out on MQTT after the text */
LOCAL const char *bc_dir_name(UB cmd)
{
	switch (cmd) {
	case S1_BC_LEFT:	return "LEFT";
	case S1_BC_RIGHT:	return "RIGHT";
	case S1_BC_FORWARD:	return "FORWARD";
	case S1_BC_UTURN:	return "U-TURN";
	default:		return "NONE";
	}
}

/* Barcode passes from subsystem 5 (IF-09) */
LOCAL void take_barcodes(UW now)
{
	BARCODE_MSG m;
	/* "<text> <direction>": up to S1_BC_TEXT_MAX + " FORWARD", which is
	   what COMMS_BARCODE_MAX is sized for, plus the '\0' */
	char	    out[COMMS_BARCODE_MAX + 1];

	while (tk_rcv_mbf(mbf_barcode, &m, TMO_POL) == (INT)sizeof m) {
		s1.bc_seen = TRUE;
		s1.bc_last_failed = !m.valid;
		if (!m.valid) {
			s1.bc_err_pending = TRUE;
			s1.bc_text[0] = '\0';
			continue;
		}
		strncpy(s1.bc_text, m.text, S1_BC_TEXT_MAX);
		s1.bc_text[S1_BC_TEXT_MAX] = '\0';
		/* The frame only carries LEFT/RIGHT, so the decoded text itself
		   goes to robocar/car1/barcode (once per barcode read), with the
		   direction THIS code commands after it: "A LEFT", "X NONE". */
		tm_sprintf((UB*)out, (UB*)"%s %s", s1.bc_text,
			   bc_dir_name(m.command));
		(void)comms_send_barcode(out);
		/* Anything that decoded is worth hearing, command or not: this
		   path is only reached for a valid read (an invalid pass has
		   already gone round the loop above). */
		if (m.command != S1_BC_NONE) {
			s1.bc_cmd = m.command;
		}
		beep_command(m.command, now);
	}
}

/* The measured object length as the frame's 6-bit field: whole centimetres,
   or DL_OBJ_NONE when the scan measured nothing.  Lengths over DL_OBJ_MAX_CM
   (62 cm) clamp -- the scan cannot produce one today (its range is 32 cm), and
   a clamped maximum is a better lie than a wrapped-around small number. */
LOCAL UB obj_len_code(const OBS_STATUS *os)
{
	INT cm;

	if (!os->object.found) {
		return (UB)DL_OBJ_NONE;
	}
	cm = (INT)((os->object.length_mm + 5) / 10);
	if (cm < 0) {
		cm = 0;
	}
	if (cm > (INT)DL_OBJ_MAX_CM) {
		cm = (INT)DL_OBJ_MAX_CM;
	}
	return (UB)cm;
}

/* One line per finished obstacle scan, on its own topic (IF-10).
   The telemetry frame is full to the bit, so the measurement cannot go in it;
   it goes out as ASCII like the barcode text does.  comms.h has the format.
   Published once per scan: S6's scan counter changing is the trigger. */
LOCAL void publish_obstacle_scan(void)
{
	static OBS_STATUS	os;		/* static: ~140 bytes, off the stack */
	static char		msg[COMMS_TEXT_MAX];
	int			r;

	obstacle_get_status(&os);

	/* The frame carries the newest measurement from the moment it exists,
	   whatever the broker happens to be doing. */
	if (os.scans != s1.obs_scans_seen) {
		s1.obs_scans_seen = os.scans;
		s1.obj_len_cm = obj_len_code(&os);
	}

	if (os.scans == s1.obs_scans_pub) {
		return;				/* this one has been published */
	}

	/* d= first: how far away the obstacle was when it stopped the car (the
	   reading that set subsystem 6's latch).  Always there, so even a scan
	   that found nothing to measure says that an obstacle was detected and
	   where. */
	if (os.object.found) {
		tm_sprintf((UB*)msg, (UB*)"d=%d w=%d n=%d l=%d r=%d s=%s",
			   (INT)os.stop_mm,
			   (INT)os.object.length_mm, (INT)os.object.near_mm,
			   (INT)os.object.left_deg, (INT)os.object.right_deg,
			   (os.profile.side == AVOID_LEFT) ? "L" : "R");
	} else {
		/* The scan ran but found nothing in range: say so rather than
		   send a zero width, which would read as "no gap at all". */
		tm_sprintf((UB*)msg, (UB*)"d=%d w=none s=%s", (INT)os.stop_mm,
			   (os.profile.side == AVOID_LEFT) ? "L" : "R");
	}

#if S1_PUBLISH_OBSTACLE
	r = comms_send_obstacle(msg);
#else
	r = COMMS_ERR_NOT_READY;	/* not published (subsystem1.h) */
#endif
	if (r == COMMS_OK || r == COMMS_ERR_NOT_READY) {
		/* Sent, or there is no network in this build at all: either way
		   this measurement has had its turn.  A COMMS_ERR_BUSY (every
		   slot waiting) leaves the counter alone, so the next pass tries
		   again with the same numbers.  The frame field is already set
		   either way. */
		s1.obs_scans_pub = os.scans;
	}
}

/* One line per hump a wheel rolled over (subsystem 4, hump_height.h), on
   robocar/car1/hump: "h=12 s=R t=5.7".  S4's counter changing is the
   trigger, as the scan counter is for the obstacle line above.  Only during
   a run: idle, the car is being handled, and those tilts are not humps. */
LOCAL void publish_hump(void)
{
	static IMU_STATUS	is;		/* static: off the task stack */
	static NAV_STATUS	ns;
	char			msg[24];
	INT			mm, t10;
	int			r;

	imu_get_status(&is);
	if (is.d.hh_count == s1.hh_seen) {
		return;				/* nothing new */
	}
	nav_get_status(&ns);
	if (ns.state == NAV_IDLE || ns.state == NAV_CALIBRATING) {
		s1.hh_seen = is.d.hh_count;	/* not a run: console only (S4) */
		return;
	}

	mm = (INT)(is.d.hh_height_mm + 0.5f);
	t10 = (INT)(is.d.hh_tilt_deg * 10.0f + 0.5f);
	tm_sprintf((UB*)msg, (UB*)"h=%d s=%c t=%d.%d", mm, is.d.hh_side,
		   t10 / 10, t10 % 10);
#if S1_PUBLISH_HUMP
	r = comms_send_hump(msg);
#else
	r = COMMS_ERR_NOT_READY;		/* not published (subsystem1.h) */
#endif
	if (r != COMMS_ERR_BUSY) {
		/* Sent, or there is no network in this build: dealt with.  BUSY
		   (every slot waiting) tries again next pass, same numbers. */
		s1.hh_seen = is.d.hh_count;
	}
}

/* A new run or RESET in navigation clears the barcode state */
LOCAL void check_run_reset(void)
{
	NAV_STATUS nav;

	nav_get_status(&nav);
	if (nav.run_count != s1.run_count) {
		s1.run_count = nav.run_count;
		s1.bc_cmd = S1_BC_NONE;
		s1.bc_seen = FALSE;
		s1.obj_len_cm = (UB)DL_OBJ_NONE;	/* no object measured in this run */
		s1.bc_err_pending = FALSE;
		s1.hump_pending = FALSE;
		s1.jolt_pending_g = 0.0f;
		(void)imu_take_hump(&s1.jolt_pending_g);	/* drop older pulses */
		s1.jolt_pending_g = 0.0f;
	}
}

/* ------------------------------------------------------------------ */
/* Error indicator                                                      */
/* ------------------------------------------------------------------ */

/* Which errors are active now, from everyone's published status */
LOCAL UW active_errors(UW now, const ODOMETRY *odo, const IMU_STATUS *imu,
		       const OBS_STATUS *obs)
{
	UW e = 0;

	/* An object ahead is not a fault, but it is why the car has stopped, so
	   it gets a lamp.  The latch (not the raw distance) is what is shown: it
	   stays set while the car waits in AVOIDING and goes out when GP20 /
	   START clears it, so the light matches what the car is doing. */
	if (obs->blocked) {
		e |= ERR_OBSTACLE;
	}

	if (odo->state == MOTION_ST_ESTOP ||
	    (imu->ok && imu->valid && (imu->d.overturned || imu->d.lifted))) {
		e |= ERR_SAFETY;
	}
	if (!odo->encoders_ok && now > MOTOR_GRACE_MS) {
		e |= ERR_MOTOR;
	}
	{
		/* IMU: missing, or reads failing for IMU_FAIL_MS in a row (one
		   stray failed read while driving is not an error) */
		static UW bad_since = 0;
		if (imu->ok && imu->valid) {
			bad_since = 0;
		} else if (bad_since == 0) {
			bad_since = now ? now : 1;
		}
		if (!imu->ok || (bad_since != 0 && now - bad_since >= IMU_FAIL_MS)) {
			e |= ERR_IMU;
		}
	}
	if (imu->cal_failed) {
		e |= ERR_CAL;
	}
	if (s1.bc_seen && s1.bc_last_failed) {
		e |= ERR_BARCODE;
	}
#if defined(TM_MQTT) && TM_MQTT
	if (comms_get_state() == COM_DISCONNECTED && now > NET_GRACE_MS) {
		e |= ERR_NET;
	}
#endif
	return e;
}

LOCAL void led_show(UB r, UB g, UB b)
{
	UW rgb = ((UW)r << 16) | ((UW)g << 8) | b;

	if (rgb != led.shown) {			/* only write when it changes */
		led.shown = rgb;
		ws2812_fill(S1_RGB_LED_COUNT,
			    (uint8_t)((UW)r * S1_RGB_BRIGHTNESS / 255u),
			    (uint8_t)((UW)g * S1_RGB_BRIGHTNESS / 255u),
			    (uint8_t)((UW)b * S1_RGB_BRIGHTNESS / 255u));
	}
}

/* Start the LEDs: after the wheel encoders, whose PIO program must be
   placed first.  Then show every colour once, so you can see the LEDs work
   and learn the colours. */
LOCAL void led_start(UW now)
{
	static ODOMETRY odo;
	INT pio = -1, i;

	motion_get_odometry(&odo);
	if (!odo.encoders_ok && now < MOTOR_GRACE_MS) {
		return;					/* not yet */
	}
	if (!ws2812_init(S1_RGB_LED_PIN, &pio)) {
		led.failed = TRUE;
		tm_printf((UB*)"comms: ERROR no free PIO room for the RGB LEDs on GP%d\n",
			  S1_RGB_LED_PIN);
		return;
	}
	led.ok = TRUE;
	tm_printf((UB*)"comms: error LEDs on GP%d (PIO%d): red SAFETY, magenta MOTOR, "
		  "yellow IMU, white CAL, cyan BARCODE, blue WIFI\n", S1_RGB_LED_PIN, pio);
	for (i = 0; i < ERR_KINDS; i++) {
		led_show(err_colours[i].r, err_colours[i].g, err_colours[i].b);
		tk_dly_tsk(150);
	}
	led_show(0, 0, 0);
}

LOCAL void error_led_update(UW now)
{
	static ODOMETRY		odo;		/* static: off the task stack */
	static IMU_STATUS	imu;
	static OBS_STATUS	obs;
	INT	i, n = 0, pick;

	if ((W)(now - led.next_ms) < 0) {
		return;
	}
	led.next_ms = now + LED_UPDATE_MS;

	motion_get_odometry(&odo);
	imu_get_status(&imu);
	obstacle_get_status(&obs);
	s1.errors = active_errors(now, &odo, &imu, &obs);

	if (!led.ok) {
		if (!led.failed) {
			led_start(now);
		}
		return;
	}
	if (s1.errors == 0) {
		led_show(0, 0, 0);			/* no error: off */
		return;
	}

	/* Several errors: take turns, LED_STEP_MS each, most serious first */
	for (i = 0; i < ERR_KINDS; i++) {
		if (s1.errors & err_colours[i].bit) n++;
	}
	pick = (INT)((now / LED_STEP_MS) % (UW)n);
	for (i = 0; i < ERR_KINDS; i++) {
		if ((s1.errors & err_colours[i].bit) && pick-- == 0) {
			led_show(err_colours[i].r, err_colours[i].g, err_colours[i].b);
			return;
		}
	}
}

/* ------------------------------------------------------------------ */
/* The telemetry frame                                                  */
/* ------------------------------------------------------------------ */
LOCAL int16_t angle_deg(float a)
{
	INT v = round_i(a);

	if (v > DL_TILT_MAX_DEG) v = DL_TILT_MAX_DEG;
	if (v < DL_TILT_MIN_DEG) v = DL_TILT_MIN_DEG;
	return (int16_t)v;
}

LOCAL uint8_t line_code(const NAV_STATUS *nav)
{
	if (nav->state == NAV_FOLLOWING && nav->mode == MODE_CROSSING) {
		return DL_LINE_CROSSING;
	}
	if (nav->left && nav->right) return DL_LINE_CROSSING;
	if (nav->left)  return DL_LINE_DRIFT_RIGHT;	/* left sensor on the line = drifted right */
	if (nav->right) return DL_LINE_DRIFT_LEFT;
	return DL_LINE_CENTRED;
}

LOCAL void build_sample(telemetry_sample_t *s, const NAV_STATUS *nav,
			const ODOMETRY *odo, const IMU_STATUS *imu,
			const OBS_STATUS *obs)
{
	INT spd;

	memset(s, 0, sizeof *s);
	s->state = (nav->state == NAV_TURNING) ? DL_STATE_BARCODE_EXECUTION :
		   (nav->state == NAV_BARCODE_RESCAN) ? DL_STATE_BARCODE_RESCAN :
		   (nav->state == NAV_FOLLOWING || nav->state == NAV_AVOIDING)
		   ? DL_STATE_LINE_FOLLOWING : DL_STATE_CALIBRATE;
	s->error = (!imu->ok || !imu->valid) ? 1u : 0u;
	s->low_batt = 0u;				/* battery not measured */
	s->hump = s1.hump_pending ? 1u : 0u;

	spd = round_i(odo->speed_cms);
	if (spd < 0) spd = 0;
	if (spd > (INT)DL_SPEED_MAX_CMS) spd = (INT)DL_SPEED_MAX_CMS;
	s->speed = (uint8_t)spd;

	s->bc_dir = (s1.bc_cmd == S1_BC_LEFT)    ? DL_BC_LEFT :
		    (s1.bc_cmd == S1_BC_RIGHT)   ? DL_BC_RIGHT :
		    (s1.bc_cmd == S1_BC_FORWARD) ? DL_BC_FORWARD : DL_BC_NONE;
	/* U-TURN goes out as DL_BC_NONE: the 2-bit field has no code left for
	   it.  The barcode topic carries it in words ("D U-TURN"). */
	s->bc_err = s1.bc_err_pending ? 1u : 0u;
	s->line = line_code(nav);
	/* Distance ahead (S6).  Nothing in range, an unknown reading and a
	   sensor that is not answering are all sent as the invalid code: the
	   frame has no way to say "more than 62 cm". */
	if (obs->dist_mm > 0) {
		INT cm = (INT)((obs->dist_mm + 5) / 10);
		if (cm > (INT)DL_DIST_MAX_CM) cm = (INT)DL_DIST_MAX_CM;
		s->dist = (uint8_t)cm;
	} else {
		s->dist = DL_DIST_INVALID;
	}

	/* The object's measured length holds its value between scans; it reads
	   DL_OBJ_NONE until the first scan of the run (see check_run_reset). */
	s->obj_len = s1.obj_len_cm;

	if (imu->ok && imu->valid) {
		INT steps = round_i(s1.jolt_pending_g * 1000.0f / (float)DL_HUMP_JOLT_STEP_MG);
		if (steps > (INT)DL_HUMP_H_MAX_MM) steps = (INT)DL_HUMP_H_MAX_MM;
		s->hump_h = (uint8_t)steps;
		s->dir = imu->calibrated ? (uint16_t)(round_i(imu->d.heading) % 360) : DL_DIR_INVALID;
		s->tilt = angle_deg(imu->d.pitch);
		s->roll = angle_deg(imu->d.roll);
	} else {
		s->hump_h = DL_HUMP_H_INVALID;
		s->dir = DL_DIR_INVALID;
		s->tilt = TELEM_ANGLE_INVALID;
		s->roll = TELEM_ANGLE_INVALID;
	}
}

LOCAL void send_telemetry(void)
{
	static NAV_STATUS	nav;		/* static: off the task stack */
	static ODOMETRY		odo;
	static IMU_STATUS	imu;
	static OBS_STATUS	obs;
	telemetry_sample_t	s;
	float			jolt;
	int			r;

	if (imu_take_hump(&jolt)) {
		s1.hump_pending = TRUE;
		if (jolt > s1.jolt_pending_g) {
			s1.jolt_pending_g = jolt;
		}
	}
	nav_get_status(&nav);
	motion_get_odometry(&odo);
	imu_get_status(&imu);
	obstacle_get_status(&obs);
	build_sample(&s, &nav, &odo, &imu, &obs);

#if S1_PUBLISH_TELEMETRY
	r = comms_send(&s);
#else
	/* Not published (subsystem1.h): handled exactly like a build with no
	   network, so the pulses below are cleared on the same schedule. */
	(void)s;
	r = COMMS_ERR_NOT_READY;
#endif
	if (r == COMMS_OK || r == COMMS_ERR_NOT_READY) {
		/* Pulses are cleared only once they have been handed over (or
		   there is no network in this build at all) */
		s1.hump_pending = FALSE;
		s1.jolt_pending_g = 0.0f;
		s1.bc_err_pending = FALSE;
	}
}

/* ------------------------------------------------------------------ */
/* Status line                                                          */
/* ------------------------------------------------------------------ */
LOCAL const char *com_name(com_state_t s)
{
	switch (s) {
	case COM_SYNC:		return "SYNC";
	case COM_CONNECTED:	return "CONNECTED";
	default:		return "DISCONNECTED";
	}
}

LOCAL const char *link_name(const T_CYW43_UTK_STATUS *r)
{
	if (!r->init_complete) return "INIT";
	if (!r->ready)         return "RADIO FAIL";
	switch (r->link_status) {
	case 3:  return "UP";
	case 2:  return "NOIP";
	case 1:  return "JOIN";
	case 0:  return "DOWN";
	case -1: return "FAIL";
	case -2: return "NONET";
	case -3: return "BADAUTH";
	default: return "?";
	}
}

LOCAL void print_status(UW now)
{
	/* static: ~800 bytes, kept off the 2 KB task stack (tm_printf itself
	   needs about 1 KB of it).  Only this task calls print_status(). */
	static NAV_STATUS		nav;
	static ODOMETRY			odo;
	static IMU_STATUS		imu;
	static comms_stats_t		cs;
	static T_CYW43_UTK_STATUS	radio;
	static OBS_STATUS		obs;
	static char st[24], imus[200], bc[40], us[32], wifi[100], errs[64];
	static char b1[12], b2[12], b3[12], b4[12], b5[12], b6[12], b7[12], b8[12];
	INT i, len;

	nav_get_status(&nav);
	motion_get_odometry(&odo);
	imu_get_status(&imu);
	obstacle_get_status(&obs);
	comms_get_stats(&cs);
	cyw43_utk_get_status(&radio);

	/* Run state */
	if (imu.state == IMU_ST_CAL_SPIN || imu.state == IMU_ST_CAL_SETTLE ||
	    imu.state == IMU_ST_CAL_ZERO) {
		tm_sprintf((UB*)st, (UB*)"%s %ds", imu_state_name(imu.state),
			   (INT)(imu.cal_left_ms / 1000 + 1));
	} else if (nav.state == NAV_AVOIDING) {
		tm_sprintf((UB*)st, (UB*)"AVOIDING");
	} else if (nav.state == NAV_FOLLOWING) {
		tm_sprintf((UB*)st, (UB*)"FOLLOW %s", nav_action_name(nav.action));
	} else {
		tm_sprintf((UB*)st, (UB*)"IDLE%s",
			   (imu.ok && !imu.calibrated) ? " (no cal)" : "");
	}

	/* IMU */
	if (!imu.ok) {
		tm_sprintf((UB*)imus, (UB*)"IMU missing (WHO_AM_I 0x%02x, want 0x49; retrying)", imu.who);
	} else if (!imu.valid) {
		tm_sprintf((UB*)imus, (UB*)"IMU read failed");
	} else {
		const imu_data_t *d = &imu.d;
		tm_sprintf((UB*)imus,
			   (UB*)"IMU P%6s R%6s H%4d %-2s V%6s humps %lu max %sg rst %lu "
			   "vib %sg lift %s %scm (last %scm %sm/s)%s%s",
			   fx(b1, d->pitch, 1, FALSE), fx(b2, d->roll, 1, FALSE),
			   round_i(d->heading) % 360, imu_compass_point(d->heading),
			   fx(b3, d->vert_acc, 2, TRUE), (unsigned long)d->hump_count,
			   fx(b4, d->hump_max, 2, FALSE), (unsigned long)imu.restarts,
			   fx(b5, d->vib_g, 3, FALSE),
			   (d->lift_phase == 1) ? "UP" : (d->lift_phase == 2) ? "HELD" : "-",
			   fx(b6, d->lift_height_cm, 1, FALSE), fx(b7, d->lift_last_cm, 1, FALSE),
			   fx(b8, d->lift_last_speed, 2, FALSE),
			   d->overturned ? " OVERTURNED" : "", d->lifted ? " LIFTED" : "");
	}

	/* Barcode */
	if (!s1.bc_seen) {
		tm_sprintf((UB*)bc, (UB*)"bc -");
	} else if (s1.bc_last_failed) {
		tm_sprintf((UB*)bc, (UB*)"bc FAIL");
	} else {
		tm_sprintf((UB*)bc, (UB*)"bc \"%s\" %s", s1.bc_text,
			   (s1.bc_cmd == S1_BC_LEFT) ? "LEFT" :
			   (s1.bc_cmd == S1_BC_RIGHT) ? "RIGHT" :
			   (s1.bc_cmd == S1_BC_FORWARD) ? "FORWARD" :
			   (s1.bc_cmd == S1_BC_UTURN) ? "U-TURN" : "-");
	}

	/* Distance ahead (subsystem 6) */
	if (obs.state == OBS_ST_FAIL) {
		tm_sprintf((UB*)us, (UB*)"us DEAD");
	} else if (obs.state == OBS_ST_FAULT) {
		tm_sprintf((UB*)us, (UB*)"us no answer");
	} else if (obs.dist_mm == US_NO_ECHO) {
		tm_sprintf((UB*)us, (UB*)"us open");
	} else if (obs.dist_mm < 0) {
		tm_sprintf((UB*)us, (UB*)"us ???");
	} else {
		tm_sprintf((UB*)us, (UB*)"us %dmm%s", (INT)obs.dist_mm,
			   obs.blocked ? " BLOCKED" : "");
	}

	/* WiFi / MQTT */
	tm_sprintf((UB*)wifi, (UB*)"wifi %s (%s) q%u ack %lu drop %lu cmd %lu/%lu bc-pub %lu",
		   com_name(comms_get_state()), link_name(&radio), (unsigned)cs.queued,
		   (unsigned long)cs.sent_acked, (unsigned long)cs.discarded,
		   (unsigned long)cs.cmds_accepted, (unsigned long)cs.cmds_rejected,
		   (unsigned long)cs.barcodes_sent);

	/* Active errors (the same ones the RGB LEDs show) */
	len = tm_sprintf((UB*)errs, (UB*)"err");
	if (s1.errors == 0) {
		tm_sprintf((UB*)errs + len, (UB*)" -");
	}
	for (i = 0; i < ERR_KINDS; i++) {
		if (s1.errors & err_colours[i].bit) {
			len += tm_sprintf((UB*)errs + len, (UB*)" %s", err_colours[i].name);
		}
	}

	tm_printf((UB*)"[%5lu.%02lu s] %-15s | IR L%d R%d %-8s jn %lu | spd %4s cm/s %s pwr L%d R%d | %s | %s | %s | %s | %s\n",
		  (unsigned long)(now / 1000), (unsigned long)((now % 1000) / 10), st,
		  nav.left ? 1 : 0, nav.right ? 1 : 0, nav_mode_name(nav.mode),
		  (unsigned long)nav.junctions, fx(b1, odo.speed_cms, 1, FALSE),
		  motion_state_name(odo.state), odo.left_pm / 10, odo.right_pm / 10,
		  imus, bc, us, wifi, errs);
}

/* ------------------------------------------------------------------ */
/* The task                                                             */
/* ------------------------------------------------------------------ */
EXPORT void telemetry_task(INT stacd, void *exinf)
{
	UW now, next_telem, next_status;

	(void)stacd; (void)exinf;

	gpio_set_pin(S1_STATUS_LED_PIN, GPIO_MODE_OUT);	/* status LED, off */
	gpio_set_val(S1_STATUS_LED_PIN, 0);
	beeper_init();					/* buzzer, silent */
	s1.obj_len_cm = (UB)DL_OBJ_NONE;		/* nothing measured yet (0 = 0 cm) */

	now = app_now_ms();
	next_telem = now + S1_TELEMETRY_MS;
	next_status = now + 1000;		/* let the others start first */

	while (1) {
		now = app_now_ms();

		comms_led_task();
		forward_commands();
		take_barcodes(now);
		publish_obstacle_scan();
		publish_hump();
		check_run_reset();
		error_led_update(now);
		beeper_step(now);

		if ((W)(now - next_telem) >= 0) {
			next_telem += S1_TELEMETRY_MS;
			if ((W)(now - next_telem) >= 0) next_telem = now + S1_TELEMETRY_MS;
			send_telemetry();
		}
		if ((W)(now - next_status) >= 0) {
			next_status = now + S1_STATUS_MS;
			print_status(now);
		}
		tk_dly_tsk(S1_PERIOD_MS);
	}
}
