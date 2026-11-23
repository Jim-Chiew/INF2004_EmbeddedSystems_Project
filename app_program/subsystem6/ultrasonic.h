/*
 * ultrasonic.h
 * Subsystem 6 -- the HC-SR04 distance sensor and the SG90 servo that aims it.
 *
 * This is the portable half of the subsystem: one ping turned into
 * millimetres, the median-of-3 filter, the invalid-ping counter, the servo
 * angle maths and the two servo scans. The hardware is in drv_ultrasonic.c
 * and the task that owns it all is in task_obstacle.c.
 *
 * How one measurement works: TRIG goes high for 10 us, the HC-SR04 answers
 * with an ECHO pulse as long as the sound's flight time, and sound travels
 * 343 m/s there and back. So mm = us * 343 / 2000.
 *
 * Plain C: no kernel and no Pico SDK headers, so both sides can include it.
 *
 * TWO READINGS ARE NOT DISTANCES and every caller must handle them:
 *   US_NO_ECHO   nothing came back: open space, which counts as "clear"
 *   US_INVALID   the sensor did not answer, or answered nonsense: UNKNOWN,
 *                which is never treated as clear. A sensor that is unplugged
 *                would otherwise look like an empty road.
 */

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdbool.h>
#include <stdint.h>

/* ---- Pins (TEAM.md section 2): the HC-SR04 on Grove 1, the servo on GP15 --
 * HC-SR04: TRIG (white) -> GP0, ECHO (yellow) -> GP1, VCC -> 3V3, GND -> GND.
 * SG90:    signal -> GP15, 50 Hz PWM (slice 7 channel B), power from the
 *          Robo Pico's servo header, NOT from the Pico's 3V3 pin.
 * WARNING: GP0/GP1 are the UART0 pins. They are free in the default build but
 *          taken by "make CRASH_REPORT=1" or CONSOLE=uart, so subsystem6.h
 *          refuses to compile in those builds. If the sensor has to move,
 *          change only these three numbers (and the TEAM.md pin table). */
#define US_TRIG_PIN           0
#define US_ECHO_PIN           1      /* edge interrupt, see drv_ultrasonic.c */
#define US_SERVO_PIN          15

/* ---- One ping ---- */
#define US_TRIG_PULSE_US      10
#define US_ECHO_TIMEOUT_US    18000  /* 18 ms: ECHO rose but never fell => clear.
                                        No rising edge at all => invalid */
#define US_ECHO_RELEASE_MS    45     /* max wait for a held-high ECHO to fall (>38 ms) */
#define US_PING_GAP_MS        20     /* between pings, so the last echo dies out */
#define US_ECHO_INT_PRI       2      /* ECHO edge interrupt priority, like the
                                        kernel's own drivers (subsystem 5's 2 kHz
                                        barcode sampler also runs at 2) */
#define US_MIN_VALID_MM       20     /* under 2 cm the HC-SR04 cannot measure */
#define US_CLEAR_MM           4000   /* the value "no echo" stands for in the filter */

/* What a reading can say instead of a distance (see the header comment) */
#define US_NO_ECHO            (-1)
#define US_INVALID            (-2)

/* ---- Sensor fault policy (the SENSOR_FAULT state in task_obstacle.c) ---- */
#define US_FAULT_INVALID_CNT  10     /* invalid pings in a row => SENSOR_FAULT */
#define US_REINIT_MAX         3      /* recovery rounds before giving up */
#define US_REINIT_DELAY_MS    100    /* wait before each round */
#define US_REINIT_GOOD_PINGS  3      /* pings per round; any valid one = recovered */

/* ---- Servo ---- */
/* The standard 1-2 ms pulse for +-90 deg, which every hobby servo can reach.
 * DO NOT widen this without checking the servo on the bench: a pulse outside
 * the servo's travel drives it into its mechanical stop, where it sits
 * buzzing and drawing its stall current (~700 mA on an SG90). It looks
 * exactly like a servo that has got stuck at one end, it gets hot, and it can
 * brown out the board.  (This was 500-2400 us, carried over from the bench
 * code this subsystem came from; the scan below is the first thing that ever
 * asked for a full +-90 deg, and that is when it showed up.) */
#define SERVO_PULSE_MIN_US    1000   /* -90 deg */
#define SERVO_PULSE_MAX_US    2000   /* +90 deg */

/* How far the scans are allowed to turn.  Kept short of 90 deg on purpose, so
 * that a servo whose centre is a little off, or a bracket that fouls, still
 * does not reach a stop.  Widen only after watching the servo do it. */
#define SERVO_SCAN_MAX_DEG    75
#define SERVO_ANGLE_LEFT      SERVO_SCAN_MAX_DEG   /* 0 = centre, +ve = left */
#define SERVO_ANGLE_RIGHT     (-SERVO_SCAN_MAX_DEG)
#define SERVO_SETTLE_LEFT_MS  250
#define SERVO_SETTLE_RIGHT_MS 400    /* the longest move: +75 -> -75 */
#define SERVO_SETTLE_CENTRE_MS 250

/* ---- Coarse scan: a five-angle fan, one ping per angle ----
 * The SG90 datasheet gives about 100 ms per 60 deg at 4.8 V, so a 60 deg move
 * gets 120 ms (20 % margin) and a 30 deg step gets 70 ms. A servo run at 3.3 V
 * is slower: raise these if the sensor is still turning when the ping is taken. */
#define US_COARSE_N           5
#define SCAN_COARSE_DEG       { -60, -30, 0, 30, 60 }   /* ordered right -> left */
#define SCAN_COARSE_SETTLE_FIRST_MS 120   /* centre -> -60 deg (a 60 deg move) */
#define SCAN_COARSE_SETTLE_STEP_MS   70   /* one 30 deg step between angles */
#define SCAN_COARSE_RECENTRE_MS     120   /* +60 deg -> centre */
#define SCAN_OBSTACLE_RANGE_MM      320   /* an angle counts as blocked at or under this */

/* Time budget for one coarse scan: the settle waits plus one ping per angle.
 * Test S6-U19 checks the sum still fits, so nobody can quietly push the
 * settle times past it later. */
#define SCAN_COARSE_BUDGET_MS       800
#define SCAN_COARSE_PING_ALLOW_MS    30   /* per ping, incl. kernel tick rounding */
#define SCAN_COARSE_SETTLE_TOTAL_MS \
        (SCAN_COARSE_SETTLE_FIRST_MS + \
         ((US_COARSE_N - 1) * SCAN_COARSE_SETTLE_STEP_MS) + \
         SCAN_COARSE_RECENTRE_MS)

/* ---- Stage 2: the fine sweep that measures the object ----
 * Stage 1 (the coarse fan) says roughly WHERE the object is.  Stage 2 sweeps a
 * window around the closest coarse angle in small steps and measures how far
 * the object reaches across the car's path -- its length.
 * Only the window is swept, not the whole fan, so this costs about as much as
 * the coarse scan did. */
#define US_FINE_STEP_DEG      5      /* angle between fine samples */
#define SCAN_FINE_WINDOW_DEG  30     /* swept either side of the object */
#define SCAN_FINE_SETTLE_MS   25     /* servo settle for one 5 deg step */
#define SCAN_FINE_POINTS      ((2 * SCAN_FINE_WINDOW_DEG) / US_FINE_STEP_DEG + 1)
#define US_FINE_MAX           ((180 / US_FINE_STEP_DEG) + 1)  /* any window fits */

/* Two neighbouring samples count as the same object while their distances are
 * within this of each other.  Bigger than the depth of a box seen cornerwise,
 * smaller than the step out to the wall behind it: that step is what tells
 * the scan where the object ends. */
#define US_OBJ_STEP_MM        120

/* The HC-SR04's beam is a cone roughly this wide, so the object still answers
 * from angles where only the edge of the cone touches it: EVERY measured
 * length reads wide by about this much of arc.  us_object_t carries the raw
 * measurement and a rough correction, so both are visible. */
#define US_BEAM_DEG           15

/* Time budget for one fine sweep: the first (big) move, the small steps, the
 * recentre and one ping per point.  Test S6-U23 checks the sum still fits. */
#define SCAN_FINE_BUDGET_MS   1000
#define SCAN_FINE_SETTLE_TOTAL_MS         (SCAN_COARSE_SETTLE_FIRST_MS +          ((SCAN_FINE_POINTS - 1) * SCAN_FINE_SETTLE_MS) +          SCAN_COARSE_RECENTRE_MS)

#define SIDE_TIE_MM           30     /* left and right within 3 cm => LEFT */

/* ---- Results ---- */
typedef enum { AVOID_LEFT = 0, AVOID_RIGHT = 1 } avoid_side_t;

/* The five-angle fan */
typedef struct
{
    int8_t   deg[US_COARSE_N];   /* servo angle of each sample, 0 = centre, +ve = left */
    int32_t  mm[US_COARSE_N];    /* distance, or US_NO_ECHO / US_INVALID */
    uint8_t  blocked_mask;       /* bit i = sample i valid and <= range threshold */
    uint8_t  invalid_mask;       /* bit i = sample i unknown (never "clear") */
    int8_t   closest_idx;        /* nearest blocked sample, -1 if none */
    uint16_t duration_ms;        /* how long the scan took (budget check) */
} us_coarse_t;

/* Clearance either side, and which way to go round */
typedef struct
{
    int32_t      clear_L_mm;     /* clearance at +90 deg (US_CLEAR_MM if open) */
    int32_t      clear_R_mm;     /* clearance at -90 deg */
    avoid_side_t side;           /* the side with more room; a tie goes LEFT */
} us_profile_t;

/* What the fine sweep measured: how far the object reaches across the path.
 * Angles keep the servo convention: 0 = straight ahead, +ve = left. */
typedef struct
{
    bool     found;           /* an object was seen and measured */
    int32_t  length_mm;       /* edge to edge, as measured */
    int32_t  length_corr_mm;  /* a LOWER bound: the beam allowance taken off.
                                 Collapses towards 0 as the span approaches
                                 US_BEAM_DEG, so length_mm is the measurement
                                 and this one is the "at least" figure. */
    int32_t  near_mm;         /* closest point on it */
    int8_t   left_deg;        /* its left-most and right-most angle */
    int8_t   right_deg;
    int8_t   centre_deg;      /* half way between them: steer away from here */
    int8_t   span_deg;        /* left_deg - right_deg */
    int8_t   hits;            /* samples that landed on it */
    uint16_t duration_ms;     /* how long the sweep took (budget check) */
} us_object_t;

/* Why every ping since power-up ended the way it did. Tells a wiring or power
 * fault apart from an empty road (printed with the health line). */
typedef struct
{
    uint32_t valid_cnt;   /* a normal distance, at or above US_MIN_VALID_MM */
    uint32_t clear_cnt;   /* ECHO rose but never fell: open space (valid) */
    uint32_t none_cnt;    /* ECHO never rose: no power, wrong pin, not wired */
    uint32_t stuck_cnt;   /* ECHO was already high and never fell */
    uint32_t close_cnt;   /* echo under 2 cm: something is touching the sensor */
} us_diag_t;

/* ---- Set-up (call from the owning task only: it enables the interrupt) ---- */
int     us_init(void);                      /* servo to 0 deg, TRIG low; 0 = ok */

/* ---- Distance ---- */
int32_t us_ping_mm(void);                   /* one ping: mm, US_NO_ECHO or US_INVALID */
int32_t us_distance_mm(void);               /* three pings, median filtered */
int32_t us_filter3(const int32_t s[3]);     /* pure: median of 3 raw samples */
void    us_diag_get(us_diag_t *p_diag);     /* copy out the counters */

/* Pure fault counter: feed it every raw ping. True once US_FAULT_INVALID_CNT
 * invalid pings have arrived in a row; a valid ping or NO_ECHO resets it. */
bool    us_fault_update(int *bad_cnt, int32_t raw_ping);

/* ---- Servo and the scans ---- */
uint32_t servo_angle_to_pulse_us(int deg);  /* pure: -90..+90 -> pulse width (clamped) */
void     servo_set_angle(int deg);          /* -90..+90, 0 = centre, +ve = left */
avoid_side_t us_pick_side(int32_t clear_L_mm, int32_t clear_R_mm);  /* pure */

int32_t us_scan_angle(int deg, uint32_t settle_ms);         /* move, settle, filtered */
int32_t us_scan_angle_single(int deg, uint32_t settle_ms);  /* move, settle, ONE ping */
void    us_scan_left_right(us_profile_t *p_out);            /* L, R, pick a side, recentre */
void    us_coarse_scan(us_coarse_t *p_out);                 /* the five-angle fan */

/* Stage 2.  us_object_from_samples() is pure: it takes one sweep's samples,
 * which must be in angle order, and measures the object in them.  Both
 * arrays hold n entries; mm[] may contain US_NO_ECHO / US_INVALID. */
void    us_object_from_samples(us_object_t *p_out, const int8_t *deg_arr,
                               const int32_t *mm_arr, int n, int32_t range_mm);

/* Sweep from_deg..to_deg in US_FINE_STEP_DEG steps (clamped to +-90), one
 * ping each, measure the object and recentre.  The car must be stopped. */
void    us_object_scan(us_object_t *p_out, int from_deg, int to_deg);
void    us_coarse_classify(us_coarse_t *p_c, int32_t range_mm);  /* pure: fill the masks */

#endif /* ULTRASONIC_H */
