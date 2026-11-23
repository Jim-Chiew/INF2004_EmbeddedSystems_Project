/*
 * ultrasonic.c
 * Subsystem 6 -- the portable logic: ping -> millimetres, the median-of-3
 * filter, the invalid-ping counter, servo angles and the two servo scans.
 * See ultrasonic.h. The hardware is behind drv_ultrasonic.h, so every
 * function here can be read (and checked by subsystem6_test.c) on its own.
 *
 * Every wait is tk_dly_tsk(), so the other tasks keep running while the servo
 * moves and while an echo is on its way. Called only by obstacle_task, which
 * owns the sensor and the servo (TEAM.md section 3).
 *
 * The kernel headers are included for tk_dly_tsk() / tk_get_tim() only; this
 * file must stay free of Pico SDK headers (the "header rule" in TEAM.md).
 */

#include "ultrasonic.h"
#include "drv_ultrasonic.h"

/* <stddef.h> and PROHIBIT_DEF_SIZE_T must come BEFORE the kernel headers:
   T-Kernel defines its own size_t (signed) and <math.h> brings in the C
   library's (unsigned); the two cannot both exist in one file.  Same rule and
   same reason as app.h (TEAM.md section 7, change 4). */
#include <stddef.h>
#define PROHIBIT_DEF_SIZE_T

#include <tk/tkernel.h>
#include <math.h>          /* sinf, cosf, sqrtf: the build links -lm (TEAM.md) */

/* ---- conversions ---- */
/* mm = us * 343 m/s / 2 = us * 0.1715 -> us * 343 / 2000 */
static int32_t us_to_mm(int32_t us) { return (int32_t)(((int64_t)us * 343) / 2000); }

int us_init(void)
{
    int er = hal_us_init();
    if (er < 0) return er;
    er = hal_servo_init();
    if (er < 0) return er;
    servo_set_angle(0);
    return 0;
}

bool us_fault_update(int *bad_cnt, int32_t raw_ping)
{
    if (raw_ping == US_INVALID) {
        if (*bad_cnt < US_FAULT_INVALID_CNT) (*bad_cnt)++;
    } else {
        *bad_cnt = 0;
    }
    return *bad_cnt >= US_FAULT_INVALID_CNT;
}

/* NOTE: written only by obstacle_task, read by us_diag_get(). A torn read of
 *       one counter is harmless for a diagnostic line. */
static volatile us_diag_t g_us_diag;

void us_diag_get(us_diag_t *p_diag)
{
    p_diag->valid_cnt = g_us_diag.valid_cnt;
    p_diag->clear_cnt = g_us_diag.clear_cnt;
    p_diag->none_cnt  = g_us_diag.none_cnt;
    p_diag->stuck_cnt = g_us_diag.stuck_cnt;
    p_diag->close_cnt = g_us_diag.close_cnt;
}

/**
 * @brief Fire one ping and convert the result to millimetres.
 *
 * Every outcome is also counted in g_us_diag, so the health line can show WHY
 * pings fail (no power and a wrong pin look identical in the distance alone).
 *
 * @return distance in mm, US_NO_ECHO (open space) or US_INVALID (unknown)
 */
int32_t us_ping_mm(void)
{
    int32_t result  = US_INVALID;
    int32_t dist_mm = 0;
    int32_t echo_us = hal_us_ping_us(US_ECHO_TIMEOUT_US);

    if (HAL_ECHO_TIMEOUT == echo_us)
    {
        /* ECHO rose and ran on: nothing in range, so open space. */
        g_us_diag.clear_cnt++;
        result = US_NO_ECHO;
    }
    else if (HAL_ECHO_STUCK == echo_us)
    {
        g_us_diag.stuck_cnt++;
        result = US_INVALID;
    }
    else if (HAL_ECHO_NONE == echo_us)
    {
        /* No rising edge at all: NOT "clear", the sensor is not answering. */
        g_us_diag.none_cnt++;
        result = US_INVALID;
    }
    else
    {
        dist_mm = us_to_mm(echo_us);
        if (dist_mm < US_MIN_VALID_MM)
        {
            g_us_diag.close_cnt++;
            result = US_INVALID;
        }
        else
        {
            g_us_diag.valid_cnt++;
            result = dist_mm;
        }
    }

    return result;
}

/* Median of up to 3 samples. No-echo counts as "clear" (US_CLEAR_MM), invalid
 * samples are dropped. With fewer than 3 valid values the smaller (more
 * careful) middle value is used. */
int32_t us_filter3(const int32_t s[3])
{
    int32_t v[3]; int n = 0;
    for (int i = 0; i < 3; i++) {
        if (s[i] == US_INVALID) continue;
        v[n++] = (s[i] == US_NO_ECHO) ? US_CLEAR_MM : s[i];
    }
    if (n == 0) return US_INVALID;
    for (int i = 1; i < n; i++)                 /* insertion sort, n <= 3 */
        for (int j = i; j > 0 && v[j-1] > v[j]; j--) {
            int32_t t = v[j]; v[j] = v[j-1]; v[j-1] = t;
        }
    int32_t m = v[(n - 1) / 2];
    return (m >= US_CLEAR_MM) ? US_NO_ECHO : m;
}

int32_t us_distance_mm(void)
{
    int32_t s[3];
    for (int i = 0; i < 3; i++) {
        s[i] = us_ping_mm();
        if (i < 2) tk_dly_tsk(US_PING_GAP_MS);   /* let the echo die out */
    }
    return us_filter3(s);
}

/* ---- servo ---- */
uint32_t servo_angle_to_pulse_us(int deg)
{
    if (deg >  90) deg =  90;
    if (deg < -90) deg = -90;
    /* -90 -> MIN, +90 -> MAX. (Flip the sign here if the bracket is mirrored.) */
    uint32_t span = SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US;
    return SERVO_PULSE_MIN_US + (uint32_t)((deg + 90) * (int)span) / 180u;
}

void servo_set_angle(int deg)
{
    hal_servo_pulse_us(servo_angle_to_pulse_us(deg));
}

avoid_side_t us_pick_side(int32_t clear_L_mm, int32_t clear_R_mm)
{
    return (clear_R_mm > clear_L_mm + SIDE_TIE_MM) ? AVOID_RIGHT : AVOID_LEFT;
}

int32_t us_scan_angle(int deg, uint32_t settle_ms)
{
    servo_set_angle(deg);
    tk_dly_tsk(settle_ms);
    return us_distance_mm();
}

/* Clearance either side: SCAN_LEFT -> SCAN_RIGHT -> pick a side -> recentre.
 * Takes about one second, so the car must be stopped. */
void us_scan_left_right(us_profile_t *p_out)
{
    int32_t l = us_scan_angle(SERVO_ANGLE_LEFT,  SERVO_SETTLE_LEFT_MS);
    int32_t r = us_scan_angle(SERVO_ANGLE_RIGHT, SERVO_SETTLE_RIGHT_MS);

    /* NO_ECHO = wide open; INVALID = unknown, so treat it as blocked */
    int32_t lv = (l == US_NO_ECHO) ? US_CLEAR_MM : (l == US_INVALID ? 0 : l);
    int32_t rv = (r == US_NO_ECHO) ? US_CLEAR_MM : (r == US_INVALID ? 0 : r);

    p_out->clear_L_mm = lv;
    p_out->clear_R_mm = rv;
    p_out->side = us_pick_side(lv, rv);

    servo_set_angle(0);
    tk_dly_tsk(SERVO_SETTLE_CENTRE_MS);
}

/* ---- the coarse scan ---- */

/* Pure: work out the blocked / invalid masks and the closest blocked sample.
 * NO_ECHO = clear (not blocked). INVALID = unknown: flagged, never "clear". */
void us_coarse_classify(us_coarse_t *p_c, int32_t range_mm)
{
    p_c->blocked_mask = 0;
    p_c->invalid_mask = 0;
    p_c->closest_idx  = -1;
    for (int i = 0; i < US_COARSE_N; i++) {
        int32_t d = p_c->mm[i];
        if (d == US_INVALID) { p_c->invalid_mask |= (uint8_t)(1u << i); continue; }
        if (d == US_NO_ECHO) continue;
        if (d > 0 && d <= range_mm) {
            p_c->blocked_mask |= (uint8_t)(1u << i);
            if (p_c->closest_idx < 0 || d < p_c->mm[p_c->closest_idx]) {
                p_c->closest_idx = (int8_t)i;
            }
        }
    }
}

/**
 * @brief Point the servo at an angle, wait for it to arrive, take ONE ping.
 *
 * NOTE: us_scan_angle() takes the median of three pings, which costs about
 *       70 ms per angle. The coarse scan only has to spot "something is over
 *       there", so one ping is enough. A single invalid ping (a garbled echo)
 *       is retried once, so one bad echo does not leave a hole in the fan; if
 *       the retry is invalid too the caller sees US_INVALID and treats that
 *       angle as unknown, never as clear.
 *
 * @param deg        servo angle, 0 = centre, positive = left
 * @param settle_ms  wait for the servo to arrive before pinging
 * @return           distance in mm, US_NO_ECHO (clear) or US_INVALID
 */
int32_t us_scan_angle_single(int deg, uint32_t settle_ms)
{
    int32_t reading;

    /* Step 1: move the servo and give it time to arrive. */
    servo_set_angle(deg);
    tk_dly_tsk(settle_ms);

    /* Step 2: take one ping. */
    reading = us_ping_mm();

    /* Step 3: retry once if it was invalid. The wait lets any echo still
     * ringing in the sensor die out before the second ping. */
    if (US_INVALID == reading)
    {
        tk_dly_tsk(US_PING_GAP_MS);
        reading = us_ping_mm();
    }

    return reading;
}

/**
 * @brief Look at five angles in turn and classify each one.
 *
 * Order of work (the servo sweeps right to left):
 *   1. note the start time;
 *   2. for each angle in SCAN_COARSE_DEG: move, settle, take one ping;
 *   3. work out which angles are blocked or unknown, and which is closest;
 *   4. put the servo back to the centre;
 *   5. note the end time and store how long the scan took.
 *
 * Time cost: the settle waits (SCAN_COARSE_SETTLE_TOTAL_MS) plus five pings.
 * Test S6-U19 checks that this fits SCAN_COARSE_BUDGET_MS.
 *
 * @param p_out  filled with the angles, distances, masks and the duration
 */
void us_coarse_scan(us_coarse_t *p_out)
{
    static const int8_t scan_deg[US_COARSE_N] = SCAN_COARSE_DEG;
    SYSTIM   start_time = { 0 };
    SYSTIM   end_time   = { 0 };
    uint32_t settle_ms  = 0u;
    int      angle_idx  = 0;

    /* Step 1: note the start time so the scan can be timed. */
    tk_get_tim(&start_time);

    /* Step 2: visit every angle. */
    for (angle_idx = 0; angle_idx < US_COARSE_N; angle_idx++)
    {
        /* The first move starts from the centre and is a 60 deg move; every
         * later move is a single 30 deg step. */
        if (0 == angle_idx)
        {
            settle_ms = SCAN_COARSE_SETTLE_FIRST_MS;
        }
        else
        {
            settle_ms = SCAN_COARSE_SETTLE_STEP_MS;
        }

        p_out->deg[angle_idx] = scan_deg[angle_idx];
        p_out->mm[angle_idx]  = us_scan_angle_single(scan_deg[angle_idx],
                                                     settle_ms);
    }

    /* Step 3: turn the five readings into masks and the closest angle. */
    us_coarse_classify(p_out, SCAN_OBSTACLE_RANGE_MM);

    /* Step 4: park the servo at the centre again. */
    servo_set_angle(0);
    tk_dly_tsk(SCAN_COARSE_RECENTRE_MS);

    /* Step 5: store the scan time. SYSTIM is a hi/lo pair; lo counts ms. */
    tk_get_tim(&end_time);

    /* Cast: the unsigned subtraction stays right if the lo counter wraps, and
     * a scan lasts well under 65 s, so the result fits in uint16_t. */
    p_out->duration_ms = (uint16_t)(end_time.lo - start_time.lo);
}

/* ---- Stage 2: how long is the object ---- */

#define DEG_TO_RAD      0.017453292f

/* Does this sample sit on the same surface as the one next to it?  A sample
 * that is clear, unknown, out of range, or a long step further away is NOT
 * part of the object: that step is where the object's edge is (or where the
 * wall behind it begins). */
static bool same_surface(int32_t d, int32_t neighbour_mm, int32_t range_mm)
{
    int32_t gap;

    if (d <= 0 || d > range_mm) {
        return false;
    }
    gap = d - neighbour_mm;
    if (gap < 0) {
        gap = -gap;
    }
    return gap <= US_OBJ_STEP_MM;
}

/**
 * @brief Measure the object in one sweep's samples. Pure: no hardware.
 *
 * How it works:
 *   1. find the closest sample that is on something inside range_mm;
 *   2. grow outwards from it while the surface carries on (same_surface),
 *      which lands on the object's two edges;
 *   3. each sample is a polar reading, so turn both edges into x (across the
 *      path, +ve left) and y (along it) and take the straight line between
 *      them: that is the object's length;
 *   4. take the beam allowance off, as a second, more careful number.
 *
 * Step 2 is why a wall behind a box does not get measured as one huge object,
 * and why two boxes side by side read as one only if their faces are within
 * US_OBJ_STEP_MM of each other.
 *
 * @param p_out     filled in; p_out->found is false when the window is clear
 * @param deg_arr   servo angle of each sample, IN ANGLE ORDER
 * @param mm_arr    distance of each sample, or US_NO_ECHO / US_INVALID
 * @param n         how many samples
 * @param range_mm  ignore anything further away than this
 */
void us_object_from_samples(us_object_t *p_out, const int8_t *deg_arr,
                            const int32_t *mm_arr, int n, int32_t range_mm)
{
    int   i, lo, hi, near_i = -1;
    int   left, right;
    float x_lo, y_lo, x_hi, y_hi, dx, dy;

    p_out->found          = false;
    p_out->length_mm      = 0;
    p_out->length_corr_mm = 0;
    p_out->near_mm        = US_NO_ECHO;
    p_out->left_deg       = 0;
    p_out->right_deg      = 0;
    p_out->centre_deg     = 0;
    p_out->span_deg       = 0;
    p_out->hits           = 0;

    /* Step 1: the closest sample that is really on something. */
    for (i = 0; i < n; i++) {
        if (mm_arr[i] <= 0 || mm_arr[i] > range_mm) {
            continue;                    /* clear, unknown, or too far */
        }
        if (near_i < 0 || mm_arr[i] < mm_arr[near_i]) {
            near_i = i;
        }
    }
    if (near_i < 0) {
        return;                          /* nothing in the window to measure */
    }

    /* Step 2: grow out to both edges. */
    lo = near_i;
    hi = near_i;
    while (lo > 0 && same_surface(mm_arr[lo - 1], mm_arr[lo], range_mm)) {
        lo--;
    }
    while (hi < n - 1 && same_surface(mm_arr[hi + 1], mm_arr[hi], range_mm)) {
        hi++;
    }

    /* Step 3: the straight line between the two edge points. */
    x_lo = (float)mm_arr[lo] * sinf((float)deg_arr[lo] * DEG_TO_RAD);
    y_lo = (float)mm_arr[lo] * cosf((float)deg_arr[lo] * DEG_TO_RAD);
    x_hi = (float)mm_arr[hi] * sinf((float)deg_arr[hi] * DEG_TO_RAD);
    y_hi = (float)mm_arr[hi] * cosf((float)deg_arr[hi] * DEG_TO_RAD);
    dx = x_lo - x_hi;
    dy = y_lo - y_hi;

    /* The sweep may run either way round, so sort the two edge angles. */
    left  = (deg_arr[lo] > deg_arr[hi]) ? deg_arr[lo] : deg_arr[hi];
    right = (deg_arr[lo] > deg_arr[hi]) ? deg_arr[hi] : deg_arr[lo];

    p_out->found      = true;
    p_out->hits       = (int8_t)(hi - lo + 1);
    p_out->near_mm    = mm_arr[near_i];
    p_out->left_deg   = (int8_t)left;
    p_out->right_deg  = (int8_t)right;
    p_out->centre_deg = (int8_t)((left + right) / 2);
    p_out->span_deg   = (int8_t)(left - right);
    p_out->length_mm  = (int32_t)(sqrtf(dx * dx + dy * dy) + 0.5f);

    /* Step 4: the beam reads the span about US_BEAM_DEG too wide, so scale the
     * length by how much of the span is left once that is taken off.  This is
     * a rough linear correction, not a model of the cone; the measurement
     * above is the real reading and both are published.  A span no wider than
     * the beam means "narrower than the sensor can resolve", so 0 here, with
     * hits == 1 saying the object fitted inside a single step. */
    if (p_out->span_deg > US_BEAM_DEG) {
        p_out->length_corr_mm = p_out->length_mm *
                                (p_out->span_deg - US_BEAM_DEG) /
                                p_out->span_deg;
    }
}

/**
 * @brief Sweep a window of angles and measure the object in it.
 *
 * The first move comes from the centre and can be up to 90 deg, so it gets the
 * long settle; every step after it is only US_FINE_STEP_DEG.
 *
 * @param p_out     the measurement
 * @param from_deg  window start, clamped to -90..+90
 * @param to_deg    window end (swapped with from_deg if they are the wrong way round)
 */
void us_object_scan(us_object_t *p_out, int from_deg, int to_deg)
{
    /* static: ~190 bytes, kept off the 2 KB task stack.  Only obstacle_task
     * calls this (it owns the sensor), so one copy is enough. */
    static int8_t  deg_arr[US_FINE_MAX];
    static int32_t mm_arr[US_FINE_MAX];
    SYSTIM   start_time = { 0 };
    SYSTIM   end_time   = { 0 };
    uint32_t settle_ms;
    int      angle, n = 0;

    /* Clamped to the scan limit, not to 90: the servo must not be asked for
       an angle it cannot reach (see SERVO_SCAN_MAX_DEG in ultrasonic.h). */
    if (from_deg < -SERVO_SCAN_MAX_DEG) from_deg = -SERVO_SCAN_MAX_DEG;
    if (from_deg >  SERVO_SCAN_MAX_DEG) from_deg =  SERVO_SCAN_MAX_DEG;
    if (to_deg   < -SERVO_SCAN_MAX_DEG) to_deg   = -SERVO_SCAN_MAX_DEG;
    if (to_deg   >  SERVO_SCAN_MAX_DEG) to_deg   =  SERVO_SCAN_MAX_DEG;
    if (to_deg < from_deg) {
        int swap = from_deg; from_deg = to_deg; to_deg = swap;
    }

    tk_get_tim(&start_time);

    for (angle = from_deg;
         angle <= to_deg && n < US_FINE_MAX;
         angle += US_FINE_STEP_DEG) {
        settle_ms = (0 == n) ? SCAN_COARSE_SETTLE_FIRST_MS : SCAN_FINE_SETTLE_MS;
        deg_arr[n] = (int8_t)angle;
        mm_arr[n]  = us_scan_angle_single(angle, settle_ms);
        n++;
    }

    us_object_from_samples(p_out, deg_arr, mm_arr, n, SCAN_OBSTACLE_RANGE_MM);

    servo_set_angle(0);
    tk_dly_tsk(SCAN_COARSE_RECENTRE_MS);

    tk_get_tim(&end_time);
    p_out->duration_ms = (uint16_t)(end_time.lo - start_time.lo);
}

/* end of file */
