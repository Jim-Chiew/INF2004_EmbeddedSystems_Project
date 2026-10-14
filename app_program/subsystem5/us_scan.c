/* ultrasonic.c - portable logic: filtering, detection, servo scan.
 * Blocking calls use tk_dly_tsk() so other uT-Kernel tasks keep running. */
#include "us_scan.h"
#include "us_config.h"
#include "drv_ultrasonic.h"
#include <tk/tkernel.h>   /* tk_dly_tsk */

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

int32_t us_ping_mm(void)
{
    int32_t us = hal_us_ping_us(US_ECHO_TIMEOUT_US);
    if (us == HAL_ECHO_TIMEOUT) return US_NO_ECHO;
    if (us == HAL_ECHO_STUCK)   return US_INVALID;
    int32_t mm = us_to_mm(us);
    return (mm < US_MIN_VALID_MM) ? US_INVALID : mm;
}

/* Median of up to 3 samples. No-echo counts as "clear" (US_CLEAR_MM),
 * invalid samples are dropped. If a sample set has <3 valid values the
 * smaller (more conservative) middle value is used. */
int32_t us_filter3(const int32_t s[3])
{
    int32_t v[3]; int n = 0;
    for (int i = 0; i < 3; i++) {
        if (s[i] == US_INVALID) continue;
        v[n++] = (s[i] == US_NO_ECHO) ? US_CLEAR_MM : s[i];
    }
    if (n == 0) return US_INVALID;
    for (int i = 1; i < n; i++)                 /* insertion sort, n<=3 */
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
        if (i < 2) tk_dly_tsk(US_PING_PERIOD_MS);   /* let echoes die out */
    }
    return us_filter3(s);
}

bool us_object_detected(int32_t threshold_mm, int32_t *dist_mm)
{
    int32_t d = us_distance_mm();
    if (dist_mm) *dist_mm = d;
    return (d > 0 && d <= threshold_mm);   /* NO_ECHO / INVALID => false */
}

/* ---- servo ---- */
uint32_t servo_angle_to_pulse_us(int deg)
{
    if (deg >  90) deg =  90;
    if (deg < -90) deg = -90;
    /* -90 -> MIN, +90 -> MAX. (Flip sign here if the bracket is mounted mirrored.) */
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

/* Statechart OBSTACLE_PROFILING: SCAN_LEFT -> SCAN_RIGHT -> SELECT_SIDE -> RECENTRE */
void us_scan_left_right(us_profile_t *out)
{
    int32_t l = us_scan_angle(SERVO_ANGLE_LEFT,  SERVO_SETTLE_LEFT_MS);
    int32_t r = us_scan_angle(SERVO_ANGLE_RIGHT, SERVO_SETTLE_RIGHT_MS);

    /* NO_ECHO = fully open; INVALID = unknown => treat as blocked (fail-safe) */
    int32_t lv = (l == US_NO_ECHO) ? US_CLEAR_MM : (l == US_INVALID ? 0 : l);
    int32_t rv = (r == US_NO_ECHO) ? US_CLEAR_MM : (r == US_INVALID ? 0 : r);

    out->clear_L_mm = lv;
    out->clear_R_mm = rv;
    out->side = us_pick_side(lv, rv);

    servo_set_angle(0);
    tk_dly_tsk(SERVO_SETTLE_CENTRE_MS);
}
