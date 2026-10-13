/* us_scan.h - Buddy 5: HC-SR04 + SG90 sensing API (portable core). */
#ifndef US_SCAN_H
#define US_SCAN_H
#include <stdint.h>
#include <stdbool.h>

#define US_NO_ECHO   (-1)   /* nothing in range => treat as clear */
#define US_INVALID   (-2)   /* stuck echo / too-close garbage */

typedef enum { AVOID_LEFT = 0, AVOID_RIGHT = 1 } avoid_side_t;

typedef struct {
    int32_t      clear_L_mm;   /* clearance at +90 deg (US_CLEAR_MM if open) */
    int32_t      clear_R_mm;   /* clearance at -90 deg */
    avoid_side_t side;         /* side with more clearance; tie (<3 cm) => LEFT */
} us_profile_t;

int     us_init(void);                      /* servo -> 0 deg, TRIG low; 0 = ok */

/* --- distance / detection (use from the owning task only) --- */
int32_t us_ping_mm(void);                   /* one raw ping, mm or US_NO_ECHO/US_INVALID */
int32_t us_distance_mm(void);               /* median-of-3 filtered, mm or US_NO_ECHO/US_INVALID */
int32_t us_filter3(const int32_t s[3]);     /* median-of-3 over raw samples (NO_ECHO=clear, INVALID dropped) */
bool    us_object_detected(int32_t threshold_mm, int32_t *dist_mm); /* dist_mm may be NULL */

/* --- servo / scan --- */
uint32_t servo_angle_to_pulse_us(int deg);  /* pure: -90..+90 -> pulse width (clamped) */
avoid_side_t us_pick_side(int32_t clear_L_mm, int32_t clear_R_mm); /* pure: tie (<3 cm) -> LEFT */
void    servo_set_angle(int deg);           /* -90..+90, 0 = centre, +ve = left */
int32_t us_scan_angle(int deg, uint32_t settle_ms);  /* move, settle, filtered distance */
void    us_scan_left_right(us_profile_t *out);       /* L, R, pick side, recentre (~1 s) */

#endif
