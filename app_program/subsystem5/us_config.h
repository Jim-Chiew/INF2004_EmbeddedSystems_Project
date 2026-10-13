/* us_config.h - Buddy 5 tunables (NFR-MNT-01: one config header).
 * Values taken from the Subsystem 5 statechart / Week 6 design review. */
#ifndef US_CONFIG_H
#define US_CONFIG_H

/* ---- Pins (design review pin table) ---- */
#define US_TRIG_GPIO          2      /* HC-SR04 TRIG, GP2 */
#define US_ECHO_GPIO          3      /* HC-SR04 ECHO, GP3 (edge IRQ) */
#define SERVO_GPIO            15     /* SG90 PWM, GP15, 50 Hz */

/* ---- Ultrasonic timing ---- */
#define US_TRIG_PULSE_US      10
#define US_ECHO_TIMEOUT_US    18000  /* 18 ms, no echo => treat as clear */
#define US_PING_PERIOD_MS     20     /* FRONT_MONITOR tick */
#define US_MIN_VALID_MM       20     /* < 2 cm => invalid echo */
#define US_CLEAR_MM           4000   /* value used for "no echo" in filters */

/* ---- Obstacle decision ---- */
#define OBSTACLE_THRESHOLD_MM 150    /* 15 cm */
#define OBSTACLE_CONFIRM_CNT  2      /* 2 consecutive filtered readings */

/* ---- Servo ---- */
#define SERVO_PULSE_MIN_US    500    /* -90 deg  (tune on bench per SG90) */
#define SERVO_PULSE_MAX_US    2400   /* +90 deg */
#define SERVO_ANGLE_LEFT      90
#define SERVO_ANGLE_RIGHT     (-90)
#define SERVO_SETTLE_LEFT_MS  250
#define SERVO_SETTLE_RIGHT_MS 400    /* longest sweep: +90 -> -90 */
#define SERVO_SETTLE_CENTRE_MS 250
#define SCAN_PINGS_PER_SIDE   3
#define SIDE_TIE_MM           30     /* tie within 3 cm => LEFT */

/* ---- Demo ---- */
#define US_DEMO               1      /* 1 = start the bench-test stand-in for Nav */

#endif
