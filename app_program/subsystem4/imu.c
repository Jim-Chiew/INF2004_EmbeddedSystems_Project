// imu.c - LSM303D (GY-511) driver implementation (subsystem 4)
//
// Plain C (see imu.h): the sensor is reached through drv_i2c, time comes
// from the RP2040 hardware timer (time_us_64, provided by the kernel's WiFi
// port), and the timed calibration phases are run by imu_task.

#include "imu.h"
#include "drv_i2c.h"
#include "lift_gate.h"
#include "hump_height.h"

#include <stddef.h>
#include <math.h>

extern uint64_t time_us_64(void);

// ---- Device registers ----
#define DEV_ADDR   0x1D
#define WHO_AM_I   0x0F
#define OUT_X_L_M  0x08
#define CTRL1      0x20
#define CTRL2      0x21
#define CTRL5      0x24
#define CTRL6      0x25
#define CTRL7      0x26
#define OUT_X_L_A  0x28

#define RAD2DEG       57.2957795f
#define ACC_G_PER_LSB 0.000061f   // +/-2 g full scale

// ---- Module state ----
static vec3  g_mag_min = { 32767,  32767,  32767};
static vec3  g_mag_max = {-32768, -32768, -32768};
static float g_pitch_off = 0.0f;
static float g_roll_off  = 0.0f;

// Hump detection state
static vec3     g_up         = {0.0f, 0.0f, 1.0f}; // unit vector pointing "up" (sensor frame)
static float    g_g_ref      = 1.0f;               // measured gravity magnitude at rest (g)
static bool     g_in_hump    = false;
static float    g_hump_peak  = 0.0f;               // running peak of the hump in progress
static float    g_hump_last  = 0.0f;
static float    g_hump_max   = 0.0f;
static uint32_t g_hump_count = 0;

// Lift / overturn detection state
#define LIFT_BASE_TC_S     1.0f   // time constant of the at-rest baseline (s)
#define LIFT_FILT_TC_S     0.03f  // low-pass on vertical accel: smooths vibration (s)
#define LIFT_VEL_TC_S      0.5f   // on the floor the speed estimate leaks away (s)
#define LIFT_END_FRACTION  0.25f  // move up is over once speed falls below this x peak
#define LIFT_COAST_G       0.05f  // |accel| below this while rising = coasting
#define LIFT_COAST_MAX_S   0.30f  // a hand never coasts this long: it is drift, not a lift
#define LIFT_MAX_DT_S      0.05f  // longer gap between reads = can't integrate

// The lift tests below are measured against how hard the car is shaking, not
// against the fixed numbers above -- those describe a car AT REST, and driving
// this one shakes it harder than IMU_LIFT_TRIGGER_G. g_vib_ms is that shaking.
// The arithmetic, and the run that proved it was needed, are in lift_gate.h.
#define TILT_FILT_TC_S     0.05f  // low-pass on the acceleration vector for the tilt check (s)

typedef enum {
    LIFT_IDLE,     // on the floor (as far as we know)
    LIFT_RISING,   // upward push seen: following the move up
    LIFT_HELD,     // a real lift finished: waiting out IMU_LIFT_CONFIRM_MS
} lift_state_t;

static lift_state_t g_lift_state = LIFT_IDLE;
static float    g_vert_base     = 1.0f;  // slow average of accel along "up" at rest (g)
static float    g_vert_filt     = 0.0f;  // low-passed vertical accel, baseline removed (g)
static float    g_vib_ms        = 0.0f;  // mean square of the vertical vibration (g^2)
static float    g_lift_vel      = 0.0f;  // vertical speed estimate, m/s (always running)
static float    g_lift_h        = 0.0f;  // rise of the move being followed (m)
static float    g_lift_vpeak    = 0.0f;  // top upward speed of that move (m/s)
static float    g_lift_coast_s  = 0.0f;  // time spent coasting during that move (s)
static uint64_t g_lift_start_us = 0;     // lift-off (the upward push)
static bool     g_lifted        = false; // latched until imu_reset_lift()
static uint64_t g_last_read_us  = 0;
static float    g_lift_last_cm    = 0.0f; // rise of the last upward move (cm)
static float    g_lift_last_speed = 0.0f; // top speed of the last upward move (m/s)

static vec3     g_tilt_acc      = {0.0f, 0.0f, 1.0f}; // low-passed acceleration vector
static uint64_t g_tilt_last_us  = 0;
static bool     g_tilt_over     = false; // past the tilt limit right now
static uint64_t g_tilt_since_us = 0;
static bool     g_overturned    = false; // past the limit for IMU_TILT_CONFIRM_MS

// Calibration in progress (see imu.h). Only imu_task calls these, so plain
// statics are enough.
static bool  g_compass_cal  = false;           // widening the magnetometer min/max
static vec3  g_mag_min_old, g_mag_max_old;     // put back by imu_compass_cal_abort()
static bool  g_zeroing      = false;           // averaging the gravity vector
static vec3  g_zero_sum;
static int   g_zero_n       = 0;
static float g_pitch_off_old, g_roll_off_old;  // put back by imu_zero_abort()

// ---- Low-level I2C helpers ----
static int reg_write(uint8_t reg, uint8_t val) {
    uint8_t b[2] = {reg, val};
    return i2c_drv_write(DEV_ADDR, b, 2, false);
}

static int reg_read(uint8_t reg, uint8_t *dst, size_t n) {
    if (n > 1) reg |= 0x80;  // auto-increment
    int r = i2c_drv_write(DEV_ADDR, &reg, 1, true);
    if (r < 0) return r;
    return i2c_drv_read(DEV_ADDR, dst, n, false);
}

// ---- Math helpers ----
static vec3 cross(vec3 a, vec3 b) {
    return (vec3){a.y * b.z - a.z * b.y,
                  a.z * b.x - a.x * b.z,
                  a.x * b.y - a.y * b.x};
}

static float dot(vec3 a, vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

static vec3 normalize(vec3 v) {
    float n = sqrtf(dot(v, v));
    if (n < 1e-6f) return v;
    return (vec3){v.x / n, v.y / n, v.z / n};
}

// Keeps angles within +/-180 degrees after subtracting offsets
static float wrap180(float a) {
    while (a > 180.0f)  a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

// ---- Sensor reading ----
// Reads acceleration (g) and raw magnetometer counts.
static bool read_sensors(vec3 *acc, vec3 *mag_raw) {
    uint8_t a[6], m[6];
    if (reg_read(OUT_X_L_A, a, 6) < 0 || reg_read(OUT_X_L_M, m, 6) < 0) {
        return false;
    }

    *acc = (vec3){
        (int16_t)((a[1] << 8) | a[0]) * ACC_G_PER_LSB,
        (int16_t)((a[3] << 8) | a[2]) * ACC_G_PER_LSB,
        (int16_t)((a[5] << 8) | a[4]) * ACC_G_PER_LSB
    };

    *mag_raw = (vec3){
        (float)(int16_t)((m[1] << 8) | m[0]),
        (float)(int16_t)((m[3] << 8) | m[2]),
        (float)(int16_t)((m[5] << 8) | m[4])
    };
    return true;
}

static void update_minmax(vec3 m) {
    if (m.x < g_mag_min.x) g_mag_min.x = m.x;
    if (m.x > g_mag_max.x) g_mag_max.x = m.x;
    if (m.y < g_mag_min.y) g_mag_min.y = m.y;
    if (m.y > g_mag_max.y) g_mag_max.y = m.y;
    if (m.z < g_mag_min.z) g_mag_min.z = m.z;
    if (m.z > g_mag_max.z) g_mag_max.z = m.z;
}

// ---- Hump detection ----
static void hump_reset_state(void) {
    g_in_hump    = false;
    g_hump_peak  = 0.0f;
    g_hump_last  = 0.0f;
    g_hump_max   = 0.0f;
    g_hump_count = 0;
}

// Feeds one vertical-acceleration sample into the detector.
// Returns true when a hump has just finished.
static bool hump_update(float vert) {
    float mag = fabsf(vert);

    if (!g_in_hump) {
        if (mag >= IMU_HUMP_THRESHOLD_G) {
            g_in_hump   = true;
            g_hump_peak = mag;
        }
        return false;
    }

    if (mag > g_hump_peak) g_hump_peak = mag;

    if (mag < IMU_HUMP_RELEASE_G) {
        g_in_hump   = false;
        g_hump_last = g_hump_peak;
        if (g_hump_peak > g_hump_max) g_hump_max = g_hump_peak;
        g_hump_count++;
        return true;
    }
    return false;
}

static void fill_hump_fields(imu_data_t *d) {
    d->hump_last  = g_hump_last;
    d->hump_max   = g_hump_max;
    d->hump_count = g_hump_count;
}

// ---- Lift / overturn detection ----

// Back to "on the floor". The speed estimate itself carries on.
static void lift_idle(void) {
    if (g_lift_state == LIFT_RISING) {     // remember the move, for tuning
        g_lift_last_cm    = g_lift_h * 100.0f;
        g_lift_last_speed = g_lift_vpeak;
    }
    g_lift_state   = LIFT_IDLE;
    g_lift_h       = 0.0f;
    g_lift_vpeak   = 0.0f;
    g_lift_coast_s = 0.0f;
}

static void safety_reset_state(void) {
    g_lift_state   = LIFT_IDLE;
    lift_idle();
    g_lift_vel     = 0.0f;
    g_lifted       = false;
    g_vert_base    = g_g_ref;
    g_vert_filt    = 0.0f;
    g_vib_ms       = 0.0f;
    g_last_read_us = 0;
    g_tilt_acc     = (vec3){g_up.x * g_g_ref, g_up.y * g_g_ref, g_up.z * g_g_ref};
    g_tilt_last_us = 0;
    g_tilt_over    = false;
    g_overturned   = false;
}

// Overturned = the car's "up" axis (from GP21 zeroing; board +Z before that)
// is more than IMU_TILT_LIMIT_DEG from vertical, in ANY direction (pitch or
// roll, either way), for IMU_TILT_CONFIRM_MS. The acceleration vector is
// low-passed first so motor vibration and bumps cannot fake it, and the test
// does not depend on how the IMU is mounted.
static void tilt_update(vec3 acc, uint64_t now) {
    float dt = g_tilt_last_us ? (float)(now - g_tilt_last_us) / 1e6f : 1.0f;
    g_tilt_last_us = now;
    float k = dt / (TILT_FILT_TC_S + dt);  // slow callbacks (calibration): ~unfiltered
    g_tilt_acc.x += (acc.x - g_tilt_acc.x) * k;
    g_tilt_acc.y += (acc.y - g_tilt_acc.y) * k;
    g_tilt_acc.z += (acc.z - g_tilt_acc.z) * k;

    float len  = sqrtf(dot(g_tilt_acc, g_tilt_acc));
    bool  over = len > 0.3f &&  // in free fall the direction means nothing
                 dot(g_tilt_acc, g_up) < len * cosf(IMU_TILT_LIMIT_DEG / RAD2DEG);
    if (!over) {
        g_tilt_over  = false;
        g_overturned = false;
        return;
    }
    if (!g_tilt_over) {
        g_tilt_over     = true;
        g_tilt_since_us = now;
    }
    if (now - g_tilt_since_us >= (uint64_t)IMU_TILT_CONFIRM_MS * 1000u)
        g_overturned = true;
}

// Lift detector. up_g: acceleration along the "up" axis including gravity (g).
//
// Driving shakes the car but never carries it UP by many cm. A hand lift has
// a clear shape, and all of it must be seen:
//   1. an upward push (filtered accel above IMU_LIFT_TRIGGER_G),
//   2. a top speed of at least IMU_LIFT_MIN_SPEED,
//   3. braking to a stop (the hand stops) within IMU_LIFT_RISE_MAX_MS,
//      without long coasting at constant speed (that is sensor drift),
//   4. a rise of at least IMU_LIFT_HEIGHT_CM,
//   5. and no lowering (faster than IMU_LIFT_DOWN_SPEED) until
//      IMU_LIFT_CONFIRM_MS after lift-off.
// The speed estimate runs all the time (leaking away on the floor so noise
// cannot build up), so the fall before a bump's landing is not forgotten and
// the landing jolt is not mistaken for a launch.
static void lift_update(float up_g, uint64_t now) {
    float dt = g_last_read_us ? (float)(now - g_last_read_us) / 1e6f : 0.0f;
    g_last_read_us = now;
    if (dt <= 0.0f || dt > LIFT_MAX_DT_S) {
        g_lift_state = LIFT_IDLE;  // first read or a long gap: can't integrate
        lift_idle();
        g_lift_vel  = 0.0f;
        g_vert_filt = 0.0f;
        return;
    }
    if (g_lifted) return;          // latched until imu_reset_lift()

    // Vertical acceleration: gravity and sensor bias removed, vibration smoothed
    float raw = up_g - g_vert_base;
    g_vert_filt += (raw - g_vert_filt) * (dt / (LIFT_FILT_TC_S + dt));
    float a = g_vert_filt;
    g_lift_vel += a * 9.81f * dt;

    float rms = sqrtf(g_vib_ms);   // how hard the car is shaking now (g)

    if (g_lift_state == LIFT_IDLE) {
        // On the floor: follow slow changes (sensor bias, gentle slopes),
        // measure the vibration, and let the speed estimate leak away
        g_vert_base += (up_g - g_vert_base) * (dt / LIFT_BASE_TC_S);
        g_vib_ms    += (raw * raw - g_vib_ms) * (dt / LIFT_BASE_TC_S);
        g_lift_vel  -= g_lift_vel * (dt / LIFT_VEL_TC_S);
        // The push has to beat the fixed gate AND the shaking
        if (a < lift_gate_trigger_g(rms, IMU_LIFT_TRIGGER_G) ||
            g_lift_vel <= 0.0f) return;
        g_lift_state    = LIFT_RISING; // pushed upward: maybe a lift, follow it
        g_lift_start_us = now;
        g_lift_h        = 0.0f;
        g_lift_vpeak    = 0.0f;
        g_lift_coast_s  = 0.0f;
    }

    uint64_t since_us = now - g_lift_start_us;

    if (g_lift_state == LIFT_RISING) {
        g_lift_h += g_lift_vel * dt;
        if (g_lift_vel > g_lift_vpeak) g_lift_vpeak = g_lift_vel;
        if (fabsf(a) < lift_gate_coast_g(rms, LIFT_COAST_G)) {
            g_lift_coast_s += dt;    // nothing pushing: see lift_gate.h
        }

        if (g_lift_coast_s > LIFT_COAST_MAX_S) {
            lift_idle();           // steady "speed" with nothing pushing: drift
            return;
        }
        if (g_lift_vel > LIFT_END_FRACTION * g_lift_vpeak) {
            // Still moving up. A jolt is never braked, so it times out here
            if (since_us > (uint64_t)IMU_LIFT_RISE_MAX_MS * 1000u) lift_idle();
            return;
        }

        // Braked: the move up is over. Fast and high enough for a lift, and
        // faster than the shaking alone could have faked over this long?
        {
            float need = lift_gate_min_speed(rms, (float)since_us / 1e6f,
                                             IMU_LIFT_MIN_SPEED);

            if (g_lift_vpeak < need ||
                g_lift_h < IMU_LIFT_HEIGHT_CM / 100.0f) {
                lift_idle();
                return;
            }
        }
        g_lift_last_cm    = g_lift_h * 100.0f;
        g_lift_last_speed = g_lift_vpeak;
        g_lift_state = LIFT_HELD;  // a real lift: the car is up in the air
                                   // (speed kept: the rest of the braking brings it to 0)
    } else if (g_lift_vel < -IMU_LIFT_DOWN_SPEED) {
        g_lift_state = LIFT_IDLE;  // HELD, but lowered again: put back down
        lift_idle();
        return;
    }

    // Off the floor continuously since lift-off for the confirm time
    if (since_us >= (uint64_t)IMU_LIFT_CONFIRM_MS * 1000u) {
        g_lifted     = true;
        g_lift_state = LIFT_IDLE;
        g_lift_vel   = 0.0f;
    }
}

static void fill_safety_fields(imu_data_t *d) {
    d->overturned      = g_overturned;
    d->lifted          = g_lifted;
    d->lift_phase      = (uint8_t)g_lift_state;
    d->lift_height_cm  = g_lift_state != LIFT_IDLE ? g_lift_h * 100.0f : 0.0f;
    d->lift_last_cm    = g_lift_last_cm;
    d->lift_last_speed = g_lift_last_speed;
    d->vib_g           = sqrtf(g_vib_ms);
}

// Turns raw sensor values into pitch/roll/heading using current calibration.
static imu_data_t compute(vec3 acc, vec3 mag_raw) {
    imu_data_t d;
    d.acc = acc;

    // Hard-iron offset: (max + min) / 2
    vec3 mag = {
        mag_raw.x - (g_mag_max.x + g_mag_min.x) / 2.0f,
        mag_raw.y - (g_mag_max.y + g_mag_min.y) / 2.0f,
        mag_raw.z - (g_mag_max.z + g_mag_min.z) / 2.0f
    };

    // Tilt angles, both with full +/-180 range
    float roll_raw  = atan2f(acc.y, acc.z) * RAD2DEG;
    float pitch_raw = atan2f(acc.x, acc.z) * RAD2DEG;
    d.roll  = wrap180(roll_raw  - g_roll_off);
    d.pitch = wrap180(pitch_raw - g_pitch_off);

    // Tilt-compensated heading of the board's +X axis
    vec3 east  = normalize(cross(mag, acc));
    vec3 north = normalize(cross(acc, east));
    float heading = atan2f(east.x, north.x) * RAD2DEG;
    if (heading < 0) heading += 360.0f;
    d.heading = heading;

    // Vertical acceleration along the zeroed "up" axis, gravity removed
    d.vert_acc      = dot(acc, g_up) - g_g_ref;
    d.hump_detected = false;
    fill_hump_fields(&d);

    // Tilt check here so it also runs during the calibration callbacks
    tilt_update(acc, time_us_64());
    fill_safety_fields(&d);

    return d;
}

// ---- Public API ----
bool imu_reconfigure(uint8_t *who_am_i) {
    uint8_t who = 0;
    int r = reg_read(WHO_AM_I, &who, 1);
    if (who_am_i) *who_am_i = who;
    if (r < 0 || who != 0x49) return false;

    // Accelerometer: 50 Hz (AODR 0101), block data update, XYZ enabled.
    // imu_task reads every 10 ms, so each sample is usually read twice.
    // Each write is checked: a NAK means the sensor is not answering.
    if (reg_write(CTRL1, 0x5F) < 0) return false;
    // CTRL2: +/-2 g; anti-alias filter 773 Hz, or 50 Hz with IMU_ACC_AA_50HZ=1
    if (reg_write(CTRL2, IMU_ACC_AA_50HZ ? 0xC0 : 0x00) < 0) return false;
    // Magnetometer: high resolution, 50 Hz, +/-2 gauss, continuous conversion
    if (reg_write(CTRL5, 0x70) < 0) return false;
    if (reg_write(CTRL6, 0x00) < 0) return false;
    if (reg_write(CTRL7, 0x00) < 0) return false;
    return true;
}

bool imu_init(uint8_t *who_am_i) {
    i2c_drv_init(400 * 1000);       // I2C1 on GP2/GP3, 400 kHz

    if (!imu_reconfigure(who_am_i)) return false;
    g_compass_cal = false;
    g_zeroing     = false;

    // Reset calibration state
    g_mag_min = (vec3){ 32767,  32767,  32767};
    g_mag_max = (vec3){-32768, -32768, -32768};
    g_pitch_off = 0.0f;
    g_roll_off  = 0.0f;

    // Default hump reference: board flat, 1 g
    g_up    = (vec3){0.0f, 0.0f, 1.0f};
    g_g_ref = 1.0f;
    hump_reset_state();
    safety_reset_state();
    hh_reset();             // hump height: learn level again from scratch
    return true;
}

void imu_compass_cal_begin(void) {
    g_mag_min_old = g_mag_min;
    g_mag_max_old = g_mag_max;
    g_mag_min = (vec3){ 32767,  32767,  32767};
    g_mag_max = (vec3){-32768, -32768, -32768};
    g_compass_cal = true;
}

void imu_compass_cal_end(void) {
    g_compass_cal = false;
}

void imu_compass_cal_abort(void) {
    if (g_compass_cal) {
        g_mag_min = g_mag_min_old;
        g_mag_max = g_mag_max_old;
    }
    g_compass_cal = false;
}

void imu_zero_begin(void) {
    // Readings in between show raw (un-zeroed) angles
    g_pitch_off_old = g_pitch_off;
    g_roll_off_old  = g_roll_off;
    g_pitch_off = 0.0f;
    g_roll_off  = 0.0f;
    g_zero_sum  = (vec3){0.0f, 0.0f, 0.0f};
    g_zero_n    = 0;
    g_zeroing   = true;
}

int imu_zero_end(void) {
    vec3 sum = g_zero_sum;
    int  n   = g_zero_n;

    g_zeroing = false;
    if (n <= 0) {
        imu_zero_abort();
        return 0;
    }

    // Average the gravity vector (not the angles) so zeroing is safe near +/-180
    g_pitch_off = atan2f(sum.x, sum.z) * RAD2DEG;
    g_roll_off  = atan2f(sum.y, sum.z) * RAD2DEG;

    // Same averaged gravity vector defines "up" and 1 g for hump detection
    float len = sqrtf(dot(sum, sum));
    if (len > 1e-6f) {
        g_up    = normalize(sum);
        g_g_ref = len / (float)n;
    }
    hump_reset_state();
    safety_reset_state();  // new "up" axis and 1 g reference
    return n;
}

void imu_zero_abort(void) {
    if (g_zeroing || g_zero_n == 0) {
        g_pitch_off = g_pitch_off_old;
        g_roll_off  = g_roll_off_old;
    }
    g_zeroing = false;
}

void imu_get_tilt_offsets(float *pitch_off, float *roll_off) {
    if (pitch_off) *pitch_off = g_pitch_off;
    if (roll_off)  *roll_off  = g_roll_off;
}

bool imu_read(imu_data_t *out) {
    vec3 acc, mag;
    if (!read_sensors(&acc, &mag)) return false;

    // Calibration steps run by imu_task (see imu.h)
    if (g_compass_cal) update_minmax(mag);
    if (g_zeroing) {
        g_zero_sum.x += acc.x;
        g_zero_sum.y += acc.y;
        g_zero_sum.z += acc.z;
        g_zero_n++;
    }

    *out = compute(acc, mag);
    out->hump_detected = hump_update(out->vert_acc);
    lift_update(dot(acc, g_up), time_us_64());
    fill_hump_fields(out);
    fill_safety_fields(out);
    return true;
}

void imu_reset_lift(void) {
    g_lift_state = LIFT_IDLE;  // drop a move in progress without recording it
    lift_idle();
    g_lift_vel = 0.0f;
    g_lifted   = false;
}

void imu_reset_hump(void) {
    hump_reset_state();
}

float imu_get_hump_max(void) {
    return g_hump_max;
}

float imu_get_g_ref(void) {
    return g_g_ref;
}

const char *imu_compass_point(float deg) {
    static const char *names[] = {"N","NE","E","SE","S","SW","W","NW"};
    return names[(int)((deg + 22.5f) / 45.0f) % 8];
}
