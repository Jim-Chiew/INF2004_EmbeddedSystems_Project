// imu.h - LSM303D (GY-511) driver (subsystem 4, micro T-Kernel build)
// Provides acceleration, full +/-180 pitch/roll, tilt-compensated heading,
// and hump (vertical jolt) detection.
//
// Plain C: no kernel and no Pico SDK headers. It talks to the sensor through
// drv_i2c (I2C1, GP2/GP3) and is called ONLY by imu_task, which owns I2C1
// (TEAM.md section 3). Other subsystems read the results through
// subsystem4.h (imu_get_status), never through these functions.
//
// Sign convention (assuming +X forward, +Y left, +Z up when flat):
//   pitch: +X end up = positive, down = negative, range +/-180
//   roll : right side down = positive, left side down = negative, range +/-180
//
// Hump detection:
//   The "up" direction and the 1 g reference are taken from the gravity vector
//   averaged by imu_zero_begin/end(). Each imu_read() projects the acceleration onto
//   that axis and subtracts gravity, giving vert_acc (g, + = upward).
//   A hump starts when |vert_acc| rises above IMU_HUMP_THRESHOLD_G and ends when
//   it falls below IMU_HUMP_RELEASE_G. The largest |vert_acc| during that window
//   is the hump's value. Call imu_read() often (e.g. every 10 ms) so short
//   jolts are not missed.

#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>

// ---- Hardware: I2C1, SDA GP2 / SCL GP3 (pins in s4_pins.h) ----
#ifndef IMU_FROZEN_READS
#define IMU_FROZEN_READS 10   // this many identical readings in a row = sensor stopped
#endif

// ---- Hump detection tuning (override with -D or before including) ----
#ifndef IMU_HUMP_THRESHOLD_G
#define IMU_HUMP_THRESHOLD_G  0.25f   // |vertical accel| that starts a hump (g)
#endif
#ifndef IMU_HUMP_RELEASE_G
#define IMU_HUMP_RELEASE_G    0.10f   // |vertical accel| that ends a hump (g)
#endif

// ---- Lift / overturn detection tuning (override with -D or before including) ----
// Overturned: the car's "up" axis (set by imu_zero_end; board +Z before that)
//   is more than IMU_TILT_LIMIT_DEG from vertical in any direction - pitch or
//   roll, either way - for IMU_TILT_CONFIRM_MS.
// Lifted: an accelerometer reads 1 g on the floor AND when held still in the
//   air, so a lift is recognised by the move itself. All of these must happen:
//   an upward push above IMU_LIFT_TRIGGER_G, a top speed of at least
//   IMU_LIFT_MIN_SPEED, braking to a stop within IMU_LIFT_RISE_MAX_MS after a
//   rise of at least IMU_LIFT_HEIGHT_CM, and no lowering (faster than
//   IMU_LIFT_DOWN_SPEED) until IMU_LIFT_CONFIRM_MS after lift-off. Driving
//   vibration and bumps are jolts with no real rise of that size, so they
//   don't pass. Lift the car briskly, 15 cm or more. "lifted" stays set until
//   imu_reset_lift().
#ifndef IMU_TILT_LIMIT_DEG
#define IMU_TILT_LIMIT_DEG    90.0f  // degrees from upright, any direction
#endif
#ifndef IMU_TILT_CONFIRM_MS
#define IMU_TILT_CONFIRM_MS   50     // ms past the limit before it counts
#endif
#ifndef IMU_LIFT_TRIGGER_G
#define IMU_LIFT_TRIGGER_G    0.12f  // filtered upward accel that may start a lift (g)
#endif
#ifndef IMU_LIFT_MIN_SPEED
#define IMU_LIFT_MIN_SPEED    0.35f  // a lift reaches at least this upward speed (m/s)
#endif
#ifndef IMU_LIFT_HEIGHT_CM
#define IMU_LIFT_HEIGHT_CM    8.0f   // ...and rises at least this much (cm)
#endif
#ifndef IMU_LIFT_RISE_MAX_MS
#define IMU_LIFT_RISE_MAX_MS  800    // ...and stops at the top within this time (ms)
#endif
#ifndef IMU_LIFT_DOWN_SPEED
#define IMU_LIFT_DOWN_SPEED   0.30f  // lowered faster than this = put back down (m/s)
#endif
#ifndef IMU_LIFT_CONFIRM_MS
#define IMU_LIFT_CONFIRM_MS   600    // must stay lifted this long after lift-off (ms)
#endif

// The accelerometer's anti-alias filter: 0 = 773 Hz (as before), 1 = 50 Hz.
// Turn on if the status line shows "vib" above ~0.06 g while driving: motor
// vibration is then folding into the readings. Also smooths hump jolts.
#ifndef IMU_ACC_AA_50HZ
#define IMU_ACC_AA_50HZ       0
#endif

typedef struct { float x, y, z; } vec3;

typedef struct {
    vec3     acc;            // acceleration in g
    float    pitch;          // degrees, +/-180 (zero offset applied)
    float    roll;           // degrees, +/-180 (zero offset applied)
    float    heading;        // degrees, 0-360, direction of the board's +X axis

    // Hump detection
    float    vert_acc;       // vertical acceleration with gravity removed (g, + = up)
    bool     hump_detected;  // true only on the sample where a hump has just ended
    float    hump_last;      // peak |vert_acc| of the most recent hump (g)
    float    hump_max;       // highest hump peak since init / last reset (g)
    uint32_t hump_count;     // number of humps detected since init / last reset

    // Lift / overturn detection
    bool     overturned;      // tilted past IMU_TILT_LIMIT_DEG (live, clears when upright)
    bool     lifted;          // held off the floor (latched until imu_reset_lift())
    uint8_t  lift_phase;      // 0 = on the floor, 1 = moving up, 2 = up, waiting out the confirm time
    float    lift_height_cm;  // estimated rise of the move in progress, else 0
    float    lift_last_cm;    // rise of the last upward move (for tuning)
    float    lift_last_speed; // top speed of the last upward move, m/s (for tuning)
    float    vib_g;           // RMS vertical vibration while on the floor (g)

    // Hump HEIGHT: one wheel up a hump tilts the car (hump_height.h).
    // NOT filled by imu_read(): imu_task works these out, because only it
    // knows whether the car is turning (and the tilt is then meaningless).
    float    hh_height_mm;    // height of the most recent hump measured (mm)
    float    hh_tilt_deg;     // its peak tilt (degrees)
    char     hh_side;         // 'R' / 'L' wheel, 'C' = car pitched, not one wheel
    uint32_t hh_count;        // humps measured since power-up (never reset)
    float    hh_tilt_now;     // tilt from level right now (degrees)
} imu_data_t;

// Initialises I2C and the sensor. Returns true on success.
// who_am_i (optional, may be NULL) receives the value read from the WHO_AM_I register.
bool imu_init(uint8_t *who_am_i);

// Writes the sensor's settings again WITHOUT touching the calibration.
// Use it when the readings stop changing: after a power dip the LSM303D
// resets to power-down and keeps returning its last values.
// Returns true if the sensor answered.
bool imu_reconfigure(uint8_t *who_am_i);

// ---- Calibration, in steps (imu_task runs these as states; nothing blocks) ----
// Compass (hard-iron): between begin and end, every imu_read() widens the
// magnetometer min/max. Spin the car meanwhile. imu_compass_cal_abort()
// puts the previous calibration back.
void imu_compass_cal_begin(void);
void imu_compass_cal_end(void);
void imu_compass_cal_abort(void);

// Tilt zero: between begin and end, every imu_read() adds its gravity vector
// to an average (the car must be still and level). begin clears the zero
// offsets, so the readings in between are raw angles. end stores the average
// as the pitch/roll zero; the same vector becomes the "up" axis and 1 g
// reference for hump, lift and tilt detection, and those are reset.
// Returns the number of samples averaged (0 = nothing stored, the old zero
// is kept).
void imu_zero_begin(void);
int  imu_zero_end(void);
void imu_zero_abort(void);

// Returns the current tilt zero offsets in degrees (either pointer may be NULL).
void imu_get_tilt_offsets(float *pitch_off, float *roll_off);

// Reads the sensor, runs hump detection and fills *out.
// Returns false on I2C failure.
bool imu_read(imu_data_t *out);

// Clears hump count, last and highest hump values.
void imu_reset_hump(void);

// Returns the highest hump value (g) detected since init / last reset.
float imu_get_hump_max(void);

// The 1 g the sensor reads at rest (from the tilt zero; 1.0 before one).
float imu_get_g_ref(void);

// Clears the "lifted" latch and any lift being tracked (call at the start of a run).
void imu_reset_lift(void);

// Converts a heading in degrees to "N", "NE", "E", ...
const char *imu_compass_point(float deg);

#endif // IMU_H
