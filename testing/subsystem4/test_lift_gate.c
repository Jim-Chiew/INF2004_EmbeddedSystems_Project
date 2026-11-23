/* test_lift_gate.c -- PC test for the lift detector's thresholds (S4-L01..).
 *
 * These are the numbers that decide whether the car e-stops in the middle of a
 * run, so they are checked here rather than by picking the car up and hoping.
 * The cases marked LOGGED are real readings from the car's console.
 *
 * Build and run from the repo root:
 *   sh testing/subsystem4/run_pc_tests.sh
 * or by hand:
 *   gcc -std=c11 -Wall -Wextra -I app_program/subsystem4 \
 *       testing/subsystem4/test_lift_gate.c app_program/subsystem4/lift_gate.c \
 *       -lm -o /tmp/t && /tmp/t
 */

#include <stdio.h>

#include "lift_gate.h"

/* The fixed thresholds from imu.h, which these gates fall back on.  Copied
 * rather than included because imu.h is the sensor's interface and this test
 * is about the arithmetic; S4-L13 pins them so a change there is noticed. */
#define FIX_TRIGGER	0.12f	/* IMU_LIFT_TRIGGER_G */
#define FIX_SPEED	0.35f	/* IMU_LIFT_MIN_SPEED */
#define FIX_COAST	0.05f	/* LIFT_COAST_G */
#define FIX_HEIGHT_M	0.08f	/* IMU_LIFT_HEIGHT_CM / 100 */

/* What the car measures, from the console */
#define VIB_STILL	0.025f	/* standing still, motors off */
#define VIB_DRIVING	0.254f	/* driving: spins, starts, stops */

static int g_fail = 0;
#define CHECK(id, cond, desc) do { if (cond) printf("[TEST] %s PASS  %s\n", id, desc); \
    else { printf("[TEST] %s FAIL  %s (line %d)\n", id, desc, __LINE__); g_fail++; } } while (0)

/* Would a rise with these numbers be called a lift? */
static int lifts(float vib, float push_g, float vpeak, float height_m, float rise_s)
{
    if (push_g < lift_gate_trigger_g(vib, FIX_TRIGGER)) {
        return 0;                               /* never even followed */
    }
    if (vpeak < lift_gate_min_speed(vib, rise_s, FIX_SPEED)) {
        return 0;                               /* too slow to be a hand */
    }
    return height_m >= FIX_HEIGHT_M;
}

int main(void)
{
    /* ---- a quiet car behaves exactly as it did before the fix ---- */
    CHECK("S4-L01", lift_gate_trigger_g(VIB_STILL, FIX_TRIGGER) == FIX_TRIGGER,
          "standing still, the start gate is the plain 0.12 g");
    CHECK("S4-L02", lift_gate_min_speed(VIB_STILL, 0.30f, FIX_SPEED) == FIX_SPEED,
          "standing still, a 0.3 s rise needs the plain 0.35 m/s");
    CHECK("S4-L03", lift_gate_coast_g(VIB_STILL, FIX_COAST) == FIX_COAST,
          "standing still, the coast test is the plain 0.05 g");
    printf("[TEST] S4-L02 info: at vib %.3f g the drift bound over 0.3 s is "
           "only %.3f m/s\n", VIB_STILL, VIB_STILL * LIFT_GATE_G * 0.30f);

    /* ---- and a real hand lift off a standing car is still caught ---- */
    CHECK("S4-L04", lifts(VIB_STILL, 0.40f, 0.60f, 0.12f, 0.30f),
          "a hand lifting a standing car 12 cm is still a lift");

    /* ---- THE BUG.  LOGGED: vib 0.254 g, "last 19.9cm 0.43m/s LIFTED",
     * car flat on the floor throughout.  Rejected now, twice over. ---- */
    CHECK("S4-L05", !lifts(VIB_DRIVING, 0.30f, 0.43f, 0.199f, 0.80f),
          "LOGGED: the 19.9 cm / 0.43 m/s false lift is rejected");
    CHECK("S4-L06", 0.30f < lift_gate_trigger_g(VIB_DRIVING, FIX_TRIGGER),
          "...it would not even start a rise");
    CHECK("S4-L07", 0.43f < lift_gate_min_speed(VIB_DRIVING, 0.80f, FIX_SPEED),
          "...and if it did, 0.43 m/s is under the drift bound");
    printf("[TEST] S4-L06 info: driving at vib %.3f g, the start gate is "
           "%.2f g (was a fixed %.2f g)\n", VIB_DRIVING,
           lift_gate_trigger_g(VIB_DRIVING, FIX_TRIGGER), FIX_TRIGGER);
    printf("[TEST] S4-L07 info: over 0.80 s of shaking, drift alone could "
           "fake %.2f m/s\n", lift_gate_min_speed(VIB_DRIVING, 0.80f, FIX_SPEED));

    /* ---- a short drift episode is rejected too: the bound shrinks with
     * time, but so does the speed noise can build up ---- */
    CHECK("S4-L08", !lifts(VIB_DRIVING, 0.80f, 0.43f, 0.199f, 0.25f),
          "a quarter-second drift at the same speed is rejected as well");

    /* ---- what is given up, stated as a test so it cannot be forgotten:
     * a brisk snatch off a DRIVING car is still caught, a gentle lift is
     * not.  See the note in lift_gate.h. ---- */
    CHECK("S4-L09", lifts(VIB_DRIVING, 1.00f, 0.80f, 0.10f, 0.20f),
          "a brisk snatch off a driving car is still a lift");
    CHECK("S4-L10", !lifts(VIB_DRIVING, 0.20f, 0.40f, 0.10f, 0.50f),
          "a gentle lift off a driving car is NOT caught (the trade-off)");

    /* ---- a bad vibration estimate must never make it easier to fire ---- */
    CHECK("S4-L11", lift_gate_trigger_g(-1.0f, FIX_TRIGGER) == FIX_TRIGGER &&
          lift_gate_trigger_g(0.0f, FIX_TRIGGER) == FIX_TRIGGER,
          "a zero or negative vibration cannot lower the start gate");
    CHECK("S4-L12", lift_gate_min_speed(VIB_DRIVING, -1.0f, FIX_SPEED) == FIX_SPEED &&
          lift_gate_min_speed(VIB_DRIVING, 0.0f, FIX_SPEED) == FIX_SPEED &&
          lift_gate_coast_g(-1.0f, FIX_COAST) == FIX_COAST,
          "nor can a zero or negative rise time, or a bad coast input");
    {
        float v, prev = 0.0f;
        int rising = 1;
        for (v = 0.0f; v < 1.0f; v += 0.01f) {
            float g = lift_gate_trigger_g(v, FIX_TRIGGER);
            if (g < prev) rising = 0;
            prev = g;
        }
        CHECK("S4-L13", rising && prev >= FIX_TRIGGER,
              "the gate only ever rises as the car shakes harder");
    }

    /* ---- the fixed thresholds this test assumes ---- */
    CHECK("S4-L14", LIFT_VIB_TRIGGER >= 2.0f && LIFT_VIB_SPEED > 0.0f,
          "the scale factors are at least a couple of RMS, and non-zero");

    printf(g_fail ? "SOME FAILED (%d)\n" : "ALL PASS\n", g_fail);
    return g_fail ? 1 : 0;
}
