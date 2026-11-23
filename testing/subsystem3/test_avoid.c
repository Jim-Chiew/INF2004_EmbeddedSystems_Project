/* test_avoid.c -- PC test for the going-round-an-object arithmetic (S3-A01..).
 * No car: avoid.c is plain C, so the plan can be checked with a ruler and a
 * calculator instead of a floor and a box.
 *
 * Build and run from the repo root:
 *   sh testing/subsystem3/run_pc_tests.sh
 * or by hand:
 *   gcc -std=c11 -Wall -Wextra -I app_program/subsystem3 \
 *       testing/subsystem3/test_avoid.c app_program/subsystem3/avoid.c -lm -o /tmp/t && /tmp/t
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "avoid.h"

/* PI is not standard C, and -std=c11 hides it. */
#define PI 3.14159265358979323846

static int g_fail = 0;
#define CHECK(id, cond, desc) do { if (cond) printf("[TEST] %s PASS  %s\n", id, desc); \
    else { printf("[TEST] %s FAIL  %s (line %d)\n", id, desc, __LINE__); g_fail++; } } while (0)

int main(void)
{
    avoid_plan_t p;

    /* ---- a believable measurement is used as it is ---- */
    avoid_plan(true, 150, AVOID_LEFT, &p);
    CHECK("S3-A01", !p.assumed && p.width_mm == 150,
          "a 150 mm measurement is believed");
    CHECK("S3-A02", p.lateral_mm == 150 / 2 + AVOID_CAR_HALF_MM + AVOID_CLEARANCE_MM,
          "the sidestep is half the object, half the car, plus clearance");
    CHECK("S3-A03", p.side == AVOID_LEFT, "it goes round the side the scan chose");

    /* ---- no scan at all: the brief says take it as 20 cm ---- */
    avoid_plan(false, 0, AVOID_RIGHT, &p);
    CHECK("S3-A04", p.assumed && p.width_mm == AVOID_ASSUMED_WIDTH_MM,
          "nothing measured: 200 mm assumed");
    CHECK("S3-A05", p.side == AVOID_RIGHT, "the chosen side still decides the direction");

    /* ---- measurements that cannot be true are not believed ----
     * 0 mm is what the scan reports for an object narrower than one angular
     * step, and a metre is a wall. Both must fall back on the assumption
     * rather than steer the car by a silly number. */
    avoid_plan(true, 0, AVOID_LEFT, &p);
    CHECK("S3-A06", p.assumed && p.width_mm == AVOID_ASSUMED_WIDTH_MM,
          "a 0 mm 'measurement' falls back on the assumption");
    avoid_plan(true, 1000, AVOID_LEFT, &p);
    CHECK("S3-A07", p.assumed && p.width_mm == AVOID_ASSUMED_WIDTH_MM,
          "a 1000 mm 'object' is not believed either");
    avoid_plan(true, AVOID_WIDTH_MIN_MM, AVOID_LEFT, &p);
    CHECK("S3-A08", !p.assumed, "the smallest believable width is believed");
    avoid_plan(true, AVOID_WIDTH_MAX_MM, AVOID_LEFT, &p);
    CHECK("S3-A09", !p.assumed, "and so is the largest");

    /* ---- the assumed case must leave room for a 20 cm object ---- */
    avoid_plan(false, 0, AVOID_LEFT, &p);
    CHECK("S3-A10", p.lateral_mm >= 100 + AVOID_CAR_HALF_MM,
          "stepping aside clears a 200 mm object and the car's own width");

    /* ---- the turn: arc = degrees/360 * pi * track ---- */
    {
        double want90 = 0.25 * PI * AVOID_TRACK_MM;
        double want180 = 0.5 * PI * AVOID_TRACK_MM;
        int32_t got90 = avoid_turn_arc_mm(90);

        CHECK("S3-A11", fabs(got90 - want90) <= 1.0,
              "a 90 degree turn rolls each wheel a quarter of the track circle");
        CHECK("S3-A12", fabs(avoid_turn_arc_mm(180) - want180) <= 1.0,
              "180 degrees is twice as far");
        CHECK("S3-A13", avoid_turn_arc_mm(-90) == got90,
              "turning the other way is the same distance");
        CHECK("S3-A14", avoid_turn_arc_mm(0) == 0, "no turn, no distance");
        printf("[TEST] S3-A11 info: 90 deg = %d mm of wheel with a %d mm track\n",
               (int)got90, AVOID_TRACK_MM);
    }

    /* ---- the plan carries the same arc the steps will use ---- */
    avoid_plan(true, 150, AVOID_LEFT, &p);
    CHECK("S3-A15", p.turn_arc_mm == avoid_turn_arc_mm(90),
          "the plan's turn matches avoid_turn_arc_mm(90)");

    /* ---- every step has a name, and they are in the right order ---- */
    CHECK("S3-A16", strcmp(avoid_step_name(AVOID_TURN_OUT), "turn away from it") == 0 &&
          strcmp(avoid_step_name(AVOID_PASS), "drive past it") == 0 &&
          strcmp(avoid_step_name(AVOID_FINISHED), "finished") == 0,
          "the steps are named for the console");
    CHECK("S3-A17", AVOID_TURN_OUT == 0 && AVOID_FINISHED == 9,
          "nine steps, then finished");

    /* ---- the two looks, and the order they come in ----
     * Each check must follow the turn that points the car at what it is about
     * to look at: a check taken while the car is still sideways tells you
     * about the floor, not the object. */
    CHECK("S3-A29", AVOID_CHECK_AHEAD == AVOID_TURN_ALONG + 1 &&
          AVOID_CHECK_PAST == AVOID_TURN_IN + 1,
          "each look comes straight after the turn that aims the car");
    CHECK("S3-A30", AVOID_RETURN > AVOID_CHECK_PAST &&
          AVOID_TURN_AHEAD > AVOID_RETURN,
          "the car only goes back to the line once the look says it is past");
    {
        int i, named = 0;
        for (i = 0; i <= (int)AVOID_FINISHED; i++) {
            if (strcmp(avoid_step_name((avoid_step_t)i), "finished") != 0) {
                named++;
            }
        }
        CHECK("S3-A31", named == 9, "all nine steps have their own name");
    }

    /* ---- the clearance check has to tell a cleared box from a far wall ---- */
    CHECK("S3-A32", AVOID_CHECK_CLEAR_MM > 150 && AVOID_CHECK_CLEAR_MM < 1000,
          "the clear threshold sits above the stopping distance and below a room");
    CHECK("S3-A33", AVOID_EXTRA_STEP_MM > 0 && AVOID_EXTRA_ALONG_MM >= 270 &&
          AVOID_MAX_TRIES >= 2,
          "a failed check has somewhere to go, more than once");
    printf("[TEST] S3-A33 info: up to %d mm of extra sidestep, %d mm of extra "
           "travel\n", AVOID_MAX_TRIES * AVOID_EXTRA_STEP_MM,
           AVOID_MAX_TRIES * AVOID_EXTRA_ALONG_MM);

    /* ---- the first pass must clear the object it assumed ---- */
    CHECK("S3-A34", AVOID_PASS_MM >= 2 * AVOID_ASSUMED_WIDTH_MM &&
          AVOID_PASS_MM >= 750,
          "the first drive along covers an object as deep as the assumed width");
    printf("[TEST] S3-A34 info: first pass %d mm, then up to %d mm more in %d "
           "tries -- %d mm of reach in all\n", AVOID_PASS_MM,
           AVOID_MAX_TRIES * AVOID_EXTRA_ALONG_MM, AVOID_MAX_TRIES,
           AVOID_PASS_MM + AVOID_MAX_TRIES * AVOID_EXTRA_ALONG_MM);

    /* ---- the slip allowance ----
     * A spin on the spot drags both tyres sideways, so the wheels roll
     * further than the car turns and the encoders finish early.  The
     * allowance is added to the ARC asked for, never to the angle the turn is
     * judged by, or the track learning would chase its own correction. */
    CHECK("S3-A35", AVOID_SPIN_EXTRA_DEG > 0 && AVOID_SPIN_EXTRA_DEG < 90,
          "the slip allowance is a real correction, not a second turn");
    CHECK("S3-A36", avoid_turn_arc_track(90 + AVOID_SPIN_EXTRA_DEG, AVOID_TRACK_MM) >
          avoid_turn_arc_track(90, AVOID_TRACK_MM),
          "asking for the allowance rolls the wheels further");
    {
        /* Judged by the real angle, a turn that lands on 90 teaches nothing,
         * whatever arc was asked for.  This is the property that keeps the
         * allowance and the learning from fighting each other. */
        int32_t t = AVOID_TRACK_MM;
        int i;
        for (i = 0; i < 20; i++) {
            t = avoid_track_update(t, 90);
        }
        CHECK("S3-A37", t == AVOID_TRACK_MM,
              "a turn that lands on 90 leaves the track alone, 20 times over");
        printf("[TEST] S3-A36 info: 90 deg asks for %d mm of wheel, 90+%d asks "
               "for %d mm\n", (int)avoid_turn_arc_track(90, AVOID_TRACK_MM),
               AVOID_SPIN_EXTRA_DEG,
               (int)avoid_turn_arc_track(90 + AVOID_SPIN_EXTRA_DEG, AVOID_TRACK_MM));
    }

    /* ---- room added 5 cm at a time, after watching it go round ---- */
    avoid_plan(false, 0, AVOID_LEFT, &p);
    CHECK("S3-A18", p.lateral_mm == 100 + AVOID_CAR_HALF_MM + AVOID_CLEARANCE_MM &&
          AVOID_CLEARANCE_MM >= 170,
          "the first sidestep carries the clearance, now 170 mm");
    printf("[TEST] S3-A18 info: a 200 mm object means stepping %d mm aside\n",
           (int)p.lateral_mm);

    /* ---- encoders and compass together: learning the track width ----
     * The arc comes from a track width.  If the car actually turned LESS than
     * 90, the real track is bigger than we thought and the next turn has to
     * roll the wheels further. */
    {
        int32_t t0 = 130;
        int32_t up = avoid_track_update(t0, 83);    /* turned short */
        int32_t dn = avoid_track_update(t0, 97);    /* overshot */

        CHECK("S3-A19", up > t0, "turning short makes the track estimate bigger");
        CHECK("S3-A20", dn < t0, "overshooting makes it smaller");
        CHECK("S3-A21", avoid_track_update(t0, 90) == t0,
              "an exact 90 leaves the estimate alone");
        printf("[TEST] S3-A19 info: 130 mm + an 83 deg turn -> %d mm, "
               "+ a 97 deg turn -> %d mm\n", (int)up, (int)dn);

        CHECK("S3-A22", up < 130 * 90 / 83,
              "it moves only part of the way towards what one turn suggests");

        /* Honest readings must converge on the truth.  A car whose real track
         * is 150 mm turns 90*130/150 = 78 deg on an arc meant for 130 mm. */
        {
            int32_t t = 130;
            int i;
            for (i = 0; i < 12; i++) {
                int went = (int)(90 * t / 150);
                t = avoid_track_update(t, went);
            }
            CHECK("S3-A23", t >= 145 && t <= 155,
                  "a dozen turns converge on the real 150 mm track");
            printf("[TEST] S3-A23 info: settled at %d mm (truth 150)\n", (int)t);
        }
    }

    /* ---- a disturbed compass must be ignored, not obeyed ---- */
    CHECK("S3-A24", avoid_track_update(130, 20) == 130 &&
          avoid_track_update(130, 170) == 130 &&
          avoid_track_update(130, 0) == 130,
          "readings outside 60..120 deg leave the estimate untouched");
    CHECK("S3-A25", avoid_track_update(130, -83) == avoid_track_update(130, 83),
          "turning the other way teaches the same thing");

    /* ---- the estimate can never run away ---- */
    {
        int32_t t = AVOID_TRACK_MAX_MM;
        int i;
        for (i = 0; i < 50; i++) {
            t = avoid_track_update(t, AVOID_TURN_TRUST_MIN_DEG);  /* always short */
        }
        CHECK("S3-A26", t <= AVOID_TRACK_MAX_MM,
              "even 50 worst-case turns cannot push it past the bound");
        printf("[TEST] S3-A26 info: 50 short turns from the top -> %d mm "
               "(bound %d)\n", (int)t, AVOID_TRACK_MAX_MM);
    }

    /* ---- the arc scales with whatever track it is given ---- */
    CHECK("S3-A27", avoid_turn_arc_track(90, 260) == 2 * avoid_turn_arc_track(90, 130),
          "twice the track, twice the wheel distance");
    CHECK("S3-A28", avoid_turn_arc_track(90, AVOID_TRACK_MM) == avoid_turn_arc_mm(90),
          "the plain version uses the track width from avoid.h");

    printf(g_fail ? "SOME FAILED (%d)\n" : "ALL PASS\n", g_fail);
    return g_fail ? 1 : 0;
}
