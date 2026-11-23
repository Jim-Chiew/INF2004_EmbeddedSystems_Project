# Subsystem 2 — test log

**Subsystem:** Motion (motors, encoders, speed sync, emergency stop)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.
Put the car on a stand, wheels off the floor, for the automatic tests.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | `mbf_motion` and `mtx_odom` exist | IF-01, IF-03 | auto | `subsystem2_test.c` | `[TEST] S2-U01 PASS` |
| S2-U02 | encoders start on the PIO block the radio left free (within 10 s) | — | auto | `subsystem2_test.c` | `[TEST] S2-U02 PASS`, console shows `motion: encoders running on PIOn` |
| S2-U03 | odometry copy refreshed every 5 ms | IF-03 | auto | `subsystem2_test.c` | `[TEST] S2-U03 PASS` |
| S2-U04 | ESTOP latches: a DRIVE after it leaves the motors braked | IF-02, safety stop | auto | `subsystem2_test.c` | `[TEST] S2-U04 PASS`, wheels do not move |
| S2-U05 | ENABLE leaves ESTOP (back to STOPPED) | IF-01 | auto | `subsystem2_test.c` | `[TEST] S2-U05 PASS` |
| S2-U06 | both wheels turn forward at the same speed in DRIVE | — | manual | normal image, START on a straight line; watch, and the status line `spd` | car drives straight, both wheels forward |
| S2-U07 | each encoder counts up when its wheel turns forward | — | manual | turn each wheel by hand forward, watch the odometry (status line / debugger) | counts rise; if a count falls, swap that encoder's A/B in `s2_pins.h` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S2-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
