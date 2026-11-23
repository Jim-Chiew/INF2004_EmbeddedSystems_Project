# Subsystem 4 — test log

**Subsystem:** IMU (tilt, heading, humps, lift/overturn safety stop)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.
The car must be upright and still on the bench for the automatic tests.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | `mbf_imu_req`, `flg_imu`, `mtx_imu` exist | IF-04, IF-05, IF-06 | auto | `subsystem4_test.c` | `[TEST] S4-U01 PASS` |
| S4-U02 | LSM303D answers on I2C1 (GP2/GP3), WHO_AM_I 0x49, within 3 s | — | auto | `subsystem4_test.c` | `[TEST] S4-U02 PASS` |
| S4-U03 | readings valid and changing (live sensor) | — | auto | `subsystem4_test.c` | `[TEST] S4-U03 PASS` |
| S4-U04 | upright: not overturned, gravity seen | — | auto | `subsystem4_test.c` | `[TEST] S4-U04 PASS` |
| S4-U05 | RESET_LIFT clears the lift latch | IF-04 | auto | `subsystem4_test.c` | `[TEST] S4-U05 PASS` |
| S4-U06 | tilted past 90° either way (pitch or roll) stops the motors at once | safety stop, IF-02, IF-05 | manual | follow the line, tip the car over | motors stop, `SAFETY STOP: overturned`, `motion: EMERGENCY STOP (from S4)` |
| S4-U07 | lifted for 0.6 s stops the motors; driving and humps do not | safety stop, IF-02, IF-05 | manual | follow the line over humps, then lift the car briskly ≥ 15 cm | no stop while driving; `SAFETY STOP: lifted off the floor` after the lift |
| S4-U08 | GP21 calibration: compass spin, settle, zero; heading and tilt valid after | — | manual | press GP21 on flat floor | `imu: calibrated from N samples`, status line shows heading |
| S4-H01..H18 | hump height arithmetic: formula, either wheel, diagonal pivot, crooked sensor, vibration + jolts, knocks, steering, tipping, held on a block, slopes | `hump_height.c` | auto (PC) | `sh testing/subsystem4/run_pc_tests.sh` | `ALL PASSED` |
| S4-U09 | calibrate `S4_HUMP_LEVER_MM` (hump_height.h) | — | manual | car idle on a flat floor: slide a block of known height (e.g. 10 mm) under the RIGHT wheel, wait 2 s, take it away | console `imu: hump N mm under the RIGHT wheel ...`; set the lever to `130 * real / N`, rebuild, repeat until N is within 1 mm. If it says LEFT, set `S4_HUMP_SIDE_FLIP 1` |
| S4-U10 | hump height on MQTT while following the line | IF-10, `robocar/car1/hump` | manual | a hump of known height under the right wheel's path; GP20 | one `h=N s=R t=...` per hump on `robocar/car1/hump`, N within ~2 mm of the real height; nothing while the car is idle |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S4-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
