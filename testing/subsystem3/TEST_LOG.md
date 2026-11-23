# Subsystem 3 — test log

**Subsystem:** Navigation (line following, buttons, run control)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.
Do not press GP20/GP21 while the automatic tests run.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | `mbf_nav_cmd` and `mtx_nav` exist | IF-07, IF-08 | auto | `subsystem3_test.c` | `[TEST] S3-U01 PASS` |
| S3-U02 | IDLE with motors stopped after power-up | — | auto | `subsystem3_test.c` | `[TEST] S3-U02 PASS` |
| S3-U03 | nav_task sleeps while idle (not busy) | — | auto | `subsystem3_test.c` | `[TEST] S3-U03 PASS` |
| S3-U04 | RESET clears the run (run_count + 1) and stays IDLE | IF-07 | auto | `subsystem3_test.c` | `[TEST] S3-U04 PASS` |
| S3-U05 | IR sensors: L/R on the status line go to 1 over the black line | — | manual | slide each sensor over the line | `IR L1` / `IR R1` over black; else set `S3_IR_LINE_IS_HIGH` |
| S3-U06 | follows a curve and crosses a junction (junction count + 1) | — | manual | normal image, GP20 on the track | stays on the line, `jn` counts up at each crossing |
| S3-U07 | after a correction the car turns slightly further (no jerking back onto the line) | `STEER_EXTRA_MS` | manual | straight line, watch / film | smooth correction, no hit-turn-hit oscillation |
| S3-U08 | GP20 starts and stops; GP21 calibrates (spins 8 s, stops, `imu: calibrated`) | — | manual | press the buttons | as described |
| S3-U10 | GP20 clears every latched error and starts a clean run; a live overturn still stops it | IF-01, IF-04, IF-05 | manual | 1. lift the car while running (SAFETY STOP), put it down, press GP20; 2. unplug the IMU, replug, press GP20; 3. hold the car on its side, press GP20 | 1: `nav: errors cleared`, car runs; 2: `imu: LSM303D found` at once, car runs; 3: motors stay stopped, `SAFETY STOP: overturned` |
| S3-U11 | after going round an obstacle the car picks the original line up again and follows it | `avoid_return_to_line()`, `line_search()` | manual | straight line, box on it, GP20; watch the console from `(8) step back to the line` | `line seen NNN mm in (MMM stepped out) -- on ... to put the axle over it`, then `line found N deg ... -- following it again` with N small (under ~20); the car then follows the line and does not drift off beside it. Same result with and without a GP21 calibration (`S3_AVOID_USE_COMPASS 0`). If the console says `SET S3_SENSOR_TO_AXLE_TRUE_MM TO ABOUT N`, put N in `task_nav.c`, rebuild and repeat: it should then find the line on the first search |
| S3-U13 | barcode rescan backs up straight along the line, both wheels at the same speed | `bc_reverse()`, `MOTION_CMD_DRIVE` backwards | manual | print a code with a weak bar so the first pass fails, on a straight stretch; GP20 | console `backed up NNN mm straight, wheels in step: left A counts, right B` with A and B within a few % of each other, and the line under each sensor a small % of the way; the car is still on the line when it stops and the forward leg follows it |
| S3-U12 | axle-to-sensor distance is right | `S3_SENSOR_TO_AXLE_TRUE_MM` | manual | ruler from the IR sensors back to the centre of the drive wheels | the value in `task_nav.c` matches within ~1 cm |
| S3-U09 | GP20 starts the car with no WiFi: hotspot off (normal image) and with `make NET=none MQTT=0` | — | manual | power on, press GP20 at once and again after 15 s | car starts both times; a press in the first seconds prints `nav: START -- waiting for the motors to be ready...` and then starts |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S3-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
