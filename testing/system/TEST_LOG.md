# System — test log

Tests of the complete product against its requirements, run on the final hardware set-up. Run the full set before every demo and before hand-in.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | full demo run: calibrate, follow the course (curves, junctions, humps, barcode), lift to end | all | manual | 1. GP21 on flat floor. 2. GP20 at the start line. 3. Lift the car at the end. | completes the course; barcode read; humps counted; stops on the lift |
| SYS-02 | lifting the car ends the run: motors stop within 0.6 s + one IMU sample | lift requirement | manual | lift the car briskly ≥ 15 cm while it follows the line | motors stop; `SAFETY STOP: lifted off the floor` |
| SYS-03 | tilting the car past 90° either way stops the motors | overturn requirement | manual | tip the car onto each side while running | motors stop at once; `SAFETY STOP: overturned` |
| SYS-04 | telemetry keeps flowing for 10 minutes; nothing dropped with the broker up | FR-COM-07, FR-COM-09 | manual | `mosquitto_sub -t "robocar/#" -v` for 10 min | 5 frames/s, status line `drop 0` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | SYS-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
