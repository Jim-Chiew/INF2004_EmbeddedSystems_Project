# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | S3 drives through S2: straight, steer, spin follow the line | IF-01, IF-03 | manual | normal image, GP20 on the track | car follows the line; status line `FOLLOW steer-L/R`, motion `DRIVE`/`WHEELS` |
| INT-02 | S4 stops the motors (ESTOP) while S3 keeps following | IF-02, IF-05 | auto | `subsystem2_test.c` S2-U04 covers the latch; plus S4-U06/S4-U07 manual | motors stay braked; S3 ends the run (`SAFETY STOP`) |
| INT-03 | GP21 calibration: S3 spins the car through S2 while S4 records, then waits for S4 | IF-01, IF-04, IF-05 | manual | press GP21 | spin stops after 8 s; `imu: calibrated`; status `IDLE` (no "(no cal)") |
| INT-04 | broker START/STOP/RESET reach S3 through S1 within 200 ms | IF-07, IF-10, FR-COM-08 | manual | `mosquitto_pub -t robocar/car1/cmd -f start.bin` (stop.bin, reset.bin) | `COMMAND RECEIVED: START`, car starts; STOP stops; RESET clears `jn` |
| INT-05 | barcode text from S5 reaches the broker through S1, bc_dir in the frame | IF-09, IF-10 | manual | read `*L*`; `mosquitto_sub -t "robocar/#" -v` | `robocar/car1/barcode L`; decoded frame shows bc_dir LEFT |
| INT-06 | telemetry frame carries S2 speed, S3 line/state, S4 tilt/roll/heading/hump | IF-03, IF-06, IF-08, IF-10 | manual | `python tools/robocar.py watch` while driving over a hump | values match the status line; hump = 1 once per hump |
| INT-07 | S6 stops the car through S3 and S2 before it hits an obstacle | IF-11, IF-01 | manual | start a run and put a box across the line ahead of the car | car stops at about 15 cm, `nav: OBSTACLE … mm ahead`, status `AVOIDING`, motion `STOPPED` |
| INT-08 | GP20 clears S6's latch and the run carries on; the latch holds while the box is there | IF-11, IF-12 | manual | from INT-07 press GP20 with the box still there, then take it away and press again | first press: `the object is still there`, car stays stopped; second: `road clear, following the line again` |
| INT-09 | S3 asks S6 for a servo scan and gets the result | IF-12, IF-13 | manual | from INT-07, read the `OBSTACLE SCAN` block on the console | five angles with distances, a closest angle, left/right clearance, `scan time:` under 800 ms |
| INT-10 | the telemetry frame carries S6's distance | IF-13, IF-10 | manual | `python tools/robocar.py watch` with something ~30 cm ahead, then nothing | distance ≈30 cm, then the "no echo" code (63) |
| INT-11 | each obstacle measurement reaches the broker once | IF-13, IF-10 | manual | `python tools/robocar.py watch` (or `mosquitto_sub -t robocar/car1/obstacle -v`) while the car stops at a box | one `object scanned: NNN mm long, ...` line per scan, matching the `[s6]` block on the console; none while the car is just driving |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
