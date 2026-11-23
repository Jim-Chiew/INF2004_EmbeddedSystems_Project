# Subsystem 6 — test log

**Subsystem:** Obstacle (HC-SR04 on GP0/GP1, SG90 servo on GP15)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem6/`

See `testing/README.md` for how to run tests and record results.
The road ahead of the car must be clear when the board is powered up, and the
sensor must point forward. Build with a plain `make TEST=1`: the sensor shares
GP0/GP1 with UART0, so `CRASH_REPORT=1` and `CONSOLE=uart` do not compile.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S6-U01 | the median of 3 rejects a single spike | — | auto | `subsystem6_test.c` | `[TEST] S6-U01 PASS` |
| S6-U02 | three invalid samples give US_INVALID, never "clear" | — | auto | `subsystem6_test.c` | `[TEST] S6-U02 PASS` |
| S6-U03 | no-echo counts as clear | — | auto | `subsystem6_test.c` | `[TEST] S6-U03 PASS` |
| S6-U04 | an invalid sample is dropped, the lower middle value used | — | auto | `subsystem6_test.c` | `[TEST] S6-U04 PASS` |
| S6-U05 | servo pulse: −90 = min, 0 = centre, +90 = max | — | auto | `subsystem6_test.c` | `[TEST] S6-U05 PASS` |
| S6-U06 | servo angle is clamped to ±90 | — | auto | `subsystem6_test.c` | `[TEST] S6-U06 PASS` |
| S6-U07 | more room on the right picks RIGHT | — | auto | `subsystem6_test.c` | `[TEST] S6-U07 PASS` |
| S6-U08 | more room on the left picks LEFT | — | auto | `subsystem6_test.c` | `[TEST] S6-U08 PASS` |
| S6-U09 | a tie within 3 cm picks LEFT | — | auto | `subsystem6_test.c` | `[TEST] S6-U09 PASS` |
| S6-U10 | 9 invalid pings do not raise a fault yet | — | auto | `subsystem6_test.c` | `[TEST] S6-U10 PASS` |
| S6-U11 | the 10th invalid ping in a row raises SENSOR_FAULT | — | auto | `subsystem6_test.c` | `[TEST] S6-U11 PASS` |
| S6-U12 | a valid ping resets the invalid counter | — | auto | `subsystem6_test.c` | `[TEST] S6-U12 PASS` |
| S6-U13 | open-space NO_ECHO is valid and resets the counter | — | auto | `subsystem6_test.c` | `[TEST] S6-U13 PASS` |
| S6-U14 | the coarse angles are 5, ascending, within ±90 | — | auto | `subsystem6_test.c` | `[TEST] S6-U14 PASS` |
| S6-U15 | all angles clear: nothing blocked, no closest | — | auto | `subsystem6_test.c` | `[TEST] S6-U15 PASS` |
| S6-U16 | the blocked mask and the closest angle are right | — | auto | `subsystem6_test.c` | `[TEST] S6-U16 PASS` |
| S6-U17 | an invalid angle is flagged, never clear or blocked | — | auto | `subsystem6_test.c` | `[TEST] S6-U17 PASS` |
| S6-U18 | the scan range limit is inclusive (≤) | — | auto | `subsystem6_test.c` | `[TEST] S6-U18 PASS` |
| S6-U19 | settle + ping time fits the 800 ms scan budget | — | auto | `subsystem6_test.c` | `[TEST] S6-U19 PASS` |
| S6-U23 | fine-sweep settle + ping time fits its budget | — | auto | `subsystem6_test.c` | `[TEST] S6-U23 PASS` |
| S6-U24 | a board 200 mm ahead over -10..+10° measures about 70 mm | — | auto | `subsystem6_test.c` | `[TEST] S6-U24 PASS`, info line shows ~71 mm |
| S6-U25 | an empty window measures nothing (not a zero-length object) | — | auto | `subsystem6_test.c` | `[TEST] S6-U25 PASS` |
| S6-U26 | a single hit is found but too narrow to measure | — | auto | `subsystem6_test.c` | `[TEST] S6-U26 PASS` |
| S6-U27 | the step out to a wall behind ends the object | — | auto | `subsystem6_test.c` | `[TEST] S6-U27 PASS` |
| S6-U28 | an object off to one side is measured at its own angles | — | auto | `subsystem6_test.c` | `[TEST] S6-U28 PASS` |
| S6-M07 | a measured length matches a tape measure to about ±20 % | the object scan | manual | stop the car with a box of known width (say 150 mm) 20 cm ahead; read `object : NNN mm long` | within roughly 30 mm of the real width; expect it to read WIDE (the beam), which is what `US_BEAM_DEG` is for |
| S6-M09 | the measurement reaches MQTT once per scan, and survives a broker outage | IF-10 | manual | stop at a box with the broker off, then start the broker | the message goes out after the reconnect (it waits in the ring), still with the right numbers |
| S6-M08 | two objects side by side are not merged into one | the object scan | manual | put two boxes 30 cm apart, both in the window | the report measures only the nearer one; its span stops at the gap |
| S6-U20 | obstacle_task really pings, at about 50 Hz | IF-13 | auto | `subsystem6_test.c` | `[TEST] S6-U20 PASS`, and the info line shows ≈50 pings/s |
| S6-U21 | obstacle_task sleeps between pings | — | auto | `subsystem6_test.c` | `[TEST] S6-U21 PASS` |
| S6-U22 | the sensor answers (wiring check) | IF-13 | auto | `subsystem6_test.c` | `[TEST] S6-U22 PASS`; a FAIL means TRIG/ECHO/power — read the `obstacle: pings ok=… no-echo=…` line |
| S6-M01 | measured distance is right to ±1 cm | the 15 cm stop | manual | put a flat board at 10, 20, 30, 50 cm from the sensor; read the status line (`us …mm`) or set `S6_TRACE 1` | each reading within ±10 mm of a tape measure |
| S6-M02 | the car stops before an obstacle while following the line | IF-11 | manual | start a run (GP20) and put a box across the line in front of it | the car stops, `nav: OBSTACLE … mm ahead` and `AVOIDING` on the status line; it does not touch the box |
| S6-M03 | GP20 carries the run on, STOP ends it | IF-11, IF-12 | manual | from S6-M02: take the box away and press GP20; repeat and send STOP instead | GP20: `nav: carrying on`, the car follows the line again. STOP: the run ends, back to IDLE |
| S6-M04 | an unplugged sensor is a fault, never an empty road | — | manual | while running, pull the ECHO wire out | `obstacle: no answer from the sensor`, then `sensor dead after 3 tries`; status line `us DEAD`; the car does **not** keep driving as if the road were clear |
| S6-M05 | the servo scan runs inside its time budget | — | manual | from S6-M02, read the scan report block | `scan time:` under 800 ms, the servo sweeps right → left and recentres |
| S6-M06 | the distance reaches the broker | IF-13, IF-10 | manual | `python tools/robocar.py watch` with something 30 cm ahead | the frame's distance field shows ≈30 cm (63 = nothing in range) |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S6-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
