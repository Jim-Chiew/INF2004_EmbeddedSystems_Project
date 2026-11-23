# Subsystem 5 — test log

**Subsystem:** Barcode (Code 39 on MH-sensor #3, ADC on GP28)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.
The barcode sensor must be over plain white floor at power-up (it learns the floor level).

The decoder also has PC tests that need no car: `sh testing/subsystem5/run_pc_tests.sh`
feeds it made-up bar/gap widths, including the mess the car's pivot steering
makes of a pass (junk bars before the code, uneven widths, a missing bar) and
200 000 random passes to check it does not invent codes.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | sampler interrupt runs at 2 kHz (±10 %) | — | auto | `subsystem5_test.c` | `[TEST] S5-U01 PASS` |
| S5-U02 | barcode_task sleeps between polls | — | auto | `subsystem5_test.c` | `[TEST] S5-U02 PASS` |
| S5-U03 | ADC in range and floor (white) level learned | — | auto | `subsystem5_test.c` | `[TEST] S5-U03 PASS` |
| S5-D01 | a one-character code is 29 bars/gaps | — | auto (PC) | `test_barcode_decode.c` | `[TEST] S5-D01 PASS` |
| S5-D02 | a clean `*L*` decodes | — | auto (PC) | `test_barcode_decode.c` | `[TEST] S5-D02 PASS` |
| S5-D03 | a clean `*R*` decodes | — | auto (PC) | `test_barcode_decode.c` | `[TEST] S5-D03 PASS` |
| S5-D04a | a backwards pass does NOT decode as it stands (Code 39 `*` is not symmetrical) | — | auto (PC) | `test_barcode_decode.c` | `[TEST] S5-D04a PASS` |
| S5-D04b | `finish_pass` turns a backwards pass round and reads it | — | auto (PC) | `test_barcode_decode.c` | `[TEST] S5-D04b PASS` |
| S5-D05 | **the known limitation:** 2 stray bars/gaps before the code make the read fail (what a correction flick produces) | the inconsistency being worked on | auto (PC) | `test_barcode_decode.c` | `[TEST] S5-D05 PASS` (it passes by failing to decode) |
| S5-D06 | 10 stray bars/gaps (a legal length again) still do not decode | same | auto (PC) | `test_barcode_decode.c` | `[TEST] S5-D06 PASS` |
| S5-D07 | a steady slow-down across the code still decodes | — | auto (PC) | `test_barcode_decode.c` | `[TEST] S5-D07 PASS` |
| S5-D08 | a sudden speed change between characters still decodes | — | auto (PC) | `test_barcode_decode.c` | `[TEST] S5-D08 PASS` |
| S5-D09 | wide bars 1.5x the narrow ones (the minimum) read | — | auto (PC) | `test_barcode_decode.c` | `[TEST] S5-D09 PASS` |
| S5-D10 | wide bars only 1.3x narrow are refused, not guessed at | — | auto (PC) | `test_barcode_decode.c` | `[TEST] S5-D10 PASS` |
| S5-D11 | a missing bar fails the read instead of changing the letter | — | auto (PC) | `test_barcode_decode.c` | `[TEST] S5-D11 PASS` |
| S5-D12 | random bar widths never decode as a command letter (200 000 passes) | safety: a wrong letter steers the car | auto (PC) | `test_barcode_decode.c` | `[TEST] S5-D12 PASS`, info line shows 0 |
| S5-U07 | a `*F*` code decodes as FORWARD, and the letters config is honoured | — | manual | read `*F*`; then swap `BARCODE_LETTERS_FORWARD` in `barcode.h` to another letter, rebuild and read that one | `barcode: "F" FORWARD` both times, with three beeps |
| S5-U08 | how often a code reads while the car is correcting its line | the inconsistency being worked on | manual | put `*L*` where the car has to correct as it crosses it, drive over it 10 times; then on a straight section, 10 times | record both hit rates: the straight one is the baseline, the correcting one is what any fix has to improve |
| S5-U04 | a printed `*L*` read forwards and backwards | — | manual | drive (or push) the car over the code both ways | `barcode: "L" LEFT`, then `... (backwards)` |
| S5-U06 | a single black line or a bump is not a barcode error; a code at least 50 % read that fails is | — | manual | 1. drive over a plain black line; 2. drive over the first half of `*L*` only, then stop / turn away | 1: no cyan light, no `barcode:` FAILED line; 2: cyan light, `barcode: pass FAILED` |
| S5-U05 | each decoded text reaches the broker once | IF-09, IF-10 | manual | `mosquitto_sub -t robocar/car1/barcode` while reading a code | one message per pass, e.g. `L` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S5-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
