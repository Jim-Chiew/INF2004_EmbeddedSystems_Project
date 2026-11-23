#!/bin/sh
# Builds and runs the PC tests of Subsystem 4. No Pico and no car needed.
# Run from the repo root:   sh testing/subsystem4/run_pc_tests.sh
# Needs gcc. Prints one result line per test program, exits 1 if any failed.
#
# Only the plain-C parts are testable here: imu.c itself talks to the LSM303D
# over I2C, so the on-target checks in subsystem4_test.c cover that. What lives
# here is the lift detector's threshold arithmetic, which decides whether the
# car e-stops in the middle of a run (see app_program/subsystem4/lift_gate.h).
T=testing/subsystem4
A=app_program/subsystem4
OUT=${TMPDIR:-/tmp}/s4_pc_tests
mkdir -p "$OUT"
FAILED=0

for t in test_lift_gate test_hump_height; do
    if ! gcc -std=c11 -Wall -Wextra -I$A $T/$t.c $A/lift_gate.c $A/hump_height.c -lm -o "$OUT/$t"; then
        echo "BUILD FAILED: $t"; FAILED=1; continue
    fi
    if "$OUT/$t" > "$OUT/$t.txt" 2>&1; then
        echo "PASS  $t   $(grep -ciE 'pass' "$OUT/$t.txt") pass lines"
    else
        echo "FAIL  $t"; tail -20 "$OUT/$t.txt"; FAILED=1
    fi
done
exit $FAILED
