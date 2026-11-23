#!/bin/sh
# Builds and runs the PC tests of Subsystem 3. No Pico and no car needed.
# Run from the repo root:   sh testing/subsystem3/run_pc_tests.sh
# Needs gcc. Prints one result line per test program, exits 1 if any failed.
T=testing/subsystem3
A=app_program/subsystem3
OUT=${TMPDIR:-/tmp}/s3_pc_tests
mkdir -p "$OUT"
FAILED=0

for t in test_avoid; do
    if ! gcc -std=c11 -Wall -Wextra -I$A $T/$t.c $A/avoid.c -lm -o "$OUT/$t"; then
        echo "BUILD FAILED: $t"; FAILED=1; continue
    fi
    if "$OUT/$t" > "$OUT/$t.txt" 2>&1; then
        echo "PASS  $t   $(grep -ciE 'pass' "$OUT/$t.txt") pass lines"
    else
        echo "FAIL  $t"; tail -20 "$OUT/$t.txt"; FAILED=1
    fi
done
exit $FAILED
