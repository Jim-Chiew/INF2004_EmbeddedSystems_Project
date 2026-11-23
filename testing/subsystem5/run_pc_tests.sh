#!/bin/sh
# Builds and runs the PC tests of Subsystem 5. No Pico and no sensor needed:
# the decoder is fed made-up bar/gap widths.
# Run from the repo root:   sh testing/subsystem5/run_pc_tests.sh
# Needs gcc. Prints one result line per test program, exits 1 if any failed.
T=testing/subsystem5
A=app_program/subsystem5
OUT=${TMPDIR:-/tmp}/s5_pc_tests
mkdir -p "$OUT"
FAILED=0

# Add -fsanitize=address,undefined if your gcc has the sanitiser libraries
# (mingw on Windows usually does not).
for t in test_barcode_decode; do
    if ! gcc -std=c11 -Wall -Wextra -I$A $T/$t.c -o "$OUT/$t"; then
        echo "BUILD FAILED: $t"; FAILED=1; continue
    fi
    if "$OUT/$t" > "$OUT/$t.txt" 2>&1; then
        echo "PASS  $t   $(grep -ciE 'pass' "$OUT/$t.txt") pass lines"
    else
        echo "FAIL  $t   (see $OUT/$t.txt)"; tail -20 "$OUT/$t.txt"; FAILED=1
    fi
done
exit $FAILED
