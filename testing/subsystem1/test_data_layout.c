/* test_data_layout.c - checks the layout header on a PC (S1-L01..). */
#include <stdio.h>
#include "data_layout.h"

#define ROW(n, p) printf("%-12s bits %2u-%2u  width %u\n", n, p##_SHIFT, p##_SHIFT + p##_BITS - 1u, p##_BITS)

int main(void)
{
    unsigned fields = DL_STATE_BITS + DL_ERROR_BITS + DL_HUMP_BITS + DL_SPEED_BITS
                    + DL_BC_DIR_BITS + DL_BC_ERR_BITS + DL_LINE_BITS + DL_DIST_BITS
                    + DL_DIR_BITS + DL_HUMP_H_BITS + DL_TILT_BITS + DL_ROLL_BITS
                    + DL_OBJ_BITS;
    unsigned extra  = DL_WIFI_LOSS_BITS + DL_LOW_BATT_BITS + DL_DROP_BITS + DL_SEQ_BITS
                    + DL_SPARE_BITS;
    int fail = 0;

    ROW("state", DL_STATE);     ROW("error", DL_ERROR);   ROW("wifi loss", DL_WIFI_LOSS);
    ROW("low batt", DL_LOW_BATT); ROW("drop", DL_DROP);   ROW("hump", DL_HUMP);
    ROW("speed", DL_SPEED);     ROW("barcode dir", DL_BC_DIR); ROW("barcode err", DL_BC_ERR);
    ROW("line pos", DL_LINE);   ROW("distance", DL_DIST); ROW("direction", DL_DIR);
    ROW("hump h", DL_HUMP_H);   ROW("tilt", DL_TILT);     ROW("roll", DL_ROLL);
    ROW("sequence", DL_SEQ);   ROW("object len", DL_OBJ); ROW("spare", DL_SPARE);

    printf("data fields = %u bits, status+seq = %u bits, total = %u\n", fields, extra, fields + extra);
    if (fields != DL_FIELD_BITS) { printf("FAIL: data fields not %u\n", DL_FIELD_BITS); fail = 1; }
    if (fields + extra != DL_TOTAL_BITS) { printf("FAIL: total not %u\n", DL_TOTAL_BITS); fail = 1; }
    if (dl_mask(DL_SEQ_BITS) != DL_SEQ_MAX) { printf("FAIL: seq mask\n"); fail = 1; }
    if (DL_FRAME_BYTES != 10u) { printf("FAIL: frame bytes\n"); fail = 1; }
    if (DL_VERSION != 2u) { printf("FAIL: version byte\n"); fail = 1; }
    /* the object length must have its own code for "nothing measured yet",
       and it must not collide with a real length */
    if (dl_mask(DL_OBJ_BITS) != DL_OBJ_NONE) { printf("FAIL: object none code\n"); fail = 1; }
    if (DL_OBJ_MAX_CM >= DL_OBJ_NONE) { printf("FAIL: object range\n"); fail = 1; }
    printf(fail ? "FAILED\n" : "S1-L01 PASS  (ALL PASS, frame = %u bytes)\n", DL_FRAME_BYTES);
    return fail;
}
