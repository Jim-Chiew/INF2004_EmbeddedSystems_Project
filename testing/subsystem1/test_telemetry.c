/* test_telemetry.c -- PC test for the binary frame. Build:
 * gcc -std=c11 -Wall -Wextra -Wpedantic -I app_program app_program/telemetry.c testing/subsystem1/test_telemetry.c */
#include <stdio.h>
#include <string.h>
#include "telemetry.h"

static int g_fail = 0;
#define CHECK(id, cond, desc) do { if (cond) printf("[TEST] %s PASS  %s\n", id, desc); \
    else { printf("[TEST] %s FAIL  %s (line %d)\n", id, desc, __LINE__); g_fail++; } } while (0)

/* field by field: the struct has padding bytes, so memcmp would be wrong */
static int same(const telemetry_sample_t *a, const telemetry_sample_t *b)
{
    return a->state == b->state && a->error == b->error && a->wifi_loss == b->wifi_loss &&
           a->low_batt == b->low_batt && a->drop == b->drop && a->hump == b->hump &&
           a->speed == b->speed && a->bc_dir == b->bc_dir && a->bc_err == b->bc_err &&
           a->line == b->line && a->dist == b->dist && a->dir == b->dir &&
           a->hump_h == b->hump_h && a->tilt == b->tilt && a->roll == b->roll &&
           a->seq == b->seq && a->obj_len == b->obj_len &&
           a->errors == b->errors && strcmp(a->bc_text, b->bc_text) == 0;
}

/* Pack, then unpack exactly the bytes that were packed (the length varies with
 * the barcode text, so the packer's own answer is the only right length). */
static int roundtrip(const telemetry_sample_t *a)
{
    uint8_t f[DL_FRAME_MAX_BYTES]; telemetry_sample_t b; uint32_t n = 0u;
    memset(&b, 0xAA, sizeof b);
    return telemetry_pack(a, f, sizeof f, &n) == TELEM_OK &&
           n >= DL_FRAME_MIN_BYTES && n <= DL_FRAME_MAX_BYTES &&
           telemetry_unpack(f, n, &b) == TELEM_OK && same(a, &b);
}

int main(void)
{
    telemetry_sample_t lo, hi, x, y;
    uint8_t f[DL_FRAME_MAX_BYTES + 4];
    uint32_t n = 0u;
    /* known answers from an independent Python encoder.  Version 3 keeps bytes
       1..9 exactly as version 2 had them (nothing in the bit block moved) and
       appends the error flags and the barcode text, so A and B are the old
       vectors with "no errors, no code" after them, and C is A with every
       error flag set and the code "A" read. */
    static const uint8_t A[12] = {0x03,0x01,0x19,0x00,0xB5,0x00,0xD0,0xA2,0x05,0x3F,0x00,0x00};
    static const uint8_t B[12] = {0x03,0xFF,0xBE,0xF7,0xCF,0x02,0xA2,0x05,0xF0,0x3E,0x00,0x00};
    static const uint8_t C[13] = {0x03,0x01,0x19,0x00,0xB5,0x00,0xD0,0xA2,0x05,0x3F,0x7F,0x01,0x41};
    uint8_t c;

    memset(&lo, 0, sizeof lo);
    hi = (telemetry_sample_t){7,1,1,1,1,1,62,2,1,3,62,359,128,180,-180,15,62,0,""};
    CHECK("S1-B01", roundtrip(&lo), "all-minimum values round trip");
    CHECK("S1-B02", roundtrip(&hi), "all-maximum values round trip");

    x = (telemetry_sample_t){7,1,1,1,1,1,63,3,1,3,63,511,255,TELEM_ANGLE_INVALID,TELEM_ANGLE_INVALID,15,DL_OBJ_NONE,0,""};
    CHECK("S1-B03", roundtrip(&x), "invalid codes round trip");

    x = (telemetry_sample_t){1,0,0,0,0,0,25,0,0,0,32,90,0,0,0,0,DL_OBJ_NONE,0,""};
    CHECK("S1-B04", telemetry_pack(&x, f, sizeof f, &n) == 0 && n == 12u && memcmp(f, A, n) == 0,
          "known frame A matches (12 bytes, no errors, no code)");
    CHECK("S1-B05", telemetry_pack(&hi, f, sizeof f, &n) == 0 && n == 12u && memcmp(f, B, n) == 0,
          "known frame B matches");
    y = x; y.errors = DL_ERRF_ALL; strcpy(y.bc_text, "A");
    CHECK("S1-B25", telemetry_pack(&y, f, sizeof f, &n) == 0 && n == 13u && memcmp(f, C, n) == 0,
          "known frame C matches (every error flag, code \"A\")");

    /* one step beyond each limit must be refused */
    y = hi; y.state = 8;     CHECK("S1-B06", telemetry_pack(&y, f, sizeof f, NULL) == TELEM_ERR_RANGE, "state 8 rejected");
    y = hi; y.speed = 64;    CHECK("S1-B07", telemetry_pack(&y, f, sizeof f, NULL) == TELEM_ERR_RANGE, "speed 64 rejected");
    y = hi; y.dist = 64;     CHECK("S1-B08", telemetry_pack(&y, f, sizeof f, NULL) == TELEM_ERR_RANGE, "distance 64 rejected");
    y = hi; y.dir = 360;     CHECK("S1-B09", telemetry_pack(&y, f, sizeof f, NULL) == TELEM_ERR_RANGE, "direction 360 rejected");
    y = hi; y.hump_h = 129;  CHECK("S1-B10", telemetry_pack(&y, f, sizeof f, NULL) == TELEM_ERR_RANGE, "hump height 129 rejected");
    y = hi; y.tilt = 181;    CHECK("S1-B11", telemetry_pack(&y, f, sizeof f, NULL) == TELEM_ERR_RANGE, "tilt 181 rejected");
    y = hi; y.roll = -181;   CHECK("S1-B12", telemetry_pack(&y, f, sizeof f, NULL) == TELEM_ERR_RANGE, "roll -181 rejected");
    y = hi; y.seq = 16;      CHECK("S1-B13", telemetry_pack(&y, f, sizeof f, NULL) == TELEM_ERR_RANGE, "sequence 16 rejected");
    y = hi; y.error = 2;     CHECK("S1-B14", telemetry_pack(&y, f, sizeof f, NULL) == TELEM_ERR_RANGE, "1-bit flag 2 rejected");
    y = hi; y.obj_len = 64;  CHECK("S1-B23", telemetry_pack(&y, f, sizeof f, NULL) == TELEM_ERR_RANGE, "object length 64 rejected");

    /* fields must not leak into each other: set one field at a time */
    {
        int ok = 1;
        y = lo; y.dir = 359;  telemetry_pack(&y, f, sizeof f, &n); { telemetry_sample_t z; telemetry_unpack(f, n, &z);
            ok &= (z.dir == 359 && z.dist == 0 && z.hump_h == 0 && z.speed == 0); }
        y = lo; y.hump_h = 128; telemetry_pack(&y, f, sizeof f, &n); { telemetry_sample_t z; telemetry_unpack(f, n, &z);
            ok &= (z.hump_h == 128 && z.dir == 0 && z.tilt == 0 + 0 && z.seq == 0); }
        y = lo; y.seq = 15; y.obj_len = 62; telemetry_pack(&y, f, sizeof f, &n);
            { telemetry_sample_t z; telemetry_unpack(f, n, &z);
            ok &= (z.obj_len == 62 && z.seq == 15 && z.roll == 0 && z.tilt == 0); }
        y = lo; y.obj_len = DL_OBJ_NONE; telemetry_pack(&y, f, sizeof f, &n);
            { telemetry_sample_t z; telemetry_unpack(f, n, &z);
            ok &= (z.obj_len == DL_OBJ_NONE && z.seq == 0); }
        /* the two appended fields must not disturb the bit block either */
        y = lo; y.errors = DL_ERRF_SAFETY; strcpy(y.bc_text, "RIGHT");
            telemetry_pack(&y, f, sizeof f, &n);
            { telemetry_sample_t z; telemetry_unpack(f, n, &z);
            ok &= (z.errors == DL_ERRF_SAFETY && strcmp(z.bc_text, "RIGHT") == 0 &&
                   z.state == 0 && z.seq == 0 && z.obj_len == 0 && z.dir == 0); }
        CHECK("S1-B15", ok, "one field set does not change its neighbours");
    }

    /* the error flags: every bit round trips, and the spare bit 7 never does */
    {
        int ok = 1;
        y = lo; y.errors = DL_ERRF_ALL; telemetry_pack(&y, f, sizeof f, &n);
            ok &= (f[DL_ERRORS_BYTE] == DL_ERRF_ALL);
            { telemetry_sample_t z; telemetry_unpack(f, n, &z); ok &= (z.errors == DL_ERRF_ALL); }
        y = lo; y.errors = 0xFFu; telemetry_pack(&y, f, sizeof f, &n);
            ok &= (f[DL_ERRORS_BYTE] == DL_ERRF_ALL);     /* bit 7 is spare: sent as 0 */
        y = lo; y.errors = DL_ERRF_IMU | DL_ERRF_NET; telemetry_pack(&y, f, sizeof f, &n);
            { telemetry_sample_t z; telemetry_unpack(f, n, &z);
            ok &= (z.errors == (DL_ERRF_IMU | DL_ERRF_NET)); }
        CHECK("S1-B26", ok, "error flags round trip, spare bit 7 dropped");
    }

    /* the decoded barcode code: lengths, and what is not sent */
    {
        int ok = 1;
        telemetry_sample_t z;
        y = lo; strcpy(y.bc_text, "A");
            telemetry_pack(&y, f, sizeof f, &n);
            ok &= (n == DL_HEAD_BYTES + 1u && f[DL_BC_LEN_BYTE] == 1u);
            telemetry_unpack(f, n, &z); ok &= (strcmp(z.bc_text, "A") == 0);
        y = lo; strcpy(y.bc_text, "ABCDEFGHIJKL");          /* DL_BC_TEXT_MAX = 12 */
            telemetry_pack(&y, f, sizeof f, &n);
            ok &= (n == DL_FRAME_MAX_BYTES && f[DL_BC_LEN_BYTE] == DL_BC_TEXT_MAX);
            telemetry_unpack(f, n, &z); ok &= (strcmp(z.bc_text, "ABCDEFGHIJKL") == 0);
        y = lo; y.bc_text[0] = '\0';                        /* nothing read */
            telemetry_pack(&y, f, sizeof f, &n);
            ok &= (n == DL_FRAME_MIN_BYTES && f[DL_BC_LEN_BYTE] == 0u);
            telemetry_unpack(f, n, &z); ok &= (z.bc_text[0] == '\0');
        y = lo; strcpy(y.bc_text, "A\tB");                  /* a control character ends it */
            telemetry_pack(&y, f, sizeof f, &n);
            ok &= (n == DL_HEAD_BYTES + 1u);
            telemetry_unpack(f, n, &z); ok &= (strcmp(z.bc_text, "A") == 0);
        CHECK("S1-B27", ok, "barcode code round trips, 0 to 12 printable characters");
    }

    CHECK("S1-B16", telemetry_pack(NULL, f, sizeof f, NULL) == TELEM_ERR_ARG &&
          telemetry_pack(&lo, f, DL_FRAME_MIN_BYTES - 1u, NULL) == TELEM_ERR_ARG,
          "NULL or too-small buffer rejected");
    y = lo; strcpy(y.bc_text, "ABCDEFGHIJKL");
    CHECK("S1-B28", telemetry_pack(&y, f, DL_FRAME_MIN_BYTES, NULL) == TELEM_ERR_ARG &&
          telemetry_pack(&y, f, DL_FRAME_MAX_BYTES, NULL) == TELEM_OK,
          "a buffer without room for the code is rejected");
    memcpy(f, A, sizeof A); f[0] = 2;         /* a version-2 (10-byte) frame */
    CHECK("S1-B17", telemetry_unpack(f, sizeof A, &y) == TELEM_ERR_ARG, "old version byte rejected");
    CHECK("S1-B18", telemetry_unpack(A, DL_FRAME_MIN_BYTES - 1u, &y) == TELEM_ERR_ARG &&
          telemetry_unpack(A, DL_FRAME_MIN_BYTES + 1u, &y) == TELEM_ERR_ARG,
          "wrong frame length rejected");
    memcpy(f, C, sizeof C);
    CHECK("S1-B29", telemetry_unpack(f, sizeof C, &y) == TELEM_OK &&
          telemetry_unpack(f, sizeof C - 1u, &y) == TELEM_ERR_ARG &&
          telemetry_unpack(f, sizeof C + 1u, &y) == TELEM_ERR_ARG,
          "a length that disagrees with the text-length byte is rejected");
    memcpy(f, A, sizeof A); f[DL_BC_LEN_BYTE] = (uint8_t)(DL_BC_TEXT_MAX + 1u);
    CHECK("S1-B30", telemetry_unpack(f, sizeof A, &y) == TELEM_ERR_ARG,
          "a text longer than the field allows is rejected");

    c = DL_CMD_STOP;
    CHECK("S1-B19", telemetry_cmd_decode(&c, 1) == DL_CMD_STOP, "command byte 2 = STOP");
    c = 0; CHECK("S1-B20", telemetry_cmd_decode(&c, 1) == 0, "command byte 0 rejected");
    c = 4; CHECK("S1-B21", telemetry_cmd_decode(&c, 1) == 0, "command byte 4 rejected");
    CHECK("S1-B22", telemetry_cmd_decode(A, 2) == 0 && telemetry_cmd_decode(NULL, 1) == 0, "wrong length or NULL rejected");
    CHECK("S1-B24", roundtrip(&(telemetry_sample_t){0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,DL_OBJ_NONE,0,""}),
          "\"no object measured yet\" round trips");

    printf(g_fail ? "SOME FAILED (%d)\n" : "ALL PASS\n", g_fail);
    return g_fail ? 1 : 0;
}
