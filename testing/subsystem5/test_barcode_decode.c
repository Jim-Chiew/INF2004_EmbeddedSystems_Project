/* test_barcode_decode.c -- PC test for the Code 39 decoder (S5-D01..).
 * No Pico and no sensor: it feeds the decoder made-up bar/gap widths, the way
 * a pass over a printed code would look.
 *
 * Build and run from the repo root:
 *   sh testing/subsystem5/run_pc_tests.sh
 * or by hand:
 *   gcc -std=c11 -Wall -Wextra -I app_program/subsystem5 \
 *       testing/subsystem5/test_barcode_decode.c -o /tmp/t && /tmp/t
 *
 * It includes barcode.c, because what is being tested (decode_symbol and the
 * character reader under it) is static to that file. The sampler is not
 * touched, so the drv_bc_adc functions barcode.c needs are stubbed below and
 * never called.
 *
 * S5-D05 and S5-D06 are the interesting ones: they pin down what the decoder
 * will NOT read, which is exactly what a correction flick in the middle of a
 * code produces. They pass by failing to decode -- so if a future change
 * teaches the decoder to tolerate stray bars, those two are the tests to
 * rewrite, and S5-D12 is the one that must still hold afterwards.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* The hardware calls barcode.c links against. The decoder never reaches them;
 * they exist so the file links. */
void     bc_adc_init(void) { }
uint16_t bc_adc_read(void) { return 0u; }
uint32_t bc_time_us(void) { return 0u; }
void     bc_alarm_start(uint32_t period_us) { (void)period_us; }
void     bc_alarm_stop(void) { }
void     bc_alarm_next(void) { }

#include "barcode.c"

static int g_fail = 0;
#define CHECK(id, cond, desc) do { if (cond) printf("[TEST] %s PASS  %s\n", id, desc); \
    else { printf("[TEST] %s FAIL  %s (line %d)\n", id, desc, __LINE__); g_fail++; } } while (0)

/* ---- making a pass out of thin air -------------------------------------
 * Code 39 patterns, written here from the standard rather than read from
 * barcode.c's table, so a wrong table would show up as a failure.
 * Bit (8 - i) set means element i is wide; elements 0,2,4,6,8 are bars and
 * 1,3,5,7 are the gaps inside the character. */
#define PAT_STAR  0x094u
#define PAT_L     0x043u
#define PAT_R     0x106u

#define NARROW    10000u                 /* us per narrow element */
#define WIDE      25000u                 /* 2.5 x narrow */

/* Appends one character's 9 elements, then the gap to the next character. */
static int put_char(uint32_t *w, int n, unsigned pattern, unsigned narrow,
                    unsigned wide, int with_gap)
{
    for (int i = 0; i < 9; i++) {
        int widebit = (pattern >> (8 - i)) & 1u;
        w[n++] = widebit ? wide : narrow;
    }
    if (with_gap) {
        w[n++] = narrow;                 /* the gap between characters */
    }
    return n;
}

/* A whole "*X*" pass. scale_num/scale_den stretch every element a little more
 * as the pass goes on, which is what a car that is slowing down produces. */
static int make_pass(uint32_t *w, unsigned middle, unsigned narrow,
                     unsigned wide, int scale_num, int scale_den)
{
    int n = 0;
    n = put_char(w, n, PAT_STAR, narrow, wide, 1);
    n = put_char(w, n, middle,   narrow, wide, 1);
    n = put_char(w, n, PAT_STAR, narrow, wide, 0);

    if (scale_den > 0) {
        for (int i = 0; i < n; i++) {
            long f = (long)scale_den * 100 + (long)scale_num * i;
            w[i] = (uint32_t)(((long)w[i] * f) / ((long)scale_den * 100));
        }
    }
    return n;
}

/* Decode a pass as it stands. Returns 1 and the text, or 0 and the reason. */
static int decode(const uint32_t *w, int n, char *text, const char **why)
{
    static bool thick[BARCODE_MAX_ELEMENTS];
    const char *err = NULL;
    int ok;

    text[0] = '\0';
    ok = decode_symbol(w, n, text, thick, &err) ? 1 : 0;
    if (why != NULL) {
        *why = err;
    }
    return ok;
}

int main(void)
{
    static uint32_t w[BARCODE_MAX_ELEMENTS];
    char text[BARCODE_MAX_TEXT + 1];
    const char *why = NULL;
    int  n;

    /* ---- a clean pass: the car driving straight over the code ---- */
    n = make_pass(w, PAT_L, NARROW, WIDE, 0, 0);
    CHECK("S5-D01", n == 29, "a one-character code is 29 bars/gaps");
    CHECK("S5-D02", decode(w, n, text, &why) && strcmp(text, "L") == 0,
          "a clean *L* decodes");

    n = make_pass(w, PAT_R, NARROW, WIDE, 0, 0);
    CHECK("S5-D03", decode(w, n, text, &why) && strcmp(text, "R") == 0,
          "a clean *R* decodes");

    /* ---- backwards: the car crossed the code the other way ---------------
     * A Code 39 '*' is NOT symmetrical (reversed it is the pattern for 'P'),
     * so a backwards pass cannot be decoded as it stands -- the stream has to
     * be turned round first, which is finish_pass()'s job. */
    {
        static uint32_t rev[BARCODE_MAX_ELEMENTS];
        barcode_result_t r;
        int fwd = make_pass(w, PAT_L, NARROW, WIDE, 0, 0);

        for (int i = 0; i < fwd; i++) rev[i] = w[fwd - 1 - i];
        CHECK("S5-D04a", !decode(rev, fwd, text, &why),
              "a backwards pass does not decode as it stands");

        for (int i = 0; i < fwd; i++) widths[i] = rev[i];  /* as the sampler left it */
        num_widths = fwd;
        too_long   = false;
        glitches   = 0;
        state      = BARCODE_IN_PASS;
        finish_pass(&r);
        CHECK("S5-D04b", r.valid && strcmp(r.text, "L") == 0 && r.reversed,
              "finish_pass turns a backwards pass round and reads it");
    }

    /* ---- THE LIMITATION: stray bars from a correction mid-code -----------
     * When the line sensor sees the line and the car pivots to correct, the
     * barcode sensor swings sideways and the pass picks up bars that are not
     * part of the code. This decoder needs the pass to BE the code, so those
     * passes fail. This is the behaviour to beat. */
    {
        static uint32_t j[BARCODE_MAX_ELEMENTS];
        int base = make_pass(w, PAT_L, NARROW, WIDE, 0, 0);
        int m = 0;

        j[m++] = 12000u;                                 /* one stray bar */
        j[m++] = 9000u;                                  /* and its gap */
        for (int i = 0; i < base; i++) j[m++] = w[i];
        CHECK("S5-D05", !decode(j, m, text, &why),
              "2 stray bars/gaps before the code make the read fail");
        printf("        (reason: %s)\n", why ? why : "?");
    }
    {
        static uint32_t j[BARCODE_MAX_ELEMENTS];
        int base = make_pass(w, PAT_L, NARROW, WIDE, 0, 0);
        int m = 0;

        /* ten stray elements: that is a legal length again (39), so the
           decoder gets as far as the characters before failing */
        for (int i = 0; i < 10; i++) j[m++] = (i & 1) ? 9000u : 12000u;
        for (int i = 0; i < base; i++) j[m++] = w[i];
        CHECK("S5-D06", !decode(j, m, text, &why),
              "10 stray bars/gaps (a legal length again) still do not decode");
        printf("        (reason: %s)\n", why ? why : "?");
    }

    /* ---- what it DOES tolerate: the car changing speed ------------------
     * Thick against thin is judged inside each character and among its own
     * kind, so an even slow-down across the code and a step change between
     * characters both read fine. Width distortion is not the problem. */
    n = make_pass(w, PAT_L, NARROW, WIDE, 1, 1);
    CHECK("S5-D07", decode(w, n, text, &why) && strcmp(text, "L") == 0,
          "a steady slow-down across the code still decodes");

    n = make_pass(w, PAT_L, NARROW, WIDE, 0, 0);
    for (int i = 10; i < n; i++) w[i] *= 2;              /* half speed after char 1 */
    CHECK("S5-D08", decode(w, n, text, &why) && strcmp(text, "L") == 0,
          "a sudden speed change between characters still decodes");

    /* ---- print quality ---- */
    n = make_pass(w, PAT_L, NARROW, (NARROW * 15u) / 10u, 0, 0);
    CHECK("S5-D09", decode(w, n, text, &why) && strcmp(text, "L") == 0,
          "wide bars 1.5 x the narrow ones (the minimum) read");

    n = make_pass(w, PAT_L, NARROW, (NARROW * 13u) / 10u, 0, 0);
    CHECK("S5-D10", !decode(w, n, text, &why),
          "wide bars only 1.3 x narrow are refused, not guessed at");

    /* ---- a missing bar must fail, not change the letter ---- */
    n = make_pass(w, PAT_L, NARROW, WIDE, 0, 0);
    for (int i = 14; i < n - 1; i++) w[i] = w[i + 1];
    n--;
    CHECK("S5-D11", !decode(w, n, text, &why),
          "a missing bar fails the read instead of changing the letter");

    /* ---- it must not invent codes out of noise --------------------------
     * A wrong letter steers the car the wrong way, so this is the number that
     * matters most. Random passes of every plausible length. */
    {
        int reads = 0, commands = 0;
        const int trials = 200000;

        srand(12345);
        for (int trial = 0; trial < trials; trial++) {
            int m = 29 + (rand() % 40);
            for (int i = 0; i < m; i++) {
                w[i] = 4000u + (uint32_t)(rand() % 30000);
            }
            if (decode(w, m, text, NULL)) {
                reads++;
                if (command_for(text) != BARCODE_CMD_NONE) {
                    commands++;
                    printf("        (noise read as the COMMAND \"%s\", %d elements)\n",
                           text, m);
                }
            }
        }
        printf("[TEST] S5-D12 info: %d of %d random passes decoded, %d of them a command\n",
               reads, trials, commands);
        CHECK("S5-D12", commands == 0,
              "random widths never decode as a command letter");
    }

    printf(g_fail ? "SOME FAILED (%d)\n" : "ALL PASS\n", g_fail);
    return g_fail ? 1 : 0;
}
