/*
 * barcode.h
 * Subsystem 5 -- barcode reader on MH-sensor #3, the right-most MH-sensor.
 * Reads Code 39 barcodes: thick/thin black bars and thick/thin white gaps,
 * framed by '*' start and stop characters (e.g. "*L*").
 *
 * The sensor's ANALOG output (AO) is sampled by the ADC at 2 kHz from a
 * timer interrupt (drv_bc_adc.c + task_barcode.c). Software thresholds with
 * hysteresis turn the voltage into black/white edges, which are timed.
 *
 * Plain C: no kernel and no Pico SDK headers, so both sides can include it.
 *
 * Pass state machine (barcode_poll):
 *   WAIT_BAR --first black bar--> IN_PASS --no edge for BARCODE_END_GAP_MS
 *   --> decode --> WAIT_BAR
 */

#ifndef BARCODE_H
#define BARCODE_H

#include <stdbool.h>
#include <stdint.h>

// Pin from the Design Review pin table (Section 2).
#define BARCODE_AO_PIN         28     // BARCODE_IR_AO: MH-sensor #3 AO (Grove 7)
#define BARCODE_ADC_INPUT      2      // GP28 = ADC input 2

// 1 = AO voltage is HIGHER over a black bar (usual for MH-sensors).
// Set to 0 if the serial output shows the opposite.
#define BARCODE_BAR_IS_HIGH    1

#define BARCODE_SAMPLE_US      500    // ADC sample period: 2 kHz (>= 1 kHz)
#define BARCODE_MIN_CONTRAST   250    // ADC counts (~0.2 V) black above white

// How fast the remembered "bar level" fades back towards the floor level.
// The sampler puts its threshold half way between the two, so whatever is the
// darkest thing seen lately decides how dark a bar has to be to count.
//
// This matters more than it looks. A narrow printed bar and a solid black
// line do NOT read the same: the sensor's spot averages a 2 mm bar with the
// white either side, so a bar might read 400 counts where the line reads
// 1000. With a slow fade, crossing the line leaves the threshold near 585 for
// seconds afterwards and every bar at 400 is invisible -- which looks exactly
// like a sensor that has stopped working, even though its own LED is still
// blinking on each bar.
//
// Time constant = (1 << this) samples, and samples are BARCODE_SAMPLE_US
// apart: 10 is about 0.5 s, 14 was about 8 s. It must stay well LONGER than
// one bar (tens of ms) so the threshold holds steady across a code, and well
// SHORTER than the gap between crossing the line and reaching a code.
#define BARCODE_BLACK_FADE     10
#define BARCODE_HYST_PCT       10     // hysteresis either side of the midpoint

// ---- What this decoder needs from a pass ---------------------------------
// The elements of one pass must BE the code: exactly 29, 39, 49 ... of them
// (10 per character less the last gap), starting on the code's first bar.
// That is reliable when the car drives straight over a code, and it is what
// makes a read fail when the car flicks to correct its line halfway across:
// one stray bar from the sensor swinging is enough to change the count.
// Widths themselves are not the problem -- thick against thin is judged
// inside each character and among its own kind (bars with bars, gaps with
// gaps), so the car speeding up or slowing down between characters is fine.
#define BARCODE_GLITCH_US      1000   // pulses shorter than this are noise
// A pass that fails to decode counts as a barcode READ ERROR (cyan error
// light, bc_err in telemetry) only if it got through at least
// BARCODE_FAIL_MIN_PCT % of a full barcode first. Shorter passes (a black
// line, a bump, the edge of a code) are ignored.
#define BARCODE_DATA_CHARS     1      // characters between the *s on the course codes (*L* = 1)
#define BARCODE_FULL_ELEMENTS  ((BARCODE_DATA_CHARS + 2) * 10 - 1)   // bars + gaps: 29 for *L*
#define BARCODE_FAIL_MIN_PCT   50     // % of a full barcode read before a failure counts
#define BARCODE_FAIL_MIN_ELEMENTS \
        ((BARCODE_FULL_ELEMENTS * BARCODE_FAIL_MIN_PCT + 99) / 100)  // 15 for *L*
#define BARCODE_END_GAP_MS     1000   // no edge for this long = pass finished
#define BARCODE_MAX_ELEMENTS   129    // up to 11 characters incl. * and *
#define BARCODE_MAX_TEXT       12

// ---- Which letters mean what ----------------------------------------------
// The FIRST data character between the *s decides the command, so "*L*" is a
// LEFT barcode. To use different letters for the course, change ONLY these
// four strings; nothing else in the car needs editing.
//   - any character in the Code 39 table works: 0-9, A-Z, space, - . $ / + %
//   - a character must appear in only ONE of the four
//   - "" switches that command off
//   - any other letter still decodes and is reported, it just commands
//     nothing (BARCODE_CMD_NONE)
// Demo table: A = turn left, B = turn right, C = go straight, D = U-turn.
// L, R, F and S are kept as well so the older test codes still work.
#define BARCODE_LETTERS_LEFT     "LA"
#define BARCODE_LETTERS_RIGHT    "RB"
#define BARCODE_LETTERS_FORWARD  "FSC"  // F = forward, S = straight, C (demo)
#define BARCODE_LETTERS_UTURN    "D"    // D (demo)

typedef enum
{
    BARCODE_CMD_NONE = 0,
    BARCODE_CMD_LEFT,
    BARCODE_CMD_RIGHT,
    BARCODE_CMD_FORWARD,
    BARCODE_CMD_UTURN
} barcode_cmd_t;

typedef enum
{
    BARCODE_WAIT_BAR = 0,   // over plain floor, waiting for a first bar
    BARCODE_IN_PASS         // timing bars and gaps
} barcode_state_t;

// Everything measured and decoded in one pass over a barcode.
typedef struct
{
    int           num_elements;                      // bars + gaps
    uint32_t      width_us[BARCODE_MAX_ELEMENTS];    // in the order scanned
    bool          is_thick[BARCODE_MAX_ELEMENTS];    // thick or thin
    bool          valid;                             // decoded OK
    bool          reversed;                          // scanned backwards
    char          text[BARCODE_MAX_TEXT + 1];        // data between the *s
    barcode_cmd_t command;
    int           glitches;                          // pulses < 1 ms removed
    const char *  error;                             // why it failed
} barcode_result_t;

// Sets up the ADC and learns the floor level: the sensor should be over
// white floor at this moment. Does NOT start the sampling (see task_barcode.c).
void        barcode_init(void);

// One 2 kHz sample: called from the timer interrupt only.
void        barcode_sample(void);

// Call often from barcode_task. Returns true when a pass has finished
// and fills *result.
bool        barcode_poll(barcode_result_t *result);

// ---- Is a bar passing under the sensor right now? ------------------------
// Set by the sampler interrupt on every black <-> white edge: when the last
// one arrived (the bc_time_us() clock, which is the same 1 MHz hardware timer
// as app_now_us()) and how many edges there have been since power-up.
// One 32-bit store each, so any task on either core may read them without a
// lock. Navigation uses them to steer gently while a code goes past, see
// barcode_crossing() in subsystem5.h.
extern volatile uint32_t barcode_edge_us;
extern volatile uint32_t barcode_edges;
// 1 while the sensor is reading black. An edge timestamp on its own is not
// enough: sitting on one wide bar can take longer than the window navigation
// waits, and the car would snap back to a hard pivot in the middle of a code.
extern volatile uint32_t barcode_black;

barcode_state_t barcode_state(void);
const char *barcode_cmd_name(barcode_cmd_t cmd);

// Latest AO reading and the learned floor (white) and bar (black) levels,
// in ADC counts (0..4095), for tuning.
void        barcode_get_levels(uint16_t *raw, uint16_t *white, uint16_t *black);

// Lowest and highest AO reading since the last call, then starts a new
// window. One caller only (barcode_task). Tells you whether the sensor is
// seeing any contrast at all, which the levels alone cannot: between two
// calls the reading may have swung from white to black and back.
void        barcode_take_swing(uint16_t *lo, uint16_t *hi);

// How the pass in progress is getting on: bars/gaps collected so far, and how
// many edges the glitch filter threw away as noise (each one also undoes the
// element before it, so a noisy signal eats a pass alive). Call from
// barcode_task only -- it is that task's own state.
void        barcode_progress(int *elements, int *glitch_count);

#endif // BARCODE_H
