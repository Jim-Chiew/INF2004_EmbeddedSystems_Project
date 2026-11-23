/*
 * data_layout.h
 * One shared layout for the MQTT telemetry frame and command (change
 * requirement: Minimum-Bit Data Representation). Plain C, constants only.
 * Every buddy includes this file; nobody invents their own bit positions.
 *
 * Frame on the wire = 1 version byte + 9 payload bytes of bit fields
 * + 1 error-flag byte + 1 text-length byte + that many characters of decoded
 * barcode text = 12 to 24 bytes.
 * The payload is a little-endian bit string: bit 0 is the least significant
 * bit of payload byte 0, bit 8 the least significant bit of byte 1, and so on.
 * (Up to version 1 it was 8 bytes and fitted one 64-bit word; version 2 added
 * the object length and the payload outgrew a single word; version 3 appended
 * the error flags and the barcode text, which do not fit in the bit block --
 * the bit positions below did not move, so a reader of bytes 1..9 is
 * unchanged.)
 *
 *   bits   field          size  unit / step           valid     invalid
 *   0-2    state            3   code 0-7              0-7       -
 *   3      error fault      1   0 / 1                 -         -
 *   4      Wi-Fi loss       1   0 / 1 (status only)   -         -
 *   5      low battery      1   0 / 1 (status only)   -         -
 *   6      drop flag        1   0 / 1                 -         -
 *   7      hump detect      1   0 / 1                 -         -
 *   8-13   speed            6   cm/s, step 1          0-62      63
 *   14-15  barcode dir      2   code 0-3              0-3       -
 *   16     barcode error    1   0 / 1                 -         -
 *   17-18  line position    2   code 0-3              0-3       -
 *   19-24  distance         6   cm, step 1            0-62      63 (no echo)
 *   25-33  direction        9   degrees, step 1       0-359     511
 *   34-41  hump jolt        8   g, step 0.02          0-128     255   (0-2.56 g)
 *   42-50  tilt             9   degrees + 180         -180..180 511
 *   51-59  roll             9   degrees + 180         -180..180 511
 *   60-63  sequence         4   counts 0-15, wraps    0-15      -
 *   64-69  object length    6   cm, step 1            0-62      63 (none yet)
 *   70-71  spare            2   always 0              -         -
 *                          --
 *                          72 bits (63 field bits + 7 spare used for the
 *                          Wi-Fi loss, low-battery, drop flag and sequence,
 *                          and 2 left over for whoever needs them next)
 *
 * "distance" and "object length" are different measurements, both from
 * subsystem 6: distance is how far ahead the nearest thing is RIGHT NOW
 * (every frame), object length is how wide the thing that stopped the car
 * measured when the servo last scanned it. The length holds its value until
 * the next scan and is 63 ("none yet") until the first one, and again after
 * every RESET or new run.
 *
 * Values marked "placeholder" are decided again after integration.
 * Wi-Fi loss and low battery are status bits only: they never cause
 * FAULT_STOP (Week 6: link loss never faults).
 * Battery voltage, distance travelled and the timestamp are not sent.
 * The barcode TEXT is not in the frame either (only its LEFT/RIGHT code is):
 * comms publishes it once per read, as plain ASCII, on COMMS_TOPIC_BARCODE
 * (robocar/car1/barcode, see comms.h).
 */
#ifndef DATA_LAYOUT_H
#define DATA_LAYOUT_H

#include <stdint.h>

#define DL_VERSION          3u     /* first byte of every frame.  1 = the
                                    * 9-byte frame without the object length,
                                    * 2 = the 10-byte frame without the error
                                    * flags and the barcode text */
#define DL_PAYLOAD_BYTES    9u     /* the bit-field block: bytes 1..9 */

/* After the bit block come the two things that cannot go in it: the error
 * flags (one bit each, so a byte of its own) and the decoded barcode text
 * (ASCII, as long as the code the camera read).
 *   byte 10     error flags, DL_ERRF_* below
 *   byte 11     how many characters of barcode text follow, 0..DL_BC_TEXT_MAX
 *   bytes 12..  that text, ASCII, no terminator ('A', 'L', '123'...)
 * The text is the only thing that makes a frame vary in length: every field
 * before it is always at the same offset, and a frame with no code read yet
 * is DL_HEAD_BYTES long. */
#define DL_ERRORS_BYTE      10u    /* index of the error-flag byte */
#define DL_BC_LEN_BYTE      11u    /* index of the text-length byte */
#define DL_HEAD_BYTES       12u    /* version + payload + flags + length byte */
#define DL_BC_TEXT_MAX      12u    /* longest code (subsystem1.h S1_BC_TEXT_MAX) */
#define DL_FRAME_MIN_BYTES  DL_HEAD_BYTES
#define DL_FRAME_MAX_BYTES  (DL_HEAD_BYTES + DL_BC_TEXT_MAX)

#define DL_BITS_PER_BYTE    8u

/* A mask of `bits` ones (typed inline function, not a macro). */
static inline uint64_t dl_mask(uint32_t bits)
{
    return (UINT64_C(1) << bits) - 1u;
}

/* ---- field positions and sizes (shift = lowest bit, bits = width) ---- */
#define DL_STATE_SHIFT        0u
#define DL_STATE_BITS         3u
#define DL_ERROR_SHIFT        3u
#define DL_ERROR_BITS         1u
#define DL_WIFI_LOSS_SHIFT    4u
#define DL_WIFI_LOSS_BITS     1u
#define DL_LOW_BATT_SHIFT     5u
#define DL_LOW_BATT_BITS      1u
#define DL_DROP_SHIFT         6u
#define DL_DROP_BITS          1u
#define DL_HUMP_SHIFT         7u
#define DL_HUMP_BITS          1u
#define DL_SPEED_SHIFT        8u
#define DL_SPEED_BITS         6u
#define DL_BC_DIR_SHIFT       14u
#define DL_BC_DIR_BITS        2u
#define DL_BC_ERR_SHIFT       16u
#define DL_BC_ERR_BITS        1u
#define DL_LINE_SHIFT         17u
#define DL_LINE_BITS          2u
#define DL_DIST_SHIFT         19u
#define DL_DIST_BITS          6u
#define DL_DIR_SHIFT          25u
#define DL_DIR_BITS           9u
#define DL_HUMP_H_SHIFT       34u
#define DL_HUMP_H_BITS        8u
#define DL_TILT_SHIFT         42u
#define DL_TILT_BITS          9u
#define DL_ROLL_SHIFT         51u
#define DL_ROLL_BITS          9u
#define DL_SEQ_SHIFT          60u
#define DL_SEQ_BITS           4u
#define DL_OBJ_SHIFT          64u
#define DL_OBJ_BITS           6u
#define DL_SPARE_SHIFT        70u   /* always sent as 0; take bits from here */
#define DL_SPARE_BITS         2u

#define DL_TOTAL_BITS         72u
#define DL_FIELD_BITS         63u   /* bits of data fields only (no status/seq/spare) */

/* ---- value limits and invalid codes ---- */
#define DL_SPEED_MAX_CMS      62u
#define DL_SPEED_INVALID      63u
#define DL_DIST_MAX_CM        62u
#define DL_DIST_INVALID       63u   /* no echo / unknown */
#define DL_DIR_MAX_DEG        359u
#define DL_DIR_INVALID        511u
/* hump_h carries the hump JOLT: peak vertical acceleration of the hump(s)
 * that ended since the previous frame, in steps of DL_HUMP_JOLT_STEP_MG
 * (0 = no hump in this frame, 128 = 2.56 g or more). The field keeps its
 * old name and range so the frame layout does not change. */
#define DL_HUMP_JOLT_STEP_MG  20u   /* 1 step = 0.02 g */
#define DL_HUMP_H_MAX_MM      128u  /* largest value (name kept: it is now jolt steps, not mm) */
#define DL_HUMP_H_INVALID     255u
#define DL_TILT_OFFSET        180   /* stored value = degrees + 180 */
#define DL_TILT_MIN_DEG       (-180)
#define DL_TILT_MAX_DEG       180
#define DL_TILT_INVALID       511u
#define DL_OBJ_MAX_CM         62u
/* The default: no object has been measured yet (and what a RESET goes back
 * to).  Note 0 means "measured, and it was under half a centimetre", which is
 * a real answer, so it must not be used for "nothing". */
#define DL_OBJ_NONE           63u
#define DL_SEQ_MAX            15u
#define DL_STATE_MAX           7u   /* 8 states in 3 bits */
#define DL_BC_DIR_MAX          3u   /* 2 bits, all four codes used */
#define DL_LINE_MAX            3u   /* 2 bits */

/* ---- state codes (the 8 Week 6 navigation states) ---- */
typedef enum
{
    DL_STATE_CALIBRATE = 0,
    DL_STATE_LINE_FOLLOWING,
    DL_STATE_LINE_LOST,
    DL_STATE_BARCODE_EXECUTION,
    DL_STATE_LINE_SEARCH,
    DL_STATE_OBSTACLE_AVOIDANCE,
    DL_STATE_BARCODE_RESCAN,
    DL_STATE_FAULT_STOP            /* = 7 */
} dl_state_t;

/* ---- barcode direction codes (2 bits) ---- */
typedef enum
{
    DL_BC_NONE = 0,
    DL_BC_LEFT = 1,
    DL_BC_RIGHT = 2,
    DL_BC_FORWARD = 3              /* the last free code in the 2-bit field */
} dl_bc_dir_t;

/* ---- line position codes (2 bits, same as ir_line_position_t) ---- */
typedef enum
{
    DL_LINE_CENTRED = 0,
    DL_LINE_DRIFT_RIGHT = 1,
    DL_LINE_DRIFT_LEFT = 2,
    DL_LINE_CROSSING = 3
} dl_line_t;

/* ---- error flags (byte DL_ERRORS_BYTE), one bit each ----
 * What the car's error indicator is showing: the SAME bit values the RGB LEDs
 * and the USB status line use (task_telemetry.c ERR_*), so nothing is
 * translated on the way to the wire.  Any number of them can be set at once.
 * They say why the car has stopped or what is wrong with it; the status bits
 * inside the bit block (error, wifi_loss, low_batt, drop, hump, bc_err) stay
 * where they are and keep their old meanings. */
#define DL_ERRF_SAFETY      0x01u  /* lifted, overturned, or the e-stop latched */
#define DL_ERRF_MOTOR       0x02u  /* the wheel encoders never started */
#define DL_ERRF_IMU         0x04u  /* IMU missing, or its reads keep failing */
#define DL_ERRF_CAL         0x08u  /* IMU calibration failed */
#define DL_ERRF_BARCODE     0x10u  /* the last barcode pass did not decode */
#define DL_ERRF_NET         0x20u  /* WiFi / broker was down when the frame was
                                    * built: the frame waits in the queue and
                                    * still says so when the link comes back */
#define DL_ERRF_OBSTACLE    0x40u  /* something within S6_STOP_MM ahead */
#define DL_ERRF_ALL         0x7Fu  /* all of the above; bit 7 spare, sent as 0 */

/* ---- command sent to robocar/car1/cmd: one byte (proposed) ---- */
#define DL_CMD_BYTES          1u
#define DL_CMD_START          1u
#define DL_CMD_STOP           2u
#define DL_CMD_RESET          3u

/* ---- compile-time checks: fields touch, none overlap, total is 64 ---- */
#define DL_NEXT(prev) (prev##_SHIFT + prev##_BITS)

_Static_assert(DL_ERROR_SHIFT   == DL_NEXT(DL_STATE),     "error follows state");
_Static_assert(DL_WIFI_LOSS_SHIFT == DL_NEXT(DL_ERROR),   "wifi loss follows error");
_Static_assert(DL_LOW_BATT_SHIFT == DL_NEXT(DL_WIFI_LOSS), "low battery follows wifi loss");
_Static_assert(DL_DROP_SHIFT    == DL_NEXT(DL_LOW_BATT),  "drop follows low battery");
_Static_assert(DL_HUMP_SHIFT    == DL_NEXT(DL_DROP),      "hump follows drop");
_Static_assert(DL_SPEED_SHIFT   == DL_NEXT(DL_HUMP),      "speed follows hump");
_Static_assert(DL_BC_DIR_SHIFT  == DL_NEXT(DL_SPEED),     "barcode dir follows speed");
_Static_assert(DL_BC_ERR_SHIFT  == DL_NEXT(DL_BC_DIR),    "barcode err follows dir");
_Static_assert(DL_LINE_SHIFT    == DL_NEXT(DL_BC_ERR),    "line follows barcode err");
_Static_assert(DL_DIST_SHIFT    == DL_NEXT(DL_LINE),      "distance follows line");
_Static_assert(DL_DIR_SHIFT     == DL_NEXT(DL_DIST),      "direction follows distance");
_Static_assert(DL_HUMP_H_SHIFT  == DL_NEXT(DL_DIR),       "hump height follows direction");
_Static_assert(DL_TILT_SHIFT    == DL_NEXT(DL_HUMP_H),    "tilt follows hump height");
_Static_assert(DL_ROLL_SHIFT    == DL_NEXT(DL_TILT),      "roll follows tilt");
_Static_assert(DL_SEQ_SHIFT     == DL_NEXT(DL_ROLL),      "sequence follows roll");
_Static_assert(DL_OBJ_SHIFT     == DL_NEXT(DL_SEQ),       "object length follows sequence");
_Static_assert(DL_SPARE_SHIFT   == DL_NEXT(DL_OBJ),       "spare follows object length");
_Static_assert(DL_NEXT(DL_SPARE) == DL_TOTAL_BITS,        "frame is exactly 72 bits");
_Static_assert(DL_TOTAL_BITS == 8u * DL_PAYLOAD_BYTES,    "payload size matches bits");
_Static_assert(DL_ERRORS_BYTE == DL_PAYLOAD_BYTES + 1u,   "error flags follow the bit block");
_Static_assert(DL_BC_LEN_BYTE == DL_ERRORS_BYTE + 1u,     "text length follows the flags");
_Static_assert(DL_HEAD_BYTES  == DL_BC_LEN_BYTE + 1u,     "the text starts after the length");
_Static_assert(DL_FRAME_MAX_BYTES == DL_HEAD_BYTES + DL_BC_TEXT_MAX, "longest frame");
_Static_assert(DL_ERRF_ALL == (DL_ERRF_SAFETY | DL_ERRF_MOTOR | DL_ERRF_IMU |
                               DL_ERRF_CAL | DL_ERRF_BARCODE | DL_ERRF_NET |
                               DL_ERRF_OBSTACLE), "DL_ERRF_ALL holds every flag");

/* the invalid codes and limits must fit the field widths */
_Static_assert(DL_SPEED_INVALID  <= (1u << DL_SPEED_BITS) - 1u,  "speed invalid fits");
_Static_assert(DL_DIST_INVALID   <= (1u << DL_DIST_BITS) - 1u,   "distance invalid fits");
_Static_assert(DL_OBJ_NONE       <= (1u << DL_OBJ_BITS) - 1u,    "object none fits");
_Static_assert(DL_OBJ_MAX_CM     <  DL_OBJ_NONE,                 "object range below none");
_Static_assert(DL_DIR_INVALID    <= (1u << DL_DIR_BITS) - 1u,    "direction invalid fits");
_Static_assert(DL_HUMP_H_INVALID <= (1u << DL_HUMP_H_BITS) - 1u, "hump height invalid fits");
_Static_assert(DL_TILT_INVALID   <= (1u << DL_TILT_BITS) - 1u,   "tilt invalid fits");
_Static_assert((DL_TILT_MAX_DEG + DL_TILT_OFFSET) < (int)DL_TILT_INVALID, "tilt range below invalid");
_Static_assert(DL_STATE_FAULT_STOP == 7, "8 states fit 3 bits");

#endif /* DATA_LAYOUT_H */