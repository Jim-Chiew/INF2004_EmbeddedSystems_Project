/*
 * beeper.c
 * Subsystem 1 -- see beeper.h.
 *
 * Pattern state machine:
 *   IDLE --beeper_play--> ON --on_ms, beeps left--> OFF --BEEP_GAP_MS--> ON
 *                            --on_ms, none left--> IDLE
 */

#include "beeper.h"
#include "drv_buzzer.h"

typedef enum { BEEP_IDLE, BEEP_ON, BEEP_OFF } beep_phase_t;

static struct {
    beep_phase_t phase;
    int          left;      /* beeps still to come after the current one */
    uint32_t     on_ms;
    uint32_t     hz;        /* 0 = the buzzer's usual tone */
    uint32_t     until_ms;  /* when this phase ends */
} b;

static void sound_on(void)
{
    if (b.hz == 0u) {
        buzzer_on();
    } else {
        buzzer_on_hz((unsigned)b.hz);
    }
}

void beeper_init(void)
{
    buzzer_init();
    b.phase = BEEP_IDLE;
    b.left = 0;
}

void beeper_play(int beeps, uint32_t on_ms, uint32_t now_ms)
{
    beeper_play_tone(beeps, on_ms, BEEP_HZ_COMMAND, now_ms);
}

void beeper_play_tone(int beeps, uint32_t on_ms, uint32_t hz, uint32_t now_ms)
{
    if (beeps <= 0) {
        return;
    }
    b.on_ms = on_ms;
    b.hz = hz;
    b.left = beeps - 1;         /* this call starts the first one */
    b.phase = BEEP_ON;
    b.until_ms = now_ms + on_ms;
    sound_on();
}

void beeper_step(uint32_t now_ms)
{
    if (b.phase == BEEP_IDLE) {
        return;
    }
    /* Signed difference: right across a wrap of the millisecond clock */
    if ((int32_t)(now_ms - b.until_ms) < 0) {
        return;                 /* this phase is not over yet */
    }

    if (b.phase == BEEP_ON) {
        buzzer_off();
        if (b.left <= 0) {
            b.phase = BEEP_IDLE;        /* the pattern is done */
            return;
        }
        b.phase = BEEP_OFF;
        b.until_ms = now_ms + BEEP_GAP_MS;
        return;
    }

    /* BEEP_OFF: the gap is over, sound the next beep */
    b.left--;
    b.phase = BEEP_ON;
    b.until_ms = now_ms + b.on_ms;
    sound_on();
}

bool beeper_busy(void)
{
    return b.phase != BEEP_IDLE;
}
