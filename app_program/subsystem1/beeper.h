/*
 * beeper.h
 * Subsystem 1 -- beep patterns on the buzzer, as a little state machine that
 * never waits.  telemetry_task starts a pattern and then calls beeper_step()
 * once per loop pass (every S1_PERIOD_MS); the beeps happen in the gaps
 * between everything else the task does.
 *
 * Why not just sleep between beeps: telemetry_task also forwards remote
 * commands, drains barcode messages and drives the LEDs.  Two beeps with a
 * gap take about half a second, and nothing else in the car should stop for
 * that long.
 *
 * What the car says.  There is one buzzer, so the beeps are told apart by how
 * many there are, how long they last, and (for the last one) their pitch:
 *   LEFT                   one long beep
 *   RIGHT                  two short beeps
 *   FORWARD                three short beeps
 *   U-TURN                 two long beeps
 *   any other letter       one short LOWER beep -- the code was read, but its
 *                          letter is not one of the commands in barcode.h
 * A pass that does NOT decode stays silent: the cyan error light and the
 * console line report that one, and a beep for it would be indistinguishable
 * from a successful read at the far end of a track.
 *
 * Plain C: it only times the pattern and calls drv_buzzer.
 */

#ifndef BEEPER_H
#define BEEPER_H

#include <stdbool.h>
#include <stdint.h>

/* Lengths.  Rounded up to whole S1_PERIOD_MS (10 ms) steps in practice. */
#define BEEP_LONG_MS	600u	/* the single LEFT beep */
#define BEEP_SHORT_MS	180u	/* each beep when there are several */
#define BEEP_GAP_MS	150u	/* silence between them */

/* Pitches, in Hz.  0 means "the buzzer's usual tone" (S1_BUZZER_HZ in
   s1_pins.h).  The lower one is for a code that carries no command: lower and
   shorter than LEFT's one long beep, so the two cannot be confused.  Raise it
   towards the usual tone if it comes out too quiet -- a piezo is loudest near
   its resonance. */
#define BEEP_HZ_COMMAND	0u
#define BEEP_HZ_OTHER	1500u

void	beeper_init(void);	/* sets up the buzzer, silent */

/* Start a pattern: 'beeps' beeps of 'on_ms' each, BEEP_GAP_MS apart.
   Starts at once (the first beep is already sounding when this returns) and
   replaces any pattern still running. */
void	beeper_play(int beeps, uint32_t on_ms, uint32_t now_ms);

/* The same, at a pitch of your own (hz = 0: the usual tone). */
void	beeper_play_tone(int beeps, uint32_t on_ms, uint32_t hz,
			 uint32_t now_ms);

/* Call often -- every telemetry_task pass.  Does nothing when idle. */
void	beeper_step(uint32_t now_ms);

bool	beeper_busy(void);	/* a pattern is still sounding */

#endif /* BEEPER_H */
