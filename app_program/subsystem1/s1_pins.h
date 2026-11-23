/*
 *	s1_pins.h
 *	Subsystem 1 -- pins.  Plain #defines only (included by comms_port_utk.c,
 *	which must stay free of the kernel's register definitions).
 */

#ifndef S1_PINS_H
#define S1_PINS_H

/* COM status LED (FR-COM-05/06): on = CONNECTED, 2 Hz blink = DISCONNECTED.
   The Pico W's own LED is wired to the radio, which no application task may
   touch (TEAM.md section 3), so an external LED + 330 R to GND is used. */
#define S1_STATUS_LED_PIN	19

/* Error indicator: the Robo Pico's two WS2812 RGB LEDs, chained on GP18.
   Off = no error; a colour per error (see task_telemetry.c, TEAM.md). */
#define S1_RGB_LED_PIN		18
#define S1_RGB_LED_COUNT	2
#define S1_RGB_BRIGHTNESS	40	/* 0..255; full brightness is blinding */

/* Buzzer: the Robo Pico's on-board piezo.  It needs a square wave, so it is
   driven by PWM (slice 3 channel A) at this tone -- near the resonance of a
   small piezo, where it is loudest.  An external buzzer on another free pin
   works too: change only this number (and the TEAM.md pin table). */
#define S1_BUZZER_PIN		22
#define S1_BUZZER_HZ		2700

#endif /* S1_PINS_H */
