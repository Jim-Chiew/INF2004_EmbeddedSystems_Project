/*
 *	subsystem1.h
 *	Subsystem 1 -- Comms: WiFi / MQTT telemetry and remote commands.
 *	Student ID: <student ID>.
 *
 *	This header is subsystem 1's PUBLIC interface.  The comms engine
 *	(comms.c, see comms.h and README.md in this folder) is split in two:
 *	  - comms_task() runs inside the kernel's WiFi task (cyw43_utk_task),
 *	    the only task allowed to touch lwIP / MQTT (TEAM.md section 3);
 *	  - telemetry_task (this subsystem's own task) feeds it: every 200 ms
 *	    a 10-byte telemetry frame, each barcode text, each obstacle
 *	    measurement, and it hands the
 *	    START / STOP / RESET commands it receives to navigation (IF-07).
 *	It also prints the status line on the USB console every 250 ms.
 *
 *	State machine: the COM state (com_state.c), driven by comms_task():
 *	  DISCONNECTED --WiFi + MQTT up--> SYNC (sending stored frames)
 *	  SYNC --queue empty--> CONNECTED --link lost--> DISCONNECTED
 *	The status LED on GP19 shows it (on = CONNECTED, 2 Hz = DISCONNECTED).
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"
#include "s1_pins.h"

#define S1_PRI_TELEMETRY	18	/* priority band 18-19 (TEAM.md) */
#define S1_PERIOD_MS		10	/* LED, commands, barcode texts */
#define S1_TELEMETRY_MS		200	/* one frame 5 times a second */

/* What goes to the broker.  The DECODED BARCODE TEXT (robocar/car1/barcode,
   one message per code read, e.g. "A LEFT") is always published.  These two
   switch the rest on or off:
     S1_PUBLISH_TELEMETRY  1 = the 10-byte status frame on
                               robocar/car1/telemetry every S1_TELEMETRY_MS
     S1_PUBLISH_OBSTACLE   1 = one line per obstacle the car stops for, on
                               robocar/car1/obstacle, once subsystem 6 has
                               scanned it: "d=142 w=184 n=133 ... s=L"
   As set: events only -- each barcode read and each obstacle, nothing
   continuous.  Everything else is unchanged -- the frame is still built (so
   the error lights and the USB status line work as before), commands from
   the broker (START / STOP / RESET) are still received, and setting
   S1_PUBLISH_TELEMETRY back to 1 restores the frames. */
#define S1_PUBLISH_TELEMETRY	0
#define S1_PUBLISH_OBSTACLE	1

/* 1 = one line per hump a wheel rolls over, on robocar/car1/hump:
   "h=12 s=R t=5.7" (height mm, wheel, peak tilt degrees; comms.h).
   Only DURING A RUN: a hump measured while the car is idle -- being picked
   up, put down, or the block test -- goes to the USB console only. */
#define S1_PUBLISH_HUMP		1
#define S1_STATUS_MS		250	/* status line on the USB console */

/* ---------------------------------------------------------------------
 *  IF-09 (S5 -> S1): each barcode pass.  Message buffer mbf_barcode,
 *  owned by S1.  One BARCODE_MSG per pass, sent with tk_snd_mbf().
 * --------------------------------------------------------------------- */
#define S1_BC_TEXT_MAX	12		/* decoded text only; COMMS_BARCODE_MAX
					   leaves room for " FORWARD" after it */

typedef enum {
	S1_BC_NONE = 0,			/* the code carries no command */
	S1_BC_LEFT,
	S1_BC_RIGHT,
	S1_BC_FORWARD,
	S1_BC_UTURN			/* not in the telemetry frame: its 2-bit
					   bc_dir field is full, see data_layout.h */
} S1_BC_COMMAND;

typedef struct {
	UB	valid;			/* 1 = decoded OK, 0 = the pass failed */
	UB	command;		/* S1_BC_COMMAND */
	UB	reversed;		/* 1 = scanned backwards */
	UB	pad;
	char	text[S1_BC_TEXT_MAX + 1];	/* data between the *s, '\0' ended */
} BARCODE_MSG;

IMPORT ID	mbf_barcode;

IMPORT ID	tid_telemetry;

/* Private to subsystem 1, shared between its own .c files */
IMPORT void	telemetry_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM1_H */
