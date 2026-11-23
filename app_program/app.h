/*
 *	app.h
 *	Definitions shared by the whole team.  Changed only by team agreement.
 *
 *	Anything that belongs to ONE subsystem goes in that subsystem's own
 *	header instead (app_program/subsystemN/subsystemN.h).
 *
 *	The car (see TEAM.md):
 *	  S1  comms       telemetry to the MQTT broker, START/STOP/RESET from it
 *	  S2  motion      motors (PWM) and wheel encoders (PIO)
 *	  S3  navigation  line following, buttons, the run state machine
 *	  S4  imu         tilt, heading, humps, lift/overturn safety stop
 *	  S5  barcode     Code 39 barcodes under the car
 *	  S6  obstacle    HC-SR04 distance ahead on a servo: stops the car
 */

#ifndef APP_H
#define APP_H

/* size_t: use the C library's one (unsigned), not T-Kernel's own (signed,
   tk/syslib.h).  The two cannot both exist in one file, and the subsystems
   need <string.h> and the Pico SDK register headers next to the kernel
   headers.  This must come before tk/tkernel.h. */
#include <stddef.h>
#define PROHIBIT_DEF_SIZE_T

#include <tk/tkernel.h>		/* micro T-Kernel API (tk_xxx_yyy) */
#include <tm/tmonitor.h>	/* T-Monitor API: tm_printf() console output */
#include <bsp/libbsp.h>		/* Board helpers: gpio_set_pin/val, pwm_... */
#include <stdint.h>
#include <stdbool.h>

/* The car needs the radio build: the WiFi port provides time_us_64() and the
   Pico SDK PIO/GPIO code the wheel encoders use.  "make" builds it (default). */
#if !defined(TM_WIFI_CYW43) || !TM_WIFI_CYW43
#error "The car needs WIFI=cyw43 (the default). Build with plain 'make'."
#endif

/* ---------------------------------------------------------------------
 *  Pins: free for projects are GP0..GP22 and GP26..GP28.  GP0/GP1 are
 *  taken by UART0 when built with "make CRASH_REPORT=1" (see README).
 *  Never use GP23/24/25/29: they are wired to the Pico W radio.
 *  Claim every pin in TEAM.md section 2 before using it.  Each subsystem
 *  defines its own pins in its own header.
 * --------------------------------------------------------------------- */

/* ---------------------------------------------------------------------
 *  Task priorities: 1 is the highest; the kernel's USB console (3) and
 *  WiFi (4) tasks sit above every application task.  Each subsystem
 *  has its own band (TEAM.md section 4), defined in its own header.
 *
 *  Cores: the kernel's WiFi task runs on core 0 every 2 ms, so the tasks
 *  that steer the car (S2, S3, S4, S5, S6) run on core 1 and only reporting
 *  (S1) shares core 0 with the radio.
 * --------------------------------------------------------------------- */
#define TASK_STACK_SIZE		2048	/* bytes; tm_printf needs ~1 KB */
#define APP_CORE_RADIO		TP_PRC1	/* core 0: WiFi, USB console, S1 */
#define APP_CORE_CONTROL	TP_PRC2	/* core 1: S2, S3, S4, S5 */

/* Create and start a task.  prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or
   0 for "any core".  Defined in app_main.c. */
IMPORT ID app_start_task(FP entry, PRI pri, UW prc, const char *name);

/* Time since power-up, from the RP2040's 1 MHz hardware timer.  Any task
   (and any interrupt handler) may call these.  Provided by the WiFi port
   (lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_compat.c). */
extern uint64_t time_us_64(void);
static inline uint32_t app_now_us(void) { return (uint32_t)time_us_64(); }
static inline uint32_t app_now_ms(void) { return (uint32_t)(time_us_64() / 1000u); }

/* ---------------------------------------------------------------------
 *  Every subsystem provides these.  usermain() calls ALL the _init()
 *  functions first and only then ALL the _start() functions, so every
 *  kernel object exists before any task can use it.
 *    subsystemN_init()   create this subsystem's kernel objects
 *    subsystemN_start()  create and start this subsystem's tasks
 *    subsystemN_test()   on-target tests, only in test builds (make TEST=1)
 * --------------------------------------------------------------------- */
IMPORT ER subsystem1_init(void);
IMPORT ER subsystem2_init(void);
IMPORT ER subsystem3_init(void);
IMPORT ER subsystem4_init(void);
IMPORT ER subsystem5_init(void);
IMPORT ER subsystem6_init(void);

IMPORT ER subsystem1_start(void);
IMPORT ER subsystem2_start(void);
IMPORT ER subsystem3_start(void);
IMPORT ER subsystem4_start(void);
IMPORT ER subsystem5_start(void);
IMPORT ER subsystem6_start(void);

#if APP_TEST
IMPORT void subsystem1_test(void);
IMPORT void subsystem2_test(void);
IMPORT void subsystem3_test(void);
IMPORT void subsystem4_test(void);
IMPORT void subsystem5_test(void);
IMPORT void subsystem6_test(void);
#endif

/* Print a readable message if a kernel call failed (negative error code) */
#define CHECK(er, what)							\
	do {								\
		INT check_er_ = (INT)(er);	/* evaluate once */	\
		if (check_er_ < E_OK) {					\
			tm_printf((UB*)"ERROR %s: %d\n", (what), check_er_); \
		}							\
	} while (0)

#endif /* APP_H */
