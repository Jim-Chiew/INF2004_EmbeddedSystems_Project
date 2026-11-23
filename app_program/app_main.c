/*
 *	app_main.c
 *	START HERE.  usermain() below is your program's entry point.
 *
 *	How micro T-Kernel starts your code
 *	-----------------------------------
 *	  reset -> kernel initialises itself -> creates the "initial task"
 *	        -> initial task calls usermain()
 *
 *	usermain() runs inside that initial task at priority 1 (the highest).
 *	On the dual-core build the OTHER core may start a new task at once, so
 *	usermain() works in two phases:
 *	  1. every subsystem creates its kernel objects  (subsystemN_init)
 *	  2. every subsystem starts its tasks             (subsystemN_start)
 *	and then puts the initial task to sleep forever.
 *	If usermain() RETURNS, the kernel shuts the whole system down.
 *
 *	The car: one subsystem per folder, each written as a state machine
 *	--------------------------------------------------------------------
 *	  subsystem1/  Comms       telemetry_task  core 0  COM state machine
 *	  subsystem2/  Motion      motion_task     core 1  STOPPED/DRIVE/WHEELS/ESTOP
 *	  subsystem3/  Navigation  nav_task        core 1  IDLE/CALIBRATING/FOLLOWING
 *	  subsystem4/  IMU         imu_task        core 1  SEARCH/RUN/CAL_SPIN/SETTLE/ZERO
 *	  subsystem5/  Barcode     barcode_task    core 1  WAIT_BAR/IN_PASS (+ 2 kHz sampler)
 *	  subsystem6/  Obstacle    obstacle_task   core 1  MONITOR/BLOCKED/FAULT/FAIL
 *	The kernel's own WiFi task (core 0) also runs subsystem 1's comms
 *	engine, comms_task(): only that task may touch lwIP and MQTT.
 *
 *	How they talk to each other (the interfaces, see TEAM.md section 5):
 *	  IF-01  S3 -> S2  message buffer mbf_motion     motion commands
 *	  IF-02  S4 -> S2  message buffer mbf_motion     emergency stop (ESTOP)
 *	  IF-03  S2 -> any shared data + mutex mtx_odom  encoder counts, speed
 *	  IF-04  S3 -> S4  message buffer mbf_imu_req    calibrate, reset lift/humps
 *	  IF-05  S4 -> S3  event flag flg_imu            overturned, lifted, cal done
 *	  IF-06  S4 -> any shared data + mutex mtx_imu   angles, humps, safety
 *	  IF-07  S1 -> S3  message buffer mbf_nav_cmd    START / STOP / RESET
 *	  IF-08  S3 -> any shared data + mutex mtx_nav   run state, line, junctions
 *	  IF-09  S5 -> S1  message buffer mbf_barcode    each barcode read
 *	  IF-10  S1 -> WiFi task  comms.c rings          telemetry frames, commands
 *	  IF-11  S6 -> S3  event flag flg_obs             obstacle ahead, sensor fault
 *	  IF-12  S3 -> S6  message buffer mbf_obs_req     clear the obstacle, scan
 *	  IF-13  S6 -> any shared data + mutex mtx_obs    distance, state, scan result
 *	The subsystem that RECEIVES or OWNS the data creates the object and
 *	publishes its ID in its header; the other side just uses it.
 *
 *	Output appears on the USB serial port (115200 8N1 in any terminal).
 */

#include "app.h"

/*
 * Create and start one task (declared in app.h for every subsystem).
 *   prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or 0 for "any core".
 */
EXPORT ID app_start_task(FP entry, PRI pri, UW prc, const char *name)
{
	T_CTSK	ctsk = {0};
	ID	tid;
	ER	er;

	ctsk.task    = entry;		/* function the task runs */
	ctsk.itskpri = pri;		/* initial priority (1 = highest) */
	ctsk.stksz   = TASK_STACK_SIZE;	/* kernel allocates the stack */
	ctsk.tskatr  = TA_HLNG		/* written in a high-level language (C) */
		     | TA_RNG3;		/* runs at user protection level */
#if TK_SUPPORT_SMP
	if (prc != 0) {
		ctsk.tskatr |= TA_ASSPRC;	/* honour the affinity below */
		ctsk.assprc  = prc;		/* which core(s) it may run on */
	}
#else
	(void)prc;			/* single-core build: no affinity */
#endif

	/* tk_cre_tsk: create the task.  It starts in the DORMANT state. */
	tid = tk_cre_tsk(&ctsk);
	if (tid < E_OK) {
		CHECK(tid, name);
		return tid;
	}

	/* tk_sta_tsk: DORMANT -> READY.  The int is passed as stacd. */
	er = tk_sta_tsk(tid, 0);
	CHECK(er, name);
	return tid;
}

EXPORT INT usermain(void)
{
	T_RVER	ver;

	/* tk_ref_ver: ask the kernel who it is */
	tk_ref_ver(&ver);
	tm_printf((UB*)"\n=== Robo car on micro T-Kernel, Pico W"
		  " (spec 0x%04x, product 0x%04x) built %s %s ===\n",
		  ver.spver, ver.prver, __DATE__, __TIME__);
#if TK_SUPPORT_SMP
	tm_printf((UB*)"dual-core (SMP) build, usermain on core %d\n",
		  tk_get_prc() - 1);	/* tk_get_prc() counts from 1 */
#else
	tm_printf((UB*)"single-core build\n");
#endif

	/* Phase 1: every subsystem creates its kernel objects */
	CHECK(subsystem1_init(), "subsystem1_init");
	CHECK(subsystem2_init(), "subsystem2_init");
	CHECK(subsystem3_init(), "subsystem3_init");
	CHECK(subsystem4_init(), "subsystem4_init");
	CHECK(subsystem5_init(), "subsystem5_init");
	CHECK(subsystem6_init(), "subsystem6_init");

	/* Phase 2: every subsystem starts its tasks */
	CHECK(subsystem1_start(), "subsystem1_start");
	CHECK(subsystem2_start(), "subsystem2_start");
	CHECK(subsystem3_start(), "subsystem3_start");
	CHECK(subsystem4_start(), "subsystem4_start");
	CHECK(subsystem5_start(), "subsystem5_start");
	CHECK(subsystem6_start(), "subsystem6_start");

	tm_printf((UB*)"all subsystems started\n\n");

#if APP_TEST
	/* Test build (make TEST=1): run every subsystem's tests in turn,
	   here in the initial task, while the system is running.  Tests may
	   call tk_dly_tsk() to let the other tasks work for a while. */
	tm_printf((UB*)"[TEST] === test run start ===\n");
	subsystem1_test();
	subsystem2_test();
	subsystem3_test();
	subsystem4_test();
	subsystem5_test();
	subsystem6_test();
	tm_printf((UB*)"[TEST] === test run end ===\n");
#endif

	/* Sleep forever.  Returning from usermain() would shut down. */
	tk_slp_tsk(TMO_FEVR);
	return 0;
}
