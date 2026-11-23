/*
 * drv_ultrasonic.c
 * Subsystem 6 -- hardware layer. See drv_ultrasonic.h.
 *
 *   TRIG / ECHO : BSP gpio_set_pin / gpio_set_val, plus a GPIO edge interrupt
 *                 registered with tk_def_int
 *   timestamp   : the RP2040 TIMER's raw low word (1 MHz), read in the handler
 *   servo       : BSP pwm_* (the BSP has no clock-divider call, so the DIV
 *                 register is written here)
 *
 * Kernel and BSP headers only, no Pico SDK headers (the "header rule" in
 * TEAM.md): the kernel's sysdef.h and the SDK's headers define the same
 * register names differently.
 *
 * Measuring a pulse needs microsecond timing, which no task can be paced to
 * (the kernel tick is 1 ms), so both ECHO edges raise an interrupt. The
 * handler does the least it can: take the time, clear the edge latch, and on
 * the falling edge wake the waiting task through an event flag.
 */

#include <tk/tkernel.h>
#include <bsp/libbsp.h>
#include "drv_ultrasonic.h"
#include "ultrasonic.h"

#define INTNO_IO_BANK0     13          /* RP2040 IO_IRQ_BANK0 */

/* IO_BANK0 registers for one pin. Pins are in groups of 8, 4 bits each.
   INTR is the raw edge latch, shared by both cores and cleared by writing 1s.
   The INTE (enable) registers come one set per core: the interrupt is taken by
   the core whose NVIC EnableInt() was called on, so the right set is the one
   belonging to the core that calls hal_us_init() -- obstacle_task, core 1. */
#define IO_INTR(n)         (IO_BANK0_BASE + 0x0F0 + 4 * ((n) >> 3))
#define IO_PROC0_INTE(n)   (IO_BANK0_BASE + 0x100 + 4 * ((n) >> 3))
#define IO_PROC1_INTE(n)   (IO_BANK0_BASE + 0x130 + 4 * ((n) >> 3))
#define EDGE_LOW(n)        (1UL << (4 * ((n) & 7) + 2))
#define EDGE_HIGH(n)       (1UL << (4 * ((n) & 7) + 3))
#define ECHO_EDGES         (EDGE_LOW(US_ECHO_PIN) | EDGE_HIGH(US_ECHO_PIN))

#define FLG_ECHO_DONE      0x01

static ID           flg_echo;
static volatile UW  t_rise, t_fall;
static volatile UW  got_rise;

/* ECHO edge interrupt: take the time, acknowledge, wake the waiting task. */
static void echo_inthdr(UINT intno)
{
    UW now = in_w(TIMER_TIMERAWL);
    UW st  = in_w(IO_INTR(US_ECHO_PIN)) & ECHO_EDGES;

    out_w(IO_INTR(US_ECHO_PIN), st);               /* write 1 to clear the latches */
    if (st & EDGE_HIGH(US_ECHO_PIN)) {
        t_rise = now;
        got_rise = 1;
    }
    if ((st & EDGE_LOW(US_ECHO_PIN)) && got_rise) {
        t_fall = now;
        tk_set_flg(flg_echo, FLG_ECHO_DONE);
    }
    ClearInt(intno);
}

/* The ECHO interrupt enable register of the core we are running on */
static UW echo_inte_reg(void)
{
#if TK_SUPPORT_SMP
    /* tk_get_prc() counts from 1: 1 = core 0, 2 = core 1 */
    if (tk_get_prc() == 2) return IO_PROC1_INTE(US_ECHO_PIN);
#endif
    return IO_PROC0_INTE(US_ECHO_PIN);
}

int hal_us_init(void)
{
    static const T_CFLG cflg = { .flgatr = TA_TFIFO | TA_WSGL, .iflgptn = 0 };
    static const T_DINT dint = { .intatr = TA_HLNG, .inthdr = (FP)echo_inthdr };
    ER er;

    flg_echo = tk_cre_flg(&cflg);
    if (flg_echo < E_OK) return flg_echo;

    gpio_set_pin(US_TRIG_PIN, GPIO_MODE_OUT);
    gpio_set_val(US_TRIG_PIN, 0);
    gpio_set_pin(US_ECHO_PIN, GPIO_MODE_IN);

    er = tk_def_int(INTNO_IO_BANK0, &dint);
    if (er < E_OK) return er;

    out_w(IO_INTR(US_ECHO_PIN), ECHO_EDGES);       /* drop any stale latch */
    set_w(echo_inte_reg(), ECHO_EDGES);            /* enable both ECHO edges */
    ClearInt(INTNO_IO_BANK0);
    EnableInt(INTNO_IO_BANK0, US_ECHO_INT_PRI);    /* on this task's core */

    return 0;
}

int32_t hal_us_ping_us(uint32_t timeout_us)
{
    UINT ptn;
    ER   er;

    /* Some modules hold ECHO high for about 38 ms after a ping into open
       space. If that is still going on, wait it out (bounded) instead of
       calling it a fault; only an ECHO that never falls is "stuck". */
    for (UINT w = 0; gpio_get_val(US_ECHO_PIN); w++) {
        if (w >= US_ECHO_RELEASE_MS) return HAL_ECHO_STUCK;
        tk_dly_tsk(1);
    }

    tk_clr_flg(flg_echo, 0);
    got_rise = 0;

    gpio_set_val(US_TRIG_PIN, 1);
    {   /* 10 us is far below one kernel tick: spin on the 1 MHz timer */
        UW t0 = in_w(TIMER_TIMERAWL);
        while ((in_w(TIMER_TIMERAWL) - t0) < US_TRIG_PULSE_US) { }
    }
    gpio_set_val(US_TRIG_PIN, 0);

    er = tk_wai_flg(flg_echo, FLG_ECHO_DONE, TWF_ORW | TWF_CLR, &ptn,
                    (timeout_us + 999) / 1000);
    if (er == E_TMOUT) return got_rise ? HAL_ECHO_TIMEOUT   /* rose, never fell: clear */
                                       : HAL_ECHO_NONE;     /* never rose: no sensor */
    if (er < E_OK)     return HAL_ECHO_STUCK;

    return (int32_t)(t_fall - t_rise);
}

/* ---- Servo: GP15 = PWM slice 7 channel B. 125 MHz / 125 = 1 MHz, TOP = 20 ms ---- */
#define SERVO_SLICE   (((US_SERVO_PIN) >> 1) & 7)
#define SERVO_DIV_REG (PWM_BASE + PWM_CHx_DIV + (SERVO_SLICE) * 0x14)

int hal_servo_init(void)
{
    /* The kernel releases the PWM block from reset only when its own physical
       timer uses it (USE_PTMR, off in this project), and a block in reset
       ignores every write. Subsystem 2's drv_motor.c releases it for the
       motors, but the two tasks start at the same time, so do it here too
       (clearing a bit that is already clear changes nothing). The TEAM.md
       "reset rule" has the whole story. */
    clr_w(RESETS_RESET, RESETS_RESET_PWM);
    while (!(in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM)) { }

    pwm_set_enabled(US_SERVO_PIN, FALSE);
    pwm_set_pin(US_SERVO_PIN);
    out_w(SERVO_DIV_REG, (UW)(CLK_PERI_FREQ / 1000000UL) << 4);  /* 8.4 fixed point */
    pwm_set_wrap(US_SERVO_PIN, 19999);                            /* 50 Hz */
    pwm_set_cc(US_SERVO_PIN, 1500);                               /* centre until told */
    pwm_set_enabled(US_SERVO_PIN, TRUE);

    return 0;
}

void hal_servo_pulse_us(uint32_t pulse_us)
{
    pwm_set_cc(US_SERVO_PIN, pulse_us);
}

/* end of file */
