/* drv_ultrasonic.c - hardware layer for the mtk3smp-rp2040 repo (no Pico SDK).
 *   TRIG/ECHO : BSP gpio_set_pin/gpio_set_val + GPIO edge IRQ via tk_def_int
 *   timestamp : RP2040 TIMER raw low word (1 MHz) read in the ISR
 *   servo     : BSP pwm_* (the BSP has no divider call, so DIV is written here)
 * Ownership (PORT_RP2040.md safety rule 2/6): IO_BANK0 IRQ, PWM slice and TIMER
 * reads are used only from the Obstacle task / its ISR, on processor 1. */

#include <tk/tkernel.h>
#include <bsp/libbsp.h>
#include "drv_ultrasonic.h"
#include "us_config.h"

#define INTNO_IO_BANK0     13          /* RP2040 IO_IRQ_BANK0 */
#define IO_INTR(n)         (IO_BANK0_BASE + 0x0F0 + 4 * ((n) >> 3))
#define IO_PROC0_INTE(n)   (IO_BANK0_BASE + 0x100 + 4 * ((n) >> 3))
#define IO_PROC0_INTS(n)   (IO_BANK0_BASE + 0x120 + 4 * ((n) >> 3))
#define EDGE_LOW(n)        (1UL << (4 * ((n) & 7) + 2))
#define EDGE_HIGH(n)       (1UL << (4 * ((n) & 7) + 3))
#define ECHO_EDGES         (EDGE_LOW(US_ECHO_GPIO) | EDGE_HIGH(US_ECHO_GPIO))

#define FLG_ECHO_DONE      0x01
#define ECHO_INT_PRI       2           /* below kernel-critical levels */

static ID           flg_echo;
static volatile UW  t_rise, t_fall;
static volatile UW  got_rise;

/* ECHO edge interrupt: timestamp in us, ack, wake the waiting task. */
static void echo_inthdr(UINT intno)
{
    UW now = in_w(TIMER_TIMERAWL);
    UW st  = in_w(IO_PROC0_INTS(US_ECHO_GPIO)) & ECHO_EDGES;

    out_w(IO_INTR(US_ECHO_GPIO), st);              /* write-1-to-clear edge latches */
    if (st & EDGE_HIGH(US_ECHO_GPIO)) { 
        t_rise = now; 
        got_rise = 1; 
    }
    if ((st & EDGE_LOW(US_ECHO_GPIO)) && got_rise) {
        t_fall = now;
        tk_set_flg(flg_echo, FLG_ECHO_DONE);
    }
    ClearInt(intno);
}

int hal_us_init(void)
{
    static const T_CFLG cflg = { .flgatr = TA_TFIFO | TA_WSGL, .iflgptn = 0 };
    static const T_DINT dint = { .intatr = TA_HLNG, .inthdr = echo_inthdr };
    ER er;

    flg_echo = tk_cre_flg(&cflg);
    if (flg_echo < E_OK) return flg_echo;

    gpio_set_pin(US_TRIG_GPIO, GPIO_MODE_OUT);
    gpio_set_val(US_TRIG_GPIO, 0);
    gpio_set_pin(US_ECHO_GPIO, GPIO_MODE_IN);

    er = tk_def_int(INTNO_IO_BANK0, &dint);
    if (er < E_OK) return er;

    out_w(IO_INTR(US_ECHO_GPIO), ECHO_EDGES);       /* drop stale latches */
    set_w(IO_PROC0_INTE(US_ECHO_GPIO), ECHO_EDGES); /* enable rise + fall for ECHO */
    ClearInt(INTNO_IO_BANK0);
    EnableInt(INTNO_IO_BANK0, ECHO_INT_PRI);

    return 0;
}

int32_t hal_us_ping_us(uint32_t timeout_us)
{
    UINT ptn;
    ER   er;

    if (gpio_get_val(US_ECHO_GPIO)) return HAL_ECHO_STUCK;

    tk_clr_flg(flg_echo, 0);
    got_rise = 0;

    gpio_set_val(US_TRIG_GPIO, 1);
    {   /* 10 us is far below one kernel tick: spin on the 1 MHz timer */
        UW t0 = in_w(TIMER_TIMERAWL);
        while ((in_w(TIMER_TIMERAWL) - t0) < US_TRIG_PULSE_US) { }
    }
    gpio_set_val(US_TRIG_GPIO, 0);

    er = tk_wai_flg(flg_echo, FLG_ECHO_DONE, TWF_ORW | TWF_CLR, &ptn,
                    (timeout_us + 999) / 1000);
    if (er == E_TMOUT) return HAL_ECHO_TIMEOUT;
    if (er < E_OK)     return HAL_ECHO_STUCK;

    return (int32_t)(t_fall - t_rise);
}

/* ---- Servo: GP15 = PWM slice 7 channel B. 125 MHz / 125 = 1 MHz, TOP = 20 ms ---- */
#define SERVO_SLICE   (((SERVO_GPIO) >> 1) & 7)
#define SERVO_DIV_REG (PWM_BASE + PWM_CHx_DIV + (SERVO_SLICE) * 0x14)

int hal_servo_init(void)
{
    pwm_set_enabled(SERVO_GPIO, FALSE);
    pwm_set_pin(SERVO_GPIO);
    out_w(SERVO_DIV_REG, (UW)(CLK_PERI_FREQ / 1000000UL) << 4);  /* 8.4 fixed point */
    pwm_set_wrap(SERVO_GPIO, 19999);                              /* 50 Hz */
    pwm_set_cc(SERVO_GPIO, 1500);                                 /* centre until told */
    pwm_set_enabled(SERVO_GPIO, TRUE);

    return 0;
}

void hal_servo_pulse_us(uint32_t pulse_us)
{
    pwm_set_cc(SERVO_GPIO, pulse_us);
}