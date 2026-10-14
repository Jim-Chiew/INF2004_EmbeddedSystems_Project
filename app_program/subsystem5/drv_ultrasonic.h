/* drv_ultrasonic.h - hardware abstraction so the logic can be host-tested. */
#ifndef DRV_ULTRASONIC_H
#define DRV_ULTRASONIC_H
#include <stdint.h>

#define HAL_ECHO_TIMEOUT  (-1)   /* no echo within timeout */
#define HAL_ECHO_STUCK    (-2)   /* ECHO already high before trigger */

int     hal_us_init(void);       /* 0 = ok, <0 = uT-Kernel error */
/* Fire one 10 us TRIG pulse and return echo pulse width in us,
 * or HAL_ECHO_TIMEOUT / HAL_ECHO_STUCK. Blocks the calling task
 * (not the CPU) until the echo ISR signals or timeout expires. */
int32_t hal_us_ping_us(uint32_t timeout_us);

int     hal_servo_init(void);
void    hal_servo_pulse_us(uint32_t pulse_us);

#endif
