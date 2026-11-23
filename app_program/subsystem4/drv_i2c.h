/*
 *	drv_i2c.h
 *	Subsystem 4 -- I2C1 master driver for the IMU (GP2 SDA, GP3 SCL).
 *	Called ONLY by imu_task (through imu.c): it owns I2C1 (TEAM.md 3).
 *
 *	Why not the kernel's I2C driver (device/i2c)?  It is switched off in
 *	config/config_device.h and is wired to GP6/GP7 for I2C1, and GP6 is a
 *	wheel encoder pin.  So, as the README asks, this subsystem brings its
 *	own small driver.  Polling, no interrupts; every wait has a timeout,
 *	so a missing sensor gives an error instead of a hang.
 *
 *	Plain C on purpose: drv_i2c.c uses Pico SDK register headers, which
 *	cannot be mixed with the kernel headers in one file (see app.h).
 */

#ifndef DRV_I2C_H
#define DRV_I2C_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#define I2C_DRV_ERR_NACK	(-1)	/* no answer (address not acknowledged) */
#define I2C_DRV_ERR_TIMEOUT	(-2)	/* the bus did not finish in time */

/* Reset I2C1, set the bus speed, give GP2/GP3 to I2C1 with pull-ups */
void	i2c_drv_init(uint32_t baud);

/* Return the number of bytes moved, or I2C_DRV_ERR_*.
   nostop = true keeps the bus for a repeated start (register read). */
int	i2c_drv_write(uint8_t addr, const uint8_t *src, size_t len, bool nostop);
int	i2c_drv_read(uint8_t addr, uint8_t *dst, size_t len, bool nostop);

#endif /* DRV_I2C_H */
