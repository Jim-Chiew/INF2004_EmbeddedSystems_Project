/*
 *	drv_i2c.c
 *	Subsystem 4 -- I2C1 master driver.  See drv_i2c.h.
 *
 *	Pico SDK side: this file must NOT include app.h or any tk/ header.
 *	The register sequence follows the Pico SDK's hardware_i2c (i2c_init,
 *	i2c_write_blocking, i2c_read_blocking), with a timeout on every wait.
 */

#include "drv_i2c.h"
#include "s4_pins.h"
#include "hardware/structs/i2c.h"
#include "hardware/structs/resets.h"
#include "hardware/gpio.h"

#define CLK_SYS_HZ	125000000u	/* set by the kernel (cpu_clock.c) */
#define BYTE_TIMEOUT_US	2000u		/* one byte takes ~25 us at 400 kHz */

extern uint64_t time_us_64(void);

static i2c_hw_t * const hw = i2c1_hw;
static bool restart_on_next = false;	/* last transfer ended with nostop */

void i2c_drv_init(uint32_t baud)
{
	uint32_t period, lcnt, hcnt, sda_hold;

	/* Reset I2C1 and wait until it is out of reset again */
	hw_set_bits(&resets_hw->reset, RESETS_RESET_I2C1_BITS);
	hw_clear_bits(&resets_hw->reset, RESETS_RESET_I2C1_BITS);
	while (!(resets_hw->reset_done & RESETS_RESET_I2C1_BITS)) {
	}

	hw->enable = 0;
	/* Master only, fast mode, repeated start allowed, and TX_EMPTY only
	   when a byte has really gone out (used to pace the writes below) */
	hw->con = I2C_IC_CON_SPEED_VALUE_FAST << I2C_IC_CON_SPEED_LSB |
		  I2C_IC_CON_MASTER_MODE_BITS |
		  I2C_IC_CON_IC_SLAVE_DISABLE_BITS |
		  I2C_IC_CON_IC_RESTART_EN_BITS |
		  I2C_IC_CON_TX_EMPTY_CTRL_BITS;
	hw->tx_tl = 0;
	hw->rx_tl = 0;
	hw->dma_cr = 0;

	/* SCL high/low counts: 3/5 low, 2/5 high, as the SDK does */
	period = (CLK_SYS_HZ + baud / 2u) / baud;
	lcnt = period * 3u / 5u;
	hcnt = period - lcnt;
	hw->fs_scl_hcnt = hcnt;
	hw->fs_scl_lcnt = lcnt;
	hw->fs_spklen = (lcnt < 16u) ? 1u : lcnt / 16u;

	/* SDA hold time: 300 ns (below 1 MHz) */
	sda_hold = (CLK_SYS_HZ * 3u) / 10000000u + 1u;
	hw->sda_hold = (hw->sda_hold & ~I2C_IC_SDA_HOLD_IC_SDA_TX_HOLD_BITS) |
		       (sda_hold << I2C_IC_SDA_HOLD_IC_SDA_TX_HOLD_LSB);

	hw->enable = 1;
	restart_on_next = false;

	gpio_set_function(S4_I2C_SDA_PIN, GPIO_FUNC_I2C);
	gpio_set_function(S4_I2C_SCL_PIN, GPIO_FUNC_I2C);
	gpio_pull_up(S4_I2C_SDA_PIN);
	gpio_pull_up(S4_I2C_SCL_PIN);
}

static void set_target(uint8_t addr)
{
	hw->enable = 0;
	hw->tar = addr;
	hw->enable = 1;
}

static bool expired(uint64_t deadline)
{
	return time_us_64() > deadline;
}

int i2c_drv_write(uint8_t addr, const uint8_t *src, size_t len, bool nostop)
{
	bool		abort = false, timeout = false;
	uint32_t	reason = 0;
	size_t		i;

	if (len == 0) {
		return 0;
	}
	set_target(addr);

	for (i = 0; i < len; i++) {
		bool first = (i == 0), last = (i == len - 1);
		uint64_t deadline = time_us_64() + BYTE_TIMEOUT_US;

		hw->data_cmd = (uint32_t)(first && restart_on_next) << I2C_IC_DATA_CMD_RESTART_LSB |
			       (uint32_t)(last && !nostop) << I2C_IC_DATA_CMD_STOP_LSB |
			       src[i];

		/* Wait until the byte has left the shift register */
		while (!(hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS)) {
			if (expired(deadline)) { timeout = true; break; }
		}
		if (!timeout) {
			reason = hw->tx_abrt_source;
			if (reason) {
				(void)hw->clr_tx_abrt;	/* clear on read */
				abort = true;
			}
			if (abort || (last && !nostop)) {
				/* Wait for the STOP (sent by hardware on abort too) */
				deadline = time_us_64() + BYTE_TIMEOUT_US;
				while (!(hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_STOP_DET_BITS)) {
					if (expired(deadline)) { timeout = true; break; }
				}
				if (!timeout) {
					(void)hw->clr_stop_det;
				}
			}
		}
		if (abort || timeout) {
			break;
		}
	}

	restart_on_next = nostop && !abort && !timeout;
	if (timeout) return I2C_DRV_ERR_TIMEOUT;
	if (abort)   return I2C_DRV_ERR_NACK;
	return (int)i;
}

int i2c_drv_read(uint8_t addr, uint8_t *dst, size_t len, bool nostop)
{
	bool	abort = false, timeout = false;
	size_t	i;

	if (len == 0) {
		return 0;
	}
	set_target(addr);

	for (i = 0; i < len; i++) {
		bool first = (i == 0), last = (i == len - 1);
		uint64_t deadline = time_us_64() + BYTE_TIMEOUT_US;

		while (hw->txflr >= 16u) {		/* room in the TX FIFO */
			if (expired(deadline)) { timeout = true; break; }
		}
		if (timeout) break;

		hw->data_cmd = (uint32_t)(first && restart_on_next) << I2C_IC_DATA_CMD_RESTART_LSB |
			       (uint32_t)(last && !nostop) << I2C_IC_DATA_CMD_STOP_LSB |
			       I2C_IC_DATA_CMD_CMD_BITS;	/* read */

		do {
			if (hw->tx_abrt_source) {
				(void)hw->clr_tx_abrt;
				abort = true;
			}
			if (expired(deadline)) {
				timeout = true;
			}
		} while (!abort && !timeout && hw->rxflr == 0u);

		if (abort || timeout) {
			break;
		}
		dst[i] = (uint8_t)hw->data_cmd;
	}

	restart_on_next = nostop && !abort && !timeout;
	if (timeout) return I2C_DRV_ERR_TIMEOUT;
	if (abort)   return I2C_DRV_ERR_NACK;
	return (int)i;
}
