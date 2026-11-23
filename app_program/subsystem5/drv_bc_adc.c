/*
 *	drv_bc_adc.c
 *	Subsystem 5 -- ADC and timer alarm 3.  See drv_bc_adc.h.
 *
 *	Pico SDK side: this file must NOT include app.h or any tk/ header.
 *	Timer alarm 3 is free: the kernel ticks with SysTick and the WiFi port
 *	only READS the timer (time_us_64), it sets no alarms.
 */

#include "drv_bc_adc.h"
#include "barcode.h"
#include "hardware/structs/adc.h"
#include "hardware/structs/timer.h"
#include "hardware/gpio.h"

static uint32_t period = BARCODE_SAMPLE_US;
static uint32_t next_at;

void bc_adc_init(void)
{
	/* Like the SDK's adc_gpio_init(): no digital function, no pulls,
	   input buffer off, so the pin is a clean analog input */
	gpio_set_function(BARCODE_AO_PIN, GPIO_FUNC_NULL);
	gpio_disable_pulls(BARCODE_AO_PIN);
	gpio_set_input_enabled(BARCODE_AO_PIN, false);

	adc_hw->fcs = 0;			/* no FIFO: we read RESULT */
	adc_hw->div = 0;			/* no free-running conversions */
	adc_hw->cs = ADC_CS_EN_BITS;
	while (!(adc_hw->cs & ADC_CS_READY_BITS)) {
	}
}

uint16_t bc_adc_read(void)
{
	adc_hw->cs = ADC_CS_EN_BITS |
		     ((uint32_t)BARCODE_ADC_INPUT << ADC_CS_AINSEL_LSB) |
		     ADC_CS_START_ONCE_BITS;
	while (!(adc_hw->cs & ADC_CS_READY_BITS)) {
	}
	return (uint16_t)adc_hw->result;
}

uint32_t bc_time_us(void)
{
	return timer_hw->timerawl;
}

void bc_alarm_start(uint32_t period_us)
{
	period = period_us;
	next_at = timer_hw->timerawl + period;
	timer_hw->intr = 1u << BC_ALARM_NUM;		/* clear an old one */
	timer_hw->alarm[BC_ALARM_NUM] = next_at;	/* writing arms it */
	hw_set_bits(&timer_hw->inte, 1u << BC_ALARM_NUM);
}

void bc_alarm_stop(void)
{
	hw_clear_bits(&timer_hw->inte, 1u << BC_ALARM_NUM);
	timer_hw->armed = 1u << BC_ALARM_NUM;		/* disarm */
	timer_hw->intr = 1u << BC_ALARM_NUM;
}

void bc_alarm_next(void)
{
	uint32_t now;

	timer_hw->intr = 1u << BC_ALARM_NUM;		/* acknowledge */
	next_at += period;
	now = timer_hw->timerawl;
	if ((int32_t)(next_at - now) < 20) {
		next_at = now + period;		/* fell behind: skip, no burst */
	}
	timer_hw->alarm[BC_ALARM_NUM] = next_at;
}
