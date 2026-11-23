/*
 *	drv_bc_adc.h
 *	Subsystem 5 -- the hardware under the barcode reader: the ADC (GP28)
 *	and RP2040 timer alarm 3, which paces the 2 kHz sampling.
 *	Called ONLY by subsystem 5 (barcode_task and its interrupt handler);
 *	S5 owns the ADC (TEAM.md section 3).
 *
 *	Why the ADC registers directly and not the kernel's ADC device
 *	(device/adc)?  A 2 kHz sample from an interrupt handler cannot wait on
 *	the device driver's event flag.  So nobody else may open "adca"
 *	either: two users of one ADC would change each other's input.
 *
 *	Plain C on purpose: drv_bc_adc.c uses Pico SDK register headers, which
 *	cannot be mixed with the kernel headers in one file (see app.h).
 */

#ifndef DRV_BC_ADC_H
#define DRV_BC_ADC_H

#include <stdint.h>

#define BC_ALARM_NUM	3u	/* timer alarm 3 -> interrupt TIMER_IRQ_3 */
#define BC_ALARM_IRQ	3u	/* NVIC interrupt number of TIMER_IRQ_3 */

void		bc_adc_init(void);		/* GP28 as analog input, ADC on */
uint16_t	bc_adc_read(void);		/* one conversion, ~2 us */
uint32_t	bc_time_us(void);		/* hardware timer, low 32 bits */

void		bc_alarm_start(uint32_t period_us);	/* first alarm + its interrupt */
void		bc_alarm_stop(void);
void		bc_alarm_next(void);		/* in the handler: clear + re-arm */

#endif /* DRV_BC_ADC_H */
