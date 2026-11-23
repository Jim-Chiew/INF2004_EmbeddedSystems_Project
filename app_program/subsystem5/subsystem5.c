/*
 *	subsystem5.c
 *	Subsystem 5 -- Barcode: starts its task.  It owns no kernel objects:
 *	it sends to subsystem 1's mbf_barcode (IF-09).
 */

#include "subsystem5.h"

EXPORT ID	tid_barcode;

EXPORT ER subsystem5_init(void)
{
	return E_OK;
}

EXPORT ER subsystem5_start(void)
{
	/* Core 1: the sampler interrupt is enabled from this task, and an
	   interrupt is enabled on the core that calls EnableInt(). */
	tid_barcode = app_start_task(barcode_task, S5_PRI_BARCODE,
				     APP_CORE_CONTROL, "barcode_task");
	return (tid_barcode < E_OK) ? tid_barcode : E_OK;
}
