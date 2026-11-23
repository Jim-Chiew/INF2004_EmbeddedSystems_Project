/*
 *	subsystem1.c
 *	Subsystem 1 -- Comms: creates its objects, sets up the comms engine
 *	and starts telemetry_task.
 */

#include "subsystem1.h"
#include "comms.h"
#include "wifi_secrets.h"

EXPORT ID	mbf_barcode;	/* IF-09, see subsystem1.h */
EXPORT ID	tid_telemetry;

/* The strings are not copied by comms_init(): they must stay valid */
LOCAL const comms_config_t comms_cfg = {
	.wifi_ssid     = WIFI_SSID,		/* the WiFi task joins with these */
	.wifi_password = WIFI_PASSWORD,
	.broker_ip     = MQTT_BROKER_IP,
	.broker_port   = MQTT_BROKER_PORT,
	.client_id     = MQTT_CLIENT_ID,
	.username      = NULL,			/* anonymous broker (bench only) */
	.password      = NULL,
};

EXPORT ER subsystem1_init(void)
{
	T_CMBF	cmbf = {0};

	/* Barcode passes from subsystem 5: room for several, they are rare */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = 8 * (sizeof(BARCODE_MSG) + 8);
	cmbf.maxmsz = sizeof(BARCODE_MSG);
	mbf_barcode = tk_cre_mbf(&cmbf);
	if (mbf_barcode < E_OK) {
		return mbf_barcode;
	}

	/* Does not block and does not touch the radio: comms_task(), called by
	   the kernel's WiFi task, does the network work from now on. */
	if (!comms_init(&comms_cfg)) {
		tm_printf((UB*)"comms: comms_init failed (check config/wifi_secrets.h)\n");
	}
	return E_OK;
}

EXPORT ER subsystem1_start(void)
{
	/* Core 0, next to the WiFi task: reporting must never delay the
	   tasks that steer the car (core 1). */
	tid_telemetry = app_start_task(telemetry_task, S1_PRI_TELEMETRY,
				       APP_CORE_RADIO, "telemetry_task");
	return (tid_telemetry < E_OK) ? tid_telemetry : E_OK;
}
