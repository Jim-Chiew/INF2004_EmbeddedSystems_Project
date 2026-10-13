/*
 * wifi_secrets.example.h  --  TEMPLATE, safe to commit.
 *
 * Copy this file to wifi_secrets.h (same folder) and put your real values in
 * the copy. wifi_secrets.h is git-ignored, so it is never uploaded.
 */
#ifndef WIFI_SECRETS_H
#define WIFI_SECRETS_H

#define WIFI_SSID        "your-2.4GHz-network-name"
#define WIFI_PASSWORD    "your-wifi-password"

/* The computer running the MQTT broker, on the SAME network as the Pico W. */
#define MQTT_BROKER_IP   "192.168.0.10"
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID   "robocar-buddy1"

#endif /* WIFI_SECRETS_H */