#pragma once

// Copy this file to Config/credentials/credentials.h and fill in real values.
// credentials.h is intentionally ignored by Git.

const char *ssid = "YOUR_WIFI_SSID";
const char *password = "YOUR_WIFI_PASSWORD";

const char *mqttServer = "192.0.2.10";
const int mqttPort = 1883;
const char *mqttUser = "YOUR_MQTT_USER";
const char *mqttPassword = "YOUR_MQTT_PASSWORD";

// Optional separate credentials for /data, /reboot and /update.
// If omitted, the main unit falls back to mqttUser / mqttPassword.
#define WEB_USERNAME "YOUR_WEB_ADMIN_USER"
#define WEB_PASSWORD "YOUR_WEB_ADMIN_PASSWORD"

#define MESH_PREFIX "pocasi-loucka"
#define MESH_PASSWORD "CHANGE_THIS_MESH_PASSWORD"
#define MESH_PORT 5555
