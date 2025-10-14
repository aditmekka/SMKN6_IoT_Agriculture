#ifndef ENV_H
#define ENV_H

// MQTT
#define MQTT_BROKER "192.168.1.1"
#define MQTT_PORT 1883
#define MQTT_TOPIC "esp32/data"
#define DEVICE_ID "ESP32-01"

// Thresholds
#define TEMP_THRESHOLD   0.2f
#define HUM_THRESHOLD    1.0f
#define PRESS_THRESHOLD  0.5f
#define SOIL_THRESHOLD   2.0f
#define AIRQ_THRESHOLD   10
#define RAIN_THRESHOLD   10

// Interval
#define MQTT_COOLDOWN_MS 1000   // Minimum time between updates
#define MQTT_FORCE_MS   60000   // Force publish every 60s

#endif
