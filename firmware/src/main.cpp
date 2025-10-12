#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <WiFi.h>
#include <WiFiManager.h>
#include <AHT20_BMP280.h>
#include <ADS1115.h>

static EventGroupHandle_t wifiEventGroup;
#define WIFI_CONNECTED_BIT (1 << 0)

TaskHandle_t xTaskWifiManager, xTaskSensorI2C, xTaskSensorAnalog, xTaskSerialPrint;

QueueHandle_t eventQueue;
QueueHandle_t databaseQueue;

ADS1115 ads1(0x48);
ADS1115 ads2(0x49);

typedef struct {
    float temperature;
    float humidity;
    float pressure;
    float altitude;
    float soilMoisture1;
    uint16_t soilMoisture2;
    uint16_t soilMoisture3;
    uint16_t soilMoisture4;
    uint16_t soilMoisture5;
    uint16_t airQuality;
    uint16_t rainLevel;
} SensorData_t;

SensorData_t sensorData;

bool lastConnected = false;

void ads1115_init(void);

void freertos_task_init(void);
void freertos_queue_init(void);
void freertos_event_init(void);

void read_analog_sensor(void);
void read_i2c_sensor(void);
void wifi_manager(void);

void task_wifi_manager(void *pvParameters);
void task_analog_sensor(void *pvParameters);
void task_serial_print(void *pvParameters);
void task_i2c_sensor(void *pvParameters);

void setup() {
    WiFi.mode(WIFI_STA);
    Serial.begin(115200);
    Wire.begin();

    ads1115_init();
    if (!AHT20_Init()) Serial.println("AHT20 init failed!");
    if (!BMP280_Init()) Serial.println("BMP280 init failed!");

    freertos_event_init();
    freertos_queue_init();
    freertos_task_init();
}

void loop() {
    // Nothing to do here; all handled by tasks
}

void freertos_queue_init(void){
    // eventQueue = xQueueCreate(10, sizeof(EventType_t));
}

void freertos_event_init(void){
    wifiEventGroup = xEventGroupCreate();
}

void freertos_task_init(void){
    xTaskCreate(
        task_wifi_manager,
        "wifi_manager",
        8192,
        NULL,
        5,
        &xTaskWifiManager
    );

    xTaskCreate(
        task_analog_sensor,
        "analog_sensor",
        2048,
        NULL,
        2,
        &xTaskSensorAnalog
    );

    xTaskCreate(
        task_serial_print,
        "serialPrintTask",
        2048,
        NULL,
        2,
        &xTaskSerialPrint
    );

    xTaskCreate(
        task_i2c_sensor,
        "i2c_sensor",
        2048,
        NULL,
        2,
        &xTaskSensorI2C
    );
}

void setTaskStates(bool wifiConnected) {
    if (wifiConnected) {
        vTaskResume(xTaskSensorI2C);
        vTaskResume(xTaskSensorAnalog);
        // vTaskResume(xTaskActuator);
        vTaskResume(xTaskSerialPrint);
    } else {
        vTaskSuspend(xTaskSensorI2C);
        vTaskSuspend(xTaskSensorAnalog);
        // vTaskSuspend(xTaskActuator);
        vTaskSuspend(xTaskSerialPrint);
    }
}

void ads1115_init(void){
    ads1.setGain(GAIN_ONE);
    ads2.setGain(GAIN_ONE);
    ads1.setDataRate(RATE_250SPS);
    ads2.setDataRate(RATE_250SPS);
}

void read_analog_sensor(void){
    sensorData.soilMoisture1 = (ads1.readVoltage(0)/ 3.3f) * 100.0f;
    sensorData.soilMoisture2 = ads1.readVoltage(1);
    sensorData.soilMoisture3 = ads1.readVoltage(2);
    sensorData.soilMoisture4 = ads1.readVoltage(3);

    if(ads2.begin()){
        sensorData.airQuality = ads2.readVoltage(0);
        sensorData.rainLevel = ads2.readVoltage(1);
    }
}

void read_i2c_sensor(void){
    AHT20_Measure();
    BMP280_Read();
    BMP280_ReadAltitude(1013.25);

    sensorData.temperature = AHT20_Temperature;
    sensorData.humidity = AHT20_Humidity;
    sensorData.pressure = BMP280_Pressure;
    sensorData.altitude = BMP280_Altitude;
}

void task_wifi_manager(void *pvParameters) {
    WiFiManager wm;
    wm.setDebugOutput(true);

    wm.setConfigPortalBlocking(false);
    wm.setShowInfoUpdate(true);
    wm.setConnectTimeout(15);           // seconds to try connecting before starting AP
    wm.setConfigPortalTimeout(300);     // portal closes after 5 minutes if idle

    wm.setTitle("SMKN6 IoT Agriculture WiFi Setup");

    // Uncomment for testing purpose:
    //wm.resetSettings();

    bool ok = wm.autoConnect();
    if (!ok) {
        Serial.println("WiFiMgr: Failed to auto-connect, starting AP portal...");

        WiFi.mode(WIFI_AP);
        WiFi.softAP("IoT Agriculture", "SMKN6BDG");
        wm.setHostname("IoT Agriculture");
    
        wm.startConfigPortal("IoT Agriculture", "SMKN6BDG");
    } else {
        Serial.println("WiFiMgr: Connected to WiFi!");
        xEventGroupSetBits(wifiEventGroup, WIFI_CONNECTED_BIT);
        setTaskStates(true);
    }

    bool wasConnected = (WiFi.status() == WL_CONNECTED);

    for (;;) {
        wm.process();

        bool isConnected = (WiFi.status() == WL_CONNECTED);

        if (isConnected && !wasConnected) {
            Serial.println("WiFiMgr: Connected to network!");
            xEventGroupSetBits(wifiEventGroup, WIFI_CONNECTED_BIT);
            setTaskStates(true);
        } else if (!isConnected && wasConnected) {
            Serial.println("WiFiMgr: Lost Wi-Fi, reopening portal...");
            xEventGroupClearBits(wifiEventGroup, WIFI_CONNECTED_BIT);
            setTaskStates(false);

            WiFi.mode(WIFI_AP);
            WiFi.softAP("IoT Agriculture", "SMKN6BDG");
            wm.setHostname("IoT Agriculture");
            wm.startConfigPortal("IoT Agriculture", "SMKN6BDG");
        }

        wasConnected = isConnected;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void task_analog_sensor(void *pvParameters) {
    for (;;) {
        read_analog_sensor();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void task_i2c_sensor(void *pvParameters) {
    for (;;) {
        read_i2c_sensor();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void task_serial_print(void *pvParameters) {
    // EventType_t evt;

    for (;;) {

        xEventGroupWaitBits(wifiEventGroup, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

        // if (xQueueReceive(eventQueue, &evt, portMAX_DELAY) != pdPASS) {
        //     continue;
        // }

        // if (evt != EV_SERIAL_PRINT) {
        //     continue;
        // }

        Serial.print("Temperature: ");
        Serial.print(sensorData.temperature);
        Serial.print(" °C, Humidity: ");
        Serial.print(sensorData.humidity);
        Serial.print(" %, Pressure: ");
        Serial.print(sensorData.pressure);
        Serial.print(" Pa, Approx Altitude: ");
        Serial.print(sensorData.altitude);
        Serial.print(" m, Soil Moisture: ");
        Serial.print(sensorData.soilMoisture1);
        Serial.print(" %, PPM: ");
        Serial.print(sensorData.airQuality);
        Serial.println(" %");

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

