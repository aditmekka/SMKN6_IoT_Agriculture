#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <AHT20_BMP280.h>

QueueHandle_t eventQueue;

#define MQ_PIN      34
#define SOILC_PIN   35

uint16_t mq_val, soil_val, prev_mq, prev_soil, old_mq, old_soil;
float old_tem, old_hum;
float alpha = 0.95;
float hum, temp;

typedef enum {
    EV_READ_SENSOR,
    EV_SERIAL_PRINT,
    EV_FIREBASE,
    EV_MARIADB
} EventType_t;

void freertos_task_init(void);
void freertos_queue_init(void);

void taskMaster(void *pvParameters);
void taskSensorRead(void *pvParameters);
void taskSerialPrint(void *pvParameters);
void taskEnvSensor(void *pvParameters);

void setup() {
    Serial.begin(115200);
    Wire.begin();
    pinMode(MQ_PIN, INPUT);
    pinMode(SOILC_PIN, INPUT);

    if (!AHT20_Init()) {
        Serial.println("AHT20 init failed!");
    }
    if (!BMP280_Init()) {
        Serial.println("BMP280 init failed!");
    }

    freertos_queue_init();
    freertos_task_init();
}

void loop() {
    // Nothing to do here; all handled by tasks
}

void freertos_queue_init(void){
    eventQueue = xQueueCreate(10, sizeof(EventType_t));
}

void freertos_task_init(void){
    xTaskCreate(
        taskMaster,
        "masterControllerTask",
        2048,
        NULL,
        24,
        NULL
    );

    xTaskCreate(
        taskSensorRead,
        "sensorReadTask",
        2048,
        NULL,
        23,
        NULL
    );

    xTaskCreate(
        taskSerialPrint,
        "serialPrintTask",
        2048,
        NULL,
        2,
        NULL
    );

    xTaskCreate(
        taskEnvSensor,
        "EnvSensorTask",
        2048,
        NULL,
        2,
        NULL
    );
}

void taskMaster(void *pvParameters) {
    EventType_t evt;

    const uint16_t MQ_THRESHOLD = 2;
    const uint16_t SOIL_THRESHOLD = 2;
    const float TEMP_THRESHOLD = 0.5;
    const float HUM_THRESHOLD = 1.0;

    for (;;) {
        bool shouldPrint = false;

        if (abs((int16_t)(mq_val - old_mq)) >= MQ_THRESHOLD) {
            old_mq = mq_val;
            shouldPrint = true;
        }

        if (abs((int16_t)(soil_val - old_soil)) >= SOIL_THRESHOLD) {
            old_soil = soil_val;
            shouldPrint = true;
        }

        if (fabs(temp - old_tem) >= TEMP_THRESHOLD) {
            old_tem = temp;
            shouldPrint = true;
        }

        if (fabs(hum - old_hum) >= HUM_THRESHOLD) {
            old_hum = hum;
            shouldPrint = true;
        }

        if (shouldPrint) {
            evt = EV_SERIAL_PRINT;
            xQueueSend(eventQueue, &evt, 0);
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void taskSensorRead(void *pvParameters) {
    for (;;) {
        mq_val = map((analogRead(MQ_PIN) + prev_mq / 2), 0, 4095, 0, 100);
        prev_mq = mq_val;

        soil_val = map((analogRead(SOILC_PIN) + prev_soil / 2), 0, 4095, 0, 100);
        prev_soil = soil_val;

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void taskEnvSensor(void *pvParameters) {
    for (;;) {
        AHT20_Measure();
        BMP280_Read();
        BMP280_ReadAltitude(1013.25);

        temp = AHT20_Temperature;
        hum = AHT20_Humidity;

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void taskSerialPrint(void *pvParameters) {
    EventType_t evt;

    for (;;) {
        if (xQueueReceive(eventQueue, &evt, portMAX_DELAY) != pdPASS) {
            continue;
        }

        if (evt != EV_SERIAL_PRINT) {
            continue;
        }

        Serial.print("Temperature: ");
        Serial.print(temp);
        Serial.print(" °C, Humidity: ");
        Serial.print(hum);
        Serial.print(" %, Pressure: ");
        Serial.print(BMP280_Pressure);
        Serial.print(" Pa, Approx Altitude: ");
        Serial.print(BMP280_Altitude);
        Serial.print(" m, Soil Moisture: ");
        Serial.print(soil_val);
        Serial.print(" %, PPM: ");
        Serial.print(mq_val);
        Serial.println(" %");
    }
}

