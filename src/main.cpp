#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <DHT22.h>

QueueHandle_t eventQueue;

#define DHT_PIN 19
#define MQ_PIN 34
#define SOILC_PIN 35

uint16_t mq_val, soil_val, prev_mq, prev_soil, old_mq, old_soil, old_tem, old_hum;
float alpha = 0.95;
float hum, temp;

typedef enum {
  EV_READ_SENSOR,
  EV_SERIAL_PRINT,
  EV_FIREBASE,
  EV_MARIADB
} EventType_t;

void taskMaster(void *pvParameters);
void taskSensorRead(void *pvParameters);
void taskSerialPrint(void *pvParameters);
void taskDHT22(void *pvParameters);

void setup() {
  Serial.begin(115200);
  pinMode(DHT_PIN, INPUT);
  pinMode(MQ_PIN, INPUT);
  pinMode(SOILC_PIN, INPUT);

  eventQueue = xQueueCreate(10, sizeof(EventType_t));

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
    taskDHT22,
    "DHT22Task",
    2048,
    NULL,
    2,
    NULL
  );
}

void loop() {

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
      xQueueSend(eventQueue, &evt, 0);  // Trigger Serial print
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}


void taskSensorRead(void *pvParameters){
  for(;;){
    mq_val = map((analogRead(MQ_PIN) + prev_mq / 2), 0, 4095, 0, 100);
    prev_mq = mq_val;
    soil_val = map((analogRead(SOILC_PIN) + prev_soil / 2), 0, 4095, 0, 100);
    prev_soil = soil_val;

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void taskDHT22(void *pvParameters){
  for(;;){
    DHT22_Read(DHT_PIN, &temp, &hum);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void taskSerialPrint(void *pvParameters) {
  EventType_t evt;
  for (;;) {
    if (xQueueReceive(eventQueue, &evt, portMAX_DELAY) == pdPASS) {
      if (evt == EV_SERIAL_PRINT) {
        Serial.print("Temperature: ");
        Serial.print(temp);
        Serial.print(" °C, Humidity: ");
        Serial.print(hum);
        Serial.print(" %, Soil Moisture: ");
        Serial.print(soil_val);
        Serial.print(" %, PPM: ");
        Serial.print(mq_val);
        Serial.println(" %");
      }
    }
  }
}
