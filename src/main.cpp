#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <WiFi.h>
#include <AHT20_BMP280.h>
#include <ADS1115.h>

QueueHandle_t eventQueue;
QueueHandle_t databaseQueue;

ADS1115_t ads1;
ADS1115_t ads2;

typedef struct {
    float temperature;
    float humidity;
    float pressure;
    float altitude;
    uint16_t soilMoisture1;
    uint16_t soilMoisture2;
    uint16_t soilMoisture3;
    uint16_t soilMoisture4;
    uint16_t soilMoisture5;
    uint16_t airQuality;
    uint16_t rainLevel;
} SensorData_t;

SensorData_t sensorData;

void ads1115_init(void);

void freertos_task_init(void);
void freertos_queue_init(void);

void read_analog_sensor(void);
void read_i2c_sensor(void);
void wifi_manager(void);

void task_wifi_manager(void *pvParameters);
void task_analog_sensor(void *pvParameters);
void task_serial_print(void *pvParameters);
void task_i2c_sensor(void *pvParameters);

void setup() {
    Serial.begin(115200);
    Wire.begin();

    ads1115_init();

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
    // eventQueue = xQueueCreate(10, sizeof(EventType_t));
}

void freertos_task_init(void){
    xTaskCreate(
        task_wifi_manager,
        "masterControllerTask",
        2048,
        NULL,
        24,
        NULL
    );

    xTaskCreate(
        task_analog_sensor,
        "sensorReadTask",
        2048,
        NULL,
        23,
        NULL
    );

    xTaskCreate(
        task_serial_print,
        "serialPrintTask",
        2048,
        NULL,
        2,
        NULL
    );

    xTaskCreate(
        task_i2c_sensor,
        "EnvSensorTask",
        2048,
        NULL,
        2,
        NULL
    );
}

void ads1115_init(void){
    ADS1115_begin(&ads1, &Wire, ADS1115_ADDR_GND);
    ADS1115_begin(&ads2, &Wire, ADS1115_ADDR_VDD);

    ADS1115_setGain(&ads1, ADS1115_PGA_2_048V);
    ADS1115_setDataRate(&ads1, ADS1115_DR_128SPS);
}

void read_analog_sensor(void){
    sensorData.soilMoisture1 = ADS1115_readVoltage(&ads1, ADS1115_MUX_SINGLE_0);
    sensorData.soilMoisture2 = ADS1115_readVoltage(&ads1, ADS1115_MUX_SINGLE_1);
    sensorData.soilMoisture3 = ADS1115_readVoltage(&ads1, ADS1115_MUX_SINGLE_2);
    sensorData.soilMoisture4 = ADS1115_readVoltage(&ads1, ADS1115_MUX_SINGLE_3);

    sensorData.airQuality = ADS1115_readVoltage(&ads2, ADS1115_MUX_SINGLE_0);
    sensorData.rainLevel = ADS1115_readVoltage(&ads2, ADS1115_MUX_SINGLE_1);
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

void wifi_manager(){
    
}

void task_wifi_manager(void *pvParameters){
    for(;;){
        if(WiFi.status() != WL_CONNECTED){
            wifi_manager();
        }else{
            vTaskDelay(5000 / portTICK_PERIOD_MS);
        }
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
        // if (xQueueReceive(eventQueue, &evt, portMAX_DELAY) != pdPASS) {
        //     continue;
        // }

        // if (evt != EV_SERIAL_PRINT) {
        //     continue;
        // }

        // Serial.print("Temperature: ");
        // Serial.print(temp);
        // Serial.print(" °C, Humidity: ");
        // Serial.print(hum);
        // Serial.print(" %, Pressure: ");
        // Serial.print(BMP280_Pressure);
        // Serial.print(" Pa, Approx Altitude: ");
        // Serial.print(BMP280_Altitude);
        // Serial.print(" m, Soil Moisture: ");
        // Serial.print(soil_val);
        // Serial.print(" %, PPM: ");
        // Serial.print(mq_val);
        // Serial.println(" %");
    }
}

