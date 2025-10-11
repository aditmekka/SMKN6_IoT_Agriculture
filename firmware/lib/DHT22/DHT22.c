/*
 * DHT22.c
 *
 *  Created on: Oct 7, 2025
 *      Author: aditr
 */

#include "DHT22.h"

static uint8_t dht_data[5];

static bool DHT22_ReadRaw(uint8_t pin) {
    uint8_t i, j;
    uint16_t counter;

    // Start signal
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delay(20);
    digitalWrite(pin, HIGH);
    delayMicroseconds(40);
    pinMode(pin, INPUT_PULLUP);

    // Wait for DHT22 response
    counter = 0;
    while (digitalRead(pin)) {
        delayMicroseconds(1);
        if (++counter > 90) return false;
    }

    counter = 0;
    while (!digitalRead(pin)) {
        delayMicroseconds(1);
        if (++counter > 90) return false;
    }

    counter = 0;
    while (digitalRead(pin)) {
        delayMicroseconds(1);
        if (++counter > 90) return false;
    }

    for (j = 0; j < 5; j++) {
        dht_data[j] = 0;
        for (i = 0; i < 8; i++) {
            counter = 0;
            while (!digitalRead(pin)) {
                delayMicroseconds(1);
                if (++counter > 90) return false;
            }

            counter = 0;
            while (digitalRead(pin)) {
                delayMicroseconds(1);
                if (++counter > 100) break;
            }

            if (counter > 30)
                dht_data[j] |= (1 << (7 - i));
        }
    }

    return true;
}

bool DHT22_Read(uint8_t pin, float *temperature, float *humidity) {
    if (!DHT22_ReadRaw(pin))
        return false;

    uint16_t rawHumidity = ((uint16_t)dht_data[0] << 8) | dht_data[1];
    uint16_t rawTemperature = ((uint16_t)dht_data[2] << 8) | dht_data[3];

    if (rawTemperature & 0x8000) {  // negative temperature
        rawTemperature &= 0x7FFF;
        *temperature = -((float)rawTemperature) / 10.0;
    } else {
        *temperature = ((float)rawTemperature) / 10.0;
    }

    *humidity = ((float)rawHumidity) / 10.0;

    uint8_t sum = dht_data[0] + dht_data[1] + dht_data[2] + dht_data[3];
    if (sum != dht_data[4]) return false;

    return true;
}
