/*
 * DHT22.H
 *
 *  Created on: Oct 7, 2025
 *      Author: aditr
 */

#ifndef DHT22_H
#define DHT22_H

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

bool DHT22_Read(uint8_t pin, float *temperature, float *humidity);

#ifdef __cplusplus
}
#endif

#endif
