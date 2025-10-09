/**
 ******************************************************************************
 * @file    AHT20_BMP280.h
 * @brief   Driver for AHT20 (Temp/Humidity) and BMP280 (Pressure) sensors using Arduino Wire I2C (C version)
 * @author  Original: InaSkills
 *          Modified by: aditmekka
 * @version 2.0
 * @date    October 2025
 * @license MIT License (Free to use with attribution)
 *
 * @details
 * Based on the AHT20 STM32 HAL driver by InaSkills (v1.0, July 2025),
 * modified and ported to the Arduino framework by aditmekka.
 * Added BMP280 functionality for temperature, pressure, and altitude.
 *
 * ----------------------------------------------------------------------------
 * Original MIT License:
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
 *
 ******************************************************************************
 */

#ifndef AHT20_BMP280_H_
#define AHT20_BMP280_H_

#include <Arduino.h>
#include <Wire.h>

/* --- I2C Addresses --- */
#define AHT20_ADDRESS   0x38
#define BMP280_ADDRESS  0x76  // some modules use 0x77

/* --- Public Variables --- */
extern float AHT20_Temperature;
extern float AHT20_Humidity;
extern float BMP280_Temperature;
extern float BMP280_Pressure;
extern float BMP280_Altitude;

/* --- AHT20 Functions --- */
bool AHT20_Init(void);
bool AHT20_Measure(void);

/* --- BMP280 Functions --- */
bool BMP280_Init(void);
bool BMP280_Read(void);
float BMP280_ReadAltitude(float seaLevel_hPa);

#endif /* AHT20_BMP280_H_ */
