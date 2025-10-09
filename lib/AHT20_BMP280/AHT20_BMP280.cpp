/**
 ******************************************************************************
 * @file    AHT20_BMP280.cpp
 * @brief   Combined driver for AHT20 (Temp/Humidity) and BMP280 (Pressure) sensors using Arduino Wire (C-style)
 * @author  Original: InaSkills
 *          Modified by: aditmekka
 * @version 2.1
 * @date    October 2025
 * @license MIT License (Free to use with attribution)
 *
 * @details
 * Based on the AHT20 STM32 HAL driver by InaSkills (v1.0, July 2025),
 * modified and ported to the Arduino framework by aditmekka.
 * Added BMP280 support and ensured full compliance with C++/ESP32 compilation.
 *
 * ----------------------------------------------------------------------------
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

#include "AHT20_BMP280.h"
#include <math.h>  // for pow()

/* -------------------------------------------------------------------------- */
/* --- Global Variables ----------------------------------------------------- */
float AHT20_Temperature = 0.0f;
float AHT20_Humidity    = 0.0f;
float BMP280_Temperature = 0.0f;
float BMP280_Pressure    = 0.0f;
float BMP280_Altitude    = 0.0f;

/* --- Internal Variables --- */
static unsigned long aht20_tick = 0;
static unsigned long aht20_tick_update = 0;

/* --- BMP280 Calibration Data --- */
static uint16_t dig_T1;
static int16_t  dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static int32_t  t_fine;

/* -------------------------------------------------------------------------- */
/* --- AHT20 Functions ------------------------------------------------------ */

bool AHT20_Init(void)
{
    delay(40); // wait after power-up

    uint8_t status = 0;

    Wire.beginTransmission((uint8_t)AHT20_ADDRESS);
    Wire.write(0x71);
    if (Wire.endTransmission(false) != 0) return false;

    Wire.requestFrom((uint8_t)AHT20_ADDRESS, (uint8_t)1);
    if (Wire.available()) status = Wire.read();

    // If not calibrated
    if (((status >> 3) & 0x01) == 0) {
        Wire.beginTransmission((uint8_t)AHT20_ADDRESS);
        Wire.write(0xBE);
        Wire.write(0x08);
        Wire.write(0x00);
        if (Wire.endTransmission() != 0) return false;
        delay(10);
    }

    return true;
}

bool AHT20_Measure(void)
{
    if (millis() - aht20_tick < 1000)
        return false; // Too soon to read again

    uint8_t measure_cmd[3] = {0xAC, 0x33, 0x00};
    Wire.beginTransmission((uint8_t)AHT20_ADDRESS);
    Wire.write(measure_cmd, 3);
    if (Wire.endTransmission() != 0) return false;

    delay(80); // Wait for conversion

    uint8_t status = 0;
    do {
        if (millis() - aht20_tick_update > 100) {
            Wire.beginTransmission((uint8_t)AHT20_ADDRESS);
            Wire.write(0x71);
            if (Wire.endTransmission(false) != 0) return false;
            Wire.requestFrom((uint8_t)AHT20_ADDRESS, (uint8_t)1);
            if (Wire.available()) status = Wire.read();
            aht20_tick_update = millis();
        }
    } while ((status >> 7) & 0x01); // Wait BUSY=0

    uint8_t RxData[7] = {0};
    Wire.requestFrom((uint8_t)AHT20_ADDRESS, (uint8_t)7);
    for (uint8_t i = 0; i < 7 && Wire.available(); i++)
        RxData[i] = Wire.read();

    uint32_t HUM_DATA = ((uint32_t)RxData[1] << 16) | ((uint32_t)RxData[2] << 8) | RxData[3];
    HUM_DATA >>= 4;
    AHT20_Humidity = ((float)HUM_DATA * 100.0f) / 1048576.0f;

    uint32_t TEMP_DATA = (((uint32_t)RxData[3] & 0x0F) << 16) | ((uint32_t)RxData[4] << 8) | RxData[5];
    AHT20_Temperature = (((float)TEMP_DATA * 200.0f) / 1048576.0f) - 50.0f;

    aht20_tick = millis();
    return true;
}

/* -------------------------------------------------------------------------- */
/* --- BMP280 Helper Functions --------------------------------------------- */

static uint8_t bmp280_read8(uint8_t reg)
{
    Wire.beginTransmission((uint8_t)BMP280_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)BMP280_ADDRESS, (uint8_t)1);
    return Wire.read();
}

static uint16_t bmp280_read16(uint8_t reg)
{
    Wire.beginTransmission((uint8_t)BMP280_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)BMP280_ADDRESS, (uint8_t)2);
    uint16_t value = (uint16_t)(Wire.read() | (Wire.read() << 8));
    return value;
}

static int16_t bmp280_readS16(uint8_t reg)
{
    return (int16_t)bmp280_read16(reg);
}

static uint32_t bmp280_read24(uint8_t reg)
{
    Wire.beginTransmission((uint8_t)BMP280_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)BMP280_ADDRESS, (uint8_t)3);
    uint32_t value = ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
    return value;
}

/* -------------------------------------------------------------------------- */
/* --- BMP280 Functions ----------------------------------------------------- */

bool BMP280_Init(void)
{
    uint8_t id = bmp280_read8(0xD0);
    if (id != 0x58) return false; // Wrong chip ID

    // Read calibration data
    dig_T1 = bmp280_read16(0x88);
    dig_T2 = bmp280_readS16(0x8A);
    dig_T3 = bmp280_readS16(0x8C);
    dig_P1 = bmp280_read16(0x8E);
    dig_P2 = bmp280_readS16(0x90);
    dig_P3 = bmp280_readS16(0x92);
    dig_P4 = bmp280_readS16(0x94);
    dig_P5 = bmp280_readS16(0x96);
    dig_P6 = bmp280_readS16(0x98);
    dig_P7 = bmp280_readS16(0x9A);
    dig_P8 = bmp280_readS16(0x9C);
    dig_P9 = bmp280_readS16(0x9E);

    // Control registers
    Wire.beginTransmission((uint8_t)BMP280_ADDRESS);
    Wire.write(0xF4);
    Wire.write(0x27); // temp/press oversampling x1, normal mode
    Wire.endTransmission();

    Wire.beginTransmission((uint8_t)BMP280_ADDRESS);
    Wire.write(0xF5);
    Wire.write(0xA0); // standby 1000ms, filter off
    Wire.endTransmission();

    delay(100);
    return true;
}

bool BMP280_Read(void)
{
    int32_t adc_T = bmp280_read24(0xFA) >> 4;
    int32_t adc_P = bmp280_read24(0xF7) >> 4;

    // Temperature compensation
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
                      ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
                    ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    BMP280_Temperature = (float)((t_fine * 5 + 128) >> 8) / 100.0f;

    // Pressure compensation
    int64_t var1p, var2p, p;
    var1p = ((int64_t)t_fine) - 128000;
    var2p = var1p * var1p * (int64_t)dig_P6;
    var2p = var2p + ((var1p * (int64_t)dig_P5) << 17);
    var2p = var2p + (((int64_t)dig_P4) << 35);
    var1p = ((var1p * var1p * (int64_t)dig_P3) >> 8) +
            ((var1p * (int64_t)dig_P2) << 12);
    var1p = (((((int64_t)1) << 47) + var1p)) * ((int64_t)dig_P1) >> 33;
    if (var1p == 0) return false;

    p = 1048576 - adc_P;
    p = (((p << 31) - var2p) * 3125) / var1p;
    var1p = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2p = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1p + var2p) >> 8) + (((int64_t)dig_P7) << 4);
    BMP280_Pressure = (float)p / 25600.0f; // hPa

    return true;
}

float BMP280_ReadAltitude(float seaLevel_hPa)
{
    BMP280_Altitude = 44330.0f * (1.0f - pow((BMP280_Pressure / seaLevel_hPa), 0.1903f));
    return BMP280_Altitude;
}
