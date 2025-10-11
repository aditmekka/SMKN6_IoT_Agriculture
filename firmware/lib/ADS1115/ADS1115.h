/**
 ******************************************************************************
 * @file    ADS1115.h
 * @brief   Driver for ADS1115 Analog to I2C Multiplexer.
 * @author  aditmekka
 * @version 1.0
 * @date    October 2025
 * @license MIT License (Free to use with attribution)
 *
 * @details
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

#ifndef ADS1115_H
#define ADS1115_H

#include <Arduino.h>
#include <Wire.h>

#define ADS1115_DEFAULT_ADDRESS 0x48

// Pointer Register
#define ADS1115_REG_CONVERSION  0x00
#define ADS1115_REG_CONFIG      0x01

// PGA (gain) settings
enum adsGain_t {
    GAIN_TWOTHIRDS = 0x0000, // ±6.144V
    GAIN_ONE       = 0x0200, // ±4.096V
    GAIN_TWO       = 0x0400, // ±2.048V
    GAIN_FOUR      = 0x0600, // ±1.024V
    GAIN_EIGHT     = 0x0800, // ±0.512V
    GAIN_SIXTEEN   = 0x0A00  // ±0.256V
};

// Data rate (samples per second)
enum adsDataRate_t {
    RATE_8SPS   = 0x0000,
    RATE_16SPS  = 0x0020,
    RATE_32SPS  = 0x0040,
    RATE_64SPS  = 0x0060,
    RATE_128SPS = 0x0080,
    RATE_250SPS = 0x00A0,
    RATE_475SPS = 0x00C0,
    RATE_860SPS = 0x00E0
};

class ADS1115 {
public:
    ADS1115(uint8_t addr = ADS1115_DEFAULT_ADDRESS, TwoWire *wirePort = &Wire);
    bool begin();
    void setGain(adsGain_t gain);
    void setDataRate(adsDataRate_t rate);

    float readMillivolt(uint8_t channel);
    float readVoltage(uint8_t channel);
    int16_t readADC(uint8_t channel);

    // Differential reads
    int16_t readDifferential_0_1();
    int16_t readDifferential_2_3();

private:
    uint8_t _addr;
    TwoWire *_wire;
    adsGain_t _gain;
    adsDataRate_t _rate;

    void writeRegister(uint8_t reg, uint16_t value);
    uint16_t readRegister(uint8_t reg);
    float getMultiplier();
};

#endif
