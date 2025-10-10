/**
 ******************************************************************************
 * @file    ADS1115.h
 * @brief   C Style Driver For ADS1115 on the Arduino Framework
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

// Default I2C address
#define ADS1115_ADDR_GND 0x48
#define ADS1115_ADDR_VDD 0x49
#define ADS1115_ADDR_SDA 0x4A
#define ADS1115_ADDR_SCL 0x4B

// Pointer Register
#define ADS1115_REG_CONVERSION 0x00
#define ADS1115_REG_CONFIG     0x01

// PGA (Programmable Gain Amplifier)
typedef enum {
    ADS1115_PGA_6_144V = 0x00, // ±6.144V
    ADS1115_PGA_4_096V = 0x01, // ±4.096V
    ADS1115_PGA_2_048V = 0x02, // ±2.048V (default)
    ADS1115_PGA_1_024V = 0x03, // ±1.024V
    ADS1115_PGA_0_512V = 0x04, // ±0.512V
    ADS1115_PGA_0_256V = 0x05  // ±0.256V
} ADS1115_PGA_t;

// Data rate
typedef enum {
    ADS1115_DR_8SPS   = 0x00,
    ADS1115_DR_16SPS  = 0x01,
    ADS1115_DR_32SPS  = 0x02,
    ADS1115_DR_64SPS  = 0x03,
    ADS1115_DR_128SPS = 0x04, // default
    ADS1115_DR_250SPS = 0x05,
    ADS1115_DR_475SPS = 0x06,
    ADS1115_DR_860SPS = 0x07
} ADS1115_DataRate_t;

// Mux Channels
typedef enum {
    ADS1115_MUX_DIFF_0_1 = 0x00,
    ADS1115_MUX_DIFF_0_3 = 0x01,
    ADS1115_MUX_DIFF_1_3 = 0x02,
    ADS1115_MUX_DIFF_2_3 = 0x03,
    ADS1115_MUX_SINGLE_0 = 0x04,
    ADS1115_MUX_SINGLE_1 = 0x05,
    ADS1115_MUX_SINGLE_2 = 0x06,
    ADS1115_MUX_SINGLE_3 = 0x07
} ADS1115_Mux_t;

typedef struct {
    uint8_t i2cAddress;
    TwoWire *wire;
    ADS1115_PGA_t gain;
    ADS1115_DataRate_t dataRate;
} ADS1115_t;

#ifdef __cplusplus
extern "C" {
#endif

// Initialization
void ADS1115_begin(ADS1115_t *dev, TwoWire *wire, uint8_t address);

// Configuration
void ADS1115_setGain(ADS1115_t *dev, ADS1115_PGA_t gain);
void ADS1115_setDataRate(ADS1115_t *dev, ADS1115_DataRate_t rate);

// Read functions
float ADS1115_readVoltage(ADS1115_t *dev, ADS1115_Mux_t channel);
int16_t ADS1115_readRaw(ADS1115_t *dev, ADS1115_Mux_t channel);

#ifdef __cplusplus
}
#endif

#endif // ADS1115_H
