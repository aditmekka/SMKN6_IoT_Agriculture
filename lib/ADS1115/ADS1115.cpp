/**
 ******************************************************************************
 * @file    ADS1115.cpp
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

#include "ADS1115.h"

static void ADS1115_writeConfig(ADS1115_t *dev, uint16_t config)
{
    dev->wire->beginTransmission(dev->i2cAddress);
    dev->wire->write(ADS1115_REG_CONFIG);
    dev->wire->write((config >> 8) & 0xFF);
    dev->wire->write(config & 0xFF);
    dev->wire->endTransmission();
}

static int16_t ADS1115_readConversion(ADS1115_t *dev)
{
    dev->wire->beginTransmission(dev->i2cAddress);
    dev->wire->write(ADS1115_REG_CONVERSION);
    dev->wire->endTransmission();
    dev->wire->requestFrom(dev->i2cAddress, (uint8_t)2);
    int16_t val = ((int16_t)dev->wire->read() << 8) | dev->wire->read();
    return val;
}

void ADS1115_begin(ADS1115_t *dev, TwoWire *wire, uint8_t address)
{
    dev->wire = wire;
    dev->i2cAddress = address;
    dev->gain = ADS1115_PGA_2_048V;
    dev->dataRate = ADS1115_DR_128SPS;
    dev->wire->begin();
}

void ADS1115_setGain(ADS1115_t *dev, ADS1115_PGA_t gain)
{
    dev->gain = gain;
}

void ADS1115_setDataRate(ADS1115_t *dev, ADS1115_DataRate_t rate)
{
    dev->dataRate = rate;
}

int16_t ADS1115_readRaw(ADS1115_t *dev, ADS1115_Mux_t channel)
{
    uint16_t config = 0;
    config |= (0x01 << 15);                       // Start single conversion
    config |= ((uint16_t)channel & 0x07) << 12;   // MUX
    config |= ((uint16_t)dev->gain & 0x07) << 9;  // PGA
    config |= (0x01 << 8);                        // Single-shot mode
    config |= ((uint16_t)dev->dataRate & 0x07) << 5;
    config |= 0x03;                               // Disable comparator

    ADS1115_writeConfig(dev, config);

    delay(1000 / (8 << dev->dataRate));

    return ADS1115_readConversion(dev);
}

float ADS1115_readVoltage(ADS1115_t *dev, ADS1115_Mux_t channel)
{
    int16_t raw = ADS1115_readRaw(dev, channel);
    float lsbSize = 0.0000625f; // default ±2.048V range => 0.0000625V/bit

    switch (dev->gain) {
        case ADS1115_PGA_6_144V: lsbSize = 0.0001875f; break;
        case ADS1115_PGA_4_096V: lsbSize = 0.000125f; break;
        case ADS1115_PGA_2_048V: lsbSize = 0.0000625f; break;
        case ADS1115_PGA_1_024V: lsbSize = 0.00003125f; break;
        case ADS1115_PGA_0_512V: lsbSize = 0.000015625f; break;
        case ADS1115_PGA_0_256V: lsbSize = 0.0000078125f; break;
    }

    return raw * lsbSize;
}
