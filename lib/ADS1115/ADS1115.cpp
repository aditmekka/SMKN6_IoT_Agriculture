#include "ADS1115.h"

ADS1115::ADS1115(uint8_t addr, TwoWire *wirePort)
    : _addr(addr), _wire(wirePort), _gain(GAIN_TWO), _rate(RATE_128SPS) {}

bool ADS1115::begin() {
    _wire->beginTransmission(_addr);
    return (_wire->endTransmission() == 0);
}

void ADS1115::setGain(adsGain_t gain) {
    _gain = gain;
}

void ADS1115::setDataRate(adsDataRate_t rate) {
    _rate = rate;
}

float ADS1115::readMillivolt(uint8_t channel) {
    int16_t adc = readADC(channel);
    return adc * getMultiplier();
}

float ADS1115::readVoltage(uint8_t channel) {
    return readMillivolt(channel) / 1000.0f;
}

int16_t ADS1115::readADC(uint8_t channel) {
    if (channel > 3) return 0;

    uint16_t config = 0x8000; // Start single conversion
    config |= _gain;
    config |= _rate;
    config |= 0x0100; // Single-shot mode
    config |= 0x0003; // Comparator disable

    // Channel MUX config
    switch (channel) {
        case 0: config |= 0x4000; break; // AIN0
        case 1: config |= 0x5000; break; // AIN1
        case 2: config |= 0x6000; break; // AIN2
        case 3: config |= 0x7000; break; // AIN3
    }

    writeRegister(ADS1115_REG_CONFIG, config);
    delay(9); // Wait for conversion (~8ms at 128SPS)

    return (int16_t)readRegister(ADS1115_REG_CONVERSION);
}

int16_t ADS1115::readDifferential_0_1() {
    uint16_t config = 0x8000 | 0x0000 | _gain | _rate | 0x0100 | 0x0003;
    writeRegister(ADS1115_REG_CONFIG, config);
    delay(9);
    return (int16_t)readRegister(ADS1115_REG_CONVERSION);
}

int16_t ADS1115::readDifferential_2_3() {
    uint16_t config = 0x8000 | 0x3000 | _gain | _rate | 0x0100 | 0x0003;
    writeRegister(ADS1115_REG_CONFIG, config);
    delay(9);
    return (int16_t)readRegister(ADS1115_REG_CONVERSION);
}

void ADS1115::writeRegister(uint8_t reg, uint16_t value) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write((uint8_t)(value >> 8));
    _wire->write((uint8_t)(value & 0xFF));
    _wire->endTransmission();
}

uint16_t ADS1115::readRegister(uint8_t reg) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->endTransmission();
    _wire->requestFrom(_addr, (uint8_t)2);
    uint16_t result = ((_wire->read() << 8) | _wire->read());
    return result;
}

float ADS1115::getMultiplier() {
    switch (_gain) {
        case GAIN_TWOTHIRDS: return 0.1875F; // mV per bit
        case GAIN_ONE:       return 0.125F;
        case GAIN_TWO:       return 0.0625F;
        case GAIN_FOUR:      return 0.03125F;
        case GAIN_EIGHT:     return 0.015625F;
        case GAIN_SIXTEEN:   return 0.0078125F;
    }
    return 0.0625F;
}
