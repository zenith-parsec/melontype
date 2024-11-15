#include "SpeedCalibration.h"

SpeedCalibration::SpeedCalibration() : 
    lastThrottle(0), 
    lastThrottleChangeTime(0) 
{
    if (EEPROM.read(EEPROM_START_ADDR) != VALID_SIGNATURE) {
        initializeArray();
    }
}

void SpeedCalibration::initializeArray() {
    EEPROM.write(EEPROM_START_ADDR, VALID_SIGNATURE);
    for (int i = 0; i < ARRAY_SIZE; i++) {
        writeRPMToEEPROM(i, 0);
    }
}

void SpeedCalibration::writeRPMToEEPROM(int index, uint16_t rpm) {
    int addr = EEPROM_START_ADDR + 1 + (index * 2);
    EEPROM.write(addr, rpm >> 8);
    EEPROM.write(addr + 1, rpm & 255);
}

uint16_t SpeedCalibration::readRPMFromEEPROM(int index) {
    int addr = EEPROM_START_ADDR + 1 + (index * 2);
    uint16_t rpm = (EEPROM.read(addr) << 8) | EEPROM.read(addr + 1);
    return rpm;
}

void SpeedCalibration::updateCalibration(float throttle, uint16_t rpm) {
    int index = constrain(int(throttle * (ARRAY_SIZE - 1)), 0, ARRAY_SIZE - 1);

    if (abs(throttle - lastThrottle) > STABLE_THRESHOLD) {
        lastThrottle = throttle;
        lastThrottleChangeTime = millis();
        writeRPMToEEPROM(index, averageRPM);
        updateAverageRPM(rpm);
    } else if (millis() - lastThrottleChangeTime >= STABLE_TIME_MS) {
        updateAverageRPM(rpm);
        writeRPMToEEPROM(index, averageRPM);
        lastThrottleChangeTime = millis();
    }
}

void SpeedCalibration::updateAverageRPM(uint16_t rpm) {
    sampleBuffer[sampleIndex] = rpm;
    sampleIndex = (sampleIndex + 1) % SAMPLE_BUFFER_SIZE;

    uint32_t sum = 0;
    for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++) {
        sum += sampleBuffer[i];
    }
    averageRPM = sum / SAMPLE_BUFFER_SIZE;
}

uint16_t SpeedCalibration::getRPMForThrottle(float throttle) {
    int index = constrain(int(throttle * (ARRAY_SIZE - 1)), 0, ARRAY_SIZE - 1);
    return readRPMFromEEPROM(index);
}

void SpeedCalibration::printCalibration() {
    Serial.println("Throttle Calibration Table:");
    Serial.println("Throttle\tRPM");
    for (int i = 0; i < ARRAY_SIZE; i += 1) {
        float throttle = float(i) / (ARRAY_SIZE - 1);
        uint16_t rpm = readRPMFromEEPROM(i);
        Serial.print(throttle, 3);
        Serial.print("\t");
        Serial.println(rpm);
    }
}