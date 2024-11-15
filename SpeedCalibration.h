#ifndef SPEED_CALIBRATION_H
#define SPEED_CALIBRATION_H

#include <Arduino.h>
#include <EEPROM.h>

class SpeedCalibration {
public:
    SpeedCalibration();
    void initializeArray();
    void updateCalibration(float throttle, uint16_t rpm);
    uint16_t getRPMForThrottle(float throttle);
    void printCalibration();

private:
    static const int ARRAY_SIZE = 100;
    static const int EEPROM_START_ADDR = 0;
    static const uint8_t VALID_SIGNATURE = 0xAA;
    static constexpr float STABLE_THRESHOLD = 0.05;
    static const int SAMPLE_BUFFER_SIZE = 10;

    float lastThrottle;
    unsigned long lastThrottleChangeTime;
    static const unsigned long STABLE_TIME_MS = 500;

    uint16_t sampleBuffer[SAMPLE_BUFFER_SIZE];
    int sampleIndex = 0;
    uint16_t averageRPM = 0;

    void writeRPMToEEPROM(int index, uint16_t rpm);
    uint16_t readRPMFromEEPROM(int index);
    void updateAverageRPM(uint16_t rpm);
};

#endif // SPEED_CALIBRATION_H