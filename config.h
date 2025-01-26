#ifndef __CONFIG_H__
#define __CONFIG_H__
#include <Arduino.h>


// deliberately not using #ifdef because I want no defaults.
// it should also fail if you try add two of them.

#define xTEENSY 0
#define xESP32 1

#if xTEENSY == 1
const int motorPin1 = 22;    // 23 // skip HardwareSerial1
const int motorPin2 = 23;    // 22 
const int ibusPin   = 21;    // 21 FAKE! Hardware5 is digital 21  if changed, update in melontype.ino too!

const int LEDpin      = 14;

const int accelGND    = 16;
const int accel3v3    = 17;
const int accelSDA    = 18;
const int accelSCL    = 19;

#endif
#if xESP32 == 1

const int LEDpin      = D1; 
const int motorPin1   = D2; 
const int motorPin2   = D3; 
const int accelSDA    = D4;
const int accelSCL    = D5;

const int otherTX     = D6;
const int ibusPin     = D7; 

void digitalWriteFast(uint8_t pin, uint8_t val);
#endif


#define ACCEL_I2C_ADDR 0x18 // default for adafruit h3lis331 board - works with 3v3 and 5v MCUs
//#define ACCEL_I2C_ADDR 0x19   // default for sparkfun h3lis331 board - works with 3v3 MCUs.

const int ESC_SERVO_RATE = 400; 
const float ZERO_THRESHOLD = 0.1;  

const int PWM_1000us = 65536 * 40 / 100;  
const int PWM_2000us = 65536 * 80 / 100;  

const float boostThreshold = 0.2;        
const float boostSpeed = 0.5;            
const unsigned long baseCycleTime = 50;  
const unsigned long coolDownTime = 100;

const float RPS_THRESHOLD = 7.0;              
const unsigned long LOOP_DELAY_MICROS = 200;  

#endif