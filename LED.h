#ifndef __LED_H__
#define __LED_H__
#include <Arduino.h>
const int numled = 16;
const int ledcols = 16;
extern byte screen[numled*3][ledcols];


void initLEDs();
void setRGB(uint16_t r, uint16_t g, uint16_t b, uint8_t idx=0);
void updateLEDs();

void setPixel(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b);
void addPixel(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b);
void fadeScreen(uint8_t m, uint8_t d);
#endif 