#ifndef __LED_H__
#define __LED_H__
#include <Arduino.h>

void initLEDs();
void setRGB(uint16_t r, uint16_t g, uint16_t b);
void updateLEDs();

#endif 