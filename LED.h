#ifndef __LED_H__
#define __LED_H__
#include <Arduino.h>
// fake change so I can recommit it with impunity. Want to get rid of commit message only.
void initLEDs();
void setRGB(uint16_t r, uint16_t g, uint16_t b, uint8_t idx=0);
void updateLEDs();

#endif 