#ifndef __LED_H__
#define __LED_H__
#include <Arduino.h>
const int numled = 12;
const int ledcols = 16;



extern byte screen[numled*3][ledcols];

void initLEDs();
void setRGB(uint16_t r, uint16_t g, uint16_t b, uint8_t idx=0);
void setCode(uint16_t code, uint8_t u_val = 255, uint8_t v_val = 0);
void updateLEDs();

// setCode will dump 12 pixels of data into the NeoPixels
// RGB RGB RGB RGB except each pixel is spread over 3 LEDs. 
// This is to let me send bigger numbers eventually.
// I also coded it to let me do some fun dynamics quite easily.
// 

// 0[r] if 1, the is reporting an error.
// 0[g] unused
// 0[b] unused
//
// 1[r] unused... we can spot errors easier if we split red and the other colors
// 1[g] if this is set, then the error is recoverable. Otherwise you need to reboot.
// 1[b] this is in startup. 
//
// 2[r] bit-2 of a 3 bit number describing the error. 
// 2[g] bit-1 of the same number.
// 2[b] bit-0 of the same number. 
//
// at least one bit must be set in pixel index 2 so you can tell it is working.

void setPixel(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b);
void addPixel(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b);
void fadeScreen(uint8_t m, uint8_t d);
#endif 