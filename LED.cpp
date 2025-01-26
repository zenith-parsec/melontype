#include <stdint.h>
#include "config.h"
#include "LED.h"

const int numled = 16;
#if TEENSY == 1
#include <WS2812Serial.h>

byte drawingMemory[numled*3];         //  3 bytes per LED
DMAMEM byte displayMemory[numled*12]; // 12 bytes per LED

WS2812Serial leds(numled, displayMemory, drawingMemory, LEDpin, WS2812_GRB);
IntervalTimer ledTimer;
#endif 

#if xESP32 == 1
#include <Adafruit_NeoPixel.h>


Adafruit_NeoPixel pixels(numled, LEDpin, NEO_GRB + NEO_KHZ800);
#endif


// Variables to store the current LED states (16-bit values)
volatile uint16_t redState   = 0x0000;
volatile uint16_t greenState = 0x0000;
volatile uint16_t blueState  = 0x0000;

#if xESP32 ==1
void initLEDs() {
  Serial.println("LEDs initialized, I guess? mostly done in the neopixel initializer.");
}
#endif

#if xTEENSY == 1
void initLEDs() {
  // Configure the LED pins as outputs
  leds.begin();
  // Start the timer to call updateLEDs() regularly
  ledTimer.begin(updateLEDs, 50000);  // 50,000 microseconds = 50 milliseconds
  // doing it here coz.. it's an LED, I guess.

  pinMode(13, OUTPUT);          // this is the Teensy built-in LED
  digitalWriteFast(13, HIGH);   // turning it high so you can see reached here.
  Serial.println("LEDs initialized in interval timer.");
}
#endif
static bool rgbIsZero = false;
static bool rgbWasZero = false;

// Function to set the RGB LED states
void setRGB(uint16_t r, uint16_t g, uint16_t b, uint8_t idx) {
  // Update the red, green, and blue states with the provided values
#if xTEENSY == 1
  uint32_t color = ((r&0xff)<<16) | ((g&0xff)<< 8) | (b&0xff);
  leds.setPixel(idx, color);
#endif
#if xESP32 == 1
  pixels.setPixelColor(idx, pixels.Color(r&0xff, g&0xff, b&0xff)); 
#endif

  if(idx == 0)
  {
    redState = r;
    greenState = g;
    blueState = b;

    rgbWasZero = rgbIsZero;
    rgbIsZero = ( redState + greenState + blueState ) == 0;
  }
}

// Function to update the LED states based on bit patterns

void updateLEDs() 
{
  static uint8_t idx = 0;  // Static variable to track the current bit (0-15)
  
  if(idx || !rgbWasZero || !rgbIsZero)
  {
  // Set the LED values based on the i-th bit of each state's bitmask
    
    uint8_t rr = ((redState   >> idx) & 0x1) ? 28 :0; 
    uint8_t gg = ((greenState >> idx) & 0x1) ? 28 :0; 
    uint8_t bb = ((blueState  >> idx) & 0x1) ? 18 :0; 

    setRGB(rr, gg, bb, 0 ); 
  }
  pixels.show();
  // Increment i and mask with 0xF to keep it within 0-15 (16 steps)
  idx = (idx + 1) & 0xF;
}
