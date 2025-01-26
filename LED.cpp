#include <stdint.h>
#include "config.h"
#include "LED.h"

const int numled = 16;
const int ledcols = 36;
#include <WS2812Serial.h>

byte drawingMemory[numled*3];         //  3 bytes per LED
DMAMEM byte displayMemory[numled*12]; // 12 bytes per LED

byte screen[numled*3][ledcols];
void setPixel(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b)
{
  screen[y*3 + 0][x] = r;
  screen[y*3 + 1][x] = g;
  screen[y*3 + 2][x] = b;
}

void addPixel(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b)
{
  uint8_t rr = screen[y*3 + 0][x];
  uint8_t gg = screen[y*3 + 1][x];
  uint8_t bb = screen[y*3 + 2][x];
  int t;
  if(t = ( rr+r>=256) ) t= 255;
  screen[y*3 + 1][x] = t;
  if(t = ( gg+r>=256) ) t= 255;
  screen[y*3 + 1][x] = t;
  if(t = ( bb+r>=256) ) t= 255;
  screen[y*3 + 2][x] = t;
}


// e.g. fadeScreen(250, 255);
void fadeScreen(uint8_t a, uint8_t b)
{
  uint16_t v;
  for(int x = 0; x < ledcols; x++)
  {
    for(int y = 0; y < numled; y++)
    {
      v=screen[y*3 + 0][x] * a;
      v/=b;
      screen[y*3 + 0][x] = v&0xff;
    }
  }
}


WS2812Serial leds(numled, displayMemory, drawingMemory, LEDpin, WS2812_GRB);
IntervalTimer ledTimer;

// Variables to store the current LED states (16-bit values)
volatile uint16_t redState   = 0x0000;
volatile uint16_t greenState = 0x0000;
volatile uint16_t blueState  = 0x0000;

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

static bool rgbIsZero = false;
static bool rgbWasZero = false;

// Function to set the RGB LED states
void setRGB(uint16_t r, uint16_t g, uint16_t b, uint8_t idx) {
  // Update the red, green, and blue states with the provided values
  uint32_t color = ((r&0xff)<<16) | ((g&0xff)<< 8) | (b&0xff);
  leds.setPixel(idx, color);

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
  leds.show();
  // Increment i and mask with 0xF to keep it within 0-15 (16 steps)
  idx = (idx + 1) & 0xF;
}