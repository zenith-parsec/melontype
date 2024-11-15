#include <stdint.h>
#include "config.h"
#include "LED.h"

// Variables to store the current LED states (16-bit values)
volatile uint16_t redState = 0xAAAA;    // Example pattern for red (alternates on/off)
volatile uint16_t greenState = 0x0F0F;  // Example pattern for green (on/off in 128ms blocks)
volatile uint16_t blueState = 0xFFFF;   // Example pattern for blue (always on)

IntervalTimer ledTimer;

void initLEDs() {
  // Configure the LED pins as outputs
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // Set initial LED states (off)
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin, LOW);

  // Start the timer to call updateLEDs() regularly
  ledTimer.begin(updateLEDs, 50000);  // 50,000 microseconds = 50 milliseconds
  // doing it here coz.. it's an LED, I guess.

  pinMode(13, OUTPUT);
  digitalWriteFast(13, HIGH);
    
  Serial.println("LEDs initialized in interval timer.");
}

// Function to set the RGB LED states
void setRGB(uint16_t r, uint16_t g, uint16_t b) {
  // Update the red, green, and blue states with the provided values

  redState = r;
  greenState = g;
  blueState = b;
}

// Function to update the LED states based on bit patterns
void updateLEDs() {
  static uint8_t i = 0;  // Static variable to track the current bit (0-15)

  // Set the LED values based on the i-th bit of each state's bitmask
  digitalWrite(redPin, !((redState >> i) & 0x1));      // Red LED
  digitalWrite(greenPin, !((greenState >> i) & 0x1));  // Green LED -- don't do green here.
  digitalWrite(bluePin, !((blueState >> i) & 0x1));    // Blue LED

  // Increment i and mask with 0xF to keep it within 0-15 (16 steps)
  i = (i + 1) & 0xF;
}
