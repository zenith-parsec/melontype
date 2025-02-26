#include "config.h"
#include "Adafruit_H3LIS331.h"
#include "accel.h"
#include "LED.h"

Adafruit_H3LIS331 lis = Adafruit_H3LIS331();

// acceleration offsets due to gravity
float accelOffsetX;
float accelOffsetY;
float accelOffsetZ;

float accelAngle = 0;
float accelMag = 0;
float angularPosition = 0;  // phase was a bad global variable name.
// angularPosition this is the current position within a single rotation, represented as a number between 0 and 1.
// The initial direction is effectively randomly chosen. you can adjust the offset with the right potentiometer on the transmitter (I'm assuming you have the FS-i6x or the FS-i6 with the 10 channel firmware patch)


extern void checkSerial();
extern void updateInputs();
extern void displayState();

void initAccel() {
  setCode(0011); 
  // remove attempt to power I2C device from GPIO: powered from 3v3 directly now
  // - higher current available and it's what it's for

  if (!lis.begin_I2C(ACCEL_I2C_ADDR)) {
    Serial.println("Failed to initialize accelerometer!");
    Serial.println("Use serial commands now if you want. I don't care.");
    Serial.flush();
    while (1) {
      checkSerial();  // for command parsing and stuff.
      updateInputs();  // update the inputs
      fadeScreen(1,4);
      displayState();      
      //updateLEDs(); 
      setCode(0203); // 2 red, 0 green, 3 blue
      delay(50);  // 
      setCode( 0230); // 2 red, 3 green, 0 blue
      delay(50);  
    }
    // We stop the system if the accelerometer initialization fails.
    // it's kind of important.
  }

  // Configure the accelerometer settings
  lis.setDataRate(LIS331_DATARATE_50_HZ); // perhaps it will be more accurate at a lower rate?. only need it about this often: once every 20ms should be fast enough at least for now.
  lis.setRange(H3LIS331_RANGE_100_G); // shouldn't reach 100g before getting lock: at 54mm radius 100G equals 1300 RPM
      
  Serial.println("Accelerometer initialized successfully.");
  setCode(0020);
}

void collectCalibrationData() {
  Serial.println("Starting accelerometer calibration. Keep still.");
  // Set LEDs to blink a low-level warning for 2 seconds
  for(int i = 0; i < 6; i++)
  {
    setCode(0012); // 0r 1g 2b
    delay(200);
    setCode(0021); // 0r 2g 1b
    delay(200);
  }
  // Set LEDs to blink twice as fast while calibrating
  setCode(0102);  // red, black, black, blue, black, etc
  // Collect sensor event samples 
  // more samples = more better? Also, soon we rewrite this.

  int sampleCount = 128;  // more samples more 
  float xSum = 0, ySum = 0, zSum = 0;

  for (int i = 0; i < sampleCount; i++) {
    sensors_event_t s;
    lis.getEvent(&s);  // Retrieve accelerometer data

    // Sum the accelerometer values to calculate the average later
    xSum += s.acceleration.x;
    ySum += s.acceleration.y;
    zSum += s.acceleration.z;

    delayMicroseconds(2500);  // Wait for 2500µs between samples
  }
  setCode(0010); 

  Serial.println("Accelerometer samples collected");

  // Calculate the average values (gravity vector calibration)
  float xOffset = xSum / (float)sampleCount;
  float yOffset = ySum / (float)sampleCount;
  float zOffset = zSum / (float)sampleCount;

  // Store these offsets for future sensor calibration
  // Assume you have a global variable to store these offsets
  accelOffsetX = xOffset;
  accelOffsetY = yOffset;
  accelOffsetZ = zOffset;

  // temp until I work out how to deal with this.
  accelOffsetX = 0;
  accelOffsetY = 0;
  accelOffsetZ = 0;

  Serial.println("Calibration completed. Offsets stored.");
}

int getSituation(float x, float y, float z)
{
  return 0;
}

float filterAlpha = 0.2;
float previousFilteredReading = 0.0;

// Apply a low-pass filter to the accelerometer reading
float lowPassFilter(float currentReading) {
  previousFilteredReading = filterAlpha * currentReading + (1.0 - filterAlpha) * previousFilteredReading;
  return previousFilteredReading;
}

// Function to get the magnitude of the acceleration due to spinning (force on the X-axis)
// while rejecting gravity (the Z-axis) and acceleration (the Y-axis).


float getSpinAcceleration() {
  static float lastMagnitude;
  uint32_t now = millis();
  static uint32_t next_sensor_time = 0; // impossible initial timestep
  if(now < next_sensor_time) return lastMagnitude; // we just return the same value for 40ms at a time, then get another one.
  next_sensor_time = now + 25; // TODO: check if making this some ratio of the cycle time is smoother. 

  sensors_event_t s = {0,};
     // Get current accelerometer readings
  lis.getEvent(&s);

  // Subtract calibration offsets
  float x = s.acceleration.x - accelOffsetX;
  float y = s.acceleration.y - accelOffsetY;
  float z = s.acceleration.z - accelOffsetZ;
  
  static uint32_t nextDisp = 0;

  if(now >= nextDisp)
  {
    nextDisp = now + 1000;
    Serial.printf("accel = %f, %f, %f\n", x, y, z);
  }

  accelAngle = atan2(y, x);
  //accelMag = sqrt(x * x + y * y); // + z * z); currently ignoring Z. see below
  accelMag = abs(x); // let's try just the x component. the accelerometer is in a known position and orientation. it should be fine.
  accelMag = lowPassFilter(accelMag);
  //if(accelMag < min_rotation_G) accelMag = min_rotation_G;
  if(accelMag < 5.0)
  {
    accelMag = 0.001;
    setRGB(0x3f, 0x1f, 0x1f, 4);
  }
  lastMagnitude = accelMag;
  return accelMag;
}

