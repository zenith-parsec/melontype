// this is terrible code. not even being self-deprecating. so use this at your own risk.
// read the line above this again, and if you keep going, remember that it's all your fault now.

// previous header comment/documentation is now in "original_header.h"

#include <IBusBM.h>
#include "Adafruit_H3LIS331.h"
#include "config.h"
#include "LED.h"
#include "font.h"

#include <ArduinoEigen.h>

// Declare objects and variables
IBusBM ibus;  // IBus object for the radio receiver
Adafruit_H3LIS331 lis = Adafruit_H3LIS331();

void pwmInterruptHandler();

//HardwareSerial &iBusSerialPort = Serial2;  // digital pin 7: update ibusPin in config.h too!!!!
HardwareSerialIMXRT &iBusSerialPort = Serial5;  // digital pin 21: update ibusPin in config.h ::::: new board only!!
IntervalTimer pwmTimer;

bool pwmIntCalled;
bool doResetStats = false;

// acceleration offsets due to gravity
float accelOffsetX = 0.0;
float accelOffsetY = 0.0;
float accelOffsetZ = 0.0;

volatile uint32_t last_ibus_seen_millis;
volatile uint8_t last_cnt_rec;
volatile uint8_t last_cnt_poll;

// Global variables for motor throttles
float motor1Throttle = 0.0;
float motor2Throttle = 0.0;
int lastMotor1 = -1;  // make them different to get the update.
int lastMotor2 = -1;  // ditto. (renamed, sorry.)


// Variables to store inputs
volatile float stickVert = 0.0;      // up/down = ch0
volatile float stickHoriz = 0.0;     // left/right = ch1
volatile float stickAngle = 0.0;     // Stick angle as a fraction of a circle
volatile float stickLength = 0.0;    // Length of the stick vector
volatile float powerInput = 0.0;       // Throttle input = ch2
volatile float sidewaysInput = 0.0;  // Sideways input = ch3
volatile float radiusInput = 0.0;    // Radius input = ch4
volatile float radiusSize = 0.01;    // Mapped radius size (0.1 cm to 20 cm) in meters
volatile float headingOffset = 0.0;  // offset around circle heading light should be = ch5
volatile bool reverseM1 = false;
volatile bool reverseM2 = false;
volatile int sw3pos = 0;

// map the throttle input so it spends more time in the low areas.
float stretchThrottle(float throttle) {
  float s = throttle > 0 ? 1.f : -1.f;
  return s * (throttle * throttle);
}

int throttleToPWM(float input, bool stretch) {
  // Constrain the throttle value between 0 and 1
  input = constrain(input, -1.0, 1.0);
  // do non-linear stretch or not?
  if (stretch) {
    input = stretchThrottle(input);
  }
  // Map the throttle value (-1 to 1) to the PWM duty cycle values (1000us and 2000us converted to the 400Hz PWM)
  return map(input, -1, 1, PWM_1000us, PWM_2000us);
}

// Modified setThrottle to only store the desired throttle, no analogWrite() here
void setThrottle(float throttle1, float throttle2, bool stretch = false) {
  if (throttle1 == 0 && throttle2 == 0) {
    lastMotor1 = -2;  // -2 is impossible to actually set
    lastMotor2 = -2;  // force an update. hack. so? shut up.
  }

  if (reverseM1) throttle1 = -throttle1;
  if (reverseM2) throttle2 = -throttle2;

  motor1Throttle = throttleToPWM(throttle1, stretch);
  motor2Throttle = throttleToPWM(throttle2, stretch);
  pwmIntCalled = false;
}


unsigned long endBoost[2] = { 0, 0 };
bool isBoost[2] = { true, true };  // Flag to track whether we're sending modSpeed or 0

// Function to modulate throttle for low-speed control
// this is intended to give a small boost when starting up if the throttle is below 0.2
// to make it easier to keep moving at low speed.
float modulateThrottle(float inputThrottle, int idx) {
  unsigned long now = millis();
  if (now > endBoost[idx] + coolDownTime) {
    endBoost[idx] = 0;
    return inputThrottle;
  }
  if (now > endBoost[idx] || inputThrottle > boostThreshold) {
    isBoost[idx] = false;
    return inputThrottle;
  }
  if (endBoost[idx] == 0) {
    isBoost[idx] = true;
    endBoost[idx] = now + baseCycleTime;
  }
  if (!isBoost[idx]) return inputThrottle;
  return boostSpeed;
}


// Function to handle tank drive logic
// convert from 2-axis stick input into two channel "tank drive" motor output.
// (converts stick to "tank mode" output)
void handleTankDrive() {
  float x = stickHoriz / 2.0;  // Horizontal stick input -0.5 - 0.5
  float y = stickVert / 2.0;   // Vertical stick input   -0.5 - 0.5

  // Calculate motor control values based on stick input
  float m1 = x + y;  // Motor 1 throttle
  float m2 = x - y;  // Motor 2 throttle

  // this one is for trying to stop it taking off so fast in tank mode.
  m1 = modulateThrottle(m1, 0);
  m2 = modulateThrottle(m2, 1);

  pwmIntCalled = false;
  setThrottle(m1, m2, true);  // Set new desired throttle for motors
}


#define motorWrite analogWrite

// Interrupt handler to send PWM signal (this triggers the actual motor control)
void pwmInterruptHandler() {
  // Apply throttle to PWM signal value
  int pwmValueMotor1 = motor1Throttle;
  int pwmValueMotor2 = motor2Throttle;

  // Write the actual PWM value to the motors
  if (lastMotor1 != pwmValueMotor1) {
    motorWrite(motorPin1, pwmValueMotor1);
    lastMotor1 = pwmValueMotor1;
  }
  if (lastMotor2 != pwmValueMotor2) {
    motorWrite(motorPin2, pwmValueMotor2);
    lastMotor2 = pwmValueMotor2;
  }
  pwmIntCalled = true;
}

// Initialize ESCs and set the throttle to 0
void initESCs() {
  // Set PWM resolution to 16 bits
  // Set the PWM frequency for each motor pin to ESC servo rate (ESC_SERVO_RATE)
  analogWriteResolution(16);
  analogWriteFrequency(motorPin1, ESC_SERVO_RATE);
  analogWriteFrequency(motorPin2, ESC_SERVO_RATE);

  pwmTimer.begin(pwmInterruptHandler, 1000000 / ESC_SERVO_RATE);
  // Set initial throttle to 0 for both motors
  setThrottle(0, 0);

  Serial.println("ESCs initialized, throttle set to 0.");
}

// for debugging/development use. add stuff you want to control via serial here.
void checkSerial() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'p') {
      // print stuff
    }
    if (cmd == 'c') {
      // clear stuff
    }
    if (cmd == 'q') {
      Serial.print("Entering infinite loop so you can update or whatever. maybe enjoy the lights?\n");
      // infinite loop... might help uploading if it was in a crash loop? 
      while(1) {  delay(1); }
    }
  }
}

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
  lis.setDataRate(LIS331_DATARATE_400_HZ); // perhaps it will be more accurate at a lower rate. only need it about this often
  lis.setRange(H3LIS331_RANGE_100_G);
  // lis.setLPFCutoff(lis331_lpf_cutoff_t cutoff) won't help: it doesn't work in NORMAL mode.
      
  Serial.println("Accelerometer initialized successfully.");
  setCode(0020);
}

Eigen::Matrix3f rotationMatrix;

void generateRotationMatrix() {
  Eigen::Vector3f gravityVector(accelOffsetX, accelOffsetY, accelOffsetZ);
  // Normalize gravity vector (Z-axis)
  Eigen::Vector3f zAxis = gravityVector.normalized();

  // Assume the robot will spin later; define Y (forward vector)
  Eigen::Vector3f yAxis(0, 1, 0);  // Placeholder forward vector

  // Compute X (rotation vector, perpendicular to Z and Y)
  Eigen::Vector3f xAxis = yAxis.cross(zAxis).normalized();

  // Recompute Y to ensure orthogonality
  yAxis = zAxis.cross(xAxis).normalized();

  // Create the rotation matrix
  rotationMatrix.col(0) = xAxis;  // X
  rotationMatrix.col(1) = yAxis;  // Y
  rotationMatrix.col(2) = zAxis;  // Z
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

  int sampleCount = 500;  // more samples more 
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

  generateRotationMatrix(); 

  Serial.println("Calibration completed. Offsets stored.");
}
// rc_signal_is_healthy() human readable flow:
// start of by intending to say we are not healthy when we return.
// if we have seen a message in the last 500ms, 
//   then we're now going to say we're healthy when we return.
// if we haven't seen one,
//   then we'll check for one.
// if either of these ibus counters have changed since the last time we checked,
//   then we're now going to say we are healthy when we return.
//   we also update our copy of those counters.
// return our health status

bool rc_signal_is_healthy() {
  uint32_t now = millis();
  bool res = false;
  if (now - last_ibus_seen_millis < 500) {
    res = true;
  } else {
    ibus.loop();
  }
  if (last_cnt_rec != ibus.cnt_rec || last_cnt_poll != ibus.cnt_poll) {
    res = true;
    last_cnt_rec = ibus.cnt_rec;
    last_cnt_poll = ibus.cnt_poll;
    last_ibus_seen_millis = now;
  }
  return res;
}

void initIBus() {
  // We need to use pin 20 so tell the hardward to use a different pin we don't care about to transmit.
  // if we don't, the FastLED code won't work.
  // once it works, variables again
  Serial5.setTX(39); // this is an alternate transmit pin for hardware serial port 5. 
  Serial5.setRX(21); // default  
  // Initialize the radio receiver using the IBusBM library
  ibus.begin(iBusSerialPort);
  setCode(0021); 
  Serial.println("IBus receiver initialized.");
  ibus.loop();  // get the first value?
  // Validate that commands are being received
  while (!ibus.readChannel(0)) {
    checkSerial();
    setCode( 0222); 
    ibus.loop();  // trying to update values
    Serial.println("Waiting for remote signal...");

    delay(100);
    setCode(405); 
    delay(100); // we're async for LEDs so this is fine
  }
  Serial.println("Remote signal received.");

  // Ensure that throttle is in the down position (assuming throttle is on channel 2)
  int throttle = ibus.readChannel(2);
  uint32_t endThrottleCheck = millis() + 100;
  while (throttle > 1050 || millis() < endThrottleCheck) {          // is the throttle up?
    checkSerial();
    setCode(0322);
    Serial.println(throttle);
    if(millis() > endThrottleCheck) Serial.println("Please lower throttle to zero.");
    throttle = ibus.readChannel(2);
    delay(250); 
    setCode(0304);
    delay(250);   // plenty of time to lower it.
    ibus.loop();  // trying to update values
  }
  setCode(0);
  Serial.println("Throttle confirmed down.");
}

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  if (CrashReport) {
    Serial.print(CrashReport);
    delay(5000);
    checkSerial();
  }
  
  // Initialize LED subsystem: immediate start feedback
  initLEDs();
  Serial.println("LED subsystem initialized.");

  // debugging test pattern black white cyan magenta yellow blue green red
  setRGB(32, 32, 32, 0);
  setRGB(28, 28, 18, 4);
  setRGB( 0, 28, 18, 5);
  setRGB(28,  0, 18, 6);
  setRGB(28, 28,  0, 7);
  setRGB( 0,  0, 18, 8);
  setRGB( 0, 28,  0, 9);
  setRGB(28,  0,  0, 10);

  // Initialize ESCs by sending zero throttle to both motors
  initESCs();
  Serial.println("ESCs initialized and motors stopped.");

  Serial.println("Initializing Meltybrain Robot... send 'q' within 2 seconds to enter infinite loop");
  delay(2000);

  checkSerial();

  // Initialize IBus so we can monitor the TX without an accelerometer.
  initIBus();
  Serial.println("IBus initialized.");
  
  // Initialize accelerometer and collect calibration data
  initAccel();               // Function to initialize the accelerometer
  collectCalibrationData();  // Function to calibrate and store offset
  Serial.println("Accelerometer initialized and calibrated. Maybe. See above.");

  // Additional setup steps can go here
  Serial.println("Setup complete. Ready for operation.");
}

// Function to check if throttle is nearly zero
bool isThrottleNearlyZero() {
  return powerInput < ZERO_THRESHOLD;
}

// Function to check if in tank drive mode
bool isInTankDriveMode() {
  // fixed sideways to check from left side instead of center. not sure why I missed this. ;/
  float s = (0.95 + sidewaysInput);
  return isThrottleNearlyZero() && (s < 0.25);
}

void updateInputs() {
  // Read the vertical and horizontal direction control (channel 0 and 1)
  float vert = ibus.readChannel(0);   // Channel 0 (vertical)
  float horiz = ibus.readChannel(1);  // Channel 1 (horizontal)

  // Convert to centered values (from 1000-2000 to -1.0 to 1.0)
  stickVert  = map(vert , 1000.0, 2000.0, -1.0, 1.0);
  stickHoriz = map(horiz, 1000.0, 2000.0, -1.0, 1.0);

  // Calculate stick angle in radians as a fraction of a circle
  // ACTUALLY swapped sign of output and argume so circle starts at top and goes clockwise on stick.
  // this might make the controls work better? 
  stickAngle = -atan2(-stickVert, stickHoriz) / (2 * PI); 

  // Calculate stick length (magnitude) using Pythagoras' theorem
  float sl = sqrt(sq(stickVert) + sq(stickHoriz));
  stickLength = min(sl, 1.0f);  // lets's just saturate it if it's in the corners.

  if (stickLength < 0.05) {
    stickAngle = 0;
    stickLength = 0;
  }

  // Read throttle (channel 2) and map it to 0 to 1
  float ch2 = ibus.readChannel(2);
  powerInput = map(ch2, 1000.0, 2000.0, 0.0, 1.0);

  // Read sideways input (channel 3) and map it to -1 to 1
  float ch3 = ibus.readChannel(3);
  sidewaysInput = map(ch3, 1000, 2000, -1.0, 1.0);

  // Read radius input (channel 4) and map it to 1mm to 100mm
  radiusInput = ibus.readChannel(4);
  radiusSize = map(radiusInput, 1000, 2000, 0.001, 0.100);  // Result in meters (1mm to 100 mm)

  // Read LED offset input (channel 5).
  float ch5 = ibus.readChannel(5);
  headingOffset = map(ch5, 1000.0, 2000.0, -0.5, 0.5);

  // left and right switches control if we should reverse M1 and M2.
  // DO NOT SWITCH THESE WHILE MOVING. Probably. It might be cool, but it might blow up your robot.
  // This is to try mitigate "plugged in a motor the wrong way round" issues.
  reverseM1 = (ibus.readChannel(6) > 1250);
  reverseM2 = (ibus.readChannel(9) > 1250);

  int ch8 = ibus.readChannel(8);
  sw3pos = abs(ch8 - 1000) / 500;  // assuming it doesn't go too far below 1000, this should work
}

void displayState() {
  static uint32_t lastIdleMessage = 0;
  uint32_t now = millis();
  if (now - lastIdleMessage > 250) {
    lastIdleMessage = now;

    Serial.print("m1Throttle: ");
    Serial.print(motor1Throttle);
    Serial.print("  m2Throttle: ");
    Serial.print(motor2Throttle);
    Serial.print("  pwmIntCalled: ");
    Serial.print(pwmIntCalled);
    Serial.print("  stickVert: ");
    Serial.print(stickVert);
    Serial.print("  stickHoriz: ");
    Serial.print(stickHoriz);
    Serial.print("  stickAngle: ");
    Serial.print(stickAngle);
    Serial.print("  stickLength: ");
    Serial.print(stickLength);
    Serial.print("  power: ");
    Serial.print(powerInput);
    Serial.print("  sidewaysInput: ");
    Serial.print(sidewaysInput);
    Serial.print("  radiusInput: ");
    Serial.print(radiusInput);
    Serial.print("  radiusSize: ");
    Serial.print(radiusSize * 1000.0f);
    Serial.print("mm  headingOffset: ");
    Serial.print(headingOffset);
    Serial.print("  reverseM1: ");
    Serial.print(reverseM1);
    Serial.print("  reverseM2: ");
    Serial.print(reverseM2);
    Serial.print("  sw3pos: ");
    Serial.print(sw3pos);
    Serial.println();
  }
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

float accelAngle = 0;
float accelMag = 0;
float angularPosition = 0; // bad global variable name, but it's the current phase

// Function to get the magnitude of the acceleration due to spinning (force on the X-axis)
// while rejecting gravity (the Z-axis) and acceleration (the Y-axis).
float getSpinAcceleration() {
  static sensors_event_t s;
  static uint32_t last_sensor_time = 0xface1e55; // impossible initial timestep
  static float lastMagnitude;
  uint32_t now = millis();
  if(now == last_sensor_time) return lastMagnitude; // check is for equality. can't do range. also need to change initial value if condition changed.
  last_sensor_time = now;
     // Get current accelerometer readings
  lis.getEvent(&s);

  // Subtract calibration offsets
  float x = s.acceleration.x - accelOffsetX;
  float y = s.acceleration.y - accelOffsetY;
  float z = s.acceleration.z - accelOffsetZ;
  
  static uint32_t nextDisp = 0;

  if(now >= nextDisp)
  {
    nextDisp = now + 250;
    Serial.printf("accel = %f, %f, %f\n", x, y, z);
  }

  accelAngle = atan2(y, x);
  accelMag = sqrt(x * x + y * y); // + z * z); currently ignoring Z. see below
  accelMag = lowPassFilter(accelMag);
  if(accelMag < min_rotation_G) accelMag = min_rotation_G;
  lastMagnitude = accelMag;
  return accelMag;
}

// Function to calculate revolutions per second based on spin acceleration
float calculateRPS() {
  // Get the spin acceleration (centripetal force)
  float spinAccel = getSpinAcceleration();

  // Use the dynamically updated radiusSize as the spin radius (in meters)
  float spinRadius = radiusSize;  // radiusSize is in meters. expected range is from 0.001m to 0.1m

  if (spinRadius < 0.001) spinRadius = 0.001;
  // Calculate angular velocity (omega) in radians per second
  float omega = sqrt(spinAccel / spinRadius);

  // Convert angular velocity to RPS
  float rps = (omega) / (2 * PI);

  return rps;  // Return the calculated RPS
}

unsigned long usRevStartTime = 0;  // Time in microseconds when the revolution started

unsigned long prevRevTimeMicros = 1000000 / 400;  // Store the previous rotation period
unsigned long prevTimeMicros = 0;                 // Store the last timestamp

// the input is the cosine of ph1 or ph2
// the output will be a shaped cosine,
//  higher up in the top than the bottom.
float reshapeCos(float a) {
  float b = a - 1.0f;
  float c = 1 - (b * b) * 0.5;
  return c;
}

void updateLEDDisplay(float phase, float rps, float throttle) {
    uint8_t columnData[8];  // Assuming 12 LEDs per column
    
    // Display RPM in first half of rotation in blue
    if (phase < 0.5) {
        uint16_t rpm = (uint16_t)(rps * 60.0f);  // Convert RPS to RPM
        getColumnData(rpm, phase * 2.0f, false, columnData);
        for (int i = 0; i < 8; i++) {
            if (columnData[i]) {
                setRGB(0, 0, 255, i+4);  // Blue for RPM
            } else {
                setRGB(0, 0, 0, i+4);    // Off if no pixel
            }
        }
    }
    // Display throttle in second half of rotation in green
    else {
        uint16_t throttleDisplay = (uint16_t)(throttle * 100.0f);  // Scale throttle to percentage
        getColumnData(throttleDisplay, phase * 2.0f - 1.0f, false, columnData);
        for (int i = 0; i < 8; i++) {
            if (columnData[i]) {
                setRGB(0, 255, 0, i+4);  // Green for throttle
            } else {
                setRGB(0, 0, 0, i+4);    // Off if no pixel
            }
        }
    }
}


void copyScreen()
{
  int now_pos = (int)(ledcols * fmod(angularPosition + accelAngle, 1.0)); // angle relative to phase, i think?
  for(int i = 4; i < numled; i++)
  {
    uint8_t r = screen[i * 3 + 0][now_pos];
    uint8_t g = screen[i * 3 + 1][now_pos];
    uint8_t b = screen[i * 3 + 2][now_pos];
    setRGB(r, g, b, i);
  }
}  

// it also does so much else. working on refactoring it. 
// TODO: refactor handleMeltybraindrive()
void handleMeltybrainDrive() {

  // Get the current time in microseconds
  static unsigned long usRevStartTime = micros();
  // Enter loop for an entire revolution
  while (true) {
    if (!rc_signal_is_healthy()) {  // we need to bail this loop if we lose signal.
//      return;                       // next loop() call will zero throttle etc // lies. just force power to zero.
      stickLength = 0.0;
      powerInput = 0.0;
    }
    else updateInputs();

    // Calculate revolutions per second (RPS)
    float rps = calculateRPS();

    // Calculate the time for one complete revolution in microseconds
    unsigned long revTimeMicros = (unsigned long)(1000000.0f / rps);

    unsigned long now = micros();
    // When revTimeMicros is updated to a new value:

    // Calculate the phase angle in the previous rotation period (0 to 1)
    angularPosition = (float)(now - usRevStartTime) / prevRevTimeMicros; 

    if (revTimeMicros != prevRevTimeMicros) {
      unsigned long elapsedTime = now - prevTimeMicros;

      float oldAngularPosition = angularPosition;
      // Account for elapsed time since last step
      oldAngularPosition += (float)elapsedTime / prevRevTimeMicros;

      // Normalize phase to 0-1 range
      oldAngularPosition = fmod(oldAngularPosition, 1.0);
      angularPosition = oldAngularPosition;

      // Convert the phase to the new rotation period
      usRevStartTime = now - (unsigned long)(oldAngularPosition * revTimeMicros);

      // Update previous values
      prevRevTimeMicros = revTimeMicros;
      prevTimeMicros = now;

    }


    // int now_pos = (int)(ledcols * fmod(phase + accelAngle  , 1.0));

    // uint8_t mag = accelMag  ;
    // int i_mag = (int)(log(accelMag) * 2.0);
    // if(i_mag > ledcols - 1) i_mag = ledcols -1;
    // i_mag += 4;
    // addPixel(now_pos, i_mag, 0, 0, mag);

    // copyScreen();

    // Calculate the time passed in the current revolution
    unsigned long currentTimeMicros = now - usRevStartTime;
    // If the current time exceeds the revolution time, exit the loop
    if (currentTimeMicros >= revTimeMicros) {
      usRevStartTime += revTimeMicros;
      break;
    }

    // Calculate the adjusted heading
    float adjustedHeading = headingOffset + stickAngle;
    adjustedHeading = fmod(adjustedHeading, 1.0);
    if (adjustedHeading < 0.0) {
      adjustedHeading += 1.0;
    }

    // Calculate timeToForward and timeToBackward
    float timeToForward = adjustedHeading * revTimeMicros;

    // for phased based speed transition for translation
    float m2PhaseOffset = 0.5;
    float timeToBackward = timeToForward + m2PhaseOffset * revTimeMicros;
    if (timeToBackward > revTimeMicros) {
      timeToBackward -= revTimeMicros;
    }

    // Calculate width for motor activation checks
    float speedLen = stickLength;

    // Set the throttle for each motor using the global throttle value
    // modulated by a cosine function
    float ph1 = ((currentTimeMicros - timeToForward) / revTimeMicros) * M_PI * 2.0f;
    float ph2 = ((currentTimeMicros - timeToBackward) / revTimeMicros) * M_PI * 2.0f;

    // reshape the cosine function so it spends more time with bigger numbers on the positive side
    // to try balance actual behavior of motors.
    float cos_ph1 = reshapeCos(cos(ph1));
    float cos_ph2 = reshapeCos(cos(ph2));

    float mixFrac = 0.25f;
    float baseLevel = powerInput + speedLen * mixFrac;
    if (baseLevel > 1) baseLevel = 1;
    if (baseLevel > (1.0f - mixFrac)) {
      mixFrac = 1.0f - baseLevel;
    }
    if (baseLevel < mixFrac) {
      mixFrac = baseLevel;
    }
    float th1 = (cos_ph1 * mixFrac * speedLen) + baseLevel;
    float th2 = (cos_ph2 * mixFrac * speedLen) + baseLevel;

    setThrottle(th1, -th2);  // switched signs for motor2.

    // Update the LEDs to visualize the current heading.
    // ph1 is the center front of the orbit.
    // cos(ph1 * 2pi)  for the region 45 degrees each side of center front are all greater
    // than 0.7071. This illuminates 1/4 of the arc for that section of each revolution.
    // also coincides with motor pulses.

    bool blueLEDOn  = (cos_ph1 > 0.7071067811) ; // 45 degrees each side
    bool greenLEDOn = cos(accelAngle) > 0.7071067811;

    // setRGB(x, y, z, 0) sets the first 4 LEDs to the same color. (the ones on the far end are first.)
    uint8_t r = 0;
    uint8_t b = blueLEDOn ? 255 : 0;
    uint8_t g = 0;// greenLEDOn ? 255 : 0;
    setRGB(r, g, b, 0);

    // draws 8 colors in a circle: black, red, green, yellow, blue, magenta, cyan, white
    int pp = (int)fmod(angularPosition * 8.0, 8.0);
    r= pp & 4 ? 32 : 0;
    g= pp & 2 ? 32 : 0;
    b= pp & 1 ? 32 : 0;
    setRGB(r, g, b, 6);

    // this should display numbers.
    updateLEDDisplay(angularPosition, rps, powerInput);

    updateLEDs(); // manually calling it here so we get it faster. lock seems to work fine.
    static unsigned long lastBreakCheck = 0;
    if (now - lastBreakCheck > 1000000) { // 1 second
        lastBreakCheck = now;
        break;
    }
    if(isInTankDriveMode()) break;

  } // end of handleMeltybrainDrive while(1) loop
}

void loop() {
  // Update the IBus object for Teensy
  checkSerial();
  ibus.loop();
  fadeScreen(3,5);
  copyScreen();
  if (!rc_signal_is_healthy()) {
    ibus.loop();
    setThrottle(0, 0);
    setCode(0402, (millis() >> 3)&0x3f ); // rrrr++bb
    delay(100);
    setCode(0222, (millis() >> 3)&0x3f ); // rr+gg+bb
    delay(100);
    setCode(0204, (millis() >> 3)&0x3f ); // rr++bbbb
    delay(100);  // stop the speedy scroll
    setCode(0222, (millis() >> 3)&0x3f ); // rr+gg+bb
    delay(100);
    displayState();
    return;
  }
  updateInputs();
  if (isInTankDriveMode()) {
    setCode(0202);
    
    handleTankDrive();
    delay(5);  // give us time to do _something_. Driving around doesn't
    displayState();
    return;
  }

  if (isThrottleNearlyZero()) {
    setRGB(0x70, 0x70, 0x00, 5); // "zero throttle" detected so LED[9] = yellow
    setThrottle(0, 0);
  } else {
    setCode(0);  // all off? this may need to change
  }
  handleMeltybrainDrive();
  
  displayState();
}