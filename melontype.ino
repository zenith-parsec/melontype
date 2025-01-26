// this is terrible code. not even being self-deprecating. so use this at your own risk.
// read the line above this again, and if you keep going, remember that it's all your fault now.

// previous header comment/documentation is now in "original_header.h"

#include <IBusBM.h>
#include "Adafruit_H3LIS331.h"
#include "config.h"
#include "LED.h"

#include <ArduinoEigen.h>

// Declare objects and variables
IBusBM ibus;  // IBus object for the radio receiver
Adafruit_H3LIS331 lis = Adafruit_H3LIS331();

void ibusLoop() {
  ibus.loop();
}

void pwmInterruptHandler();

//HardwareSerial &iBusSerialPort = Serial2;  // digital pin 7: update ibusPin in config.h too!!!!
HardwareSerial &iBusSerialPort = Serial5;  // digital pin 21: update ibusPin in config.h ::::: new board only!!
IntervalTimer pwmTimer;

bool pwmIntCalled;
bool doResetStats = false;

// acceleration offsets due to gravity
float accelOffsetX;
float accelOffsetY;
float accelOffsetZ;

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
volatile float throttle = 0.0;       // Throttle input = ch2
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

int throttleToPWM(float throttle, bool stretch) {
  // Constrain the throttle value between 0 and 1
  throttle = constrain(throttle, -1.0, 1.0);
  // do non-linear stretch or not?
  if (stretch) {
    // throttle = stretchThrottle(throttle);
  }
  // Map the throttle value (-1 to 1) to the PWM duty cycle values (1000us and 2000us converted to the 400Hz PWM)
  return map(throttle, -1, 1, PWM_1000us, PWM_2000us);
}

// Modified setThrottle to only store the desired throttle, no analogWrite() here
void setThrottle(float throttle1, float throttle2, bool stretch = true) {
  if (throttle1 == 0 && throttle2 == 0) {
    lastMotor1 = -1;
    lastMotor2 = -1;  // force an update. hack. so? shut up.
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
  float x = stickHoriz / 5.0;  // Horizontal stick input 0 - 0.2
  float y = stickVert / 5.0;   // Vertical stick input   0 - 0.2

  // Calculate motor control values based on stick input
  float m1 = x + y;  // Motor 1 throttle
  float m2 = x - y;  // Motor 2 throttle

  // this one is for trying to stop it taking off so fast in tank mode.
  m1 = modulateThrottle(m1, 0);
  m2 = modulateThrottle(m2, 1);

  pwmIntCalled = false;
  setThrottle(m1, m2, false);  // Set new desired throttle for motors
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
      // infinite loop... might help uploading if it was in a crash loop? 
      while(1) {  delay(1); }
    }
  }
}

void initAccel() {
  pinMode(accel3v3, OUTPUT);
  pinMode(accelGND, OUTPUT);
  digitalWrite(accel3v3, HIGH);
  digitalWrite(accelGND, LOW);
  delay(10);

  if (!lis.begin_I2C(ACCEL_I2C_ADDR)) {
    Serial.println("Failed to initialize accelerometer!");
    Serial.println("Use serial commands now if you want. I don't care.");
    setRGB(0x7777, 0x8888, 0x0000);  // rrrgrrrgrrrgrrrg
    Serial.flush();
    while (1) {
      checkSerial();  // for command parsing and stuff.
                      // crashes here, but doesn't when ibus.loop() is removed. then it crashes in delay() below.
      ibus.loop();
      updateInputs();  // update the inputs
      displayState();
      updateLEDs(); // still crashes. ;/
      delay(250);  // stop the speedy scroll
      // crash in delay is crash in interrupt handler ;/
    }
    // We stop the system if the accelerometer initialization fails.
    // it's kind of important.
  }

  // Configure the accelerometer settings
  lis.setDataRate(LIS331_DATARATE_400_HZ);
  lis.setRange(H3LIS331_RANGE_200_G);
  lis.setLPFCutoff(LIS331_LPF_292_HZ);

  Serial.println("Accelerometer initialized successfully.");
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
  Serial.println("about to start calibration... oh noes!");
  // Set LEDs to blink a low-level warning for 2 seconds
  setRGB(0x739c, 0x1ce7, 0x0000);  // 0b0111001110011100, 0b0001110011100111, 0b0000000000000000
  delay(2000);                     // red and green 3 on, 2 off out of phase. slow blink.
  setRGB(0x6666, 0x9999, 0x0000);  // 0b0110011001100110, 0b1001100110011001, 0b0000000000000000
  delay(2000);                     // red and green 2 on 2 off. medium blink.
  Serial.println("Starting calibration. Keep still.");
  // Set LEDs to blink twice as fast while calibrating
  setRGB(0x5555, 0xAAAA, 0x0000);  // 0b0101010101010101, 0b1010101010101010, 0b0000000000000000
  // red and green 1 on 1 off fast blink.
  // Collect sensor event samples
  int sampleCount = 150;  // Between 100 and 200 samples
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
  Serial.println("Accelerometer samples collected");

  // Calculate the average values (gravity vector calibration)
  float xOffset = xSum / sampleCount;
  float yOffset = ySum / sampleCount;
  float zOffset = zSum / sampleCount;

  // Store these offsets for future sensor calibration
  // Assume you have a global variable to store these offsets
  accelOffsetX = xOffset;
  accelOffsetY = yOffset;
  accelOffsetZ = zOffset;

  generateRotationMatrix();

  setRGB(0x0000, 0x0007, 0x0000);  // 0, 0b0000000000000111, 0  little green blip, no red or blue.
  Serial.println("Calibration completed. Offsets stored.");
}

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
  // Initialize the radio receiver using the IBusBM library
  ibus.begin(iBusSerialPort);
  Serial.println("IBus receiver initialized.");
  ibus.loop();  // get the first value?
  // Validate that commands are being received
  while (!ibus.readChannel(0)) {
    setRGB(0xf0f6, 0x0001, 0x0f00);  // red long on, long off, long on, very short off, short on, very short off. blue blinks during first gap.
    Serial.println("Waiting for remote signal...");

    ibus.loop();  // trying to update values
    delay(100);   // stop spamming serial
  }
  Serial.println("Remote signal received.");

  // Ensure that throttle is in the down position (assuming throttle is on channel 2)
  int throttle = ibus.readChannel(2);
  while (throttle > 1050) {          // is the throttle up?
    setRGB(0xf6f6, 0xffff, 0x0f00);  // red long on, long off, long on, very short off, short on, very short off. blue blinks during first gap. now green is always on. weird.
    Serial.println(throttle);
    Serial.println("Please lower throttle to zero.");
    throttle = ibus.readChannel(2);
    delay(500);   // Wait for throttle to go to zero
    ibus.loop();  // trying to update values
  }
  setRGB(0, 0, 0);
  Serial.println("Throttle confirmed down.");
}

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  Serial.println("Initializing Meltybrain Robot...");
  Serial.println("5...");
  delay(1000);
  Serial.println("4.");
  delay(1000);
  Serial.println("3?");
  delay(1000);
  Serial.println("2!");
  delay(1000);
  Serial.println("1,");
  delay(1000);
  Serial.println("0");
  // Initialize LED subsystem
  initLEDs();
  Serial.println("LED subsystem initialized.");

  // debugging test pattern black white cyan magenta yellow blue green red
  setRGB( 0,  0,  0, 0);
  setRGB(28, 28, 18, 1);
  setRGB( 0, 28, 18, 2);
  setRGB(28,  0, 18, 3);
  setRGB(28, 28,  0, 4);
  setRGB( 0,  0, 18, 5);
  setRGB( 0, 28,  0, 6);
  setRGB(28,  0,  0, 7);
  updateLEDs();

  // Initialize ESCs by sending zero throttle to both motors
  initESCs();
  Serial.println("ESCs initialized and motors stopped.");

  // Initialize IBus so we can monitor the TX without an accelerometer.
  // It's a long story. No, it's short: I made a mistake on a PCB. Now we have this code change.
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
  return throttle < ZERO_THRESHOLD;
}

// Function to check if in tank drive mode
bool isInTankDriveMode() {
  // fixed sideways to check from left side instead of center. not sure why I missed this. ;/
  float s = (0.95 + sidewaysInput);
  return isThrottleNearlyZero() && (s < 0.25);
}

void updateInputs() {
  // Read the vertical and horizontal direction control (channel 0 and 1)
  int vert = ibus.readChannel(0);   // Channel 0 (vertical)
  int horiz = ibus.readChannel(1);  // Channel 1 (horizontal)

  // Convert to centered values (from 1000-2000 to -1.0 to 1.0)
  stickVert = (vert - 1500) / 500.0;
  stickHoriz = (horiz - 1500) / 500.0;

  // Calculate stick angle in radians as a fraction of a circle
  // swapped sign of arguments so circle starts at top and goes clockwise on stick.
  stickAngle = atan2(-stickVert, stickHoriz) / (2 * PI);

  // Calculate stick length (magnitude) using Pythagoras' theorem
  float sl = sqrt(sq(stickVert) + sq(stickHoriz));
  stickLength = min(sl, 1.0f);  // lets's just saturate it if it's in the corners.

  if (stickLength < 0.05) {
    stickAngle = 0;
    stickLength = 0;
  }

  // Read throttle (channel 2) and map it to 0 to 1
  throttle = (ibus.readChannel(2) - 1000) / 1000.0;

  // Read sideways input (channel 3) and map it to -1 to 1
  sidewaysInput = (ibus.readChannel(3) - 1500) / 500.0;

  // Read radius input (channel 4) and map it to 0.1cm to 20cm
  radiusInput = ibus.readChannel(4);
  radiusSize = map(radiusInput, 1000, 2000, 0.001, 0.200);  // Result in centimeters (1mm to 200 mm)

  // Read LED offset input (channel 5).
  headingOffset = (ibus.readChannel(5) - 1500) / 1000.0;  // fraction of the way around a circle the joystick points: offset used for moving

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

    Serial.print("pwmIntCalled : ");
    Serial.print(pwmIntCalled);
    Serial.print("\tstickVert    : ");
    Serial.print(stickVert);
    Serial.print("\tstickHoriz   : ");
    Serial.print(stickHoriz);
    Serial.print("\tstickAngle   : ");
    Serial.print(stickAngle);
    Serial.print("\tstickLength  : ");
    Serial.print(stickLength);
    Serial.print("\tthrottle     : ");
    Serial.print(throttle);
    Serial.print("\tsidewaysInput: ");
    Serial.print(sidewaysInput);
    Serial.print("\tradiusInput  : ");
    Serial.print(radiusInput);
    Serial.print("\tradiusSize   : ");
    Serial.print(radiusSize * 100.0f);
    Serial.print("cm\theadingOffset: ");
    Serial.print(headingOffset);
    Serial.print("\treverseM1: ");
    Serial.print(reverseM1);
    Serial.print("\treverseM2: ");
    Serial.print(reverseM2);
    Serial.print("\tsw3pos: ");
    Serial.print(sw3pos);
    Serial.println();
  }
}

float filterAlpha = 0.1;
float previousFilteredReading = 0.0;

// Apply a low-pass filter to the accelerometer reading
float lowPassFilter(float currentReading) {
  previousFilteredReading = filterAlpha * currentReading + (1.0 - filterAlpha) * previousFilteredReading;
  return previousFilteredReading;
}


float rawAngle = 0;

// Function to get the magnitude of the acceleration due to spinning (force on the X-axis)
// while rejecting gravity (the Z-axis) and acceleration (the Y-axis).
float getSpinAcceleration() {
  // Get current accelerometer readings
  sensors_event_t s;
  lis.getEvent(&s);

  // Subtract calibration offsets
  float x = s.acceleration.x - accelOffsetX;
  float y = s.acceleration.y - accelOffsetY;
  float z = s.acceleration.z - accelOffsetZ;

  rawAngle = atan2(y, x);
  // Calculate the magnitude of the acceleration vector (centripetal force)
  float spinAccel = lowPassFilter(sqrt(x * x + y * y + z * z));

  return spinAccel;  // This is the magnitude of the centripetal acceleration component(in m/s²)
}

// Function to calculate revolutions per second based on spin acceleration
float calculateRPS() {
  // Get the spin acceleration (centripetal force)
  float spinAccel = getSpinAcceleration();

  // Use the dynamically updated radiusSize as the spin radius (in meters)
  float spinRadius = radiusSize;  // radiusSize is in meters. expected range is from 0.1cm to 10cm

  if (spinRadius < 0.010) spinRadius = 0.010;
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

// so many changes since last time.
// it's become more machine than man. again.
void handleMeltybrainDrive() {

  // Get the current time in microseconds
  static unsigned long usRevStartTime = micros();
  // Enter loop for an entire revolution
  while (true) {
    if (!rc_signal_is_healthy()) {  // we need to bail this loop if we lose signal.
      return;                       // next loop() call will zero throttle etc
    }
    updateInputs();

    // Calculate revolutions per second (RPS)
    float rps = calculateRPS();

    // If RPS is below the threshold, pretend it's at the threshold and blink on the red light so we know we can't translate.
    // we also set the stickLength (the desired translation amount) to zero.
    if (rps < RPS_THRESHOLD) {
      rps = RPS_THRESHOLD;
      setRGB(0xaaaa, 0x0000, 0x0000);
      stickLength = 0.0;
    } else {
      setRGB(0x0000, 0x0000, 0x0000);
    }
    updateLEDs();
    // Calculate the time for one complete revolution in microseconds
    unsigned long revTimeMicros = (1000000 / rps);

    unsigned long now = micros();
    // When revTimeMicros is updated to a new value:
    if (revTimeMicros != prevRevTimeMicros) {
      unsigned long elapsedTime = now - prevTimeMicros;

      // Calculate the phase angle in the previous rotation period (0 to 1)
      float oldPhase = (float)(now - usRevStartTime) / prevRevTimeMicros;

      // Account for elapsed time since last step
      oldPhase += (float)elapsedTime / prevRevTimeMicros;

      // Normalize phase to 0-1 range
      oldPhase = fmod(oldPhase, 1.0);

      // Convert the phase to the new rotation period
      usRevStartTime = now - (unsigned long)(oldPhase * revTimeMicros);

      // Update previous values
      prevRevTimeMicros = revTimeMicros;
      prevTimeMicros = now;
    }

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
    float baseLevel = throttle + speedLen * mixFrac;
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

<<<<<<< HEAD
    bool blueLEDOn = (cos_ph1 > 0.7071067811);  // 45 degrees each side
    bool greenLEDOn = cos(rawAngle) > 0.7071067811;
=======
    bool blueLEDOn  = (cos_ph1 > 0.7071067811) ; // 45 degrees each side
    // bool greenLEDOn = (cos_ph2 > 0.9238795325) ; // 22.5 degrees each side

    digitalWriteFast(bluePin , !blueLEDOn ); // phase 1 - blue
    // digitalWriteFast(greenPin, !greenLEDOn); // phase 2 - green
>>>>>>> 2d13526fd074d816545bb020675b20cf66276f54

    uint8_t r = 0;
    uint8_t b = !blueLEDOn;
    uint8_t g = !greenLEDOn;
    setRGB(r, g, b);
  }
}

void loop() {
  // Update the IBus object for Teensy
  ibus.loop();
  if (!rc_signal_is_healthy()) {
    setRGB(0xdede, 0x0000, 0x8282);
    ibus.loop();
    setThrottle(0, 0);
    delay(50);  // stop the speedy scroll
    displayState();
    return;
  }
  updateInputs();
  if (isInTankDriveMode()) {
    setRGB(0xff00, 0xffff, 0x00ff);  // red and blue LEDs flash slowly with green steady: tank drive
    handleTankDrive();
    delay(5);  // give us time to do _something_. Driving around doesn't
    displayState();
    return;
  }

  if (isThrottleNearlyZero()) {
    setRGB(0x0000, 0x2148, 0x1236);  // no error, green and blue blinking means ready, but no throttle
    setThrottle(0, 0);
    // add code to do whatever stuff
    // we should do when not moving.

  } else {
    setRGB(0, 0, 0);  // all off. at least as far as updateLEDs() is concerned.
    handleMeltybrainDrive();
  }


  displayState();

  checkSerial();
}