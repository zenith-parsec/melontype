// this is terrible code.
// not even being self-deprecating.
// I haven't even read all of it.
// Well, I have _read_ it, but not actually _looked_ at it.
// so use this at your own risk.
// I'll update it later when I've had a chance to test it more (one of ESCs is out, waiting on a replacement.)

// This code was written by me, zenith-parsec, and ChatGPT using the GPT-4o model.
// It took nearly two days worth of ChatGPT access, with limitations kicking in on 
// multiple occasions, only to expire a few hours later when 24h had passed since 
// the last coding burst. I asked for functions which performed various tasks, and
// pointed out the myriad coding errors the AI made and asked for corrections. On
// a few occasions I gave up trying to get it to solve the last few minor issues 
// and just told it what I had done to the code on my side.

// The only testing I have done so far is to boot it up with a 6-channel IBUS capable
// receiver bound to my transmitter. First I removed the accelerometer chip from 
// the socket and it went into an infinite loop because it failed to detect the 
// accelerometer. good. Then I tried with the accelerometer plugged in but no radio.
// It stalled at the expected location, waiting for a signal.
// Then I plugged in the receiver and rebooted it. It made it to the main loop.
// In the main loop, with the left stick held in the lower left corner, i could see
// the debug values scrolling past in the serial monitor.
// the values made sense for the inputs. I verified that the radiusSize varied from 
// 1mm to 100mm and that the transmitter was sensitive enough to enter sub-mm values
// for the distance from the accellerometer to the center of rotation. This should 
// allow rock solid direction lock. 

// The robot should have red, green, and blue LEDs (all with resistors) on -ve side 
// of the appropriately named ports. The other side should be connected to 5v.
// The logic driving the LEDs in updateLEDs is inverted, with literal 1 turning the 
// LED off, and a 0 turning it on. Red LEDs have fwd voltage of about 1.6-2.2v, but
// we are connecting the negative lead to 3.3v, and the positive lead to 5v. With 
// the negative lead supplying 3.3v, the potential difference over the LED is only 
// 1.7v, which isn't enough to brightly light up. When the LED pin goes to 0v, the
// difference becomes 5v, and the LED is brightly illuminated.

// the IBus connection needs a hardware serial port. The IBusBM library doesn't 
// support Teensy by default. I couldn't easily add perfect support, but I removed
// the warning part in the begin() method about being an unsupported board so it
// compiles cleanly, and I call the ibus.loop() method manually. (it might already
// be called on the timer but I have no idea how to tell at the moment.) It's not
// like this makes the code any worse though.


#include <IBusBM.h>
#include "Adafruit_H3LIS331.h"

// Declare objects and variables
IBusBM ibus;  // IBus object for the radio receiver
HardwareSerial &serialPort = Serial2;  // Use Serial1 for IBus communication (adjust as needed)
Adafruit_H3LIS331 lis = Adafruit_H3LIS331();

// Define the ESC servo rate (typically 50Hz for most ESCs)
const int ESC_SERVO_RATE = 400;  

// Threshold for considering "near zero"
const float ZERO_THRESHOLD = 0.05;
const float SIGNIFICANT_CHANGE = 0.05;  // Threshold for detecting significant change

// Pin definitions for motor outputs
const int motorPin1 = 2;  // Example pin for motor 1
const int motorPin2 = 3;  // Example pin for motor 2

// Pin definitions for the RGB LED channels
const int redPin = 14;    // Example pin for red channel
const int greenPin = 15; // Example pin for green channel
const int bluePin = 16;  // Example pin for blue channel

// Variables to store the current LED states (16-bit values)
volatile uint16_t redState = 0xAAAA;    // Example pattern for red (alternates on/off)
volatile uint16_t greenState = 0x0F0F;  // Example pattern for green (on/off in 128ms blocks)
volatile uint16_t blueState = 0xFFFF;   // Example pattern for blue (always on)

// IntervalTimer objects
IntervalTimer ledTimer;
IntervalTimer pwmTimer;

// acceleration offsets due to gravity
float accelOffsetX;
float accelOffsetY;
float accelOffsetZ;

volatile uint32_t last_ibus_seen_millis;
volatile uint8_t last_cnt_rec;
volatile uint8_t last_cnt_poll;

// PWM frequency is 400Hz, so 1 cycle is 2500us.
const int PWM_1000us = 65536 * 40 / 100; // 40% of a 2500us cycle = 1000us 
const int PWM_2000us = 65536 * 80 / 100; // 80% of a 2500us cycle = 1000us

// Global variables for motor throttles
float motor1Throttle = 0.0;
float motor2Throttle = 0.0;
float lastMotor1Throttle = 0.0;
float lastMotor2Throttle = 0.0;


// Variables to store inputs
volatile float stickVert     = 0.0; // up/down = ch0
volatile float stickHoriz    = 0.0; // left/right = ch1
volatile float stickAngle    = 0.0; // Stick angle as a fraction of a circle
volatile float stickLength   = 0.0; // Length of the stick vector
volatile float throttle      = 0.0; // Throttle input = ch2
volatile float sidewaysInput = 0.0; // Sideways input = ch3
volatile float radiusInput   = 0.0; // Radius input = ch4
volatile float radiusSize    = 0.0; // Mapped radius size (0.1 cm to 10 cm)
volatile float headingOffset = 0.0; // offset around circle heading light should be = ch5



void initLEDs() {
  // Configure the LED pins as outputs
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  
  // Set initial LED states (off)
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin, LOW);

  // Start the timer to call updateLEDs() every 32 milliseconds
  ledTimer.begin(updateLEDs, 32000);  // 32,000 microseconds = 32 milliseconds

  Serial.println("LEDs initialized with 32ms interval timer.");
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
  digitalWrite(redPin,   !((redState   >> i) & 0x1)); // Red LED
  digitalWrite(greenPin, !((greenState >> i) & 0x1)); // Green LED
  digitalWrite(bluePin,  !((blueState  >> i) & 0x1)); // Blue LED

  // Increment i and mask with 0xF to keep it within 0-15 (16 steps)
  i = (i + 1) & 0xF;
}

int throttleToPWM(float throttle) {
  // Constrain the throttle value between 0 and 1
  throttle = constrain(throttle, 0.0, 1.0);

  // Map the throttle value (0 to 1) to the PWM duty cycle values (1000us and 2000us converted to the 400Hz PWM)
  return map(throttle * 1000, -1000, 1000, PWM_1000us, PWM_2000us);
}

// Modified setThrottle to only store the desired throttle, no analogWrite() here <
void setThrottle(float throttle1, float throttle2) {
  motor1Throttle = throttleToPWM(throttle1);          // <= FIRST patch.
  motor2Throttle = throttleToPWM(throttle2);
}

// Function to handle tank drive logic
void handleTankDrive() {
  float x = stickHoriz;  // Horizontal stick input
  float y = stickVert;   // Vertical stick input

  // Calculate motor control values based on stick input
  float m1 = x + y;  // Motor 1 throttle
  float m2 = x - y;  // Motor 2 throttle

  // Only update if there is a significant change
  if (abs(m1 - lastMotor1Throttle) > SIGNIFICANT_CHANGE) {
    setThrottle(m1, motorPin1);  // Set new desired throttle for motor 1
    lastMotor1Throttle = m1;     // Store the last value for comparison
  }

  if (abs(m2 - lastMotor2Throttle) > SIGNIFICANT_CHANGE) {
    setThrottle(m2, motorPin2);  // Set new desired throttle for motor 2
    lastMotor2Throttle = m2;     // Store the last value for comparison
  }
}

// Interrupt handler to send PWM signal (this triggers the actual motor control)
void pwmInterruptHandler() {
  // Convert throttle to PWM signal value
  int pwmValueMotor1 = motor1Throttle;  // Convert desired throttle to PWM value
  int pwmValueMotor2 = motor2Throttle;

  // Write the actual PWM value to the motors
  analogWrite(motorPin1, pwmValueMotor1);
  analogWrite(motorPin2, pwmValueMotor2);
}

// Initialize ESCs and set the throttle to 0
void initESCs() {
  // Set PWM resolution to 16 bits
  analogWriteResolution(16);

  // Set the PWM frequency for each motor pin to ESC servo rate (ESC_SERVO_RATE)
  analogWriteFrequency(motorPin1, ESC_SERVO_RATE);
  analogWriteFrequency(motorPin2, ESC_SERVO_RATE);

  pwmTimer.begin(pwmInterruptHandler, 2500);  

  // Set initial throttle to 0 for both motors
  setThrottle(0, 0);

  Serial.println("ESCs initialized, throttle set to 0.");
}


void initAccel() {
  if (!lis.begin_I2C()) {
    Serial.println("Failed to initialize accelerometer! System halt.");
    setRGB(0x7777, 0x8888, 0x0000); // rrrgrrrgrrrgrrrg 
    while (1);  // Stop the system if the accelerometer initialization fails
  }

  // Configure the accelerometer settings
  lis.setDataRate(LIS331_DATARATE_400_HZ);
  lis.setRange(H3LIS331_RANGE_100_G);
  lis.setLPFCutoff(LIS331_LPF_292_HZ);

  Serial.println("Accelerometer initialized successfully.");
}

void collectCalibrationData() {
  // Set LEDs to blink a low-level warning for 2 seconds
  setRGB(0x6666, 0x9999, 0x0000);  // 0b0110011001100110, 0b1001100110011001, 0b0000000000000000
  for (int i = 0; i < 62; i++) {   // ~2 seconds (62 calls at 32ms each)
    delay(32);
  }

  // Set LEDs to blink twice as fast while calibrating
  setRGB(0x5555, 0xAAAA, 0x0000);  // 0b0101010101010101, 0b1010101010101010, 0b0000000000000000

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

  // Calculate the average values (gravity vector calibration)
  float xOffset = xSum / sampleCount;
  float yOffset = ySum / sampleCount;
  float zOffset = zSum / sampleCount;

  // Store these offsets for future sensor calibration
  // Assume you have a global variable to store these offsets
  accelOffsetX = xOffset;
  accelOffsetY = yOffset;
  accelOffsetZ = zOffset;

  setRGB(0x0000 ,0x0007, 0x0000); // 0, 0b0000000000000111, 0  little green blip, no red or blue.
  Serial.println("Calibration completed. Offsets stored.");
}


bool rc_signal_is_healthy()
{
  uint32_t now = millis();
  bool res = false;
  if(now - last_ibus_seen_millis < 500)
  {
    res = true;
  }
  else
  {
    ibus.loop();
  }
  if(last_cnt_rec != ibus.cnt_rec || last_cnt_poll != ibus.cnt_poll)
  {
    res = true;
    last_cnt_rec = ibus.cnt_rec;
    last_cnt_poll = ibus.cnt_poll;
    last_ibus_seen_millis = now;
  }
  return res;
}

void initIBus() {
  // Initialize the radio receiver using the IBusBM library
  ibus.begin(serialPort);  // Start IBus communication
  Serial.println("IBus receiver initialized.");
  ibus.loop(); // get the first value?

  // Validate that commands are being received
  while (!ibus.readChannel(0)) {
  	setRGB(0xf0f6, 0x0000, 0x0f00); // red long on, long off, long on, very short off, short on, very short off. blue blinks during first gap.
    Serial.println("Waiting for remote signal...");
    ibus.loop(); // trying to update values
  }
  Serial.println("Remote signal received.");

  // Ensure that throttle is in the down position (assuming throttle is on channel 2)
  int throttle = ibus.readChannel(2);
  while (throttle > 1050) {
    Serial.println(throttle);
    Serial.println("Please lower throttle to zero.");
    throttle = ibus.readChannel(2);
    delay(500);  // Wait for throttle to go to zero
    ibus.loop(); // trying to update values
  }
  Serial.println("Throttle confirmed down.");
}

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  Serial.println("Initializing Meltybrain Robot...");

  // Initialize LED subsystem
  initLEDs();
  Serial.println("LED subsystem initialized.");

  // Initialize ESCs by sending zero throttle to both motors
  initESCs();
  setThrottle(0, 0);  // Send 0 throttle to both motors
  Serial.println("ESCs initialized and motors stopped.");

  // Initialize accelerometer and collect calibration data
  initAccel();  // Function to initialize the accelerometer
  collectCalibrationData();  // Function to calibrate and store offset
  Serial.println("Accelerometer initialized and calibrated.");

  initIBus();

  // Additional setup steps can go here
  Serial.println("Setup complete. Ready for operation.");
}

// Function to check if throttle is nearly zero
bool isThrottleNearlyZero() {
  return abs(throttle) < ZERO_THRESHOLD;
}

// Function to check if in tank drive mode
bool isInTankDriveMode() {
  return (abs(throttle) < ZERO_THRESHOLD) && (abs(sidewaysInput) < ZERO_THRESHOLD);
}

void updateInputs()
{
  // Read the vertical and horizontal direction control (channel 0 and 1)
  int vert  = ibus.readChannel(0);  // Channel 0 (vertical)
  int horiz = ibus.readChannel(1);  // Channel 1 (horizontal)

  // Convert to centered values (from 1000-2000 to -1.0 to 1.0)
  stickVert  = (vert  - 1500) / 500.0;
  stickHoriz = (horiz - 1500) / 500.0;

  // Calculate stick angle in radians as a fraction of a circle
  stickAngle = atan2(stickVert, -stickHoriz) / (2 * PI);

  // Calculate stick length (magnitude) using Pythagoras' theorem
  stickLength = sqrt(sq(stickVert) + sq(stickHoriz)) / 500.0; // Normalize to 0-1

  // Read throttle (channel 2) and map it to 0 to 1
  throttle = (ibus.readChannel(2) - 1000) / 1000.0;

  // Read sideways input (channel 3) and map it to -1 to 1
  sidewaysInput = (ibus.readChannel(3) - 1500) / 500.0;

  // Read radius input (channel 4) and map it to 0.1cm to 10cm
  radiusInput = ibus.readChannel(4);
  radiusSize = map(radiusInput, 1000, 2000, 0.001, 0.10) / 10.0; // Result in centimeters (0.1 to 10 cm)

  // Read LED offset input (channel 5).
  headingOffset  = (ibus.readChannel(5) - 1000) / 1000.0; // fraction of the way around a circle the joystick points: offset used for moving
}


void handleIdle()
{
	static uint32_t lastIdleMessage = 0;
	uint32_t now = millis();
	if(now - lastIdleMessage > 250)
	{
		lastIdleMessage = now;
		// message stuff
    Serial.printf("stickVert    : %f\t", (double)stickVert    );
    Serial.printf("stickHoriz   : %f\t", (double)stickHoriz   );
    Serial.printf("stickAngle   : %f\t", (double)stickAngle   );
    Serial.printf("stickLength  : %f\t", (double)stickLength  );
    Serial.printf("throttle     : %f\t", (double)throttle     );
    Serial.printf("sidewaysInput: %f\t", (double)sidewaysInput);
    Serial.printf("radiusInput  : %f\t", (double)radiusInput  );
    Serial.printf("radiusSize   : %f\t", (double)radiusSize   );
    Serial.printf("headingOffset: %f\n", (double)headingOffset);
  }	
}



// Function to get the magnitude of the acceleration due to spinning (centripetal force)
float getSpinAcceleration() {
  // Get current accelerometer readings
  sensors_event_t s;
  lis.getEvent(&s);

  // Subtract calibration offsets
  float x = s.acceleration.x - accelOffsetX;
  float y = s.acceleration.y - accelOffsetY;
  float z = s.acceleration.z - accelOffsetZ;

  // Calculate the magnitude of the acceleration vector (centripetal force)
  float spinAccel = sqrt(x * x + y * y + z * z);  // Ignore Z since it's aligned with gravity

  return spinAccel;  // This is the magnitude of the centripetal acceleration (in m/s²)
}

// Function to calculate revolutions per second based on spin acceleration
float calculateRPS() {
  // Get the spin acceleration (centripetal force)
  float spinAccel = getSpinAcceleration();

  // Use the dynamically updated radiusSize as the spin radius (in meters)
  float spinRadius = radiusSize;  // radiusSize is in meters. expected range is from 0.1cm to 10cm

  // Calculate angular velocity (omega) in radians per second
  float omega = sqrt(spinAccel / spinRadius);

  // Convert angular velocity to RPS
  float rps = (omega) / (2 * PI);

  return rps;  // Return the calculated RPS
}



// Constants
const float RPS_THRESHOLD = 7.0;  // 7 revolutions per second = 420 RPM
const unsigned long LOOP_DELAY_MICROS = 200;  // Delay for each loop iteration in microseconds

// Variables
unsigned long usRevStartTime = 0;  // Time in microseconds when the revolution started

// Generalized function to determine if the current time is within a time slice
bool isWithinTimeslice(float currentTime, float targetTime, float width, float revDuration) {
  float startTime = targetTime - width;
  float endTime = targetTime + width;

  // Correct for wrapping around the revolution time
  if (startTime < 0) {
    startTime += revDuration;
  }
  if (endTime >= revDuration) {
    endTime -= revDuration;
  }

  // Check if current time is within the pulse time
  return isPulseTime(currentTime, startTime, endTime, revDuration);
}

// Function to determine if the motors should be pulsed on
bool isPulseTime(float currentTime, float startTime, float endTime, float revDuration) {
  if (startTime > endTime) {
    // Case when the start time is after the end time
    return (currentTime >= startTime || currentTime <= endTime);
  } else {
    // Case when the start time is before the end time
    return (currentTime >= startTime && currentTime <= endTime);
  }
}

void handleMeltybrainDrive() {
  // Calculate revolutions per second (RPS)
  float rps = calculateRPS();
  
  // Calculate the time for one complete revolution in microseconds
  unsigned long revTimeMicros = (1.0 / rps) * 1000000.0;

  // Get the current time in microseconds
  unsigned long usCurrentTime = micros();
  if (usRevStartTime == 0) {
    // Initialize the start time of the revolution if not set
    usRevStartTime = usCurrentTime;
  }

  // If RPS is below the threshold, set motors to stay on indefinitely
  if (rps < RPS_THRESHOLD) {
    setThrottle(throttle, -throttle);  // Both motors on with throttle values
    delayMicroseconds(LOOP_DELAY_MICROS);  // Add delay before next iteration
    return;
  }

  // Enter loop for an entire revolution
  while (true) {
    // Calculate the time passed in the current revolution
    unsigned long currentTimeMicros = micros() - usRevStartTime;

    // If the current time exceeds the revolution time, exit the loop
    if (currentTimeMicros >= revTimeMicros) {
	  	break;
    }

    // Normalize currentTime as a fraction of the revolution (0 to 1)
    // float currentTime = float(currentTimeMicros) / revTimeMicros;

    // Calculate the adjusted heading
    float adjustedHeading = headingOffset + stickAngle;
    if (adjustedHeading > 1.0) {
      adjustedHeading -= 1.0;
    }

    // Calculate timeToForward and timeToBackward
    float timeToForward = adjustedHeading * revTimeMicros;
    float timeToBackward = timeToForward + 0.5 * revTimeMicros;
    if (timeToBackward > revTimeMicros) {
      timeToBackward -= revTimeMicros;
    }

    // Calculate width for motor activation checks
    float width = 0.25 * stickLength * revTimeMicros;

    // Determine if the motors should be activated
    bool motor1Active = isWithinTimeslice(currentTimeMicros, timeToForward, width, revTimeMicros);
    bool motor2Active = isWithinTimeslice(currentTimeMicros, timeToBackward, width, revTimeMicros);

    // Set the throttle for each motor using the global throttle value
    setThrottle(motor1Active ? throttle : -throttle, motor2Active ? throttle : -throttle);

    // Update the LEDs to visualize the current and adjusted heading
    bool blueLEDOn = isWithinTimeslice(currentTimeMicros, headingOffset * revTimeMicros, width, revTimeMicros);
    bool greenLEDOn = isWithinTimeslice(currentTimeMicros, adjustedHeading * revTimeMicros, width, revTimeMicros);

    digitalWrite(bluePin, blueLEDOn);
    digitalWrite(greenPin, greenLEDOn);

    // Add a 200-microsecond delay at the end of each loop iteration
    delayMicroseconds(LOOP_DELAY_MICROS);
  }
}


void loop() {
  // Update the IBus object for Teensy
  ibus.loop();
  
  updateInputs();
  
  if(isInTankDriveMode())
  {
  	setRGB(0x0000, 0xaaf0, 0x550f); // blue and green alternating blinking
  	handleTankDrive();
  	delay(5); // give us time to do _something_
  	return;
  }
  
  if(isThrottleNearlyZero())
  {
    setRGB(0x0001, 0x1246, 0x1236);
  	handleIdle();
  }
  else
  {
    setRGB(0,0,0);       // all off. at least as far as updateLEDs() is concerned.
  	handleMeltybrainDrive();
  }  
}
