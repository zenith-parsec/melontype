// this is terrible code. not even being self-deprecating.
// so use this at your own risk.

// as if 10/21/2024 this code "works"

// I have actually tested it now and after various tweaks have had the robot 
// translate! It seemed quite stable. There isn't much room in my test box to
// see how stable it is before it hits a wall. (it's well after midnight and
// i live in an apartment complex. ;/ it sounds like a power tool or hammering
// when it hits the walls.)

// This code was written by me, zenith-parsec, and ChatGPT using the GPT-4o model.
// For a transcript of the chat that resulted in most of the code (first commit)
// look at:
//
//     https://chatgpt.com/share/67132632-9b8c-8000-8483-046ebea0f9ee
//
// Testing was performed using "the arena floor", a 3lb class combat robot.
// 
// - FPVDrone Fli14+ 14CH Mini Receiver 2A
// https://www.amazon.com/dp/B086W6LG8Z
// - Adafruit H3LIS331 accelerometer
// https://www.adafruit.com/product/4627
// - 2 x Repeat AM32 Drive ESC
// https://repeat-robotics.com/buy/am32/
// - Teensy 4.0 (which runs this code)
// https://www.pjrc.com/store/teensy40.html
// = red, green, and blue indicator LEDs. one each of for top (and bottom if you are invertible)
// - 2 x brushless motors
// - 2 x wheels
// - an approptiate battery (I'm using a 3S 750mAh LiPo)
// - a body to mount these things on. 
// - various connectors and tools

// The receiver should connect to RX2 (pin 7) of the Teensy and a common ground pin.
// The signal pins from the ESCs should be connected to pins 2 and 3, and the ground
// pins to ground. 

// I use a custom circuit board to hold things together, but it's still evolving
// so I won't share it yet.

// The robot should have red, green, and blue LEDs (all with resistors) on -ve side 
// of the appropriately named ports. The other side should be connected to 5v.
// The logic driving the LEDs in updateLEDs is inverted, with literal 1 turning the 
// LED off, and a 0 turning it on. Red LEDs have fwd voltage of about 1.6-2.2v, but
// we are connecting the negative lead to 3.3v, and the positive lead to 5v. With 
// the negative lead supplying 3.3v, the potential difference over the LED is only 
// 1.7v, which isn't enough to brightly light up a red LED, and nowhere near enough
// to light up a green or blue one. When the LED pin goes to 0v, the difference 
// becomes 5v, and all the LEDs are brightly illuminated. This is probably bad
// for the Teensy.

// the IBus connection needs a hardware serial port. The IBusBM library doesn't 
// support Teensy by default: it does compile, but it sends a warning message.
// I couldn't easily add perfect support, but I removed the warning part in the 
// begin() method about being an unsupported board so it compiles cleanly, and 
// I call the ibus.loop() method manually. (it might already be called on the timer
// I added as well, but I have no idea how to tell at the moment.) It's not like 
// this makes the code any worse.

#include <IBusBM.h>
#include "Adafruit_H3LIS331.h"

// Declare objects and variables
IBusBM ibus;  // IBus object for the radio receiver
HardwareSerial &serialPort = Serial2;  // Use Serial1 for IBus communication (adjust as needed)
Adafruit_H3LIS331 lis = Adafruit_H3LIS331();

// Define the ESC servo rate (typically 50Hz for most ESCs)
const int ESC_SERVO_RATE = 400;  

const float ZERO_THRESHOLD = 0.1;  // Threshold for detecting significant change. This is usually between -1 and 1 so 1/20th each step

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
bool pwmIntCalled;

// acceleration offsets due to gravity
float accelOffsetX;
float accelOffsetY;
float accelOffsetZ;

volatile uint32_t last_ibus_seen_millis;
volatile uint8_t last_cnt_rec;
volatile uint8_t last_cnt_poll;

// PWM frequency is 400Hz, so 1 cycle is 2500us.
const int PWM_1000us = 65536 * 40 / 100; // 40% of a 2500us cycle = 1000us 
const int PWM_2000us = 65536 * 80 / 100; // 80% of a 2500us cycle = 2000us

// Global variables for motor throttles
float motor1Throttle = 0.0;
float motor2Throttle = 0.0;
int lastMotor1 = -1; // make them different to get the update.
int lastMotor2 = -1; // ditto. (renamed, sorry.)


// Variables to store inputs
volatile float stickVert     = 0.0; // up/down = ch0
volatile float stickHoriz    = 0.0; // left/right = ch1
volatile float stickAngle    = 0.0; // Stick angle as a fraction of a circle
volatile float stickLength   = 0.0; // Length of the stick vector
volatile float throttle      = 0.0; // Throttle input = ch2
volatile float sidewaysInput = 0.0; // Sideways input = ch3
volatile float radiusInput   = 0.0; // Radius input = ch4
volatile float radiusSize    = 0.01; // Mapped radius size (0.1 cm to 10 cm) in meters
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

// map the throttle input so it spends more time in the low areas.
float stretchThrottle(float throttle)
{
  float s = throttle > 0 ? 1.f : -1.f;
  return s*(throttle * throttle);
}

int throttleToPWM(float throttle, bool stretch) {
  // Constrain the throttle value between 0 and 1
  throttle = constrain(throttle, -1.0, 1.0);
  // do non-linear stretch or not?
  if(stretch)
  {
    throttle = stretchThrottle(throttle);
  }
  // Map the throttle value (-1 to 1) to the PWM duty cycle values (1000us and 2000us converted to the 400Hz PWM)
  return map(throttle, -1, 1, PWM_1000us, PWM_2000us); // human simplified.
}

// Modified setThrottle to only store the desired throttle, no analogWrite() here
void setThrottle(float throttle1, float throttle2, bool stretch = true) {
  if(throttle1 == 0 && throttle2 == 0)
  {
    lastMotor1 = -1; lastMotor2 = -1; // force an update. hack. so? shut up.
  }
  motor1Throttle = throttleToPWM(throttle1, stretch);
  motor2Throttle = throttleToPWM(throttle2, stretch);
  pwmIntCalled = false;
}

// Function to handle tank drive logic
void handleTankDrive() {
  float x = stickHoriz;  // Horizontal stick input
  float y = stickVert;   // Vertical stick input

  // Calculate motor control values based on stick input
  float m1 = x + y;  // Motor 1 throttle
  float m2 = x - y;  // Motor 2 throttle

  // human modified. removed 'big enough change" check
  // pwm timer will keep it smooth enough. fixed use of
  // setThrottle so it passes both motors, instead of
  // throttle and motor pin.

  setThrottle(m1, m2, false);  // Set new desired throttle for motors
  pwmIntCalled = false;
}

// Interrupt handler to send PWM signal (this triggers the actual motor control)
void pwmInterruptHandler() {
  // Apply throttle to PWM signal value
  int pwmValueMotor1 = motor1Throttle; 
  int pwmValueMotor2 = motor2Throttle;

  // Write the actual PWM value to the motors
  if(lastMotor1 != pwmValueMotor1)
  {
    analogWrite(motorPin1, pwmValueMotor1);
    lastMotor1 = pwmValueMotor1;
  }
  if(lastMotor2 != pwmValueMotor2)
  {
    analogWrite(motorPin2, pwmValueMotor2);
    lastMotor2 = pwmValueMotor2;
  }
  pwmIntCalled = true;
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
    delay(100); // stop spamming serial
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
  // fixed sideways to check from left side instead of center. not sure why I missed this. ;/
  float s = (1  + sidewaysInput);
  return (throttle < ZERO_THRESHOLD) && (s < ZERO_THRESHOLD);
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
  // swapped sign of arguments so circle starts at top and goes clockwise on stick.
  stickAngle = atan2(-stickVert, stickHoriz) / (2 * PI);

  // Calculate stick length (magnitude) using Pythagoras' theorem
  stickLength = sqrt(sq(stickVert) + sq(stickHoriz)); // [ removed / 500 as it's already normalized from -1 to 1]
  stickLength /= 1.4142135623; // divide by sqrt(2) to put it into 0..1 range
  
  if(stickLength < ZERO_THRESHOLD)
  {
    stickAngle = 0;
    stickLength = 0;
  }

  // Read throttle (channel 2) and map it to 0 to 1
  throttle = (ibus.readChannel(2) - 1000) / 1000.0;

  // Read sideways input (channel 3) and map it to -1 to 1
  sidewaysInput = (ibus.readChannel(3) - 1500) / 500.0;

  // Read radius input (channel 4) and map it to 0.1cm to 10cm
  radiusInput = ibus.readChannel(4);
  radiusSize = map(radiusInput, 1000, 2000, 0.001, 0.10); // Result in centimeters (1mm to 10 cm)

  // Read LED offset input (channel 5).
  headingOffset  = (ibus.readChannel(5) - 1500) / 1000.0; // fraction of the way around a circle the joystick points: offset used for moving
}

// this function was human written i think, at least all the print parts.
void handleIdle()
{
	static uint32_t lastIdleMessage = 0;
	uint32_t now = millis();
	if(now - lastIdleMessage > 250)
	{
		lastIdleMessage = now;
		
    Serial.print("pwmIntCalled : "); Serial.print(pwmIntCalled);
    Serial.print("\tstickVert    : "); Serial.print(stickVert    );
    Serial.print("\tstickHoriz   : "); Serial.print(stickHoriz   );
    Serial.print("\tstickAngle   : "); Serial.print(stickAngle   );
    Serial.print("\tstickLength  : "); Serial.print(stickLength  );
    Serial.print("\tthrottle     : "); Serial.print(throttle     );
    Serial.print("\tsidewaysInput: "); Serial.print(sidewaysInput);
    Serial.print("\tradiusInput  : "); Serial.print(radiusInput  );
    Serial.print("\tradiusSize   : "); Serial.print(radiusSize   *100.0f);
    Serial.print("cm\theadingOffset: "); Serial.println(headingOffset);
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
  float spinAccel = sqrt(x * x + y * y + z * z); 
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

  // If RPS is below the threshold, pretend it's at the threshold and blink on the green light so we know we can't translate.
  if (rps < RPS_THRESHOLD) {
    rps = RPS_THRESHOLD;
    setRGB(0, 0xaaaa, 0x0000);
  }

  // Calculate the time for one complete revolution in microseconds
  unsigned long revTimeMicros = (1000000 / rps);

  // Get the current time in microseconds
  unsigned long usCurrentTime = micros();
  usRevStartTime = usCurrentTime;

  // Enter loop for an entire revolution
  while (true) {
    // Calculate the time passed in the current revolution
    unsigned long currentTimeMicros = micros() - usRevStartTime;

    // If the current time exceeds the revolution time, exit the loop
    if (currentTimeMicros >= revTimeMicros) {
	  	break;
    }

    // Calculate the adjusted heading
    float adjustedHeading = headingOffset + stickAngle;

    // human code: added check for negative offfset. if stickangle is to the left it will be negative.
    // also changed if to while in next one; input may have been close to -2.0 or 2.0. loop allows other modifiers
    // to offset

    while (adjustedHeading < 1.0) {
      adjustedHeading += 1.0; 
    }
    while (adjustedHeading > 1.0) {
      adjustedHeading -= 1.0;
    }

    // Calculate timeToForward and timeToBackward
    float timeToForward = adjustedHeading * revTimeMicros;
    float timeToBackward = timeToForward + 0.5 * revTimeMicros;
    if (timeToBackward > revTimeMicros) {
      timeToBackward -= revTimeMicros;
    }

    // Calculate width for motor activation checks
    float widthScale = max(stickLength, throttle); // human addition. was stickLength which doesn't make sense for not moving.
    float width = 0.25 * widthScale * revTimeMicros;

    // Determine if the motors should be activated
    bool motor1Active = isWithinTimeslice(currentTimeMicros, timeToForward, width, revTimeMicros);
    bool motor2Active = isWithinTimeslice(currentTimeMicros, timeToBackward, width, revTimeMicros);

    // Set the throttle for each motor using the global throttle value
//    setThrottle(motor1Active ? throttle : -throttle, motor2Active ? -throttle : throttle); // switched signs for motor2.
    setThrottle(motor1Active ? throttle : 0, motor2Active ? -throttle : 0); // switched signs for motor2.

    // Update the LEDs to visualize the current and adjusted heading
    bool blueLEDOn = isWithinTimeslice(currentTimeMicros, headingOffset * revTimeMicros, width, revTimeMicros);
    bool greenLEDOn = isWithinTimeslice(currentTimeMicros, adjustedHeading * revTimeMicros, width, revTimeMicros);

    digitalWrite(bluePin, blueLEDOn);
    // digitalWrite(greenPin, greenLEDOn);

    // Add a 200-microsecond delay at the end of each loop iteration
    delayMicroseconds(LOOP_DELAY_MICROS);
  }
}


void loop() {
  // Update the IBus object for Teensy
  ibus.loop();
  while(!rc_signal_is_healthy())
  {
    setRGB(0xdede,0x0000,0x8282);
    ibus.loop();
    setThrottle(0,0);
    delay(5); // stop the speedy scroll
    handleIdle();

  }
  updateInputs();
  
  if(isInTankDriveMode())
  {
  	setRGB(0x0000, 0xaaf0, 0x550f); // blue and green alternating blinking
  	handleTankDrive();
  	delay(5); // give us time to do _something_
    handleIdle();
  	return;
  }
  
  if(isThrottleNearlyZero())
  {
    setRGB(0x0001, 0x1246, 0x1236);
    setThrottle(0,0);
    // add code to do whatever stuff
    // we should do when not moving.
  }
  else
  {
    setRGB(0,0,0);       // all off. at least as far as updateLEDs() is concerned.
  	handleMeltybrainDrive();
  }  
  handleIdle();
}
