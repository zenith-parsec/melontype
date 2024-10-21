# melontype
melontype is an open source meltybrain library, collaboratively written with ChatGPT


* THIS IS NOT PRODUCTION QUALITY CODE
  * it has worked in testing
    * it still needs a lot more testing
      * **it might not work properly** all the time.
  * **Use at your own risk!**

* NOT FOR USE IN MEDICAL OR MILITARY APPLICATIONS.
  * it's probably a bad idea for everyone.

## Current status
This code has now been tested, and it seems to work. It can get "lock" when spinning now, 
and it demonstrated translation in a controlled direction after tuning earlier today in a 
test box. (The noise was too much to test it for long late at night. More later.)

## How to tune/configure the robot.

When spinning fast enough a blue arc appears indicating the front of the robot. 

if you are spinning quite fast and see this arc holding still, and you push the right stick
forward when and you move in the direction of the arc, then you are tuned. You should also 
be able to drive in other directions using the right stick. And of course you should be able
to turn the throttle up some more. You'll need to experiment to find the optimal spin speed
for translation. I have no clue yet.

to spin fast enough push the left stick (channel 3) up slightly from the bottom, increasing 
it until you can only see blue LEDs. If you can see green LEDs it is not going fast enough yet.

if you think are already going fast enough and the green LED is still on, if input 5 is really 
low, try turning it up some. eventually the lights should be only blue. at worst, turn it up 
all the way and it should turn on way earlier... but then you'll have to adjust the throttle
as you go, so aim for having it close to the correct value. ;]

when the LEDs are only blue, if they are sitting stationary, you are at least partially tuned. 

To stop the blue leds from spinning, adjust the input on channel 5 (map channel 5 to a 
potentiometer/dial in your transmitter config). Channel 5 tells the robot how far away the 
accelerometer is from the center of rotation. It varies from 1mm to 100mm.

The blue arc is supposed to be the front of the robot. When you press the right stick forward,
the robot should move forward. 

If the robot moves in another direction, use the channel 6 input to rotate the indicator so it
points in the right direction. The input value ranges from -180 degrees to 180 degrees. If you
need to rotate further, first rotate it all the way back around. This moves one revolution 
back around the robot. I map this input to a the other dial on my transmitter.

(note to self: add transmitter to list of parts: FS-i6)

TODO: images would show some ideas better

## Transcript of the coding session
Here is the transcript of me interacting with ChatGPT that resulted in the first version
of the code:

  https://chatgpt.com/share/67132632-9b8c-8000-8483-046ebea0f9ee

It wasn't just me saying "write me the code for a meltybrain combat robot." 

This transcript might help you understanding the code. You could just search for the 
function name, for example. Some code has been modified from what the bot wrote, and
I've done some clean up. 

It took on the order of 10 hours to write, which is much faster than it would've taken
me to get to this point. I got frustrated several times with the AI for not being able
to do anything right. 

This is also the 2nd attempt at doing it, and it's only as good (ha!) as it
is because I learned so much from how badly it did with the first attempt.

## comments from initial commit
below is the comment From the top of the source file when I initially committed it.
this comment is out of date, and reflects the initial commit state of the code
the current comment is likely to shrink as things are documented elsewhere and
problems are solved. 


```
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

// currently it uses ibus for reading but still uses PWM to do the servo control.
// eventually moving to dshot, but no rush.
```
