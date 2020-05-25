
/*******************************************************
* Main file for Carl's version of the 2016 DPRG Club Robot
* AKA "ClubCar DonkeyBot"
* Reference these:
*   BranchPlanning.txt
*   Planning Notes
 *******************************************************/

// #include <SoftwareServo.h>
// #include <stdio.h>

#include "libtaskMemoryTest.h"
#include "motorTasks.h"
#include "PS2X_controller.h"
#include <arduino.h>
#include <avr/io.h>     // per Dale Wheat / Arduino Internals page 35.  Explicitly included to reference Arduion registers, even though Arduino automatically picks it up when not included

#include <Adafruit_NeoPixel.h>  // using Adafruit NeoPixel by Adafruit Version 1.4.0
#define NeoPixel_PIN 37    // Groups the NeoPixel LED in the same 'connector grouping' as the RGB LED and Switch
#define NeoPixel_LEDS 8
#define Neo_applyJoystickLocally 0            // 1st neoPixel
#define Neo_applySPIHeadingThrottleLocally 1  // 2nd NeoPixel
#define Neo_SPIlinkActive 2                   // 3rd NeoPixel
unsigned int SPIlinkActivityCounter = 0;         // counter to detect a minimum number of received values per second, to declare that the SPI link is active

Adafruit_NeoPixel neoPixelStrip = Adafruit_NeoPixel(NeoPixel_LEDS, NeoPixel_PIN, NEO_GRB + NEO_KHZ800);


// Using SPI slave code reference code from https://github.com/cottjr/piMegaSPI
//  starting with baseline version, commit 00163e8 branch refactorToSpiSlaveClass
#include "megaSPIslave/spiSlave.h"

// attached a SparkFun COM-11120 10mm diffused RGB LED, with common/cathode to ground, and RGB pins as follows with resistors to approximately balance light intensity
#define cpuStatusLEDredPin 30     // 325 ohm
#define cpuStatusLEDgreenPin 31   // 1.2K ohm
#define cpuStatusLEDbluePin 32    // 1K ohm

// attached an Adafruit 3350 Rugged Metal Pushbutton http://adafru.it/3350
// =>  with common/anode to supply votage, and RGB pins with built-in current limiting resistors
#define RGBswitchRedPin 33      // 219 ohm
#define RGBswitchGreenPin 34    // 150 ohm
#define RGBswitchBluePin 35     // zero ohm
#define RGBswitchSwitchPin 36   // RC 4.7K 1 uF
bool RGBswitchPriorSecond = HIGH;  //  track the prior RGB Switch state to enable detecting transitions

// test points on breakout board
#define potA8 62    // analog voltage in - Pot #1  (A8)
#define potA9 63    // analog voltage in - Pot #2  (A9)
#define potA10 64   // analog voltage in - Pot #3  (A10)
#define potA11 65   // analog voltage in - Pot #4  (A11)
#define anaTPa12 66 // analog voltage in or digital i/o - test point #1  (A12)
#define anaTPa13 67 // analog voltage in or digital i/o - test point #2  (A13)
#define anaTPa14 68 // analog voltage in or digital i/o - test point #3  (A14)
#define anaTPa15 69 // analog voltage in or digital i/o - test point #4  (A15)

#define digTP26 26  // digital test point #1 (pin 26) // CPU status monitoring via o'scope
#define digTP27 27  // digital test point #2 (pin 27) // CPU status monitoring via o'scope
#define digTP28 28  // digital test point #3 (pin 28)
#define digTP29 29  // digital test point #4(pin 29)


unsigned int cpuCycleHeadroom20ms;            // most recent number of spare cycles betewen 10ms interrupt periods
unsigned int cpuCycleHeadroom20msIncrement;   // working count
unsigned int cpuCycleHeadroom1000ms;           // most recent number of spare cycles betewen 100ms interrupt periods
unsigned int cpuCycleHeadroom1000msIncrement;  // working count

// interrupt handler sketch per Dale Wheat - Arduino Internals, page 145
//  -> except changed from timer 1 to timer 5 (since timer 1 is incompatible with delay() and other functions, 
//  -> and timers 3 and 4 are needed to drive PWM on club robot Monster Motor Shield via ATMega2560 pins 5 and 6 )
//  -> for timer/PWM/pin compatibility ref also https://www.robotshop.com/community/forum/t/arduino-101-timers-and-interrupts/13072
ISR(TIMER5_OVF_vect) {
  bitSet(PINB, 7); // toggle ATMega2560 PB7/D13 LED pin state by writing to the input read register // per Dale Wheat - Arduino Internals page 145
}

volatile bool ISR20msActive = false;
volatile bool cpuStatusLEDisWhite = false;

// Use timer 5 to periodically trigger the main loop with well controlled timing
// using timer 5 to avoid messing with timers needed for other purposes
// interrupt handler via timer compare match method, per https://www.robotshop.com/community/forum/t/arduino-101-timers-and-interrupts/13072
ISR(TIMER5_COMPA_vect){
  if (!cpuStatusLEDisWhite){
    digitalWrite(cpuStatusLEDredPin, digitalRead(cpuStatusLEDredPin) ^ 1);        // toggle red LED pin
    digitalWrite(cpuStatusLEDgreenPin, digitalRead(cpuStatusLEDgreenPin) ^ 1);    // toggle green LED pin

  }

  ISR20msActive = true;
}

char taskLoopCounter;    // used to divide the 20ms tick into a 1000ms loop
long int tick20msCounter;     // used to track absolute number of 20ms ticks since power up
long int QuickTripStartCounter = 0; // track the start time of a Quick Trip run, in 20ms Counter ticks
bool runContinuousMotorStepResponseTest;  // simple test turns motors on and off with an impulse runContinuousMotorStepResponseTest. Handy e.g. for PID tuning
bool runQuickTrip;    // simply go out and back once shortly after power up
bool readAndViewAllPS2Buttons;  // diagnostics flag to view any PS2 controller button presses - use to figure out which button is which - writes to serial monitor - not intended for normal operation
PS2JoystickValuesType PS2JoystickValues;  // structure to track most recent values from a PS2 controller
bool applyJoystickLocally = true;  // flag to track whether or not to apply Joystick values to motor setpoints
bool applySPIHeadingThrottleLocally = true; // flag to track whether or not to apply Heading and Throttle commands from SPI to motor setpoints
bool SPIlinkActive = false; // flag to track whether or not the SPI link has active & valid traffic

// functions which run every 20ms
void tasks20ms () {
  // cpuStatusLEDisWhite = true;
  // digitalWrite(cpuStatusLEDredPin, HIGH);
  // digitalWrite(cpuStatusLEDgreenPin, HIGH);
  // digitalWrite(cpuStatusLEDbluePin, HIGH);
  // sampleMotorShield();          // read encoders, calculate PID, send commands to motors
  // cpuStatusLEDisWhite = false;
  // noInterrupts();
  // digitalWrite(cpuStatusLEDredPin, HIGH);
  // digitalWrite(cpuStatusLEDgreenPin, LOW);
  // digitalWrite(cpuStatusLEDbluePin, LOW);
  // interrupts();

  // verify the RGB Switch by writing it's value to these digital test point pins
  // digitalWrite(digTP28, digitalRead(RGBswitchSwitchPin));
  // digitalWrite(digTP29, digitalRead(RGBswitchSwitchPin));

  digitalWrite(digTP27, HIGH);
  filterTurnAndThrottleRequestValues(); // lowpass Throttle and Turn Velocity commands to match platform capability
  sampleMotorShield();

  // Query the SPI slave port receiver to retrieve fresh data if any has been received but not yet examined/handled
  if (spiSlavePort.getLatestDataFromPi())
  {
    SPIlinkActivityCounter += 1;
    // Serial.println("-----------> fresh data");    
  } else
  {
    // Serial.println("-----------> NO fresh data - Nope - Not at all! The yellow should go out now!...");
  }
  spiSlavePort.handleCommandsFromPi();

  if (taskLoopCounter == 49) {
    taskLoopCounter = 0;
    digitalWrite(digTP26, HIGH);
    tasks1000ms();
    digitalWrite(digTP26, LOW);
  } else {
    taskLoopCounter += 1;
  }

  if ((taskLoopCounter == 4) && digitalRead(cpuStatusLEDbluePin)){
    // cpuStatusLEDisWhite = false;
    // digitalWrite(cpuStatusLEDredPin, HIGH);
    // digitalWrite(cpuStatusLEDgreenPin, LOW);
    // digitalWrite(cpuStatusLEDbluePin, LOW);
  }

  unsigned char quickTripSpeed = 100;            // max speed for Quick Trip
  unsigned char quickTripSpeedThreeQuarter = 75; // kludge (should make part of filters & loop response), start & stop quick trip slowly
  unsigned char quickTripSpeedHalf = 50;         // kludge (should make part of filters & loop response), start & stop quick trip slowly
  unsigned char quickTripSpeedQuarter = 25;      // kludge (should make part of filters & loop response), start & stop quick trip slowly

// quickly stop or re-initialize autonomous modes by tapping the RGB button or the PS2 controller select button
//  This will immediately stop local automous modes like Quick Trip
//  And although it may not stop remote commanded automonous modes like DonkeyCar,
//  it will at least re-initialize values for received command buffers.
//  This can help in cases such as when the SPI link hangs, by allowing you to locally reset the most recent commands to zero.
  if (!digitalRead(RGBswitchSwitchPin) || selectButtonState) 
  {
      // Reset the QuickTrip Routine
      digitalWrite(RGBswitchRedPin, HIGH);       
      digitalWrite(RGBswitchGreenPin, LOW);   // turn green on - indicate ready to run
      digitalWrite(RGBswitchBluePin, HIGH);      
      spiSlavePort.initializeValuesCacheFromPi();
      setAutomaticVelocityLoopSetpoints(0, 0, true);
      setVelocityLoopLowPassCutoff( -1 , true);    // return the setpoint command lowpass filter to it's default value
      runQuickTrip = false;
  }

// enable start QuickTrip on RGB button release, ie. rising edge of RGBswitchSwitchPin
  if ( !RGBswitchPriorSecond && digitalRead(RGBswitchSwitchPin) && !runQuickTrip )   
  {
    // start a QuickTrip Routine
    digitalWrite(RGBswitchRedPin, LOW);       // turn Red on during the run
    digitalWrite(RGBswitchGreenPin, HIGH);
    digitalWrite(RGBswitchBluePin, HIGH);      
    QuickTripStartCounter = tick20msCounter;
    runQuickTrip = true;
  }

// start a quick trip if requested and not already in progress
  if (ps2ControllerUseable && startAndTriangle && !runQuickTrip) 
  {
    QuickTripStartCounter = tick20msCounter;
    runQuickTrip = true;
  };
  if (runQuickTrip)
  {
    digitalWrite(RGBswitchRedPin, LOW);       // turn Red on during the run
    digitalWrite(RGBswitchGreenPin, HIGH);
    digitalWrite(RGBswitchBluePin, HIGH);      

    if (tick20msCounter == QuickTripStartCounter + 25)
    {
      setVelocityLoopLowPassCutoff(0.6, true);    // smooth out the setpoint commands for more accurate acceleration    
    }

    //  Timed about-face move. Does pretty close to a 180 pretty quickly.
    // if (tick20msCounter == QuickTripStartCounter + 75)
    // {
    //   setAutomaticVelocityLoopSetpoints( quickTripSpeed, 0, true);
    // }
    // if (tick20msCounter == QuickTripStartCounter + 128)
    // {
    //   setAutomaticVelocityLoopSetpoints( 0, 0, true);
    // }

    // start moving 1.5 seconds after power up
    // move forward
    if (tick20msCounter == QuickTripStartCounter + 75)
    {
      setAutomaticVelocityLoopSetpoints( 1, quickTripSpeedQuarter, true);
    }
    if (tick20msCounter == QuickTripStartCounter + 100)
    {
      setAutomaticVelocityLoopSetpoints( 0, quickTripSpeedHalf, true);
    }
    if (tick20msCounter == QuickTripStartCounter + 125)
    {
      setAutomaticVelocityLoopSetpoints( 0, quickTripSpeedThreeQuarter, true);
    }
    if (tick20msCounter == QuickTripStartCounter + 150)
    {
      setAutomaticVelocityLoopSetpoints(0, quickTripSpeed, true);
    }
    if (tick20msCounter == QuickTripStartCounter + 305) // 250)
    {
      setAutomaticVelocityLoopSetpoints(0, quickTripSpeedThreeQuarter, true);
    }
    if (tick20msCounter == QuickTripStartCounter + 330) // 275)
    {
      setAutomaticVelocityLoopSetpoints(0, quickTripSpeedHalf, true);
    }
    if (tick20msCounter == QuickTripStartCounter + 355) // 300)
    {
      setAutomaticVelocityLoopSetpoints(0, quickTripSpeedQuarter, true);
    }
    if (tick20msCounter == QuickTripStartCounter + 380) // 325)
    {
      setAutomaticVelocityLoopSetpoints(0, 0, true);
    }
    // wait for 1.5 seconds
    // then head backwards
    if (tick20msCounter == QuickTripStartCounter + 455) // 400)
    {
      setAutomaticVelocityLoopSetpoints( -1, -quickTripSpeedQuarter, true);
    }
    if (tick20msCounter == QuickTripStartCounter + 480) // 425)
    {
      setAutomaticVelocityLoopSetpoints( -2, -quickTripSpeedHalf, true);
    }
    if (tick20msCounter == QuickTripStartCounter + 505) // 450)
    {
      setAutomaticVelocityLoopSetpoints( 0, -quickTripSpeedThreeQuarter, true);
    }
    if (tick20msCounter == QuickTripStartCounter + 530) // 475)
    {
      setAutomaticVelocityLoopSetpoints(0, -quickTripSpeed, true);
    }
    if (tick20msCounter == QuickTripStartCounter + 685) // 575)
    {
      setAutomaticVelocityLoopSetpoints(0, -quickTripSpeedThreeQuarter, true);
    }
    if (tick20msCounter == QuickTripStartCounter + 710) // 600)
    {
      setAutomaticVelocityLoopSetpoints(0, -quickTripSpeedHalf, true);
    }
    if (tick20msCounter == QuickTripStartCounter + 735) // 625)
    {
      setAutomaticVelocityLoopSetpoints(0, -quickTripSpeedQuarter, true);
    }
    if (tick20msCounter == QuickTripStartCounter + 760) // 650)
    {
      setAutomaticVelocityLoopSetpoints(0, 0, true);
    }    
    if (tick20msCounter == QuickTripStartCounter + 950)
    {
      // Reset the QuickTrip Routine
      digitalWrite(RGBswitchRedPin, HIGH);       
      digitalWrite(RGBswitchGreenPin, LOW);   // turn green on - indicate ready to run
      digitalWrite(RGBswitchBluePin, HIGH);      
      setVelocityLoopLowPassCutoff( -1 , true);    // return the setpoint command lowpass filter to it's default value
      runQuickTrip = false;
    }
  } else
  {
    if (applySPIHeadingThrottleLocally) 
    {
      // Set & mix values From Pi SPI master together with operator values, as input to the velocity loops
      setAutomaticVelocityLoopSetpoints(spiSlavePort.getTurnVelocityFromPi(),spiSlavePort.getForwardThrottleFromPi(),false);
    }
  }
  
  // Toggle enable/disable apply joystick values locally, versus simply send joystick values to piSPImaster
  if (ps2ControllerUseable && selectButtonState && L3buttonJustPressed)
  {
    applyJoystickLocally = !applyJoystickLocally;
    if (applyJoystickLocally)
    {
      // Green
      neoPixelStrip.setPixelColor(Neo_applyJoystickLocally, neoPixelStrip.Color(0,10,0));
      Serial.println("---------------------> Enabled joystick values to local drive.");
    } else
    {
      // Red
      neoPixelStrip.setPixelColor(Neo_applyJoystickLocally, neoPixelStrip.Color(10,0,0));
      Serial.println("-> Disabled local drive from joystick values. Auto-drive only.");
    }
  }

  // Toggle enable/disable whether or not to apply Heading and Throttle commands from SPI to motor setpoints
  if (ps2ControllerUseable && selectButtonState && R3buttonJustPressed)
  {
    applySPIHeadingThrottleLocally = !applySPIHeadingThrottleLocally;
    if (applySPIHeadingThrottleLocally)
    {
      // Green
      neoPixelStrip.setPixelColor(Neo_applySPIHeadingThrottleLocally, neoPixelStrip.Color(0,10,0));
      Serial.println("-> Enabled Auto-drive / SPI commanded Heading and Throttle values to local drive.");
    } else
    {
      // Red
      neoPixelStrip.setPixelColor(Neo_applySPIHeadingThrottleLocally, neoPixelStrip.Color(10,0,0));
      Serial.println("------> Disabled Auto-drive / ignoring SPI commanded Heading and Throttle values.");
    }
  }



  // ToDo-> clean this up, to write appropriate values even if PS2 controller is not available
  // readAndViewAllPS2Buttons costs about 2.2ms, plus another 1.1ms to write out values on push
  if ( ps2ControllerUseable && readAndViewAllPS2Buttons )
  {
    readAllPS2xControllerValues();                          // show any PS2 controller events/data if present
    readPS2Joysticks( &PS2JoystickValues );                 // read the latest PS2 controller joystick values
    // signed char twoThirdsTurnVelocity = (signed char) (double) joystickToTurnVelocity(PS2JoystickValues.rightX) * (double) 0.66;
    signed char halfTurnVelocity = (signed char) (double) joystickToTurnVelocity(PS2JoystickValues.rightX) / (double) 2;
    // signed char quarterTurnVelocity = (signed char) (double) joystickToTurnVelocity(PS2JoystickValues.rightX) / (double) 4;
    
    signed char selectedTurnVelocity = halfTurnVelocity;    // by default select a function of the joystick
    if (circleButtonState || rightButtonState) { selectedTurnVelocity = +15; }  // override joystick with slight steer to the right
    if (squareButtonState || leftButtonState) { selectedTurnVelocity = -15; }  // override joystick with slight steer to the left

    if ( L2button ) // "L2button press defines turbo mode, use the closer to actual raw joystick values for maximum speed"
    {
      signed char selectedThrottle = joystickToThrottle(PS2JoystickValues.leftY);  // by default, select a function of the joystick
      if (upButtonState || (triangleButtonState && !startAndTriangle))   { selectedThrottle = +15; }    // override joystick with slight forward
                                          // except don't move forward via Triangle button if the start button is also pressed at the same time as Triangle indicating to start Quick Trip
      if (downButtonState || xButtonState) { selectedThrottle = -15; }    // override joystick with slight backwards

      // always send operator values To the Pi SPI master
      spiSlavePort.setDataForPi('T', selectedTurnVelocity, selectedThrottle, 0, 0, 0, 0);
      // apply joystick commands locally if enabled
      if (applyJoystickLocally)
      {
        setManualVelocityLoopSetpoints(selectedTurnVelocity, selectedThrottle, false);
      }
    } else  // normally, just use half the values provide by the joystick
    {
      signed char halfThrottle = (signed char) (double) joystickToThrottle(PS2JoystickValues.leftY) / (double) 2;
      signed char selectedThrottle = halfThrottle;        // by default, select a function of the joystick
      if (upButtonState || (triangleButtonState && !startAndTriangle))   { selectedThrottle = +15; }    // override joystick with slight forward
                                          // except don't move forward via Triangle button if the start button is also pressed at the same time as Triangle indicating to start Quick Trip
      if (downButtonState || xButtonState) { selectedThrottle = -15; }    // override joystick with slight backwards

      // always send operator values To the Pi SPI master
      spiSlavePort.setDataForPi('T', selectedTurnVelocity, selectedThrottle, 0, 0, 0, 0);
      // apply joystick commands locally if enabled
      if (applyJoystickLocally)
      {
        setManualVelocityLoopSetpoints(selectedTurnVelocity, selectedThrottle, false);
      }
    }   
  }

  tick20msCounter += 1;

  cpuCycleHeadroom20ms = cpuCycleHeadroom20msIncrement;
  cpuCycleHeadroom20msIncrement = 0;
  digitalWrite(digTP27, LOW);
}

int sampleMotorShieldCount = 0;

// functions which run every 1000ms
void tasks1000ms () {
  // Serial.print("\n\n---sampleMotorShieldCount ");
  // Serial.println(sampleMotorShieldCount);
  sampleMotorShieldCount += 1;

  digitalWrite(cpuStatusLEDbluePin, digitalRead(cpuStatusLEDbluePin) ^ 1);      // toggle the blue pin
  // digitalWrite(RGBswitchBluePin, digitalRead(RGBswitchBluePin) ^ 1);      // toggle the blue pin

  // Moved following code to 20ms ISR.
  // -->> ToDo: adapt error detection into 20ms ISR
  // spiSlavePort.getLatestDataFromPi();
  // spiSlavePort.handleCommandsFromPi();
  // if (digitalRead(cpuStatusLEDbluePin) ){

  //     if (spiSlavePort.getNextSPIxferToPiReserved())
  //     {
  //       Serial.println("Started to queue for Pi, but did not since getNextSPIxferToPiReserved() was true.");
  //     } else
  //     {
  //       Serial.println("queuing for PI: P, Max burst duration, -9, +13, 248, 399, 425");
  //       spiSlavePort.setDataForPi('P', spiSlavePort.getMaxBurstDuration(), -9, +13, 248, 399, 425);
  //     }
  // } else
  // {
  //     if (spiSlavePort.getNextSPIxferToPiReserved())
  //     {
  //       Serial.println("Started to queue for Pi, but did not since getNextSPIxferToPiReserved() was true.");
  //     } else
  //     {
  //       Serial.println("queuing for PI: Q, Max burst duration, 51, -87, 13987, 22459, spiSlavePort.getMaxDelayBetweenBursts()");
  //       spiSlavePort.setDataForPi('Q', spiSlavePort.getMaxBurstDuration(), 51, -87, 13987, 22459, (long) spiSlavePort.getMaxDelayBetweenBursts()); //note: loss of fidelty from casting unsigned long to long...
  //     }      
  // }
  
  // Serial.print(" xfer error count, num bursts rejected too long ");
  // Serial.print(spiSlavePort.getErrorCountSPIrx());
  // Serial.print(", ");
  // Serial.println(spiSlavePort.getNumBurstsRejectedTooLong());
  // Serial.print(" max SPI burst duration (ms), max delay between SPI bursts (ms) ");
  // Serial.print(spiSlavePort.getMaxBurstDuration());
  // Serial.print(", ");
  // Serial.println(spiSlavePort.getMaxDelayBetweenBursts());      

  // Serial.println("Received from Pi: cmd, turn, fwd, sidewys, param1, param2, param3");
  // Serial.print("-> ");
  // Serial.print( spiSlavePort.getCommandFromPi());
  // Serial.print(", ");
  // Serial.print( spiSlavePort.getTurnVelocityFromPi());
  // Serial.print(", ");
  // Serial.print( spiSlavePort.getForwardThrottleFromPi());
  // Serial.print(", ");
  // Serial.print( spiSlavePort.getSidewaysThrottleFromPi());
  // Serial.print(", ");
  // Serial.print( spiSlavePort.getParam1FromPi());
  // Serial.print(", ");
  // Serial.print( spiSlavePort.getParam2FromPi());
  // Serial.print(", ");
  // Serial.println( spiSlavePort.getParam3FromPi());


  if (runContinuousMotorStepResponseTest && digitalRead(cpuStatusLEDbluePin) ){
      // setMotorVelocityByPWM(0,30);
      setAutomaticVelocityLoopSetpoints(0,50,true);
  } 
  if (runContinuousMotorStepResponseTest && !digitalRead(cpuStatusLEDbluePin) ){
      // setMotorVelocityByPWM(0,0);
      setAutomaticVelocityLoopSetpoints(0,0,true);
  } 

  if (SPIlinkActivityCounter > 10)
  {
    // Redish Yellow
    neoPixelStrip.setPixelColor(Neo_SPIlinkActive, neoPixelStrip.Color(25,15,0));
    // Serial.println("-----------> SPI link had solid activity in the last second");    
  } else
  {
    // Off
    neoPixelStrip.setPixelColor(Neo_SPIlinkActive, neoPixelStrip.Color(0,0,0));    
  //   Serial.println("-----------> It appears that the SPI link is offline - no solid activity in the last second");
  }
  SPIlinkActivityCounter = 0;

  // Always refresh the neoPixel strip once per second with whatever colors have been recently set
  //  careful - do not refresh much more often, because both NeoPixels and SPI stop interrupts,
  //  hence, it's easy to see that trying to update the Neopixels every 20ms causes a significant increase in SPI transfer errors
  neoPixelStrip.show();

  RGBswitchPriorSecond = digitalRead(RGBswitchSwitchPin);   // poor mans debouncer, read once per second, hope the switch transtion & bounce doesn't happen on a 1 second boundary.

  Serial.println();
  Serial.print("AnaTP 1..4: ");
  Serial.print(analogRead(potA8), DEC);
  Serial.print(" ");
  Serial.print(analogRead(potA9), DEC);
  Serial.print(" ");
  Serial.print(analogRead(potA10), DEC);
  Serial.print(" ");
  Serial.println(analogRead(potA11), DEC);

  Serial.print("Lx ");
  // Left stick, Y axis. Other options: LX, RY, RX  
  Serial.print(PS2JoystickValues.leftX, DEC); 
  Serial.print(" y ");
  Serial.print(PS2JoystickValues.leftY, DEC);
  
  Serial.print(" Rx ");
  Serial.print(PS2JoystickValues.rightX, DEC); 
  Serial.print(" y ");
  Serial.println(PS2JoystickValues.rightY, DEC); 

  // printRobotOdometerTicks(); // view initial values, BUT clobbers 1st sampleMotorShield() iteration
  // printVelocityLoopValues(); // view initial values, BUT clobbers 1st sampleMotorShield() iteration

  // digitalRead(cpuStatusLEDbluePin) ^ 1;      // toggle the blue pin

  cpuCycleHeadroom1000ms = cpuCycleHeadroom1000msIncrement;

  // Serial.print("\nmillis(): ");
  // Serial.print(millis());
  // Serial.print(", free cycles 20ms: ");
  // Serial.print(cpuCycleHeadroom20ms);
  // Serial.print(", 1000ms: ");
  // Serial.println(cpuCycleHeadroom1000ms);

  cpuCycleHeadroom1000msIncrement = 0;  
}


// make sure all NeoPixels are off
void NeoPixel_Clear()
{
  for (uint16_t i = 0; i < neoPixelStrip.numPixels(); i++)
  {
    neoPixelStrip.setPixelColor(i, neoPixelStrip.Color(0,0,0));  
  }  
  neoPixelStrip.show();
}


// set Neopixels to default values
void NeoPixel_Initialize()
{
  for (uint16_t i = 0; i < neoPixelStrip.numPixels(); i++)
  {
    neoPixelStrip.setPixelColor(i, neoPixelStrip.Color(0,0,0));  
  }  
  // Green
  neoPixelStrip.setPixelColor(Neo_applyJoystickLocally, neoPixelStrip.Color(0,10,0));
  // Green
  neoPixelStrip.setPixelColor(Neo_applySPIHeadingThrottleLocally, neoPixelStrip.Color(0,10,0));
  neoPixelStrip.show();
}

// briefly cycle through R G B for all NeoPixels
void NeoPixel_StripTest()
{
  for (uint16_t i = 0; i < neoPixelStrip.numPixels(); i++)
  {
    neoPixelStrip.setPixelColor(i, neoPixelStrip.Color(100,0,0));  
    neoPixelStrip.show();
    delay(100);
  }  
  for (uint16_t i = neoPixelStrip.numPixels() -1; i > 0 ; i--)
  {
    neoPixelStrip.setPixelColor(i, neoPixelStrip.Color(0,100,0));  
    neoPixelStrip.show();
    delay(100);
  }  
  for (uint16_t i = 0; i < neoPixelStrip.numPixels(); i++)
  {
    neoPixelStrip.setPixelColor(i, neoPixelStrip.Color(0,0,100));  
    neoPixelStrip.show();
    delay(100);
  }  
  for (uint16_t i = 0; i < neoPixelStrip.numPixels(); i++)
  {
    neoPixelStrip.setPixelColor(i, neoPixelStrip.Color(0,0,0));  
    neoPixelStrip.show();
    delay(100);
  }  
}


void setup()
{

  // aha- the all important class initializer...
  neoPixelStrip.begin();

  Serial.begin(250000);   // Serial:  0(RX), 1(TX) => use the highest possible rate to minimize drag on the CPU
                          // e.g. https://forum.arduino.cc/index.php?topic=76359.0
                          // e.g. https://www.quora.com/What-is-the-baud-rate-and-why-does-Arduino-have-a-baud-rate-of-9-600
  Serial3.begin(57600); // => ToDo - Set This up for communication to the Display
                        // Serial3:  15(RX), 14(TX)
                        // https://www.arduino.cc/reference/en/language/functions/communication/serial/
                        // https://www.arduino.cc/reference/en/language/functions/communication/serial/print/
                        // https://www.arduino.cc/reference/en/language/functions/communication/serial/write/#howtouse  
  printFreeBytesOfRAM();

  taskLoopCounter = 0;

// chase this down, why does it look like uint8_t appear mapped to unsigned int, but behave more like byte or char?
  // uint8_t sillyTest = 289;
  // Serial.print("silly Test is ");
  // Serial.println(sillyTest);

  // Digital Test Points
  pinMode (digTP26, OUTPUT);
  pinMode (digTP27, OUTPUT);
  pinMode (digTP28, OUTPUT);
  pinMode (digTP29, OUTPUT);
  digitalWrite(digTP26, LOW);
  digitalWrite(digTP27, LOW);
  digitalWrite(digTP28, LOW);
  digitalWrite(digTP29, LOW);

  // CPU Status RGB LED
  pinMode (cpuStatusLEDredPin, OUTPUT);
  pinMode (cpuStatusLEDgreenPin, OUTPUT);
  pinMode (cpuStatusLEDbluePin, OUTPUT);

  // RGB Switch
  pinMode (RGBswitchRedPin, OUTPUT);
  pinMode (RGBswitchGreenPin, OUTPUT);
  pinMode (RGBswitchBluePin, OUTPUT);
  pinMode (RGBswitchSwitchPin, INPUT);

  // cpuStatusLED -> active HIGH
  digitalWrite(cpuStatusLEDredPin, HIGH);   // start with Red on
  digitalWrite(cpuStatusLEDgreenPin, LOW);
  digitalWrite(cpuStatusLEDbluePin, LOW);

  // RGBswitchLED -> active LOW
  digitalWrite(RGBswitchRedPin, HIGH);
  digitalWrite(RGBswitchGreenPin, HIGH);
  digitalWrite(RGBswitchBluePin, LOW);      // start with Blue on

  // Initialize timer 5 to periodically trigger the main loop via ISR(TIMER5_COMPA_vect)
  //    note arduino defines for cli() and sei() per https://forum.arduino.cc/index.php?topic=96156.0
  noInterrupts();           // defined as cli() temporarily disable all interrupts during initialization
  TCCR5A = 0;
  TCCR5B = 0;
  TCNT5 = 0;


  //Reminder -> take care -> motorTasks.cpp has a filter dependency on this sampling period!
  OCR5A = 1250;             // compare match register 16MHz/256 -> 1250 counts => 20ms ~50Hz
  // OCR5A = 62500;             // compare match register 16MHz/256 -> 62500 counts => 1Hz
  // OCR5A = 625;             // compare match register 16MHz/256 -> 625 counts => 100Hz
  TCCR5B |= (1 << WGM12);   // select CTC mode
  TCCR5B |= (1 << CS12);    // select 256 prescaler 
  TIMSK5 |= (1 << OCIE5A);  // enable timer compare interrupt

  interrupts();             // defined as sei() enable all interrupts


  // initialize counters to keep approximate tabs on available CPU cycles between interrupts
  cpuCycleHeadroom20ms = 0;
  cpuCycleHeadroom20msIncrement = 0;
  cpuCycleHeadroom1000ms = 0;
  cpuCycleHeadroom1000msIncrement = 0;

  tick20msCounter = 0;

  // turn off all NeoPixels
  NeoPixel_Clear();
  delay(750);

  // cycle through R G B for each NeoPixel
  NeoPixel_StripTest();

// set Neopixels to default values
  NeoPixel_Initialize();

  // Initialize and check for a PS2 Controller
  initPS2xController();

  // Kludgy switches to run one or another thing when first power up
  runContinuousMotorStepResponseTest = false;  // remember to set velocity_setpoint_lowpass_cutoff_freq to 20 Hz to do a step response test
  runQuickTrip = false;
  readAndViewAllPS2Buttons = true;

  initializeMotorTasks();
  periodicSampleMotorShield_Start();
  Serial.println ("\nStarting periodic loops.");
  Serial.println("\n-----\n");

  Serial.println("Enabling the SPI as a slave.");
  spiSlavePort.enable();
  Serial.println("SPI initialization complete.");

  digitalWrite(RGBswitchRedPin, HIGH);       
  digitalWrite(RGBswitchGreenPin, LOW);   // turn green on - indicate ready to run
  digitalWrite(RGBswitchBluePin, HIGH);      
}

void loop()
{
  if (ISR20msActive){
    tasks20ms ();
    ISR20msActive = false;
  }





  cpuCycleHeadroom20msIncrement += 1;
  cpuCycleHeadroom1000msIncrement += 1;
}

