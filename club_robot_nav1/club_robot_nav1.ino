
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

  if (!digitalRead(RGBswitchSwitchPin) && runQuickTrip)   // quickly stop QuickTrip
  {
      // Reset the QuickTrip Routine
      digitalWrite(RGBswitchRedPin, HIGH);       
      digitalWrite(RGBswitchGreenPin, LOW);   // turn green on - indicate ready to run
      digitalWrite(RGBswitchBluePin, HIGH);      
      setAutomaticVelocityLoopSetpoints(0, 0, true);
      runQuickTrip = false;
  }

  if ( !RGBswitchPriorSecond && digitalRead(RGBswitchSwitchPin) && !runQuickTrip )   // allow start QuickTrip on RGB button release, ie. rising edge of RGBswitchSwitchPin
  {
    // start a QuickTrip Routine
    digitalWrite(RGBswitchRedPin, LOW);       // turn Red on during the run
    digitalWrite(RGBswitchGreenPin, HIGH);
    digitalWrite(RGBswitchBluePin, HIGH);      
    QuickTripStartCounter = tick20msCounter;
    runQuickTrip = true;
  }


  if (ps2ControllerUseable && startAndTriangle && !runQuickTrip) // start a quick trip if requested and not already in progress
  {
    QuickTripStartCounter = tick20msCounter;
    runQuickTrip = true;
  };
  if (runQuickTrip)
  {
    digitalWrite(RGBswitchRedPin, LOW);       // turn Red on during the run
    digitalWrite(RGBswitchGreenPin, HIGH);
    digitalWrite(RGBswitchBluePin, HIGH);      
    // start moving 1.5 seconds after power up
    // move forward
    if (tick20msCounter == QuickTripStartCounter + 75)
    {
      setAutomaticVelocityLoopSetpoints( 1, quickTripSpeedQuarter, true);
    }
    if (tick20msCounter == QuickTripStartCounter + 100)
    {
      setAutomaticVelocityLoopSetpoints( 1, quickTripSpeedHalf, true);
    }
    if (tick20msCounter == QuickTripStartCounter + 125)
    {
      setAutomaticVelocityLoopSetpoints( 1, quickTripSpeedThreeQuarter, true);
    }
    if (tick20msCounter == QuickTripStartCounter + 150)
    {
      setAutomaticVelocityLoopSetpoints(0, quickTripSpeed, true);
    }
    if (tick20msCounter == QuickTripStartCounter + 250)
    {
      setAutomaticVelocityLoopSetpoints(0, quickTripSpeedThreeQuarter, true);
    }
    if (tick20msCounter == QuickTripStartCounter + 275)
    {
      setAutomaticVelocityLoopSetpoints(0, quickTripSpeedHalf, true);
    }
    if (tick20msCounter == QuickTripStartCounter + 300)
    {
      setAutomaticVelocityLoopSetpoints(0, quickTripSpeedQuarter, true);
    }
    if (tick20msCounter == QuickTripStartCounter + 325)
    {
      setAutomaticVelocityLoopSetpoints(0, 0, true);
    }
    // wait for 1.5 seconds
    // then head backwards
    if (tick20msCounter == QuickTripStartCounter + 400)
    {
      setAutomaticVelocityLoopSetpoints( -2, -quickTripSpeedQuarter, true);
    }
    if (tick20msCounter == QuickTripStartCounter + 425)
    {
      setAutomaticVelocityLoopSetpoints( -2, -quickTripSpeedHalf, true);
    }
    if (tick20msCounter == QuickTripStartCounter + 450)
    {
      setAutomaticVelocityLoopSetpoints( -2, -quickTripSpeedThreeQuarter, true);
    }
    if (tick20msCounter == QuickTripStartCounter + 475)
    {
      setAutomaticVelocityLoopSetpoints(0, -quickTripSpeed, true);
    }
    if (tick20msCounter == QuickTripStartCounter + 575)
    {
      setAutomaticVelocityLoopSetpoints(0, -quickTripSpeedThreeQuarter, true);
    }
    if (tick20msCounter == QuickTripStartCounter + 600)
    {
      setAutomaticVelocityLoopSetpoints(0, -quickTripSpeedHalf, true);
    }
    if (tick20msCounter == QuickTripStartCounter + 625)
    {
      setAutomaticVelocityLoopSetpoints(0, -quickTripSpeedQuarter, true);
    }
    if (tick20msCounter == QuickTripStartCounter + 650)
    {
      setAutomaticVelocityLoopSetpoints(0, 0, true);

      // Reset the QuickTrip Routine
      digitalWrite(RGBswitchRedPin, HIGH);       
      digitalWrite(RGBswitchGreenPin, LOW);   // turn green on - indicate ready to run
      digitalWrite(RGBswitchBluePin, HIGH);      
      runQuickTrip = false;
    }
  }

  // readAndViewAllPS2Buttons costs about 2.2ms, plus another 1.1ms to write out values on push
  if ( ps2ControllerUseable && readAndViewAllPS2Buttons )
  {
    readAllPS2xControllerValues();  // show any PS2 controller events/data if present
    readPS2Joysticks( &PS2JoystickValues );  // read the latest PS2 controller joystick values
    if ( L2button ) // "L2button press defines turbo mode, use the actual raw joystick values for maximum speed"
    {
      setManualVelocityLoopSetpoints(joystickToTurnVelocity(PS2JoystickValues.rightX),joystickToThrottle(PS2JoystickValues.leftY), false);
    } else  // normally, just use half the values provide by the joystick
    {
      signed char halfTurnVelocity = (signed char) (double) joystickToTurnVelocity(PS2JoystickValues.rightX) / (double) 2;
      signed char halfThrottle = (signed char) (double) joystickToThrottle(PS2JoystickValues.leftY) / (double) 2;
      setManualVelocityLoopSetpoints(halfTurnVelocity,halfThrottle, false);
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


  if (runContinuousMotorStepResponseTest && digitalRead(cpuStatusLEDbluePin) ){
      // setMotorVelocityByPWM(0,30);
      setAutomaticVelocityLoopSetpoints(0,50,true);
  } 
  if (runContinuousMotorStepResponseTest && !digitalRead(cpuStatusLEDbluePin) ){
      // setMotorVelocityByPWM(0,0);
      setAutomaticVelocityLoopSetpoints(0,0,true);
  } 
 
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



void setup()
{
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

