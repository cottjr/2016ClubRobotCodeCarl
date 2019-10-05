
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

#define cpuStatusPin48 48         // simple digital pin for o'scope monitoring
#define cpuStatusPin50 50         // simple digital pin for o'scope monitoring
// attached a SparkFun COM-11120 10mm diffused RGB LED, with common/cathode to ground, and RGB pins as follows with resistors to approximately balance light intensity
#define cpuStatusLEDredPin 51     // 325 ohm
#define cpuStatusLEDgreenPin 52   // 1.2K ohm
#define cpuStatusLEDbluePin 53    // 1K ohm


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

  digitalWrite(cpuStatusPin50, HIGH);
  filterTurnAndThrottleRequestValues(); // lowpass Throttle and Turn Velocity commands to match platform capability
  sampleMotorShield();

  if (taskLoopCounter == 49) {
    taskLoopCounter = 0;
    digitalWrite(cpuStatusPin48, HIGH);
    tasks1000ms();
    digitalWrite(cpuStatusPin48, LOW);
  } else {
    taskLoopCounter += 1;
  }

  if ((taskLoopCounter == 4) && digitalRead(cpuStatusLEDbluePin)){
    // cpuStatusLEDisWhite = false;
    // digitalWrite(cpuStatusLEDredPin, HIGH);
    // digitalWrite(cpuStatusLEDgreenPin, LOW);
    // digitalWrite(cpuStatusLEDbluePin, LOW);
  }

  unsigned char quickTripSpeed = 100;
  if (runQuickTrip){
    if (tick20msCounter == 150){
      // start moving 3 seconds after power up
      // move forward
      setVelocityLoopSetpoints(0,quickTripSpeed,true);
    }
    if (tick20msCounter == 250){
      // turn motors off
      setVelocityLoopSetpoints(0,0,true);
      // wait for 3 seconds
    }
    if (tick20msCounter == 350){
      // then move backwards
        setVelocityLoopSetpoints(0,-quickTripSpeed,true);
    }
    if (tick20msCounter == 450){
      // stop moving, please...    
      setVelocityLoopSetpoints(0,0,true);
    }
  }

  if (readAndViewAllPS2Buttons){
    readAllPS2xControllerValues();  // show any PS2 controller events/data if present
  }

  // readPS2Joysticks( &PS2JoystickValues );  // read the latest PS2 controller joystick values

  digitalWrite(cpuStatusPin50, LOW);
  tick20msCounter += 1;

  cpuCycleHeadroom20ms = cpuCycleHeadroom20msIncrement;
  cpuCycleHeadroom20msIncrement = 0;
}

int sampleMotorShieldCount = 0;

// functions which run every 500ms
void tasks1000ms () {
  Serial.print("\n\n---sampleMotorShieldCount ");
  Serial.println(sampleMotorShieldCount);
  sampleMotorShieldCount += 1;

  digitalWrite(cpuStatusLEDbluePin, digitalRead(cpuStatusLEDbluePin) ^ 1);      // toggle the blue pin

  if (runContinuousMotorStepResponseTest && digitalRead(cpuStatusLEDbluePin) ){
      // setMotorVelocityByPWM(0,30);
      setVelocityLoopSetpoints(0,50,true);
  } 
  if (runContinuousMotorStepResponseTest && !digitalRead(cpuStatusLEDbluePin) ){
      // setMotorVelocityByPWM(0,0);
      setVelocityLoopSetpoints(0,0,true);
  } 
 
  // printRobotOdometerTicks(); // view initial values, BUT clobbers 1st sampleMotorShield() iteration
  // printVelocityLoopValues(); // view initial values, BUT clobbers 1st sampleMotorShield() iteration

  // digitalRead(cpuStatusLEDbluePin) ^ 1;      // toggle the blue pin

  cpuCycleHeadroom1000ms = cpuCycleHeadroom1000msIncrement;

  Serial.print("\nmillis(): ");
  Serial.print(millis());
  Serial.print(", free cycles 20ms: ");
  Serial.print(cpuCycleHeadroom20ms);
  Serial.print(", 1000ms: ");
  Serial.println(cpuCycleHeadroom1000ms);

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


  pinMode (cpuStatusPin48, OUTPUT);
  pinMode (cpuStatusPin50, OUTPUT);
  digitalWrite(cpuStatusPin48, LOW);
  digitalWrite(cpuStatusPin50, LOW);

  pinMode (cpuStatusLEDredPin, OUTPUT);
  pinMode (cpuStatusLEDgreenPin, OUTPUT);
  pinMode (cpuStatusLEDbluePin, OUTPUT);

  digitalWrite(cpuStatusLEDredPin, HIGH);
  digitalWrite(cpuStatusLEDgreenPin, LOW);
  digitalWrite(cpuStatusLEDbluePin, LOW);

  // Initialize timer 5 to periodically trigger the main loop via ISR(TIMER5_COMPA_vect)
  //    note arduino defines for cli() and sei() per https://forum.arduino.cc/index.php?topic=96156.0
  noInterrupts();           // defined as cli() temporarily disable all interrupts during initialization
  TCCR5A = 0;
  TCCR5B = 0;
  TCNT5 = 0;

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
  // initPS2xController();

  // Kludgy switches to run one or another thing when first power up
  runContinuousMotorStepResponseTest = false;  // remember to set velocity_setpoint_lowpass_cutoff_freq to 20 Hz to do a step response test
  runQuickTrip = true;
  readAndViewAllPS2Buttons = false;

  initializeMotorTasks();
  periodicSampleMotorShield_Start();
  Serial.println ("\nStarting periodic loops.");
  Serial.println("\n-----\n");
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

