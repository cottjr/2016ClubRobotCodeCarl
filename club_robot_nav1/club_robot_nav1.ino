
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
#include <avr/io.h>     // per Dale Wheat / Arduino Internals page 35.  Explicitly included to reference Arduion registers, even though Arduino automatically picks it up when not included

// attached a SparkFun COM-11120 10mm diffused RGB LED, with common/cathode to ground, and RGB pins as follows with resistors to approximately balance light intensity
#define cpuStatusLEDredPin 51     // 325 ohm
#define cpuStatusLEDgreenPin 52   // 1.2K ohm
#define cpuStatusLEDbluePin 53    // 1K ohm


// unsigned int cpuCycleHeadroom10ms;            // most recent number of spare cycles betewen 10ms interrupt periods
// unsigned int cpuCycleHeadroom10msIncrement;   // working count
// unsigned int cpuCycleHeadroom100ms;           // most recent number of spare cycles betewen 100ms interrupt periods
// unsigned int cpuCycleHeadroom100msIncrement;  // working count

// interrupt handler sketch per Dale Wheat - Arduino Internals, page 145
//  -> except changed from timer 1 to timer 5 (since timer 1 is incompatible with delay() and other functions, 
//  -> and timers 3 and 4 are needed to drive PWM on club robot Monster Motor Shield via ATMega2560 pins 5 and 6 )
//  -> for timer/PWM/pin compatibility ref also https://www.robotshop.com/community/forum/t/arduino-101-timers-and-interrupts/13072
ISR(TIMER5_OVF_vect) {
  bitSet(PINB, 7); // toggle ATMega2560 PB7/D13 LED pin state by writing to the input read register // per Dale Wheat - Arduino Internals page 145
}


char taskLoopCounter;      // used to divide the 100 Hz loop into a 10 Hz loop

// Use timer 5 to periodically trigger the main loop with well controlled timing
// using timer 5 to avoid messing with timers needed for other purposes
// interrupt handler via timer compare match method, per https://www.robotshop.com/community/forum/t/arduino-101-timers-and-interrupts/13072
ISR(TIMER5_COMPA_vect){
  digitalWrite(cpuStatusLEDredPin, digitalRead(cpuStatusLEDredPin) ^ 1);        // toggle red LED pin
  digitalWrite(cpuStatusLEDgreenPin, digitalRead(cpuStatusLEDgreenPin) ^ 1);    // toggle green LED pin

  tasks100Hz ();

  if (taskLoopCounter == 49) {
    taskLoopCounter = 0;
    tasks2Hz();
  } else {
    taskLoopCounter += 1;
  }

  if ((taskLoopCounter == 4) && digitalRead(cpuStatusLEDbluePin)){
    digitalWrite(cpuStatusLEDbluePin, LOW);
  }
}

// functions which run at 100 Hz
void tasks100Hz () {
}

// functions which run at 2 Hz
void tasks2Hz () {
  digitalWrite(cpuStatusLEDbluePin, HIGH);
}



void setup()
{
  // Serial.begin(57600);  // Serial:  0(RX), 1(TX)
  // Serial3.begin(57600); // => ToDo - Set This up for communication to the Display
  //                       // Serial3:  15(RX), 14(TX)
  //                       // https://www.arduino.cc/reference/en/language/functions/communication/serial/
  //                       // https://www.arduino.cc/reference/en/language/functions/communication/serial/print/
  //                       // https://www.arduino.cc/reference/en/language/functions/communication/serial/write/#howtouse  
  // printFreeBytesOfRAM();

  taskLoopCounter = 0;

  pinMode (cpuStatusLEDredPin, OUTPUT);
  pinMode (cpuStatusLEDgreenPin, OUTPUT);
  pinMode (cpuStatusLEDbluePin, OUTPUT);

  digitalWrite(cpuStatusLEDredPin, HIGH);
  digitalWrite(cpuStatusLEDgreenPin, LOW);
  digitalWrite(cpuStatusLEDbluePin, LOW);

  // Initialize timer 5 to periodically trigger the main loop via ISR(TIMER5_COMPA_vect)
  noInterrupts();           // temporarily disable all interrupts during initialization
  TCCR5A = 0;
  TCCR5B = 0;
  TCNT5 = 0;

  OCR5A = 625;             // compare match register 16MHz/256 -> 625 counts => 100Hz
  TCCR5B |= (1 << WGM12);   // select CTC mode
  TCCR5B |= (1 << CS12);    // select 256 prescaler 
  TIMSK5 |= (1 << OCIE5A);  // enable timer compare interrupt

  interrupts();             // enable all interrupts


  // initialize counters to keep approximate tabs on available CPU cycles between interrupts
  // cpuCycleHeadroom10ms = 0;
  // cpuCycleHeadroom10msIncrement = 0;
  // cpuCycleHeadroom100ms = 0;
  // cpuCycleHeadroom100msIncrement = 0;


  // initializeMotorTasks();
}

void loop()
{

  // cpuCycleHeadroom10msIncrement += 1;
  // cpuCycleHeadroom100msIncrement += 1;
}

