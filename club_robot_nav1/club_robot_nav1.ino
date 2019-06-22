
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
#include <arduino.h>
#include <avr/io.h>     // per Dale Wheat / Arduino Internals page 35.  Explicitly included to reference Arduion registers, even though Arduino automatically picks it up when not included

// attached a SparkFun COM-11120 10mm diffused RGB LED, with common/cathode to ground, and RGB pins as follows with resistors to approximately balance light intensity
#define cpuStatusLEDredPin 51     // 325 ohm
#define cpuStatusLEDgreenPin 52   // 1.2K ohm
#define cpuStatusLEDbluePin 53    // 1K ohm


unsigned int cpuCycleHeadroom10ms;            // most recent number of spare cycles betewen 10ms interrupt periods
unsigned int cpuCycleHeadroom10msIncrement;   // working count
unsigned int cpuCycleHeadroom500ms;           // most recent number of spare cycles betewen 100ms interrupt periods
unsigned int cpuCycleHeadroom500msIncrement;  // working count

// interrupt handler sketch per Dale Wheat - Arduino Internals, page 145
//  -> except changed from timer 1 to timer 5 (since timer 1 is incompatible with delay() and other functions, 
//  -> and timers 3 and 4 are needed to drive PWM on club robot Monster Motor Shield via ATMega2560 pins 5 and 6 )
//  -> for timer/PWM/pin compatibility ref also https://www.robotshop.com/community/forum/t/arduino-101-timers-and-interrupts/13072
ISR(TIMER5_OVF_vect) {
  bitSet(PINB, 7); // toggle ATMega2560 PB7/D13 LED pin state by writing to the input read register // per Dale Wheat - Arduino Internals page 145
}

volatile bool ISR10msActive = false;
volatile bool cpuStatusLEDisWhite = false;

// Use timer 5 to periodically trigger the main loop with well controlled timing
// using timer 5 to avoid messing with timers needed for other purposes
// interrupt handler via timer compare match method, per https://www.robotshop.com/community/forum/t/arduino-101-timers-and-interrupts/13072
ISR(TIMER5_COMPA_vect){
  if (!cpuStatusLEDisWhite){
    digitalWrite(cpuStatusLEDredPin, digitalRead(cpuStatusLEDredPin) ^ 1);        // toggle red LED pin
    digitalWrite(cpuStatusLEDgreenPin, digitalRead(cpuStatusLEDgreenPin) ^ 1);    // toggle green LED pin
  }

  ISR10msActive = true;
}

char taskLoopCounter;      // used to divide the 100 Hz loop into a 10 Hz loop

// functions which run every 10ms
void tasks10ms () {
  if (taskLoopCounter == 49) {
    taskLoopCounter = 0;
    tasks500ms();
  } else {
    taskLoopCounter += 1;
  }

  if ((taskLoopCounter == 4) && digitalRead(cpuStatusLEDbluePin)){
    cpuStatusLEDisWhite = false;
    digitalWrite(cpuStatusLEDredPin, HIGH);
    digitalWrite(cpuStatusLEDgreenPin, LOW);
    digitalWrite(cpuStatusLEDbluePin, LOW);
  }

  cpuCycleHeadroom10ms = cpuCycleHeadroom10msIncrement;
  cpuCycleHeadroom10msIncrement = 0;
}

// functions which run every 500ms
void tasks500ms () {
  cpuStatusLEDisWhite = true;
  digitalWrite(cpuStatusLEDredPin, HIGH);
  digitalWrite(cpuStatusLEDgreenPin, HIGH);
  digitalWrite(cpuStatusLEDbluePin, HIGH);

  cpuCycleHeadroom500ms = cpuCycleHeadroom500msIncrement;

  Serial.print("millis(): ");
  Serial.print(millis());
  Serial.print(", free cycles 10ms: ");
  Serial.print(cpuCycleHeadroom10ms);
  Serial.print(", 500ms: ");
  Serial.println(cpuCycleHeadroom500ms);

  cpuCycleHeadroom500msIncrement = 0;  
}



void setup()
{
  Serial.begin(57600);  // Serial:  0(RX), 1(TX)
  Serial3.begin(57600); // => ToDo - Set This up for communication to the Display
                        // Serial3:  15(RX), 14(TX)
                        // https://www.arduino.cc/reference/en/language/functions/communication/serial/
                        // https://www.arduino.cc/reference/en/language/functions/communication/serial/print/
                        // https://www.arduino.cc/reference/en/language/functions/communication/serial/write/#howtouse  
  printFreeBytesOfRAM();

  taskLoopCounter = 0;

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

  OCR5A = 625;             // compare match register 16MHz/256 -> 625 counts => 100Hz
  TCCR5B |= (1 << WGM12);   // select CTC mode
  TCCR5B |= (1 << CS12);    // select 256 prescaler 
  TIMSK5 |= (1 << OCIE5A);  // enable timer compare interrupt

  interrupts();             // defined as sei() enable all interrupts


  // initialize counters to keep approximate tabs on available CPU cycles between interrupts
  cpuCycleHeadroom10ms = 0;
  cpuCycleHeadroom10msIncrement = 0;
  cpuCycleHeadroom500ms = 0;
  cpuCycleHeadroom500msIncrement = 0;


  // initializeMotorTasks();
}

void loop()
{
  if (ISR10msActive){
    tasks10ms ();
    ISR10msActive = false;
  }

  cpuCycleHeadroom10msIncrement += 1;
  cpuCycleHeadroom500msIncrement += 1;
}

