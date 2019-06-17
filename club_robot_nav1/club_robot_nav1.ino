
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

// attached a SparkFun COM-11120 10mm diffused RGB LED, with common/cathode to ground, and RGB pins as follows with resistors to approximately balance light intensity
#define cpuStatusLEDredPin 51     // 325 ohm
#define cpuStatusLEDgreenPin 52   // 1.2K ohm
#define cpuStatusLEDbluePin 53    // 1K ohm


void setup()
{
  // Serial.begin(57600);  // Serial:  0(RX), 1(TX)
  // Serial3.begin(57600); // => ToDo - Set This up for communication to the Display
  //                       // Serial3:  15(RX), 14(TX)
  //                       // https://www.arduino.cc/reference/en/language/functions/communication/serial/
  //                       // https://www.arduino.cc/reference/en/language/functions/communication/serial/print/
  //                       // https://www.arduino.cc/reference/en/language/functions/communication/serial/write/#howtouse  
  // printFreeBytesOfRAM();

  bitSet(DDRB, 7); // set ATMega2560 PB7/D13 as an output // saves 4 bytes over Arduino pinMode() per Dale Wheat - Arduino Internals page 101
  TCCR5A = 0;
  TCCR5B = 1<<CS12;       // force overflow around once per second
  bitSet(TIMSK5, TOIE5);  // enable overflow interrupt

  pinMode (cpuStatusLEDredPin, OUTPUT);
  pinMode (cpuStatusLEDgreenPin, OUTPUT);
  pinMode (cpuStatusLEDbluePin, OUTPUT);


  // initialize counters to keep approximate tabs on available CPU cycles between interrupts
  // cpuCycleHeadroom10ms = 0;
  // cpuCycleHeadroom10msIncrement = 0;
  // cpuCycleHeadroom100ms = 0;
  // cpuCycleHeadroom100msIncrement = 0;


  // initializeMotorTasks();
}

void loop()
{
  digitalWrite(cpuStatusLEDredPin, HIGH);
  digitalWrite(cpuStatusLEDgreenPin, LOW);
  digitalWrite(cpuStatusLEDbluePin, LOW);
  delay(500);  
  digitalWrite(cpuStatusLEDredPin, LOW);
  digitalWrite(cpuStatusLEDgreenPin, HIGH);
  digitalWrite(cpuStatusLEDbluePin, LOW);  
  delay(500);  
  digitalWrite(cpuStatusLEDredPin, LOW);
  digitalWrite(cpuStatusLEDbluePin, HIGH);
  digitalWrite(cpuStatusLEDgreenPin, LOW);
  delay(500);  

  digitalWrite(cpuStatusLEDredPin, HIGH);
  digitalWrite(cpuStatusLEDgreenPin, HIGH);
  digitalWrite(cpuStatusLEDbluePin, LOW);
  delay(500);  
  digitalWrite(cpuStatusLEDredPin, HIGH);
  digitalWrite(cpuStatusLEDgreenPin, LOW);
  digitalWrite(cpuStatusLEDbluePin, HIGH);
  delay(500);  
  digitalWrite(cpuStatusLEDredPin, LOW);
  digitalWrite(cpuStatusLEDgreenPin, HIGH);
  digitalWrite(cpuStatusLEDbluePin, HIGH);
  delay(500);  
  digitalWrite(cpuStatusLEDredPin, HIGH);
  digitalWrite(cpuStatusLEDgreenPin, HIGH);
  digitalWrite(cpuStatusLEDbluePin, HIGH);
  delay(500);  
  digitalWrite(cpuStatusLEDredPin, LOW);
  digitalWrite(cpuStatusLEDgreenPin, LOW);
  digitalWrite(cpuStatusLEDbluePin, LOW);
  delay(500);  


  // cpuCycleHeadroom10msIncrement += 1;
  // cpuCycleHeadroom100msIncrement += 1;
}

