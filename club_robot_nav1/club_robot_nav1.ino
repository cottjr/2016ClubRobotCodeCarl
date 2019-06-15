
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
ISR(TIMER1_OVF_vect) {
  bitSet(PINB, 7); // toggle ATMega2560 PB7/D13 LED pin state by writing to the input read register // per Dale Wheat - Arduino Internals page 145
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

  bitSet(DDRB, 7); // set ATMega2560 PB7/D13 as an output // saves 4 bytes over Arduino pinMode() per Dale Wheat - Arduino Internals page 101
  TCCR1A = 0;
  TCCR1B = 1<<CS12;       // force overflow around once per second
  bitSet(TIMSK1, TOIE1);  // enable overflow interrupt

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

