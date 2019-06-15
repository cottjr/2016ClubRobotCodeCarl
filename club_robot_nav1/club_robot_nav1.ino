
/*******************************************************
* Main file for Carl's version of the 2016 DPRG Club Robot
* AKA "ClubCar DonkeyBot"
* Reference these:
* BranchPlanning.txt
* Planning Notes
 *******************************************************/

// #include <SoftwareServo.h>
// #include <stdio.h>

#include "libtaskMemoryTest.h"
#include "motorTasks.h"


void setup()

{
  Serial.begin(57600);  // Serial:  0(RX), 1(TX)
  Serial3.begin(57600); // => ToDo - Set This up for communication to the Display
                        // Serial3:  15(RX), 14(TX)
                        // https://www.arduino.cc/reference/en/language/functions/communication/serial/
                        // https://www.arduino.cc/reference/en/language/functions/communication/serial/print/
                        // https://www.arduino.cc/reference/en/language/functions/communication/serial/write/#howtouse
  printFreeBytesOfRAM();



  initializeMotorTasks();

}

void loop()
{

}

