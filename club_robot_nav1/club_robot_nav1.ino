
/*******************************************************
*  club_robot_nav1.ino
*
*  LMX multi-tasking on ARDUINO 
*  04 Sep 2015 dpa Created.  Mega2560
*
*  version: 20161011-0           Doug Paradis
*  LMX for Arduino Mega2560 version of DPRG Club Robot 2016 .
*  
*
 *******************************************************/

// #include <SoftwareServo.h>
// #include <stdio.h>

#include <task.h>
#include "libtaskMemoryTest.h"
#include "motorTasks.h"

void printkbuf(char *s)
{
#if ((MACHINE == MACH_AVR) || (MACHINE == MACH_ARM))
  // Serial.println(s);
  PRINTF(s);
#endif
}

// system_init
void system_init(void)
{
  Serial.begin(57600);  // Serial:  0(RX), 1(TX)
  Serial3.begin(57600); // => ToDo - Set This up for communication to the Display
                        // Serial3:  15(RX), 14(TX)
                        // https://www.arduino.cc/reference/en/language/functions/communication/serial/
                        // https://www.arduino.cc/reference/en/language/functions/communication/serial/print/
                        // https://www.arduino.cc/reference/en/language/functions/communication/serial/write/#howtouse

#if ((MACHINE == MACH_AVR) || (MACHINE == MACH_ARM))
  // Serial.println("...before sysclock_init");
  sysclock_init();
#endif
}

/* ----------------------------------------- */
/* main */

#if ((MACHINE == MACH_AVR) || (MACHINE == MACH_ARM)) /* ARM is Teensy3.1 */
// function definition when running in the Arduino IDE "sketch" environment, using setup() and loop()
void setup()
#else
// function definiton for other environments which require main()
int main()
#endif
{
  //  Serial.println("\n\nclub_robot_nav1.ino => main():\n");
  //  Serial.println("... starting system_init()");
  system_init();
  // printv = printkbuf;

  printFreeBytesOfRAM();

  // Serial.println("\n... Completed system_init...\n");

#if ((MACHINE == MACH_AVR) || (MACHINE == MACH_ARM)) /* ARM is Teensy3.1 */
  // Serial.println("... Starting kludge (?) delay to allow libtask sysclock to start running:\n");
  delay(1500); // uses a fully blocking - Arduino native hardware delay, sysclock not running yet - what good does this really do?
               // https://learn.adafruit.com/multi-tasking-the-arduino-part-1/ditch-the-delay
  // Serial.println("... Completed kludge (?) delay to allow libtask sysclock to start running:\n");
#endif

  // Sample available CPU cycles once per second, updates global idleCPUcountPerSec
  Serial.println("... motorTasks.cpp -> launching task monitorCPUidle");
  int monitorCPUidle_ProcessID = -1;
  monitorCPUidle_ProcessID = create_task("monitorCPUidle", monitorCPUidle, 0, MINSTACK);
  Serial.println("launched monitorCPUidle");
  printFreeBytesOfRAM();

  // keep an eye to the list of running tasks and their stack utilization
  int monitorResourcesForAllTasks_ProcessID = -1;
  monitorResourcesForAllTasks_ProcessID = create_task("monitorResourcesForAllTasks", monitorResourcesForAllTasks, 1000, MINSTACK * 3);
  Serial.println("launched monitorResourcesForAllTasks");
  printFreeBytesOfRAM();


  // trigger launching a task to periodically sample the motor shield & run PID loop(s)
  periodicSampleMotorShield_Start();
  printFreeBytesOfRAM();

  // Run a simple test of the velocity PID loop for a limited time. Then stop the periodicSampleMotorShield task
  int testVelocityPIDloop_ProcessID = -1;
  testVelocityPIDloop_ProcessID = create_task("testVelocityPIDloop", testVelocityPIDloop, 0, MINSTACK * 3);
  Serial.println("launched testVelocityPIDloop");
  printFreeBytesOfRAM();


  initializeMotorTasks();

  Serial.println("\nLaunching scheduler\n\n");
  scheduler();
  Serial.println("\nopps- scheduler fail\n");

  // PRINTF("Should never get here.");

  while (1)
    ;
#if ((MACHINE != MACH_AVR) && (MACHINE != MACH_ARM))
  return 0;
#endif
}

void loop()
{
  /* nothing to see here, move along */
  // ie. with libtask (presumably), loop() sits and waits for an appropriate interrupt event
  // steady state operating code is managed by the libtask scheduler()
  asm("nop");
}

/* ----------------------------------------- */
/* EOF */
