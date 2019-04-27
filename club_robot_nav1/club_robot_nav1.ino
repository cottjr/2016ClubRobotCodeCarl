
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

#include <SoftwareServo.h>
#include <PID_v1.h>
#include <stdio.h>
#include <Encoder.h>

#include <task.h>
#include "task_def.h"
#include "motor_funcs.h"
// #include "ultrasonic_funcs.h"
// #include "bumper_funcs.h"
// #include "claw_funcs.h"
// #include "wpt_funcs.h"

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

  Serial.println("\n\n\nclub_robot_nav1.ino => executing system_init()...\n");

#if ((MACHINE == MACH_AVR) || (MACHINE == MACH_ARM))
  /* AVR & ARM Teesy3.1  */
  Serial.println("...before init_motor_driver_shield");
  init_motor_driver_shield();
  clearRobotOdometerTicks();

  Serial.println("...before sysclock_init");
  sysclock_init();
  // Serial.println("...before ultrasonic-bumper-claw");
  // 	init_ultrasonic (ptr_us_R,ptr_us_L);
  // 	init_bumper();
  // 	init_claw();

  // Serial.println("...before init_temp_waypoint");
  // init_temp_waypoint(ptr_temp_wpt);

  Serial.println("...before initialzeMotorTasks()");
  initializeMotorTasks();

  Serial.println("...after initialzeMotorTasks()");

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

  Serial.println("\n... Completed system_init...\n");

#if ((MACHINE == MACH_AVR) || (MACHINE == MACH_ARM)) /* ARM is Teensy3.1 */
  Serial.println("... Starting kludge (?) delay to allow libtask sysclock to start running:\n");
  delay(1500); // uses a fully blocking - Arduino native hardware delay, sysclock not running yet - what good does this really do?
               // https://learn.adafruit.com/multi-tasking-the-arduino-part-1/ditch-the-delay
  Serial.println("... Completed kludge (?) delay to allow libtask sysclock to start running:\n");
#endif

  pid_count = 0;
  current = 0;

  Serial.println("... Starting libtask Tasks:\n");

  //CAUTION: When no sensors are installed, the ultrasonic sensor hits a "fail_cntr_bump > fail_limit_bump" threshold, and sets stop_movement_flg |= (0x04) => shuts down movement...
  //  Serial.println("...before create_task 'SENSORS'");
  //	create_task("SENSORS",sensors,20,MINSTACK * 2);  //40

  // Claw doesn't hang the system, but we'll disable it as there's no need to complicate things
  //   Serial.println("...before create_task 'CLAW'");
  // create_task("CLAW",claw,45,MINSTACK);        //45

  //    Serial.println("...before create_task 'MOVE'");
  //	create_task("MOVE",move,10,MINSTACK*2);        //25 //20

  // move forward 3 seconds. wait 3 seconds. move backwards 3 seconds
  // Serial.println("...before create_task 'motorTest'");
  // create_task("motorTest",motorTest,10,MINSTACK*2);        //25 //20

  //   Serial.println("... time to try motorTasks.cpp, launching open loop motor test -> task testMotorTasks");
  // int testMotorTasks_ProcessID = -1;
  // Serial.println("... motorTasks.cpp -> launching task testMotorTasks");
  // testMotorTasks_ProcessID = create_task("testMotorTasks", testMotorTasks, 10, MINSTACK * 2);
  // Serial.print("... testMotorTasks_ProcessID is ");
  // Serial.println(testMotorTasks_ProcessID);
  // if (testMotorTasks_ProcessID == -1)
  // {
  //   Serial.println("... motorTasks.cpp -> OPPS -> error in create_task(testMotorTasks)");
  // }
  // create_task("printTaskStats", printTaskStats, testMotorTasks_ProcessID, MINSTACK);

  int measureMinMaxMotorSpeeds_ProcessID = -1;
  Serial.println("... motorTasks.cpp -> launching task measureMinMaxMotorSpeeds");
  measureMinMaxMotorSpeeds_ProcessID = create_task("measureMinMaxMotorSpeeds", measureMinMaxMotorSpeeds, 10, MINSTACK * 20);
  Serial.print("... measureMinMaxMotorSpeeds_ProcessID is ");
  Serial.println(measureMinMaxMotorSpeeds_ProcessID);
    if (measureMinMaxMotorSpeeds_ProcessID == -1)
  {
    Serial.println("... motorTasks.cpp -> OPPS -> error in create_task(measureMinMaxMotorSpeeds)");
  }
  create_task("printTaskStats", printTaskStats, measureMinMaxMotorSpeeds_ProcessID, MINSTACK);

  // => seems used only by logging for libtask?
  // Serial.println("...before create_task 'IDLE'");
  // create_task("IDLE",cpu_idle,0,MINSTACK);            // apparently needed by libtask? Left as-is in task_def.cpp

  //create_task("STATS",stats_task,10000,MINSTACK*4);
  //create_task("SIGNON",signon,1,MINSTACK*4);

  Serial.println("\nLaunching scheduler...\n\n");
  scheduler();
  Serial.println("\n...after launch scheduler - should never get here...\n");

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
