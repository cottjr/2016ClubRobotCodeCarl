/******************************************************************
*
*  motorTasks.h
*  Carl Ott
*  tasks to manage and control robot drive motors with different closed loop tasks and methods
*
******************************************************************/

// start pragma alternative: ensure the C pre-processor only reads this file once
#ifndef motorTasks_H
#define motorTasks_H

// ToDo - decide whether to keep & use this
// #define VERSION "BMARK-2.1.3 (clubRobot-0.0.1)"

#include <arduino.h>
#include <task.h>
#include <log.h>
#include <sysclock.h>

#include "commonTypes.h"
#include "motor_funcs.h"      // provides access to motor motor drivers.  needs also helpful_defines.h     // ToDo, add #include helpful_defines.h to motor_funcs.h due to it's obvious dependency?
#include "nav_funcs.h"        // provides access encoder functions
#include "helpful_defines.h"  // common defines used also in motor_funcs.h, like R_MTR


// ToDo - decide whether to keep & use this
// need / used by libtask logging functions?
#if (MACHINE == MACH_AVR) /* Mega2560, Mega328 Teensy-LC */
#define PRINTF Serial3.println
#define SPRINTF sprintf
#endif


typedef struct encoderMeasurementsStruct
{
    unsigned long msTimestamp;    // timestamp in milliseconds
    int encoderCountRight;        // raw encoder count
    int encoderCountLeft;         // raw encoder count
    unsigned long msDeltaToPrior; // ms difference from this sample to prior
    int encoderDeltaToPriorRight; // encoder difference from this sample to prior
    int encoderDeltaToPriorLeft;  // encoder difference from this sample to prior
};

// class MotorLoops         // initial failed attempt to do this as an elegant class...
// {

//   public:
void initializeMotorTasks();
// encoderValues getEncoders(); // ToDo - define array [ left, right ]  // returns current absolute encoder values
bool velocityLoopStart();
bool velocityLoopStop();
bool setMotorVelocity(signed char, signed char); // turnVelocity, throttle.
                                                 // -100 to +100.
                                                 //  ToDo - need to dump this

void measureMinMaxMotorSpeeds(ASIZE dummyPlaceholder);
void testMotorTasks(ASIZE);

//  ToDo - need to dump these
//    void motorTest(ASIZE);
//    void move(ASIZE);

// private:
//bool velocityLoopEnabled;
//bool positionLoopEnabled;

// clamp input whatValue to +/- limitingValue
signed char clamp(signed char, signed char);

// }; // end class motor

// end pragma alternative: ensure the C pre-processor only reads this file once
#endif
