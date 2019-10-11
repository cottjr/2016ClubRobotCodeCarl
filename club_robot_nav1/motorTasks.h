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
// #include <task.h>
// #include <log.h>
// #include <sysclock.h>

#include "motor_funcs.h"    // provides access to motor motor drivers.  needs also helpful_defines.h     // ToDo, add #include helpful_defines.h to motor_funcs.h due to it's obvious dependency?
                            // Interface layer to Monster Motor Shield
#include <PID_v1.h>
#include "nav_funcs.h"        // provides access encoder functions
#include "helpful_defines.h"  // common defines used also in motor_funcs.h, like R_MTR


// ToDo - decide whether to keep & use this
// need / used by libtask logging functions?
// #if (MACHINE == MACH_AVR) /* Mega2560, Mega328 Teensy-LC */
// #define PRINTF Serial3.println
// #define SPRINTF sprintf
// #endif


typedef struct encoderMeasurementsStruct
{
    unsigned long msTimestamp;    // timestamp in milliseconds
    int encoderCountRight;        // raw encoder count
    int encoderCountLeft;         // raw encoder count
    unsigned long msDeltaToPrior; // ms difference from this sample to prior
    int encoderDeltaToPriorRight; // encoder difference from this sample to prior
    int encoderDeltaToPriorLeft;  // encoder difference from this sample to prior
};

// extern int periodicSampleMotorShield_ProcessID;
// extern int monitorVelocityLoop_ProcessID;

void printRobotOdometerTicks();
void printVelocityLoopValues();

// void monitorVelocityLoop(ASIZE msLoopPeriod);
void initializeMotorTasks();

void filterTurnAndThrottleRequestValues();
// bool updateVelocityLoopSetpoints(bool printNewSettings);
void sampleMotorShield();
// void periodicSampleMotorShield(ASIZE msLoopPeriod);

extern int msOfPriorPID, msOfCurrentPID, msBetweenPID, msExecutePID; // track velocity PID loop execution timing and periodicity...

// clamp input whatValue to +/- limitingValue
signed char clamp(signed char, signed char);

void periodicSampleMotorShield_Start();
void periodicSampleMotorShield_Stop();
bool setMotorVelocityByPWM(signed char, signed char); // turnVelocity, throttle.
                                                 // -100 to +100.
                                                 //  ToDo - need to dump this

signed char joystickToTurnVelocity(unsigned char); // map joystick to turn velocity setpoint space values
signed char joystickToThrottle(unsigned char);      // map joystick to throttle setpoint space values

bool setManualVelocityLoopSetpoints(signed char TurnVelocity, signed char Throttle, bool printNewSettings);
bool setAutomaticVelocityLoopSetpoints(signed char TurnVelocity, signed char Throttle, bool printNewSettings);
extern int leftLoopPWM;        // used internally by sendVelocityLoopPWMtoMotorShield(), shared globall for diagnostics
extern int rightLoopPWM;       // used internally by sendVelocityLoopPWMtoMotorShield(), shared globall for diagnostics
bool sendVelocityLoopPWMtoMotorShield();

// void testVelocityPIDloop(ASIZE dummyArgumentPlaceholder);
void testVelocityLoopSetpointsMath();
// void measureMinMaxMotorSpeeds(ASIZE dummyPlaceholder);
// void testMotorTasks(ASIZE dummyArgumentPlaceholder);





// end pragma alternative: ensure the C pre-processor only reads this file once
#endif
