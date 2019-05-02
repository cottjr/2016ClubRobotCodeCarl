// start pragma alternative: ensure the C pre-processor only reads this file once
#ifndef libtaskMemoryTest_H
#define libtaskMemoryTest_H


#include <arduino.h>
#include <task.h>
#include <log.h>
#include <sysclock.h>

// ToDo - decide whether to keep & use this
// need / used by libtask logging functions?
#if (MACHINE == MACH_AVR) /* Mega2560, Mega328 Teensy-LC */
#define PRINTF Serial3.println
#define SPRINTF sprintf
#endif



extern TSIZE idleCPUcountPerSec;  // crude count of CPU idle cycles available for use
void monitorCPUidle(ASIZE ignored);

void printTaskStatsByTaskPointer(TASK *t, int dummyArgumentPlaceholder);

void printFreeBytesOfRAM();

void monitorResourcesForAllTasks(ASIZE msLoopPeriod);


// end pragma alternative: ensure the C pre-processor only reads this file once
#endif
