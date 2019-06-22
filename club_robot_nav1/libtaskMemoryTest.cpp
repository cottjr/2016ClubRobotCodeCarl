
#include "libtaskMemoryTest.h"

// TSIZE idleCPUcountPerSec; // crude count of CPU idle cycles available for use
// Purpose: track CPU idle cycles available e.g. for use by additional tasks
// Algorithm:
//	- once per second
//	- trap the current value of the libtask library proc_counter
// Output: update global variable idleCPUcountPerSec with sampled proc_counter
// void monitorCPUidle(ASIZE dummyPlaceholder)
// {
//     TSIZE t;
//     t = sysclock + 1000;
//     while (1)
//     {
//         idleCPUcountPerSec = proc_counter;
//         proc_counter = 0;
//         PERIOD(&t, 1000);
//     }
// }

// Purpose: print values related to a specific task
//          this version requires that you provide pointer to a TASK structure
// void printTaskStatsByTaskPointer(TASK *t, int dummyArgumentPlaceholder)
// {
//     Serial.print("processID: ");
//     Serial.print(t->pid);
//     Serial.print(" name: ");
//     Serial.print(t->name);
//     Serial.print(" stack_usage(): ");
//     Serial.println(stack_usage(t->pid));
// }

// freeBytesOfRAM()
// Purpose: determine how much RAM is available
// Algorithm
//  - followed this guide https://jeelabs.org/2011/05/22/atmega-memory-use/?utm_source=rb-community&utm_medium=forum&utm_campaign=arduino-memory-usage
//  -- relies on 'At any point in time, there is a highest point in RAM occupied by the heap. This value can be found in a system variable called __brkval'
//  -- and uses another system variable __heap_start
//  - note additional resource
//  -- http://www.avr-developers.com/mm/memoryusage.html
//  -- https://learn.adafruit.com/memories-of-an-arduino/measuring-free-memory
//  .. https://github.com/McNeight/MemoryFree/blob/master/MemoryFree.cpp
// Output:
//  - number of unused bytes, ie. the number of bytes between the heap and the stack
void printFreeBytesOfRAM()
{
    // // followed this guide https://jeelabs.org/2011/05/22/atmega-memory-use/?utm_source=rb-community&utm_medium=forum&utm_campaign=arduino-memory-usage
    // // => doesn't seem to work on ATmega2560 with this program, always returns -173
    // extern int __heap_start, *__brkval;
    // int v;
    // return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);

    //  -- http://www.avr-developers.com/mm/memoryusage.html
    int free_memory;
    extern char __bss_end;
    extern char *__brkval;
    extern char *__heap_start;
    extern char *__heap_end;
    int	heap_end	=	(int)&free_memory - (int)&__malloc_margin;
	int	heap_size	=	heap_end - (int)&__bss_end;
	int	stack_size	=	RAMEND - (int)&free_memory + 1;
	Serial.print("\n+----------------+  __bss_end     =");	Serial.println((int)&__bss_end);
	Serial.print("+----------------+  __heap_start  =");	Serial.println((int)&__heap_start);
	Serial.print("+                +");						Serial.println();
	Serial.print("+       heap     +  heap_size     =");	Serial.println(heap_size);
	Serial.print("+                +");						Serial.println();
	Serial.print("+----------------+  heap_end      =");	Serial.println(heap_end);
	Serial.print("+----------------+  Current STACK =");	Serial.println((int)&free_memory);
	Serial.print("+                +");						Serial.println();
	Serial.print("+      stack     +  stack_size    =");	Serial.println(stack_size);

    if ((int)__brkval == 0)
        free_memory = ((int)&free_memory) - ((int)&__bss_end);
    else
        free_memory = ((int)&free_memory) - ((int)__brkval);

    Serial.print(" freeBytesOfRAM: ");
    Serial.println(free_memory);
    Serial.println();
}

// void monitorResourcesForAllTasks(ASIZE msLoopPeriod)
// {
//     TSIZE t;
//     t = sysclock + msLoopPeriod;
//     while (1)
//     {
//         Serial.print("\n\nidleCPUcountPerSec: ");
//         Serial.println(idleCPUcountPerSec);
//         // printFreeBytesOfRAM();

//         iterate_tasks(printTaskStatsByTaskPointer, 0);
//         PERIOD(&t, msLoopPeriod);
//     }
// }