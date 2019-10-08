/******************************************************************
*
*  PS2X_controller.h
*  Carl Ott
*  interface to PS2X controller from Lynxmotion
*
******************************************************************/

// start pragma alternative: ensure the C pre-processor only reads this file once
#ifndef PS2X_controller_H
#define PS2X_controller_H

#include "PS2X_controller.h"

typedef struct PS2JoystickValuesType
{
    int leftX;  // 0 full left, 255 full right, 127/128 in the middle
    int leftY;  // 0 full forward, 255 full backwards, 127/128 in the middle
    int rightX; // 0 full left, 255 full right, 127/128 in the middle
    int rightY; // 0 full forward, 255 full backwards, 127/128 in the middle
};

void initPS2xController();
void readPS2Joysticks( PS2JoystickValuesType* );
void readAllPS2xControllerValues();

extern bool startAndTriangle; // true while start button and triangle are both held at the same time

// end pragma alternative: ensure the C pre-processor only reads this file once
#endif
