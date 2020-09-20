//ps2controllerIntentMap.h
// Class maps user behavior from a PS2 controller to abstract robot command intents
#ifndef ps2controllerIntentMap_H
#define ps2controllerIntentMap_H

#include <arduino.h> // needed to support PS2 library use of alias 'byte'

#include <PS2X_lib.h> // using locally modified library derived from v1.6 -> LynxMotion PS2 V4 controller https://github.com/Lynxmotion/Arduino-PS2X
                      // modifications made to improve initial controller registration

// typedef struct robotCommandIntent
// {
//     bool start;
//     bool stopOrCancel;
//     bool selectModeOrRoutine;
//     bool accept;
//     signed int turn;
//     signed int forward;
//     signed int sideways;
// };

class ps2controllerIntentMap
{
public:
    // Default Constructor
    //  Note that only a single instance of this class should be constructed
    ps2controllerIntentMap();

    void initPS2xControllerValues();

    void initPS2xController();

    // read all basic values (but not every option e.g. 'just pressed')
    //  and compute derived values
    // simply always keep most recent values available to be used whenever needed
    bool readAllPS2xControllerValues(bool showValues);

    //temporarily expose these values to external functions, while refactoring MoebiusTech functions into better classes

    bool ps2ControllerUseable = false; // flag to determine if appears safe to read values from the ps2 controller

    // Raw values from PS2X controller
    unsigned char joystickLeftX = 127;  // 0 full left, 255 full right, 127/128 in the middle
    unsigned char joystickLeftY = 127;  // 0 full forward, 255 full backwards, 127/128 in the middle
    unsigned char joystickRightX = 127; // 0 full left, 255 full right, 127/128 in the middle
    unsigned char joystickRightY = 127; // 0 full forward, 255 full backwards, 127/128 in the middle

    
    // current state of button (true === pressed)
    bool selectButtonState = false;
    bool startButtonState = false;
    // note: it appears that the mode Button is not exposed

    bool L1button = false;
    bool L2button = false;
    bool L3button = false;

    bool R1button = false;
    bool R2button = false;
    bool R3button = false;

    bool upButtonState = false;
    bool rightButtonState = false;
    bool downButtonState = false;
    bool leftButtonState = false;

    bool triangleButtonState = false;
    bool circleButtonState = false;
    bool xButtonState = false;
    bool squareButtonState = false;

    // Buttons just pressed (ie. pressed since last time were sampled)
    bool selectButtonPressed = false;
    bool startButtonPressed = false;
    // note: it appears that the mode Button is not exposed

    bool upButtonPressed = false;
    bool rightButtonPressed = false;
    bool downButtonPressed = false;
    bool leftButtonPressed = false;

    bool triangleButtonPressed = false;
    bool circleButtonPressed = false;
    bool xButtonPressed = false;
    bool squareButtonPressed = false;

    bool L1buttonPressed = false;
    bool L2buttonPressed = false;
    bool L3buttonPressed = false; // left joystick button just pressed

    bool R1buttonPressed = false;
    bool R2buttonPressed = false;
    bool R3buttonPressed = false; // right joystick button just pressed

    //ToDo -> fix scale from +/- 100 to +/- 255
    // Derived values from PS2X controller

    //  these are intended to deduce operator intent assuming certain modes of operation
    // 0, stop (no motion)
    // 1 to 255, forward translation rate when looking down on platform towards floor
    // -1 to -255, backwards translation rate when looking down on platform towards floor
    int forward;

    //ToDo -> fix scale from +/- 100 to +/- 255
    // 1 to 255, translation rate towards right when looking down on platform towards floor
    // -1 to -255, translation rate towards left when looking down on platform towards floor
    int sideways;

    //ToDo -> fix scale from +/- 100 to +/- 255
    // 1 to 255, CW rotation rate when looking down on platform towards floor
    // -1 to -255, CCW rotation rate when looking down on platform towards floor
    int turn;

    // Purpose: map left to right joystick values to a TurnVelocity setpoint
    int joystickToTurnVelocity(unsigned char joystick);

    // Purpose: map front to back or sideways joystick values a translation Throttle setpoint
    int joystickToThrottle(unsigned char joystick);


private:
    // // ToDo- Refactor the PS2X library so that it behaves well as part of a proper class.
    // //  for now, move instantiation to global scope outside of the class, since that seems to work much more consistantly
    // // create a single instance of the PS2 Controller Class via PS2X_lib.h
    // PS2X ps2x;

    int ps2xError = -1;          // keeps track of any errors from most recent PS2 controller initialization attempt. Default to unknown error.
    byte PS2controllerType = -1; // keeps track of detected PS2 controller type from most recent initalization attempt. Default to unknown type.
    byte vibrateShake = 0;       // value indicating amount to vibrate controller

};

#endif
