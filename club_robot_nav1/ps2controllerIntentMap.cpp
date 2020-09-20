//ps2ontrollerIntentMap.cpp
#include "ps2controllerIntentMap.h"

// #include "LCDsparkfun10862-RGB16x2.h"

#include <arduino.h> // needed to support PS2 library use of alias 'byte'

#include <PS2X_lib.h> // using locally modified library derived from v1.6 -> LynxMotion PS2 V4 controller https://github.com/Lynxmotion/Arduino-PS2X
                      // modifications made to improve initial controller registration
                      // use personal version commit df8d1ad /  df8d1addefaa6f03ffd6f949fb76c024cd97bfa5 8 March 2020 or newer
                      // Why? be CAREFUL!!!  the original PS2X Library uses Arduino delay() -> which means it apparently depends on Timer0 for the Arduino Mega!
                      // -> this updated version uses delayMicroseconds(), because delayMicroseconds() works on processor cycles  https://forum.arduino.cc/index.php?topic=437411.0

/*
  note, this PS2 library likely does NOT support hot pluggable controllers, meaning 
  you must always either restart your Arduino after you conect the controller, 
  or call config_gamepad(pins) again after connecting the controller.
*/

// ps2controllerIntentMap class concept extended from PS2X_controller.cpp in 2016ClubRobotCodeCarl circa commit fa3ae25

//PS2 controller pinmap
// hardwire pin assignment in this class for MoebiusTech motor driver board for Arduino Mega 2560
// schematic and reference code at https://github.com/MoebiusTech/MecanumRobot-ArduinoMega2560
// driver board was recently found on AliExpress - called "4 Channel Motor 9 Channel Servo Shield Driver Board for Arduino MEGA2560 Mecanum Wheel Smart Robot Arm Car Expansion Board"
#define PS2_DAT 52
#define PS2_CMD 51
#define PS2_SEL 53
#define PS2_CLK 50
#define pressures true
#define rumble true

// // ToDo- Refactor the PS2X library so that it behaves well as part of a proper class.
// //  for now, move instantiation to global scope outside of the class, since that seems to work much more consistantly
// create a single instance of the PS2 Controller Class via PS2X_lib.h
PS2X ps2x_cottjr;

// // create a single instance of this ps2controllerIntentMap
// ps2controllerIntentMap operatorIntentMap;

// Constructor
//  use a default constructor, automatically invoked when an instance is declared
//  Note that only a single instance of this class should be constructed
ps2controllerIntentMap::ps2controllerIntentMap()
{
    // // create a single instance of the PS2 Controller Class via PS2X_lib.h
    // PS2X ps2x;

    // ps2controllerIntentMap::initPS2xController();
}

void ps2controllerIntentMap::initPS2xControllerValues()
{
    joystickLeftX = 127;  // middle
    joystickLeftY = 127;  // middle
    joystickRightX = 127; // middle
    joystickRightY = 127; // middle

    selectButtonState = false;
    startButtonState = false;

    upButtonState = false;
    rightButtonState = false;
    downButtonState = false;
    leftButtonState = false;

    triangleButtonState = false;
    circleButtonState = false;
    xButtonState = false;
    squareButtonState = false;

    L1button = false;
    L2button = false;
    L3button = false;

    R1button = false;
    R2button = false;
    R3button = false;
}

// Initialization
//  (re)initialize this instance of the PS2 controller class
void ps2controllerIntentMap::initPS2xController()
{
    // // // ToDo- Refactor the PS2X library so that it behaves well as part of a proper class.
    // // //  for now, move instantiation to global scope outside of the class, since that seems to work much more consistantly
    // // create a single instance of the PS2 Controller Class via PS2X_lib.h
    // PS2X ps2x;

    // set null controller values -> give clean initial values in case no controller is detected
    int ps2xError = -1; // keeps track of any errors from most recent PS2 controller initialization attempt. Default to unknown error.
    PS2controllerType = -1;
    vibrateShake = 0;

    ps2ControllerUseable = false;

    ps2controllerIntentMap::initPS2xControllerValues();

    Serial.println("trying to initialize gamepad");
    delayMicroseconds(500000); // delay() causes issues with the serial port, and relies on Mega Timer0.  delayMicroseconds() works on processor cycles  https://forum.arduino.cc/index.php?topic=437411.0
    // // initialize  pins:  GamePad(clock, command, attention, data, Pressures, Rumble)
    ps2xError = ps2x_cottjr.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);

    Serial.println();
    switch (ps2xError)
    {
    case 0:
        ps2ControllerUseable = true;
        Serial.println("ps2Error 0: all good- Found Controller- PS2 init success");
        // backlight8ColorDigital(0, 0, 255);
        // write16x2toLCD("PS2 cntrl OK (0)", "   Ready to Go  ");
        // delay(2500);
        break;
    case 1:
        ps2ControllerUseable = false;
        Serial.println("ps2Error 1: very bad- No controller found, check wiring, readme.txt-> enable PS2 debug");
        // backlight8ColorDigital(255, 0, 0);
        // write16x2toLCD("PS2 cntrl BAD: 1", "Init Fail  :-(  ");
        // delay(2500);
        break;
    case 2:
        ps2ControllerUseable = false;
        Serial.println("ps2Error 2: very bad- Controller found but not accepting commands. readme.txt-> enable PS2 debug");
        // backlight8ColorDigital(255, 0, 0);
        // write16x2toLCD("PS2 cntrl BAD: 2", "Init Fail  :-(  ");
        // delay(2500);
        break;
    case 3:
        ps2ControllerUseable = true;
        Serial.println("ps2Error 3: partly ok- Controller refusing Pressures mode- may not support it");
        // backlight8ColorDigital(0, 0, 255);
        // write16x2toLCD("PS2 cntrl OK (3)", "   Ready to Go  ");
        // delay(2500);
        break;
    default:
        ps2ControllerUseable = false;
        Serial.println("ps2Error unknown: very bad- unexpected PS2 init result");
        // backlight8ColorDigital(255, 0, 0);
        // write16x2toLCD("PS2 cntrl BAD: ?", "Init Fail  :-(  ");
        // delay(2500);
    }

    PS2controllerType = ps2x_cottjr.readType();
    switch (PS2controllerType)
    {
    case 0:
        Serial.println("PS2 controller: Unknown type result 0");
        break;
    case 1:
        Serial.println("PS2 controller: DualShock found");
        break;
    case 2:
        Serial.println("PS2 controller: GuitarHero found");
        break;
    default:
        Serial.println("PS2 controller: unknown type result other");
    }

    Serial.println();
}

// ToDo: Refactor from +/-100 space to full space of ~ +/-128
// Purpose: map left to right joystick values to a TurnVelocity setpoint
// Input:  accepts values from PS2 controller V4 by Lynxmotion
// Algorithm: see "Joystick to Velocity Loop Setpoints" sketch 2019 oct 6 (2016ClubRobotCarl)
// Returns: TurnVelocity values between -100 and +100
int ps2controllerIntentMap::joystickToTurnVelocity(unsigned char joystick)
{
    if (joystick < 126)
    {
        return (int)((double)0.792 * (double)joystick - (double)100);
    }
    if (joystick > 130)
    {
        return (int)((double)0.798 * (double)joystick - (double)103.5);
    }
    return (int)0;
};

// ToDo: Refactor from +/-100 space to full space of ~ +/-128
// Purpose: map front to back or sideways joystick values a translation Throttle setpoint
// Input:  accepts values from PS2 controller V4 by Lynxmotion
// Algorithm: see "Joystick to Velocity Loop Setpoints" sketch 2019 oct 6 (2016ClubRobotCarl)
// Returns: Throttle values between -100 and +100
int ps2controllerIntentMap::joystickToThrottle(unsigned char joystick)
{
    if (joystick < 126)
    {
        return (int)((double)-0.792 * (double)joystick + (double)100);
    }
    if (joystick > 130)
    {
        return (int)((double)-0.798 * (double)joystick + (double)103.5);
    }
    return (int)0;
};

// Poll and sture current button states
bool ps2controllerIntentMap::readAllPS2xControllerValues(bool showValues)
{
    // initialize all values and bail from this function if it's clearly not possible to read values from this controller
    if (ps2ControllerUseable == false)
    {
        ps2controllerIntentMap::initPS2xControllerValues();
        Serial.println(">>>>>>>  failed to read PS2 values- controller failed initialization");
        return false;
    }

    // else assume is a DualShock Controller
    // => read controller and set large motor to spin at 'vibrateShake' speed

    // Poll PS2 Controller to get new values and set vibration level
    // if rumble is disabled, use ps2x_cottjr.read_gamepad(); with no values
    // you should query the PS2 controller at least once a second, else the library will attempt to automatically reconfigure the control
    // Initialize all values & bail from this function if the attempted gamepad read appeared to fail
    if (!ps2x_cottjr.read_gamepad(false, vibrateShake))
    {
        // NOTE: this branch was intended to catch scenarios like user switching off controller
        //  however, the PS2X_lib library apparently does NOT trap that as an error
        //  hence, this branch does NOT trap loss of controller for PS2X_lib commit 7ab2fa0, 8 March 2020
        // This means that a self-driving routine will continue even if hte user turns off the PS2 controller after starting the self-driving routine
        ps2ControllerUseable = false;
        ps2controllerIntentMap::initPS2xControllerValues();
        Serial.println(">>>>>>>  psx2.read_gamepad- read attempt failed");
        return false;
    }

    joystickLeftX = ps2x_cottjr.Analog(PSS_LX);
    joystickLeftY = ps2x_cottjr.Analog(PSS_LY);
    joystickRightX = ps2x_cottjr.Analog(PSS_RX);
    joystickRightY = ps2x_cottjr.Analog(PSS_RY);

    // Buttons currently being pressed
    selectButtonState = ps2x_cottjr.Button(PSB_SELECT);
    startButtonState = ps2x_cottjr.Button(PSB_START);
    // note: it appears that the mode Button is not exposed

    upButtonState = ps2x_cottjr.Button(PSB_PAD_UP);       // UP is currently being pressed
    rightButtonState = ps2x_cottjr.Button(PSB_PAD_RIGHT); // Square is currently being pressed
    downButtonState = ps2x_cottjr.Button(PSB_PAD_DOWN);   // Down is currently being pressed
    leftButtonState = ps2x_cottjr.Button(PSB_PAD_LEFT);   // Circle is currently being pressed

    triangleButtonState = ps2x_cottjr.Button(PSB_GREEN); // UP is currently being pressed
    circleButtonState = ps2x_cottjr.Button(PSB_RED);     // Circle is currently being pressed
    xButtonState = ps2x_cottjr.Button(PSB_BLUE);         // Down is currently being pressed
    squareButtonState = ps2x_cottjr.Button(PSB_PINK);    // Square is currently being pressed

    L1button = ps2x_cottjr.Button(PSB_L1);
    L2button = ps2x_cottjr.Button(PSB_L2);
    L3button = ps2x_cottjr.Button(PSB_L3); // left joystick press

    R1button = ps2x_cottjr.Button(PSB_R1);
    R2button = ps2x_cottjr.Button(PSB_R2);
    R3button = ps2x_cottjr.Button(PSB_R3); // right joystick press

    // Buttons just pressed (ie. pressed since last time were sampled)
    selectButtonPressed = ps2x_cottjr.ButtonPressed(PSB_SELECT);
    startButtonPressed = ps2x_cottjr.ButtonPressed(PSB_START);
    // note: it appears that the mode Button is not exposed

    upButtonPressed = ps2x_cottjr.ButtonPressed(PSB_PAD_UP);       // UP is currently being pressed
    rightButtonPressed = ps2x_cottjr.ButtonPressed(PSB_PAD_RIGHT); // Square is currently being pressed
    downButtonPressed = ps2x_cottjr.ButtonPressed(PSB_PAD_DOWN);   // Down is currently being pressed
    leftButtonPressed = ps2x_cottjr.ButtonPressed(PSB_PAD_LEFT);   // Circle is currently being pressed

    triangleButtonPressed = ps2x_cottjr.ButtonPressed(PSB_GREEN); // UP is currently being pressed
    circleButtonPressed = ps2x_cottjr.ButtonPressed(PSB_RED);     // Circle is currently being pressed
    xButtonPressed = ps2x_cottjr.ButtonPressed(PSB_BLUE);         // Down is currently being pressed
    squareButtonPressed = ps2x_cottjr.ButtonPressed(PSB_PINK);    // Square is currently being pressed

    L1buttonPressed = ps2x_cottjr.ButtonPressed(PSB_L1);
    L2buttonPressed = ps2x_cottjr.ButtonPressed(PSB_L2);
    L3buttonPressed = ps2x_cottjr.ButtonPressed(PSB_L3); // left joystick press

    R1buttonPressed = ps2x_cottjr.ButtonPressed(PSB_R1);
    R2buttonPressed = ps2x_cottjr.ButtonPressed(PSB_R2);
    R3buttonPressed = ps2x_cottjr.ButtonPressed(PSB_R3); // right joystick press

    // Compute derived values

    int selectedTurnVelocity;
    int selectedForward;
    int selectedSideways;

    if (L1button || R1button)
    {
        // invoke max speed "turbo mode" when press L1 or R1 buttons
        selectedTurnVelocity = joystickToTurnVelocity(joystickRightX);
        selectedForward = joystickToThrottle(joystickLeftY);
        selectedSideways = -1 * joystickToThrottle(joystickLeftX);

        // override joystick commands
        if (circleButtonState && !startButtonState)
        {
            selectedTurnVelocity = +40; // slight steer to the right
        }
        if (squareButtonState && !startButtonState)
        {
            selectedTurnVelocity = -40; //  slight steer to the left
        }

        // override joystick commands
        // except don't move forward via Triangle button if the start button is also pressed at the same time as Triangle indicating to start Quick Trip
        if (upButtonState && !startButtonState)
        {
            selectedForward = +40; // slide forwards
        }
        if ((downButtonState || xButtonState) && !startButtonState)
        {
            selectedForward = -40; // slight backwards
        }

        // override joystick commands
        if (rightButtonState && !startButtonState)
        {
            selectedSideways = +40; // slight slide to right
        }
        if (leftButtonState && !startButtonState)
        {
            selectedSideways = -40; // slight slide to left
        }
    }
    else
    {
        // just go at some nominal fraction of max speed
        selectedTurnVelocity = (int)((double)joystickToTurnVelocity(joystickRightX) / (double)2);
        selectedForward = (int)((double)joystickToThrottle(joystickLeftY) / (double)2);
        selectedSideways = -1 * (int)((double)joystickToThrottle(joystickLeftX) / (double)2);

        // override joystick commands
        if (circleButtonState && !startButtonState)
        {
            selectedTurnVelocity = +20; // slight steer to the right
        }
        if (squareButtonState && !startButtonState)
        {
            selectedTurnVelocity = -20; //  slight steer to the left
        }

        // override joystick commands
        // except don't move forward via Triangle button if the start button is also pressed at the same time as Triangle indicating to start Quick Trip
        if ((upButtonState || triangleButtonState) && !startButtonState)
        {
            selectedForward = +20; // slide forwards
        }
        if ((downButtonState || xButtonState) && !startButtonState)
        {
            selectedForward = -20; // slight backwards
        }

        // override joystick commands
        if (rightButtonState && !startButtonState)
        {
            selectedSideways = +20; // slight slide to right
        }
        if (leftButtonState && !startButtonState)
        {
            selectedSideways = -20; // slight slide to left
        }
    }

    forward = selectedForward;
    sideways = selectedSideways;
    turn = -1 * selectedTurnVelocity;


    return true; // assume could successfully read all values

    // Reference code from PS2 library
    //  hints at rich variety of useable control state events, and reading additional values such as pressure for certain switches

    // // this will set the large motor vibrateShake speed based on how hard you press the blue (X) button
    // // read through the PS2 libarary code to observe that many buttons are apparently pressure sensitive
    // vibrateShake = ps2x_cottjr.Analog(PSAB_BLUE);

    // // TRUE if button was JUST pressed OR released
    // if (ps2x_cottjr.NewButtonState(PSB_BLUE))
    //     Serial.println("X changed");

    // // TRUE if any button changes state (on to off, or off to on)
    // if (ps2x_cottjr.NewButtonState())
    // {
    //     if (ps2x_cottjr.Button(PSB_L3))
    //         Serial.println("L3");
    //     if (ps2x_cottjr.Button(PSB_R3))
    //         Serial.println("R3");
    //     if (ps2x_cottjr.Button(PSB_L2))
    //         Serial.println("L2");
    //     if (ps2x_cottjr.Button(PSB_R2))
    //         Serial.println("R2");
    //     if (ps2x_cottjr.Button(PSB_GREEN))
    //         Serial.println("Triangle pressed");
    // }

    // // TRUE if button was JUST pressed
    // if (ps2x_cottjr.ButtonPressed(PSB_RED))
    //     Serial.println("Circle pressed");

    // // TRUE if button was JUST released
    // if (ps2x_cottjr.ButtonReleased(PSB_PINK))
    //     Serial.println("Square released");

    // // print joystick values if either is TRUE
    // if (ps2x_cottjr.Button(PSB_L1) || ps2x_cottjr.Button(PSB_R1))
    // {
    //     Serial.print("Lx ");
    //     // Left stick, Y axis. Other options: LX, RY, RX
    //     Serial.print(ps2x_cottjr.Analog(PSS_LX), DEC);
    //     Serial.print(" y ");
    //     Serial.print(ps2x_cottjr.Analog(PSS_LY), DEC);

    //     Serial.print(" Rx ");
    //     Serial.print(ps2x_cottjr.Analog(PSS_RX), DEC);
    //     Serial.print(" y ");
    //     Serial.println(ps2x_cottjr.Analog(PSS_RY), DEC);
    // }
}
