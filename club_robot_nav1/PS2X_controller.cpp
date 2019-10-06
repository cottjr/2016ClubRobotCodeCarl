#include "PS2X_controller.h"
#include <PS2X_lib.h>  //for v1.6 -> LynxMotion PS2 V4 controller https://github.com/Lynxmotion/Arduino-PS2X


/*
  right now, the library does NOT support hot pluggable controllers, meaning 
  you must always either restart your Arduino after you conect the controller, 
  or call config_gamepad(pins) again after connecting the controller.
*/

// create PS2 Controller Class
PS2X ps2x;
int ps2xError = 0; 
byte PS2controllerType = 0;
byte vibrateShake = 0;

void initPS2xController()
{
  // Serial.begin(250000);
  // CHANGES for v1.6 HERE!!! **************PAY ATTENTION*************
  // setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for ps2xError
  ps2xError = ps2x.config_gamepad(22,23,24,25, true, true);

  if(ps2xError == 0)
  {
    Serial.println();
    Serial.println("ps2xError 0- all good- Found Controller- init success");
    // Serial.println("Try out buttons:");
    // Serial.println("X shakes faster as you press harder");
    // Serial.println("hold L1 or R1 to view analog stick values");
    // Serial.println("www.billporter.info- troubleshooting tips/ updates/ bugs");
    // Serial.println();
  }
  else if(ps2xError == 1)
  {
    Serial.println("ps2xError 1- very bad- No controller found, check wiring, readme.txt-> enable debug");
    Serial.println();
  }
  else if(ps2xError == 2)
  {
    Serial.println("ps2xError 2- very bad- Controller found but not accepting commands. readme.txt-> enable debug");
    Serial.println();
  }
  else if(ps2xError == 3)
  {
    Serial.println("ps2xError 3- partly ok- Controller refusing Pressures mode- may not support it");
    // Serial.println();
  }

  // Serial.print(ps2x.Analog(1), HEX);
  PS2controllerType = ps2x.readType();
  switch(PS2controllerType)
  {
    case 0:
      Serial.println("Unknown Controller type found");
      Serial.println();
      break;
    case 1:
      Serial.println("DualShock Controller found");
      Serial.println();
      break;
    case 2:
      Serial.println("GuitarHero Controller found");
      Serial.println();
      break;
  }
}

void readPS2Joysticks( PS2JoystickValuesType* joystickValues ){
      joystickValues->leftX = ps2x.Analog(PSS_LX);
      joystickValues->leftY = ps2x.Analog(PSS_LY);
      joystickValues->rightX = ps2x.Analog(PSS_RX);
      joystickValues->rightY = ps2x.Analog(PSS_RY);
}

void readAllPS2xControllerValues()
{
  /*
    You must Read Gamepad to get new values
    Read GamePad and set vibration values
    ps2x.read_gamepad(small motor on/off, larger motor strenght from 0-255)
    if you don't enable the rumble, use ps2x.read_gamepad(); with no values
    you should call this at least once a second
  */
  // skip loop if no controller found
  if(ps2xError == 1)
    return; 
  
  if(PS2controllerType == 2)
  {
    // Guitar Hero Controller
    // read controller
    ps2x.read_gamepad();
    
    if(ps2x.ButtonPressed(GREEN_FRET))
      Serial.println("Green Fret");
    if(ps2x.ButtonPressed(RED_FRET))
      Serial.println("Red Fret");
    if(ps2x.ButtonPressed(YELLOW_FRET))
      Serial.println("Yellow Fret");
    if(ps2x.ButtonPressed(BLUE_FRET))
      Serial.println("Blue Fret");
    if(ps2x.ButtonPressed(ORANGE_FRET))
      Serial.println("Orange Fret");
    
    if(ps2x.ButtonPressed(STAR_POWER))
      Serial.println("Star Power");
    
    // will be TRUE as long as button is pressed
    if(ps2x.Button(UP_STRUM))
      Serial.println("Up Strum");
    if(ps2x.Button(DOWN_STRUM))
      Serial.println("DOWN Strum");
    
    // will be TRUE as long as button is pressed
    if(ps2x.Button(PSB_START))
      Serial.println("Start being held");
    if(ps2x.Button(PSB_SELECT))
      Serial.println("Select being held");
    
    // print stick value IF TRUE
    if(ps2x.Button(ORANGE_FRET))
    {
      Serial.print("Wammy Bar Position:");
      Serial.println(ps2x.Analog(WHAMMY_BAR), DEC); 
    } 
  }
  else
  {
    // DualShock Controller
    // read controller and set large motor to spin at 'vibrateShake' speed
    ps2x.read_gamepad(false, vibrateShake);
    
    // will be TRUE as long as button is pressed
    if(ps2x.Button(PSB_START))
      Serial.println("Start being held");
    if(ps2x.Button(PSB_SELECT))
      Serial.println("Select being held");
    
    // will be TRUE as long as button is pressed
    if(ps2x.Button(PSB_PAD_UP))
    {
      Serial.print("Up held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
    }
    if(ps2x.Button(PSB_PAD_RIGHT))
    {
      Serial.print("Right held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
    }
    if(ps2x.Button(PSB_PAD_LEFT))
    {
      Serial.print("LEFT held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
    }
    if(ps2x.Button(PSB_PAD_DOWN))
    {
      Serial.print("DOWN held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
    }
    
    // this will set the large motor vibrateShake speed based on how hard you press the blue (X) button
    vibrateShake = ps2x.Analog(PSAB_BLUE);
    
    // will be TRUE if any button changes state (on to off, or off to on)
    if (ps2x.NewButtonState())
    {
      if(ps2x.Button(PSB_L3))
        Serial.println("L3");
      if(ps2x.Button(PSB_R3))
        Serial.println("R3");
      if(ps2x.Button(PSB_L2))
        Serial.println("L2");
      if(ps2x.Button(PSB_R2))
        Serial.println("R2");
      if(ps2x.Button(PSB_GREEN))
        Serial.println("Triangle pressed");
    }
    
    // will be TRUE if button was JUST pressed
    if(ps2x.ButtonPressed(PSB_RED))
    Serial.println("Circle pressed");
    
    // will be TRUE if button was JUST released
    if(ps2x.ButtonReleased(PSB_PINK))
      Serial.println("Square released");     
    
    // will be TRUE if button was JUST pressed OR released
    if(ps2x.NewButtonState(PSB_BLUE))
      Serial.println("X changed");    
    
    // print stick values if either is TRUE
    if(ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1))
    {
      Serial.print("Lx ");
      // Left stick, Y axis. Other options: LX, RY, RX  
      Serial.print(ps2x.Analog(PSS_LX), DEC); 
      Serial.print(" y ");
      Serial.print(ps2x.Analog(PSS_LY), DEC);
      
      Serial.print(" Rx ");
      Serial.print(ps2x.Analog(PSS_RX), DEC); 
      Serial.print(" y ");
      Serial.println(ps2x.Analog(PSS_RY), DEC); 
    }
  }
  // delay(50);
}
