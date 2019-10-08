/************************************************************
*
* motor_funcs.cpp
*  base version: 20161020-0           Doug Paradis
*  Motor/ Encoder/PID functions for DPRG Club Robot 2016.
*
*  In process of refactoring
*    -> will become motorHAL.cpp
*
*  Original Purpose
*  Motor/Encoder functions are derived from:
*    - Teensy Encoder library by pjrc
*    - MonsterMoto Shield Example Sketch by Jim Lindblom
*
*    note: 
*      enc => 3292.4 pulse/rev
*      wheel is 100mm => c = pi*dia = 31.4159cm
*      pulse/cm = 3292.4pulse/rev / 31.4259cm/rev = 104.8pulse/cm 
*      if period of measure is 10ms (i.e., 100 periods/sec)
*         speed in cm/sec = cm / 100 period * 104.8 pulse/cm = 1.048 pulse/period
*
************************************************************/

#include "motor_funcs.h"

// Legacy variables => Keep in motorHAL.cpp
Encoder encRight(2, 3); // Mega INT4 and INT5
// Encoder encLeft(21, 20); // Mega INT0 and INT1   (Standard Club Pin Assignment)
Encoder encLeft(19, 18); // Mega INT2 and INT3   (Carl Wiring Pin Assignment)

robotEncoderTicksType robotOdometerTicks;       // cumulative odometer in robot coordinates in units of encoder ticks
robotEncoderTicksType priorEncoderSample;       // prior sample of cumulative odometer in robot coordinates in units of encoder ticks
robotEncoderTicksType deltaOdometerTicks;       // most recent motor velocity between most recent encoder samples, in long ticks
robotEncoderVelocityType robotOdometerVelocity; // most recent motor velocity between most recent encoder samples in Double

// Legacy variables => keep in motorHAL.cpp
// Motor control pins
//  VNH2SP30 pin definitions
int inApin[2] = {7, 4}; // INA: Clockwise input
int inBpin[2] = {8, 9}; // INB: Counter-clockwise input
int pwmpin[2] = {5, 6}; // PWM input
int cspin[2] = {2, 3};  // CS: Current sense ANALOG input (note: CS is ~0.13 V/A)
int enpin[2] = {0, 1};  // EN: Status of switches output (Analog pin)

// Legacy Function => keep in motorHAL.cpp
// Purpose: Initializes motor shield
// Output:
//  - motor shield chips left in a proper initial state
//  - motor drive pins set as proper type
//  - motor drive pin state set to 'brake mode'
void init_motor_driver_shield()
{
  // motor shield pin setup
  // Initialize digital pins as outputs
  for (int i = 0; i < 2; i++)
  {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }
  // Initialize braked
  motorOff(L_MTR);
  motorOff(R_MTR);
}

// Legacy Function => Keep in motorHAL.cpp
// Purpose: Turn motor off
// Output:
//  - set motor drive pins to 'brake mode'
void motorOff(int motor)
{
  digitalWrite(inApin[motor], LOW);
  digitalWrite(inBpin[motor], LOW);
  analogWrite(pwmpin[motor], 0);
}

// New Function => Keep in motorHAL.cpp
// Purpose: reset the robot odometer
// Algorithm:
//  - establish a calibration of odometer zero to current encoder tick count
//  - assume long integers provide orders of magnitude more dynamic range than is needed for practical DPRG style robot runs
//  - do not worry about arithmetic overflow
// Output:
//  - initialize variables used to store recent encoder readings
//  - reset the odometer to zero
void clearRobotOdometerTicks()
{
  priorEncoderSample.rightMotor = encRight.read(); // right motor orientation same polarity as robot orientation
  priorEncoderSample.leftMotor = -encLeft.read();  // negatve maps encoder orientation to robot orientation

  robotOdometerTicks.rightMotor = 0;
  robotOdometerTicks.leftMotor = 0;

  deltaOdometerTicks.rightMotor = 0;
  deltaOdometerTicks.leftMotor = 0;

  robotOdometerVelocity.leftMotor = 0;
  robotOdometerVelocity.rightMotor = 0;
}

// Purpose: print current robot odometer values
void printRobotOdometerTicks()
{
  Serial.println("\n.printRobotOdometerTicks() ");

  Serial.print(" robotOdometerTicks.leftMotor ");
  Serial.print(robotOdometerTicks.leftMotor);
  Serial.print(" right ");
  Serial.println(robotOdometerTicks.rightMotor);

  Serial.print(" robotOdometerVelocity.leftMotor ");
  Serial.print(robotOdometerVelocity.leftMotor);
  Serial.print(" right ");
  Serial.print(robotOdometerVelocity.rightMotor);
  Serial.println();
}

// New Function => Keep in motorHAL.cpp
// Purpose: Read encoders and update odometer
// Input:
//  - reads Encoders
// Processing:
//  - calculates change in ticks from current to prior encoder sample
// Algorithm:
//  - assume long integers provide orders of magnitude more dynamic range than is needed for practical DPRG style robot runs
//  - do not worry about arithmetic overflow
// Output:
//  - updates odometer and deltaOdometer measurements
void updateRobotOdometerTicks()
{
  robotEncoderTicksType currentEncoderSample;
  currentEncoderSample.rightMotor = encRight.read();
  currentEncoderSample.leftMotor = -encLeft.read();

  deltaOdometerTicks.rightMotor = currentEncoderSample.rightMotor - priorEncoderSample.rightMotor;
  deltaOdometerTicks.leftMotor = currentEncoderSample.leftMotor - priorEncoderSample.leftMotor;

  robotOdometerTicks.rightMotor += deltaOdometerTicks.rightMotor;
  robotOdometerTicks.leftMotor += deltaOdometerTicks.leftMotor;

  robotOdometerVelocity.leftMotor = -(double)deltaOdometerTicks.leftMotor;
  robotOdometerVelocity.rightMotor = (double)deltaOdometerTicks.rightMotor;

  // calculate turn velocity
  // e.g. right motor 11, left motor -9, gives a slight left turn == -2 (robot turning CCW)
  // ie. positive 'turnDeltaTicks' => robot is turning right ie. robot is turning CW
  robotOdometerVelocity.turnDeltaTicks = -1* (robotOdometerVelocity.rightMotor + robotOdometerVelocity.leftMotor);

  priorEncoderSample.rightMotor = currentEncoderSample.rightMotor;
  priorEncoderSample.leftMotor = currentEncoderSample.leftMotor;
}

// Legacy Function => Keep in motorHAL.cpp
/* motorGo() will set a motor going in a specific direction
 the motor will continue going in that direction, at that speed
 until told to do otherwise.
 
 motor: this should be either 0 or 1, will selet which of the two
 motors to be controlled
 
 direct: Should be between 0 and 3, with the following result
 0: Brake to VCC
 1: Clockwise
 2: CounterClockwise
 3: Brake to GND
 
 pwm: should be a value between 0 and 255, higher the number, the faster
 it'll go
 */
void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
{
  if (motor <= 1)
  {
    if (direct <= 4)
    {
      // Set inA[motor]
      if (direct <= 1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      // Set inB[motor]
      if ((direct == 0) || (direct == 2))
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      analogWrite(pwmpin[motor], pwm);

      // note: CS is ~0.13 V/A
      //if ((analogRead(cspin[0]) < CS_THRESHOLD) && (analogRead(cspin[1]) < CS_THRESHOLD))
      //digitalWrite(statpin, HIGH);
      //else
      //digitalWrite(statpin, LOW);
    }
  }
}

// Legacy stuff => NOT part of motorHAL.cpp
//---------- Instantiation of PID loop and associated variables

// PID Controls
double setpoint[2] = {0, 0}; // setpoint to PID loop summing node // {55, 55};  // original club robot code had default setpoint of a midrange speed
double pid_input[2];         // feedback to PID loop summing node
double pid_output[2];        // PID loop control output

double kp[2] = {3, 3}; // these are not optimized
double ki[2] = {5, 5};
//double kp[2] = {3,3};   // these are not optimized
//double ki[2] = {16,16};
double kd[2] = {0, 0};

PID right_mtr_pid(&pid_input[0], &pid_output[0], &setpoint[0], kp[0], ki[0], kd[0], DIRECT);
PID left_mtr_pid(&pid_input[1], &pid_output[1], &setpoint[1], kp[1], ki[1], kd[1], DIRECT);

// Legacy Function => NOT part of motorHAL.cpp
void init_pids()
{
  right_mtr_pid.SetSampleTime(10);
  right_mtr_pid.SetMode(AUTOMATIC);
  //right_mtr_pid.SetOutputLimits(20,240);   //min, max
  left_mtr_pid.SetSampleTime(10);
  left_mtr_pid.SetMode(AUTOMATIC);
  //left_mtr_pid.SetOutputLimits(20,240);   //min, max
}

/* --------------------------------------------------------------- */
/* motor command utilities - taken from work by D. Anderson        */
/* --------------------------------------------------------------- */

// Legacy Function => believe may not belong in motorHAL.cpp
/* clip int value to min and max */
int clip(int val, int min, int max)
{
  if (val > max)
    return max;
  if (val < min)
    return min;
  return val;
}

// Legacy Function => believe may not belong in motorHAL.cpp
/* clip float value to min and max */
int clip_f(float val, float min, float max)
{
  if (val > max)
    return max;
  if (val < min)
    return min;
  return val;
}

// Legacy Function => believe may not belong in motorHAL.cpp
/* clip float value to min and max */
/* --------------------------------------------------------------- */
/* slew rate generator */
/* --------------------------------------------------------------- */
int slew(int from, int to, int rate)
{
  int x;

  if (to > from)
  {
    x = from + rate;
    if (x <= to)
      return (x);
  }
  else if (to < from)
  {
    x = from - rate;
    if (x >= to)
      return (x);
  }
  return to;
}
