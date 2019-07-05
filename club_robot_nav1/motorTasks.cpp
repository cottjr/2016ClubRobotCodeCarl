
/******************************************************************
*
*  motorTasks.h
*  Carl Ott
*  tasks to manage and control robot drive motors with different closed loop tasks and methods
*  accepts abstrated +- 100 ranged commands for velocity control loop or position control loop
*  maps those commands to PWM scaled commands as needed by the motor shield
*
*  NOTE: Review ANY 'Velocity' values when changing the sample rate, motor type or motor drive voltage!
*   e.g. 'maxEncoderVelocityTicks' change when those parameters change...
*   => would be nice to abstract those to a single compile or run-time initialization HAL place
*   see "measureMinMaxMotorSpeed results 2019 April 21.txt" for initial baseline data
*
******************************************************************/

#include "motorTasks.h"

// These values belong in a HAL layer file -> leaving here for now
// by definition of motor_funcs.cpp, the maximum motor speed is 255
#define maxPWM 255    // maximum speed for either motor
#define maxTurnPWM 80 // maximum allowed robot pivot speed
// calibration factors specific to a particular instance of the 2016 Club Robot
//  => determined empirically
#define minLeftPWM 20               // slowest speed for the left motor, below this value motor stalls
#define minRightPWM 20              // slowest speed for the right motor, below this value motor stalls
#define minEncoderVelocityTicks 15  // abs(steady state # encoder ticks) per 10 ms when motor at minPWM,           \
                                    // defined as the bigger of left or right motor value in forward or backwards, \
                                    // to ensure that both motors can achieve the lowest speed.                    \
                                    // measured over 100 ms
#define maxEncoderVelocityTicks 200     // updated per quick measurement 2019 June 23 circa commit c786d6e 
#define maxTurnEncoderVelocityTicks 78  // One would expcet this = maxTurnPWM / maxPWM * maxEncoderVelocityTicks;
                                        // however, encoderVelocityTicks do not follow that formula in practice
                                        // hence this value needs to be set by manual calibration
                                    // abs(steady state # encoder ticks) per 10 ms when motor at maxPWM,            \
                                    // defined as the smaller of left or right motor value in forward or backwards, \
                                    // to ensure that both motors can achieve the highest speed                     \
                                    // measured over 100 ms

bool velocityLoopEnabled = false;
bool positionLoopEnabled = false;

// int periodicSampleMotorShield_ProcessID = -1; // handle for the libtask library process ID
// int monitorVelocityLoop_ProcessID = -1;       // handle for the libtask library process ID
// int positionLoopProcessID = -1; // handle for the libtask library process ID

location currentLocation; // location === structure defined in nav_funcs.h  // ToDo- Deprecate this with code below

// Velocity loop PID variables
double rightEncVelocitySetpoint = 0;
double leftEncVelocitySetpoint = 0;
double rightVelocityLoopOutPWM = 0;
double leftVelocityLoopOutPWM = 0;

int leftLoopPWM = 0;  // used by sendVelocityLoopPWMtoMotorShield(), shared globall for diagnostics
int rightLoopPWM = 0; // used by sendVelocityLoopPWMtoMotorShield(), shared globall for diagnostics

// Aggressive and conservative Tuning Parameters
// [0] == right, [1] == left
// double aggKp=4, aggKi=0.2, aggKd=1;
double conservativeVelocityKp[2] = {1.75,1.75};  //r 1.75 note: {right, left}
double conservativeVelocityKi[2] = {12, 12};  //r 12  note: {right, left}
double conservativeVelocityKd[2] = {0, 0};  //r 0    note: {right, left}

PID leftVelocityPID(&robotOdometerVelocity.leftMotor, &leftVelocityLoopOutPWM, &leftEncVelocitySetpoint, conservativeVelocityKp[1], conservativeVelocityKi[1], conservativeVelocityKd[1], DIRECT);
PID rightVelocityPID(&robotOdometerVelocity.rightMotor, &rightVelocityLoopOutPWM, &rightEncVelocitySetpoint, conservativeVelocityKp[0], conservativeVelocityKi[0], conservativeVelocityKd[0], DIRECT);

// Purpose:
//      stop any form of power going to the motor
//      this eliminates motor noise which otherwise occurs from sending 0 velocity commands to motors via the PID loops
void stopMotors()
{
    motorOff(L_MTR);
    motorOff(R_MTR);
}

// Purpose: zero the PID loop integrator
// Input: none
// Algorithm:
//      because the PID library does not expose it's internal "Initialize" method,
//      Simply flip PID mode to manual, zero the PID output, then enable the PID again.
//      This follows the library authors intention,
//      ie. per http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-initialization/
//      note: it feels "dirty" to mainpulate the PID loop output variables this way,
//          -> but it's what the author explicitly intended
// Output:
//      internal ITerm variable in the PID components are set to
//      either the most recent actual feedback values
//      or the PID loop output limits
void initializePIDs () 
{
    leftVelocityPID.SetMode(MANUAL);
    leftVelocityLoopOutPWM = 0;
    leftVelocityPID.SetMode(AUTOMATIC);

    rightVelocityPID.SetMode(MANUAL);
    rightVelocityLoopOutPWM = 0;
    rightVelocityPID.SetMode(AUTOMATIC);
}

byte zeroSpeedCache = 0;
// Purpose:
//      re-initialize the PIDs when the motors and robot appear to be at restart
//      results in quickly disabling the annoying motor hum when motors are not getting commands
//      relies on inherent inertial braking to more or less hold position steady
//      but as the PID loops are left on this function,
//      in case some external disturbance turns the motors, 
//      the loops should automatically kick in and attempt to correct any motion and maintain zero velocity
// Input:
//      velocity loop setpoints
//      actual encoder velocity measurements
// Algorithm
//      declare zero motion on several consecutive samples,
//      where command and actual velocity is zero
//      hopefully this is sufficient to detect an actual stop,
//      as opposed to a motor command and velocity swinging through zero
// Output:
//      sets motor shield values and PWM commands to zero
//      sets the PID loop integrator to zero
void resetAtRestMotorsAndPIDs ()
{
    bool isCurrentSampleZero = false;

    zeroSpeedCache = zeroSpeedCache << 1;

    isCurrentSampleZero = (leftEncVelocitySetpoint == 0) && (robotOdometerVelocity.leftMotor == 0)
                        && (rightEncVelocitySetpoint == 0) && (robotOdometerVelocity.rightMotor == 0);
   
    zeroSpeedCache |= isCurrentSampleZero;

    if ( (zeroSpeedCache & 0x07) == 0x07) {
        stopMotors();
        initializePIDs();
    }
}

// Purpose: print values related to the Velocity PID loop
void printVelocityLoopValues()
{
    Serial.println("\nprintVelocityLoopValues() ");

    Serial.print(".leftEncVelocitySetpoint ");
    Serial.print(leftEncVelocitySetpoint);
    Serial.print(" right ");
    Serial.println(rightEncVelocitySetpoint);

    Serial.print(".robotOdometerVelocity.leftMotor ");
    Serial.print(robotOdometerVelocity.leftMotor);
    Serial.print(" right ");
    Serial.print(robotOdometerVelocity.rightMotor);
    Serial.println();

    Serial.print(".leftVelocityLoopOutPWM ");
    Serial.print(leftVelocityLoopOutPWM);
    Serial.print(" right ");
    Serial.print(rightVelocityLoopOutPWM);
    Serial.println();
}

// Purpose: periodically print values related to the Velocity PID loop
// void monitorVelocityLoop(ASIZE msLoopPeriod)
// {
//     TSIZE t;
//     t = sysclock + msLoopPeriod;
//     while (1)
//     {
//         printVelocityLoopValues();
//         PERIOD(&t, msLoopPeriod);
//     }
// }

// Purpose: Initialize the code in this module
// Processing:
//  - disable motor movement
//  - reset the absolute odometer to zero
//  - start PID loop sample rate tasks & leave them running whether needed or not
//      => e.g., to provide a consistent mechanism for reading encoder values & sending motor commands
//      => whether open loop or part of a control loop
// Output:
//  - updates process handles to spawned processes
void initializeMotorTasks()
{
    Serial.println("\ninitializeMotorTasks()");
    velocityLoopEnabled = false;
    positionLoopEnabled = false;

    setVelocityLoopSetpoints(0, 0, true);
}

// Purpose: Periodic service to read and write motor shield values,
//          and perform other synchronous tasks at e.g. PID loop sample intervals
// Input:
//  - uses motor velocity targets and an open/closed loop flag which are updated elsewhere in this module
// Algorithm Assumptions
//  - velocity is in units of encoder ticks
//  - velocity commands global to this module are always maintained to reasonable values
// Processing:
//  - at each PID loop sample interval =>
//  - request current encoder values from nav_funcs.cpp
//  - calculate motor commands per PID loop calculations or open loop as appropriate
// Output:
//  - send resulting motor commands to the motor shield
// int msOfPriorPID, msOfCurrentPID, msBetweenPID, msExecutePID; // track velocity PID loop execution timing and periodicity...
void sampleMotorShield(){
        updateRobotOdometerTicks();
        // printRobotOdometerTicks(); // ToDo - remove this at full loop speed

        resetAtRestMotorsAndPIDs();

        // leftVelocityPID.Compute();
        // rightVelocityPID.Compute();
        // use preceeding lines normally - use following lines to verify PID is actually computing
        Serial.print("\nLeftSetPnt/OdVel/PID/PWM: ");
        Serial.print(leftEncVelocitySetpoint);
        Serial.print(" ");
        Serial.print(robotOdometerVelocity.leftMotor);
        Serial.print(" ");
        Serial.print(leftVelocityPID.Compute());
        Serial.print(" ");
        Serial.print(leftVelocityLoopOutPWM);

        Serial.print(", Rght ");
        Serial.print(rightEncVelocitySetpoint);
        Serial.print(" ");
        Serial.print(robotOdometerVelocity.rightMotor);
        Serial.print(" ");
        Serial.print(rightVelocityPID.Compute());
        Serial.print(" ");
        Serial.println(rightVelocityLoopOutPWM);

        // printVelocityLoopValues(); // ToDo - remove this at full loop speed
        sendVelocityLoopPWMtoMotorShield();
}
// void periodicSampleMotorShield(ASIZE msLoopPeriod)
// {
//     TSIZE t;
//     t = sysclock + msLoopPeriod;
//     while (1)
//     {
//         msOfCurrentPID = millis();
//         msBetweenPID = msOfCurrentPID - msOfPriorPID;
//         msOfPriorPID = msOfCurrentPID;

//         updateRobotOdometerTicks();
//         // printRobotOdometerTicks(); // ToDo - remove this at full loop speed

//         leftVelocityPID.Compute();
//         rightVelocityPID.Compute();

//         // printVelocityLoopValues(); // ToDo - remove this at full loop speed
//         sendVelocityLoopPWMtoMotorShield();

//         msExecutePID = millis() - msOfCurrentPID;
//         PERIOD(&t, msLoopPeriod);
//     }
// }

// ToDo - deprecate this, replace with new HAL layer stuff in motor_funcs.cpp
// Purpose: Fetch curent encoder values
// Processing:
//  - request current encoder from nav_funcs.cpp
// Output:
//  - update data structure with current values

void sampleEncoders(encoderMeasurementsStruct *whichOnes)
{
    whichOnes->msTimestamp = millis();
    getOdometer(&currentLocation);
    whichOnes->encoderCountRight = currentLocation.encoderCountRight;
    whichOnes->encoderCountLeft = currentLocation.encoderCountLeft;
}

// ToDo - deprecate this, replace with new HAL layer stuff in motor_funcs.cpp
// Purpose: Calculate differences between encoder samples
// Input:
//  - pointers to two diferent encoder measurement structures
// Output:
//  - update the current measurement with deltas to prior
void calculateEncoderMeasurementDeltas(encoderMeasurementsStruct *prior, encoderMeasurementsStruct *currentMeasures)
{
    currentMeasures->msDeltaToPrior = currentMeasures->msTimestamp - prior->msTimestamp;
    currentMeasures->encoderDeltaToPriorRight = currentMeasures->encoderCountRight - prior->encoderCountRight;
    currentMeasures->encoderDeltaToPriorLeft = currentMeasures->encoderCountLeft - prior->encoderCountLeft;
}

void periodicSampleMotorShield_Start()
{
    Serial.println("\nperiodicSampleMotorShield_Start()");
    velocityLoopEnabled = true;

    setVelocityLoopSetpoints(0, 0, true);

    //turn the PIDs on
    leftVelocityPID.SetMode(AUTOMATIC);
    rightVelocityPID.SetMode(AUTOMATIC);

    //sample the PIDs at 50 Hz -> ie. rely on the timer/counter interrupt scheme to sample every 20ms
    leftVelocityPID.SetSampleTime(19);
    rightVelocityPID.SetSampleTime(19);

    // Serial.println("launching periodicSampleMotorShield()");
    // kill_process(periodicSampleMotorShield_ProcessID); // cleanly restart this task in case an instance is already running
    // // start the task with a nominal 10ms period
    // periodicSampleMotorShield_ProcessID = create_task("periodicSampleMotorShield", periodicSampleMotorShield, 10, MINSTACK * 1);
    // Serial.println("launched periodicSampleMotorShield");
    // if (periodicSampleMotorShield_ProcessID == -1)
    // {
    //     Serial.println("-> error in create_task(periodicSampleMotorShield)");
    // }

    // // start the task with a nominal 200ms period
    // monitorVelocityLoop_ProcessID = create_task("monitorVelocityLoop", monitorVelocityLoop, 200, MINSTACK * 3);
    // Serial.println("launched monitorVelocityLoop");
    // if (periodicSampleMotorShield_ProcessID == -1)
    // {
    //     Serial.println("-> error in create_task(monitorVelocityLoop)");
    // }
}

void periodicSampleMotorShield_Stop()
{
    Serial.println("\nperiodicSampleMotorShield_Stop()");
    velocityLoopEnabled = false;

    setMotorVelocityByPWM(0, 0); // gracefully stop the motors when stopping this loop
    setVelocityLoopSetpoints(0, 0, true);

    //turn the PIDs off
    leftVelocityPID.SetMode(MANUAL);
    rightVelocityPID.SetMode(MANUAL);

    // kill_process(periodicSampleMotorShield_ProcessID); // cleanly end this task
    // Serial.println("killed periodicSampleMotorShield");

    // kill_process(monitorVelocityLoop_ProcessID); // cleanly end this task
    // Serial.println("killed monitorVelocityLoop");
}

// clamp input whatValue to +/- limitingValue
//      version for (signed char)
//      whatValue may be + or -
//      limitingValue must be positive
signed char clamp(signed char whatValue, signed char limitingValue)
{
    if (whatValue < -limitingValue)
        return -limitingValue;
    else if (whatValue > limitingValue)
        return limitingValue;
    else
        return whatValue;
}

// clamp input whatValue to +/- limitingValue
//      version for (double)
//      whatValue may be + or -
//      limitingValue must be positive
double clampDouble(double whatValue, double limitingValue)
{
    if (whatValue < -limitingValue)
        return -limitingValue;
    else if (whatValue > limitingValue)
        return limitingValue;
    else
        return whatValue;
}

// Orientation Conventions
// when looking down on robot    when looking from wheel towards motor
// robot spin CW                left    CCW     right   CCW
// robot spin CCW               left    CW      right   CW
// robot move forward           left    CCW     right   CW
// robot move backwards         left    CW      right   CCW

// setVelocityLoopSetpoints()
// Purpose: map heading and throttle commands to encoder velocity setpoints
// Input:   accepts abstract speed and steering commands
//          turnVelocity; // abstract speed from -100 to +100. + values robot spins CW, - values CCW
//          throttle;     // abstract speed from -100 to +100. + values robot moves forward, - values backwards
// Algorithm:   maps abstract command range into encoder speed value range
//              limits values to provide more robust behavior
// Output:
//  - updates local storage encoder value domain velocity setpoints,
//  - intended as reference signal for a PID loop
bool setVelocityLoopSetpoints(signed char TurnVelocity, signed char Throttle, bool printNewSettings)
{
    signed char turnVelocity; // abstract speed from -100 to +100. + values robot spins CW, - values CCW
    signed char throttle;     // abstract speed from -100 to +100. + values robot moves forward, - values backwards

    turnVelocity = clamp(TurnVelocity, 100); // keep this function tolerant of mimalformed input
    throttle = clamp(Throttle, 100);         // keep this function tolerant of mimalformed input

    // coding convention ->
    //  Encoder values are signed Long...
    //  + values => turn motor CW, - values => turn motor CCW
    //  => positive turnVelocity => robot spins CW => turn both motors CCW
    double rightEncFromTurn = -1 * ((double)turnVelocity) / 100 * maxTurnEncoderVelocityTicks;
    double leftEncFromTurn = -1 * ((double)turnVelocity) / 100 * maxTurnEncoderVelocityTicks;

    double limitedMaxVelocityTicks = maxEncoderVelocityTicks - abs(((double)turnVelocity) / 100 * maxTurnEncoderVelocityTicks);

    double rightEncFromThrottle = ((double)throttle) / 100 * limitedMaxVelocityTicks;     // limit throttle for high rates of robot turn
    double leftEncFromThrottle = -1 * ((double)throttle) / 100 * limitedMaxVelocityTicks; // limit throttle for high rates of robot turn

    rightEncVelocitySetpoint = rightEncFromTurn + rightEncFromThrottle;
    leftEncVelocitySetpoint = leftEncFromTurn + leftEncFromThrottle;

    if (printNewSettings)
    {
        Serial.print("\nsetVelLoopSetpnts()");
        Serial.print("clampd turnVel ");
        Serial.print(turnVelocity);
        Serial.print(", thrttl ");
        Serial.println(throttle);

        Serial.print("lftEncFrmTurn: ");
        Serial.print(leftEncFromTurn);
        Serial.print(" rght ");
        Serial.print(rightEncFromTurn);
        
        Serial.print(", lftEncFrmThrttl: ");
        Serial.print(leftEncFromThrottle);
        Serial.print(" rght ");
        Serial.print(rightEncFromThrottle);

        Serial.print(", lftEncVelSetpnt: ");
        Serial.print(leftEncVelocitySetpoint);
        Serial.print(" rght ");
        Serial.println(rightEncVelocitySetpoint);
        Serial.println();

        // printVelocityLoopValues();  // print new setpoint, and current encoder velocity and loop out PWM
    }
}

// sendVelocityLoopPWMtoMotorShield()   // ToDo -> belongs in HAL layer ???
// Purpose: map velocity PID loop output to  & send PWM commands to the motor shield
// Input:   'signed PWM' domain velocity setpoints
//          values expected to range from -255 to +255
//          values outside that range will be clamped
// Algorithm:
//          limits values to provide more robust behavior
// Output:  set robot motor PWM values and direction as commanded
bool sendVelocityLoopPWMtoMotorShield()
{

    // PID  -> takes Encoder domain setpoint and feedback
    //      -> outputs PWM

    // coding convention ->
    //  PWM values sent to motors must always be positive 0 .. 255
    //  PID loop output uses a 'signed PWM' value to keep track of motor direction and provide smooth calculations through zero
    //  + values => turn motor CW, - values => turn motor CCW
    //  => positive turnVelocity => robot spins CW => turn both motors CCW

    // Consider disabling the following lines -> rely on clamping function built-into PID_v1 library...
    leftLoopPWM = (signed int) clampDouble(leftVelocityLoopOutPWM, maxPWM);
    rightLoopPWM = (signed int) clampDouble(rightVelocityLoopOutPWM, maxPWM);

Serial.print("velLoopEnabled ");
Serial.print(velocityLoopEnabled);
Serial.print(" lftLoopPWM ");
Serial.print(leftLoopPWM);
Serial.print(" rght ");
Serial.println(rightLoopPWM);

    if (leftLoopPWM > 0)
    {
        motorGo(L_MTR, CW, (uint8_t)(leftLoopPWM));
    }
    if (leftLoopPWM < 0)
    {
        motorGo(L_MTR, CCW, (uint8_t)(-leftLoopPWM));
    }

    if (rightLoopPWM > 0)
    {
        motorGo(R_MTR, CW, (uint8_t)(rightLoopPWM));
    }
    if (rightLoopPWM < 0)
    {
        motorGo(R_MTR, CCW, (uint8_t)(-rightLoopPWM));
    }

    return true;
}

// setMotorVelocityByPWM()
// Purpose: compute & set motor velocity using motor Open Loop by directly sending PWM domain values
// Assumptions:
//  - this function assumes that no other tasks are sending PWM commands to the motor shield.
//  - e.g. unpredictable results will likely occur if motor PID loops are running when this function is called
// Input:   accepts abstract speed and steering commands
//          turnVelocity; // abstract speed from -100 to +100. + values robot spins CW, - values CCW
//          throttle;     // abstract speed from -100 to +100. + values robot moves forward, - values backwards
// Algorithm:   maps abstract command range into motor PWM value range
//              limits values to provide more robust behavior
//              Initial implementation - open loop
//              Target implementation - closed loop (but then based on what feedback?)
// Output:  set robot motors according to speed and sterering commands
bool setMotorVelocityByPWM(signed char TurnVelocity, signed char Throttle)
{
    signed char turnVelocity; // abstract speed from -100 to +100. + values robot spins CW, - values CCW
    signed char throttle;     // abstract speed from -100 to +100. + values robot moves forward, - values backwards

    turnVelocity = clamp(TurnVelocity, 100); // keep this function tolerant of mimalformed input
    throttle = clamp(Throttle, 100);         // keep this function tolerant of mimalformed input

    // coding convention ->
    //  PWM values are always positive 0 .. 255
    //  this code uses a 'signed PWM' value to keep track of motor direction
    //  + values => turn motor CW, - values => turn motor CCW
    //  => positive turnVelocity => robot spins CW => turn both motors CCW
    double rightPWMfromTurn = -1 * ((double)turnVelocity) / 100 * maxTurnPWM;
    double leftPWMfromTurn = -1 * ((double)turnVelocity) / 100 * maxTurnPWM;

    double limitedMaxPWM = maxPWM - abs(((double)turnVelocity) / 100 * maxTurnPWM);

    double rightPWMfromThrottle = ((double)throttle) / 100 * limitedMaxPWM;     // limit throttle for high rates of robot turn
    double leftPWMfromThrottle = -1 * ((double)throttle) / 100 * limitedMaxPWM; // limit throttle for high rates of robot turn

    int rightPWM = (int)rightPWMfromTurn + (int)rightPWMfromThrottle;
    int leftPWM = (int)leftPWMfromTurn + (int)leftPWMfromThrottle;

    Serial.println("setMotorVelocityByPWM()");

    Serial.print("clmpd turnVelocity ");
    Serial.print(turnVelocity);
    Serial.print(" thrttl ");
    Serial.print(throttle);
    Serial.println();

    //        Serial.print("... leftPWMfromTurn is ");
    //        Serial.print(leftPWMfromTurn);
    //        Serial.print("... rightPWMfromTurn is ");
    //        Serial.print(rightPWMfromTurn);
    //        Serial.println();
    //
    //        Serial.print("... limitedMaxPWM is ");
    //        Serial.print(limitedMaxPWM);
    //                Serial.println();
    //
    //        Serial.print("... leftPWMfromThrottle is ");
    //        Serial.print(leftPWMfromThrottle);
    //        Serial.print("... rightPWMfromThrottle is ");
    //        Serial.print(rightPWMfromThrottle);
    //        Serial.println();

    //        Serial.println("\nmotorTasks.cpp - setMotorVelocityByPWM() - velocityLoopEnabled === true...");
    Serial.print(" LeftPWM ");
    Serial.print(leftPWM);
    Serial.print(" Rght ");
    Serial.print(rightPWM);
    Serial.println();

    // set the left motor
    if (abs(leftPWM) < minLeftPWM) // define the left motor deadband
    {
        Serial.print(" left motor off");
        motorOff(L_MTR);
    }
    if (leftPWM > 0)
    {
        Serial.print(" left motor CW");
        motorGo(L_MTR, CW, leftPWM);
    }
    if (leftPWM < 0)
    {
        Serial.print(" left motor CCW");
        motorGo(L_MTR, CCW, -leftPWM);
    }

    // set the right motor
    if (abs(rightPWM) < minRightPWM) // define the right motor deadband
    {
        Serial.println(" right motor off");
        motorOff(R_MTR);
    }
    if (rightPWM > 0)
    {
        Serial.println(" right motor CW");
        motorGo(R_MTR, CW, rightPWM);
    }
    if (rightPWM < 0)
    {
        Serial.println(" right motor CCW");
        motorGo(R_MTR, CCW, -rightPWM);
    }


    return true;
}

// Purpose:
//  - baseline velocity PID loop verification test
// void testVelocityPIDloop(ASIZE dummyArgumentPlaceholder)
// {
//     wake_after(2000);
//     Serial.println("\n> testVelocityPIDloop() - start");
//     setVelocityLoopSetpoints(0, 50, true);

//     wake_after(4000);
//     Serial.println("\n> testVelocityPIDloop() - stop");
//     setVelocityLoopSetpoints(0, 0, true);

//     periodicSampleMotorShield_Stop();
//     terminate();
// }

// testVelocityLoopSetpointsMath()
// Purpose: check the math
// Algorithm:
//  - simply set the loop to various values
//  - manually inspect / verify the console log output
void testVelocityLoopSetpointsMath()
{
    setVelocityLoopSetpoints(0, 50, true);
    setVelocityLoopSetpoints(0, -50, true);

    setVelocityLoopSetpoints(50, 0, true);
    setVelocityLoopSetpoints(-50, 0, true);

    setVelocityLoopSetpoints(20, 90, true);
    setVelocityLoopSetpoints(20, -90, true);

    setVelocityLoopSetpoints(90, 20, true);
    setVelocityLoopSetpoints(-90, 20, true);

    setVelocityLoopSetpoints(0, 0, true);
}

// Purpose:
//  - used by measureMinMaxMotorSpeeds()
//  - show values of an encoder measurement sample set
void printEncoderMeasurements(encoderMeasurementsStruct measurementsPointer[], int maxIndex)
// per http://www.c4learn.com/c-programming/c-passing-array-of-structure-to-function/
{
    Serial.println();
    int i;
    for (i = 1; i <= maxIndex; i++)
    {
        Serial.print("sample ");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(measurementsPointer[i].msDeltaToPrior);
        Serial.print(" ms, ");
        Serial.print(measurementsPointer[i].encoderDeltaToPriorLeft);
        Serial.print(" left ticks, ");
        Serial.print(measurementsPointer[i].encoderDeltaToPriorRight);
        Serial.print(" right ticks, ");
        Serial.println();
    }
}

// Purpose:
//  - measure the minimum and maximum velocity of both motors, in both forward and reverse directions
//  - use result to calibrate throttle to speed transfer function scale for a particular robot hardware set
//  - measure velocity in terms of encoder ticks per 100 ms
//  - measure repeatability of task the libtask / task.h / task.cpp library
//  - verify dependencies on nav_funcs.cpp and motor_funcs.cpp
// Input:
//  - none
// Output:
//  - console logs and motor movement
//  - an estimate of the maximum rotational speed for each motor
// void measureMinMaxMotorSpeeds(ASIZE dummyArgumentPlaceholder)
// {
//     int const numSamples = 30; // note: sample window has a considerable impact on libtask stacksize requirements in club_robot_nav1
//                                // e.g. going from numSamples === 10 to numSamples === 30 required updating in club_robot_nav1.ino
//                                // (MINSTACK * 10) to something <= 20, ie. create_task("measureMinMaxMotorSpeeds", measureMinMaxMotorSpeeds, 10, MINSTACK * 20);
//     int const msSampleWindow = 100;
//     int const pauseBetweenTests = 3000;

//     encoderMeasurementsStruct tempMeasurements[numSamples + 1];
//     int i;

//     Serial.println("\n\nStarting measureMinMaxMotorSpeeds()\n");

//     Serial.println("\n... running open loop, hence -> periodicSampleMotorShield_Stop()\n");
//     periodicSampleMotorShield_Stop();

//     // First - measure Max Forward speed
//     Serial.print("\n\n\nMaximum Forward Speeds, with a sample period of ");
//     Serial.print(msSampleWindow);
//     Serial.println("ms");
//     setMotorVelocityByPWM(0, 100);
//     wake_after(msSampleWindow); // give ample time for motor speed to settle, or shrink to measure acceleration...
//     sampleEncoders(&tempMeasurements[0]);
//     for (i = 1; i <= numSamples; i++)
//     {
//         wake_after(msSampleWindow);
//         sampleEncoders(&tempMeasurements[i]);
//         calculateEncoderMeasurementDeltas(&tempMeasurements[i - 1], &tempMeasurements[i]);
//     }
//     setMotorVelocityByPWM(0, 0);
//     printEncoderMeasurements(tempMeasurements, numSamples);

//     // Next - measure Max Backwards speed
//     wake_after(pauseBetweenTests);
//     Serial.print("\n\n\nMaximum Reverse Speeds, with a sample period of ");
//     Serial.print(msSampleWindow);
//     Serial.println("ms");
//     setMotorVelocityByPWM(0, -100);
//     wake_after(msSampleWindow); // give ample time for motor speed to settle, or shrink to measure acceleration...
//     sampleEncoders(&tempMeasurements[0]);
//     for (i = 1; i <= numSamples; i++)
//     {
//         wake_after(msSampleWindow);
//         sampleEncoders(&tempMeasurements[i]);
//         calculateEncoderMeasurementDeltas(&tempMeasurements[i - 1], &tempMeasurements[i]);
//     }
//     setMotorVelocityByPWM(0, 0);
//     printEncoderMeasurements(tempMeasurements, numSamples);

//     // Next - measure Min Forward speed
//     wake_after(pauseBetweenTests);
//     Serial.print("\n\n\nMinimum Forward Speeds, with a sample period of ");
//     Serial.print(msSampleWindow);
//     Serial.println("ms");
//     setMotorVelocityByPWM(0, 8);
//     wake_after(msSampleWindow); // give ample time for motor speed to settle, or shrink to measure acceleration...
//     sampleEncoders(&tempMeasurements[0]);
//     for (i = 1; i <= numSamples; i++)
//     {
//         wake_after(msSampleWindow);
//         sampleEncoders(&tempMeasurements[i]);
//         calculateEncoderMeasurementDeltas(&tempMeasurements[i - 1], &tempMeasurements[i]);
//     }
//     setMotorVelocityByPWM(0, 0);
//     printEncoderMeasurements(tempMeasurements, numSamples);

//     // Next - measure Min Backwards speed
//     wake_after(pauseBetweenTests);
//     Serial.print("\n\n\nMinimum Reverse Speeds, with a sample period of ");
//     Serial.print(msSampleWindow);
//     Serial.println("ms");
//     setMotorVelocityByPWM(0, -8);
//     wake_after(msSampleWindow); // give ample time for motor speed to settle, or shrink to measure acceleration...
//     sampleEncoders(&tempMeasurements[0]);
//     for (i = 1; i <= numSamples; i++)
//     {
//         wake_after(msSampleWindow);
//         sampleEncoders(&tempMeasurements[i]);
//         calculateEncoderMeasurementDeltas(&tempMeasurements[i - 1], &tempMeasurements[i]);
//     }
//     setMotorVelocityByPWM(0, 0);
//     printEncoderMeasurements(tempMeasurements, numSamples);

//     // ToDo - decide whether to presume or test ahead of time, and restart the PID loops. For now, just leave it off after this task...
//     // Serial.println("\n... finished running open loop, start periodic sampling again -> periodicSampleMotorShield_Start()\n");
//     // periodicSampleMotorShield_Start();

//     terminate(); // end this libtask task -> free up RAM and CPU resource
// }

// Purpose:
//  - verify basic open-loop control of the motors
//  - verify movement polarity
//  - verify basic utilization of the libtask / task.h / task.cpp library
// Input:
//  - none
// Output:
//  - console logs and motor movement
// void testMotorTasks(ASIZE dummyArgumentPlaceholder)
// {

//     Serial.println("\nmotorTasks.cpp -> testMotorTasks() => let's go...\n");

//     Serial.println("\n... running open loop, hence -> periodicSampleMotorShield_Stop()\n");
//     periodicSampleMotorShield_Stop();

//     Serial.println("\n\n... Pivot CW -> setMotorVelocityByPWM(0,50) for 2 seconds");
//     setMotorVelocityByPWM(0, 50);

//     wake_after(2000);
//     Serial.println("... setMotorVelocityByPWM(0,0) for 1 second");
//     setMotorVelocityByPWM(0, 0);

//     wake_after(1000);
//     Serial.println("... Go Forwards -> setMotorVelocityByPWM(50,0) for 2 seconds");
//     setMotorVelocityByPWM(50, 0);

//     wake_after(2000);
//     Serial.println("... setMotorVelocityByPWM(0,0) for 1 second");
//     setMotorVelocityByPWM(0, 0);

//     wake_after(1000);
//     Serial.println("... Go Backwards -> setMotorVelocityByPWM(0,-90) for 2 seconds");
//     setMotorVelocityByPWM(0, -90);

//     wake_after(2000);
//     Serial.println("... setMotorVelocityByPWM(0,0) for 1 second");
//     setMotorVelocityByPWM(0, 0);

//     wake_after(1000);
//     Serial.println("... Pivot CCW -> setMotorVelocityByPWM(-50,0) for 2 seconds");
//     setMotorVelocityByPWM(-50, 0);

//     wake_after(2000);
//     Serial.println("... setMotorVelocityByPWM(0,0) for 1 second");
//     setMotorVelocityByPWM(0, 0);

//     wake_after(1000);
//     Serial.println("... forward & Pivot CW -> setMotorVelocityByPWM(50,100) for 2 seconds");
//     setMotorVelocityByPWM(50, 100);

//     wake_after(2000);
//     Serial.println("... shut down tihs test - setMotorVelocityByPWM(0,0) and leave it there...");
//     setMotorVelocityByPWM(0, 0);

//     // ToDo - decide whether to presume or test ahead of time, and restart the PID loops. For now, just leave it off after this task...
//     // Serial.println("\n... finished running open loop, start periodic sampling again -> periodicSampleMotorShield_Start()\n");
//     // periodicSampleMotorShield_Start();

//     Serial.println("... completed testMotorTasks() stopping this task with terminate()");
//     terminate();
// }
