
/******************************************************************
*
*  motorTasks.h
*  Carl Ott
*  tasks to manage and control robot drive motors with different closed loop tasks and methods
*
******************************************************************/

#include "motorTasks.h"

// by definition of motor_funcs.cpp, the maximum motor speed is 255
#define maxPWM 255     // maximum speed for either motor
#define maxTurnPWM 80  // maximum allowed robot pivot speed
#define minLeftPWM 20  // slowest speed for the left motor, below this value motor stalls
#define minRightPWM 20 // slowest speed for the right motor, below this value motor stalls

bool velocityLoopEnabled = false;
bool positionLoopEnabled = false;

bool runInitialTestOnce = true;

void initializeMotorTasks()
{
    Serial.println("\nnow running initializeMotorTasks()...\n");
    velocityLoopEnabled = false;
    positionLoopEnabled = false;
    runInitialTestOnce = true;
}

// ToDo - define array [ left, right ]  // returns current absolute encoder values
// encoderValues motor::getEncoders();
// {
//     return curentMotorEncoderValues;
// }

// bool motor::velocityLoopTaskStart()
bool velocityLoopStart()
{
    velocityLoopEnabled = true;

    setMotorVelocity(0, 0); // require that using code set motor velocity AFTER initializing this task

    return true;
}

// bool motor::velocityLoopTaskStop()
bool velocityLoopStop()
{
    velocityLoopEnabled = false;

    setMotorVelocity(0, 0); // gracefully stop the motors when stopping this loop

    return true;
}

// clamp input whatValue to +/- limitingValue
//  whatValue may be + or -
//  limitingValue must be positive
signed char clamp(signed char whatValue, signed char limitingValue)
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

// setMotorVelocity()
// Input:   accepts abstract speed and steering commands
// Algorithm:   maps abstract command range into motor PWM value range
//              limits values to provide more robust behavior
//              Initial implementation - open loop
//              Target implementation - closed loop
// Output:  set robot motors according to speed and sterering commands
// bool motor::setMotorVelocity(signed char TurnVelocity, signed char Throttle)
bool setMotorVelocity(signed char TurnVelocity, signed char Throttle)
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

    Serial.println("motorTasks.cpp - setMotorVelocity()");

    Serial.print("... clamped: turnVelocity is ");
    Serial.print(turnVelocity);
    Serial.print("... throttle is ");
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

    int rightPWM = (int)rightPWMfromTurn + (int)rightPWMfromThrottle;
    int leftPWM = (int)leftPWMfromTurn + (int)leftPWMfromThrottle;

    if (velocityLoopEnabled)
    {

        //        Serial.println("\nmotorTasks.cpp - setMotorVelocity() - velocityLoopEnabled === true...");
        Serial.print("... leftPWM is ");
        Serial.print(leftPWM);
        Serial.print("... rightPWM is ");
        Serial.print(rightPWM);
        Serial.println();

        // set the left motor
        if (leftPWM == 0)
        {
            Serial.print("... left motor off");
            motorOff(L_MTR);
        }
        if (leftPWM > 0)
        {
            Serial.print("... left motor CW");
            motorGo(L_MTR, CW, leftPWM);
        }
        if (leftPWM < 0)
        {
            Serial.print("... left motor CCW");
            motorGo(L_MTR, CCW, -leftPWM);
        }

        // set the right motor
        if (rightPWM == 0)
        {
            Serial.print("... right motor off");
            motorOff(R_MTR);
        }
        if (rightPWM > 0)
        {
            Serial.print("... right motor CW");
            motorGo(R_MTR, CW, rightPWM);
        }
        if (rightPWM < 0)
        {
            Serial.println("... right motor CCW");
            motorGo(R_MTR, CCW, -rightPWM);
        }
    }
    else // stop the motors
    {
        motorOff(R_MTR);
        motorOff(L_MTR);
    }

    Serial.println();
    Serial.println();
    Serial.println();

    return true;
}

void testMotorTasks(ASIZE dummyPlaceholder)
{
    while (1)
    {
        if (runInitialTestOnce)
        {
            Serial.println("\nmotorTasks.cpp -> testMotorTasks() -> runInitialTestOnce == true => let's go...\n");
            Serial.println("\n... velocityLoopStart()\n");
            velocityLoopStart();
            Serial.println("... Pivot CW -> setMotorVelocity(0,50) for 2 seconds");
            setMotorVelocity(0, 50);

            wake_after(2000);
            Serial.println("... setMotorVelocity(0,0) for 1 second");
            setMotorVelocity(0, 0);

            wake_after(1000);
            Serial.println("... Go Forwards -> setMotorVelocity(50,0) for 2 seconds");
            setMotorVelocity(50, 0);

            wake_after(2000);
            Serial.println("... setMotorVelocity(0,0) for 1 second");
            setMotorVelocity(0, 0);

            wake_after(1000);
            Serial.println("... Go Backwards -> setMotorVelocity(0,-90) for 2 seconds");
            setMotorVelocity(0, -90);

            wake_after(2000);
            Serial.println("... setMotorVelocity(0,0) for 1 second");
            setMotorVelocity(0, 0);

            wake_after(1000);
            Serial.println("... Pivot CCW -> setMotorVelocity(-50,0) for 2 seconds");
            setMotorVelocity(-50, 0);

            wake_after(2000);
            Serial.println("... setMotorVelocity(0,0) for 1 second");
            setMotorVelocity(0, 0);

            wake_after(1000);
            Serial.println("... forward & Pivot CW -> setMotorVelocity(50,100) for 2 seconds");
            setMotorVelocity(50, 100);

            wake_after(2000);
            Serial.println("\n... velocityLoopStop()\n");
            velocityLoopStop();
            runInitialTestOnce = false;
            Serial.println("... completed testMotorTasks() and set runInitialTestOnce = false");
            Serial.println("... testMotorTasks() will now sit and loop with nothing to do in the libtask scheduler()...");
        };
    };
}
