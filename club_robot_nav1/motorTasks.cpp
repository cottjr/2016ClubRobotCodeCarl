
/******************************************************************
*
*  motorTasks.h
*  Carl Ott
*  tasks to manage and control robot drive motors with different closed loop tasks and methods
*  accepts abstrated +- 100 ranged commands for velocity control loop or position control loop
*  maps those commands to PWM scaled commands as needed by the motor shield
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

location currentLocation; // location === structure defined in nav_funcs.h

// Velocity loop PID variables
// [0] == right, [1] == left
double velocitySetpoint[2] = {0, 0}; // setpoint to PID loop summing node // {55, 55};  // original club robot code had default setpoint of a midrange speed
double velocityActual[2] = {0, 0};   // feedback to PID loop summing node
double velocityLoopOut[2] = {0, 0};  // PID loop control output

// Aggressive and conservative Tuning Parameters
// [0] == right, [1] == left
// double aggKp=4, aggKi=0.2, aggKd=1;
double conservativeVelocityKp[2] = {1, 1};
double conservativeVelocityKi[2] = {0.05, 0.05};
double conservativeVelocityKd[2] = {0.25, 0.25};

//PID leftVelocityPID(&velocityActual[1], &velocityLoopOut[1], &velocitySetpoint[1], conservativeVelocityKp[1], conservativeVelocityKi[1], conservativeVelocityKd[1], DIRECT);
//PID rightVelocityPID(&velocityActual[0], &velocityLoopOut[0], &velocitySetpoint[0], conservativeVelocityKp[0], conservativeVelocityKi[0], conservativeVelocityKd[0], DIRECT);

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
    Serial.println("\n... >> velocityLoopStart()\n");
    velocityLoopEnabled = true;

    setMotorVelocity(0, 0); // require that using code set motor velocity AFTER initializing this task

    return true;
}

// bool motor::velocityLoopTaskStop()
bool velocityLoopStop()
{
    Serial.println("\n... >> velocityLoopStop()\n");
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

    Serial.println("\nmotorTasks.cpp - setMotorVelocity()");

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
        if (abs(leftPWM) < minLeftPWM) // define the left motor deadband
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
        if (abs(rightPWM) < minRightPWM) // define the right motor deadband
        {
            Serial.println("... right motor off");
            motorOff(R_MTR);
        }
        if (rightPWM > 0)
        {
            Serial.println("... right motor CW");
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

    return true;
}

// Purpose:
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
void measureMinMaxMotorSpeeds(ASIZE dummyPlaceholder)
{
    int const numSamples = 30;            // note: sample window has a considerable impact on libtask stacksize requirements in club_robot_nav1
                                          // e.g. going from numSamples === 10 to numSamples === 30 required updating in club_robot_nav1.ino
                                          // (MINSTACK * 10) to something <= 20, ie. create_task("measureMinMaxMotorSpeeds", measureMinMaxMotorSpeeds, 10, MINSTACK * 20);
    int const msSampleWindow = 50;
    int const pauseBetweenTests = 3000;
    encoderMeasurementsStruct maxForwardMeasurements[numSamples + 1];
    encoderMeasurementsStruct maxBackwardsMeasurements[numSamples + 1];
    encoderMeasurementsStruct minForwardMeasurements[numSamples + 1];
    encoderMeasurementsStruct minBackwardsMeasurements[numSamples + 1];
    int i;
    int haltMeasurementSemaphore;

    Serial.println("\nStarting measureMinMaxMotorSpeeds()\n");

    while (1)
    {
        velocityLoopStart();

        // First - measure Max Forward speed
        Serial.print("\n\n\nMaximum Forward Speeds, with a sample period of ");
        Serial.print(msSampleWindow);
        Serial.println("ms");
        setMotorVelocity(0, 100);
        wake_after(msSampleWindow); // give ample time for motor speed to settle, or shrink to measure acceleration...
        sampleEncoders(&maxForwardMeasurements[0]);
        for (i = 1; i <= numSamples; i++)
        {
            wake_after(msSampleWindow);
            sampleEncoders(&maxForwardMeasurements[i]);
            calculateEncoderMeasurementDeltas(&maxForwardMeasurements[i - 1], &maxForwardMeasurements[i]);
        }
        setMotorVelocity(0, 0);
        printEncoderMeasurements(maxForwardMeasurements, numSamples);

        // Next - measure Max Backwards speed
        wake_after(pauseBetweenTests);
        Serial.print("\n\n\nMaximum Reverse Speeds, with a sample period of ");
        Serial.print(msSampleWindow);
        Serial.println("ms");
        setMotorVelocity(0, -100);
        wake_after(msSampleWindow); // give ample time for motor speed to settle, or shrink to measure acceleration...
        sampleEncoders(&maxBackwardsMeasurements[0]);
        for (i = 1; i <= numSamples; i++)
        {
            wake_after(msSampleWindow);
            sampleEncoders(&maxBackwardsMeasurements[i]);
            calculateEncoderMeasurementDeltas(&maxBackwardsMeasurements[i - 1], &maxBackwardsMeasurements[i]);
        }
        setMotorVelocity(0, 0);
        printEncoderMeasurements(maxBackwardsMeasurements, numSamples);

        // Next - measure Min Forward speed
        wake_after(pauseBetweenTests);
        Serial.print("\n\n\nMinimum Forward Speeds, with a sample period of ");
        Serial.print(msSampleWindow);
        Serial.println("ms");
        setMotorVelocity(0, 8);
        wake_after(msSampleWindow); // give ample time for motor speed to settle, or shrink to measure acceleration...
        sampleEncoders(&minForwardMeasurements[0]);
        for (i = 1; i <= numSamples; i++)
        {
            wake_after(msSampleWindow);
            sampleEncoders(&minForwardMeasurements[i]);
            calculateEncoderMeasurementDeltas(&minForwardMeasurements[i - 1], &minForwardMeasurements[i]);
        }
        setMotorVelocity(0, 0);
        printEncoderMeasurements(minForwardMeasurements, numSamples);

        // Next - measure Min Backwards speed
        wake_after(pauseBetweenTests);
        Serial.print("\n\n\nMinimum Reverse Speeds, with a sample period of ");
        Serial.print(msSampleWindow);
        Serial.println("ms");
        setMotorVelocity(0, -8);
        wake_after(msSampleWindow); // give ample time for motor speed to settle, or shrink to measure acceleration...
        sampleEncoders(&minBackwardsMeasurements[0]);
        for (i = 1; i <= numSamples; i++)
        {
            wake_after(msSampleWindow);
            sampleEncoders(&minBackwardsMeasurements[i]);
            calculateEncoderMeasurementDeltas(&minBackwardsMeasurements[i - 1], &minBackwardsMeasurements[i]);
        }
        setMotorVelocity(0, 0);
        printEncoderMeasurements(minBackwardsMeasurements, numSamples);


        velocityLoopStop();
        semaphore_obtain(&haltMeasurementSemaphore); // stop this task, but don't kill it
    }
}

// Purpose:
//  - verify basic open-loop control of the motors
//  - verify movement polarity
//  - verify basic utilization of the libtask / task.h / task.cpp library
// Input:
//  - none
// Output:
//  - console logs and motor movement
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
