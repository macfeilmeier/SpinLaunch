#include "controller.h"

int Controller::Init()
{
    pinMode(MOTOR_CTRL, OUTPUT);
    pinMode(RELEASE, OUTPUT);

    pinMode(STATUS_LED, OUTPUT);
    
    pinMode(ENGAGE_BTN, INPUT);
    pinMode(ARMS_CLOSED, INPUT);
    //attachInterrupt(digitalPinToInterrupt(ARMS_CLOSED), UpdateQuad, CHANGE);

    //  Set up Quad pins and interupt
    pinMode(QUAD_A, INPUT_PULLUP);
    pinMode(QUAD_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(QUAD_A), UpdateQuad, CHANGE);
    attachInterrupt(digitalPinToInterrupt(QUAD_B), UpdateQuad, CHANGE);

    if(digitalReadFast(ARMS_CLOSED))
    {
        payloadLatched = true;
    }

    lastTime = micros();
    
    return 0;
}

void Controller::Loop()
{
    // Copy quadAngle from volitle to non-volitile.
    lastQuadAngle = quadAngle;
    quadAngle = quadAngle_volitile;
    deltaTime = millis() - lastTime;
    rotationalSpeed = (quadAngle - lastQuadAngle) / deltaTime;  // Calculate rotational speed (ticks per millisec)


    char buf[1];
    uint8_t cmd = 255;
    if(Serial1.available())
    {
        Serial1.readBytes(buf, 1);
        cmd = buf[0];
    }

    if(cmd == _cmd_abort)
    {
        state = ABORT;
    }

    switch (state)
    {
    case IDLE:
        digitalWrite(MOTOR_CTRL, LOW);  // Turn off Motor if its on
        motorTargetSpeed = 0;           // Set Motor Speed to 0, in case its not.

        //  Check if user btn for latch is pressed 
        if(digitalRead(ENGAGE_BTN))
        {
            ActuateLatch();
        }
        payloadLatched = digitalRead(ARMS_CLOSED);  // check if payload arms are closed

        if(cmd == _cmd_start_spin)
        {
            accumulatedError = 0;   // Reset accumulated error
            if(payloadLatched == true)   
                state = SPIN_UP;
            else
                Serial1.write(_cmd_error);
        }
        
        break;
    case SPIN_UP:
        if(!digitalRead(ARMS_CLOSED))   
        {   // Potential Failure of Latch
            state = ABORT;
            return;
        }

        MotorSpeedController();
        if(quadAngle >= maxSpeed)
        {
            Serial1.write(_cmd_ready);// Let computer know that spinup is completed.
            state = READY;
        }

        break;
    case READY:

        //  Maintain speed
        MotorSpeedController();

        // check is speed is not within specs.
        if(rotationalSpeed < (maxSpeed - launchSpeedTolerance))
        {
            Serial1.write(_cmd_not_ready);// Let computer know that launch is no longer available
            state = READY;
        }

        // if computer sends release command move to next stage
        if(cmd == _cmd_release)
        {
            state = LAUNCH;
        }

        break;
    case LAUNCH:
        if(abs(quadAngle - launchAngle) < launchAngleTolerance)
        {
            ActuateLatch();
            state = SPIN_DOWN;
        }

        break;
    case SPIN_DOWN:
        // Reduce Speed (Regenerative Breaking?)
        MotorSpeedController();
        
        if(abs(rotationalSpeed) < idleSpeedTolerance)
        {
            state = IDLE;
        }
        break;
    case ABORT:
        Serial1.write(_cmd_abort);  // Let computer know that launch is not available.
        state = SPIN_DOWN;
        motorTargetSpeed = 0;
        break;

    default:
        break;
    }
}

void Controller::MotorSpeedController()
{
    // Controller Logic for Motor PID
    //
    //...
    int deltaAngle = (quadAngle - lastQuadAngle);
    
    accumulatedError += (motorTargetSpeed - quadAngle) / deltaTime;


    int motorCommand = P * (motorTargetSpeed - quadAngle) + I * accumulatedError + D * deltaAngle;

}
void Controller::ActuateLatch()
{
    // Controller Logic for Latch Actuation
    //
    //...
}

void UpdateQuad()
{
    int newState = digitalRead(QUAD_A) | (digitalRead(QUAD_B) << 1);
    if (newState != quadlastState) {
        if (quadlastState == 0b00 && newState == 0b01) {
        quadAngle_volitile++;
        } else if (quadlastState == 0b01 && newState == 0b00) {
        quadAngle_volitile--;
        } else if (quadlastState == 0b00 && newState == 0b10) {
        quadAngle_volitile--;
        } else if (quadlastState == 0b10 && newState == 0b00) {
        quadAngle_volitile++;
        }
        if(quadAngle_volitile > PULSE_PER_ROTATION)
        {
            quadAngle_volitile = 0;
        }
        else if(quadAngle_volitile < 0)
        {
            quadAngle_volitile = PULSE_PER_ROTATION;
        }
        quadlastState = newState;
    }
}