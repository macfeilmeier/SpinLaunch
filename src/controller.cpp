#include "controller.h"

int Controller::Init()
{
    Serial4.begin(115200);
    pinMode(MOTOR_CTRL, OUTPUT);
    pinMode(RELEASE_0, OUTPUT);
    pinMode(RELEASE_1, OUTPUT);

    pinMode(STATUS_LED, OUTPUT);
    
    pinMode(ENGAGE_BTN, INPUT);
    pinMode(ARMS_CLOSED, INPUT);
    attachInterrupt(digitalPinToInterrupt(ARMS_CLOSED), UpdateQuad, CHANGE);

    //  Set up Quad pins and interupt
    pinMode(QUAD_A, INPUT_PULLUP);
    pinMode(QUAD_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(QUAD_A), UpdateQuad, CHANGE);

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

    // Account for the angle resetting to 0 after reaching PUSLE_PER_ROTATION
    deltaAngle = (quadAngle > lastQuadAngle) ? (quadAngle - lastQuadAngle) : (quadAngle + PULSE_PER_ROTATION -lastQuadAngle);
    rotationalSpeed = deltaAngle / deltaTime;  // Calculate rotational speed (ticks per millisec)

    char buf[1];
    uint8_t cmd = 255;
    if(Serial4.available())
    {
        Serial4.readBytes(buf, 1);
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
            ActuateLatch(0);
        }
        payloadLatched = digitalRead(ARMS_CLOSED);  // check if payload arms are closed

        if(cmd == _cmd_start_spin)
        {
            accumulatedError = 0;   // Reset accumulated error
            if(payloadLatched == true)   
                state = SPIN_UP;
            else
                Serial4.write(_cmd_error);
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
            Serial4.write(_cmd_ready);// Let computer know that spinup is completed.
            state = READY;
        }

        break;
    case READY:

        //  Maintain speed
        MotorSpeedController();

        // check is speed is not within specs.
        if(rotationalSpeed < (maxSpeed - launchSpeedTolerance))
        {
            Serial4.write(_cmd_not_ready);// Let computer know that launch is no longer available
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
            ActuateLatch(0);
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
        Serial4.write(_cmd_abort);  // Let computer know that launch is not available.
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
    accumulatedError += (motorTargetSpeed - quadAngle) / deltaTime;

    int motorCommand = P * (motorTargetSpeed - quadAngle) + I * accumulatedError + D * deltaAngle;

    // map motor command to PWM duty cycle.
    int map = motorCommand * 256.0 / PULSE_PER_ROTATION * accelerationFactor;
    // bound input
    map %= 256;

    // write pwm.    
    analogWrite(MOTOR_CTRL, P);
}
void Controller::ActuateLatch(int latchNum)
{
    // Controller Logic for Latch Actuation
    //
    //...
    digitalWriteFast((latchNum ? RELEASE_1: RELEASE_0), LOW);
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
        quadAngle_volitile %= PULSE_PER_ROTATION;
        quadlastState = newState;
    }
}