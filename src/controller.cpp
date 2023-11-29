#include "controller.h"

int Controller::Init()
{
    Serial4.begin(115200);
    pinMode(MOTOR_CTRL, OUTPUT);
    pinMode(RELEASE_0, OUTPUT);
    pinMode(RELEASE_1, OUTPUT);

    pinMode(STATUS_LED, OUTPUT);
    
    //pinMode(ENGAGE_BTN, INPUT);
    //pinMode(ARMS_CLOSED, INPUT);
    //attachInterrupt(digitalPinToInterrupt(ARMS_CLOSED), UpdateQuad, CHANGE);

    //  Set up Quad pins and interupt
    pinMode(QUAD_A, INPUT_PULLUP);
    pinMode(QUAD_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(QUAD_A), UpdateQuad, CHANGE);
    attachInterrupt(digitalPinToInterrupt(QUAD_B), UpdateQuad, CHANGE);

    // if(digitalReadFast(ARMS_CLOSED))
    // {
    //     payloadLatched = true;
    // }

    digitalWrite(MOTOR_CTRL, LOW);

    lastTime = micros();
    lastMessage = 0;
    
    return 0;
}

void Controller::Loop()
{
    // Copy quadAngle from volitle to non-volitile.
    uint32_t pulseDeltaTime = pulseDeltaTime_volatile;
    rotationalSpeed = ((float)1000000 / PULSE_PER_ROTATION) / pulseDeltaTime; // (1000000 / 12000) / pulseDeltaTime
    lastQuadAngle = quadAngle;
    quadAngle = quadAngle_volatile;
    deltaTime = micros() - lastTime;
    lastTime = micros();

    // Account for the angle resetting to 0 after reaching PUSLE_PER_ROTATION
    //deltaAngle = (quadAngle > lastQuadAngle) ? (quadAngle - lastQuadAngle) : (quadAngle + PULSE_PER_ROTATION - lastQuadAngle);

    deltaAngle = quadAngle - lastQuadAngle;
    rotationalSpeed = (float)deltaAngle / deltaTime;  // Calculate rotational speed (ticks per microsec)

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
        // if(digitalRead(ENGAGE_BTN))
        // {
        //     ActuateLatch(0);
        // }
        //payloadLatched = digitalRead(ARMS_CLOSED);  // check if payload arms are closed

        if(cmd == _cmd_start_spin)
        {
            accumulatedError = 0;   // Reset accumulated error
            motorTargetSpeed = maxDeltaAngle;
            // if(payloadLatched == true)   
            //     state = SPIN_UP;
            // else
            //     Serial4.write(_cmd_error);
            state = SPIN_UP;
        }
        
        break;
    case SPIN_UP:
        // if(!digitalRead(ARMS_CLOSED))   
        // {   // Potential Failure of Latch
        //     state = ABORT;
        //     return;
        // }

        MotorSpeedController();
        if(rotationalSpeed >= maxDeltaAngle)
        {
            Serial4.print(_cmd_cmd);
            Serial4.print('\t');
            Serial4.println(_cmd_ready);// Let computer know that spinup is completed.
            state = READY;
        }

        break;
    case READY:

        //  Maintain speed
        MotorSpeedController();

        // check is speed is not within specs.
        if(deltaAngle < (maxDeltaAngle - launchSpeedTolerance))
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
        MotorSpeedController();
        if(abs(quadAngle - launchAngle) < launchAngleTolerance)
        {
            ActuateLatch(0);
            released1 = true;
        }
        if(released1 && (quadAngle - launchAngle + (PULSE_PER_ROTATION/2)) < launchAngleTolerance)
        {
            ActuateLatch(1);
            state = SPIN_DOWN;
            motorTargetSpeed = 0;
        }

        break;
    case SPIN_DOWN:
        // Reduce Speed (Regenerative Breaking?)
        MotorSpeedController();
        
        if(abs(deltaAngle) < idleSpeedTolerance)
        {
            state = IDLE;
        }
        break;
    case ABORT:
        Serial4.write(_cmd_abort);  // Let computer know that launch is not available.
        state = IDLE;
        motorTargetSpeed = 0;
        digitalWrite(MOTOR_CTRL, LOW);
        break;

    default:
        break;
    }

    if(millis() > lastMessage + messageFrequency)
    {
        LogData();
        lastMessage = millis();
    }
    delayMicroseconds(200);
}

void Controller::MotorSpeedController()
{
    // Controller Logic for Motor PID
    //
    //...    
    // accumulatedError += (motorTargetSpeed - rotationalSpeed) / deltaTime;

    // int motorCommand = P * (motorTargetSpeed - rotationalSpeed) + I * accumulatedError + D * deltaAngle;

    // // map motor command to PWM duty cycle.
    // int map = motorCommand * 256.0 / PULSE_PER_ROTATION * accelerationFactor;
    // // bound input
    // map %= 256;
    // if(map > lastDutyCycle + maxAcceleration)
    //     map = lastDutyCycle + maxAcceleration;

    // // write pwm.    
    // analogWrite(MOTOR_CTRL, P);
    digitalWrite(MOTOR_CTRL, HIGH);
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
        if (quadlastState == 0b00 && newState == 0b01) 
        {
            quadAngle_volatile--;
        } 
        else if (quadlastState == 0b01 && newState == 0b00) 
        {
            quadAngle_volatile++;
        } 
        else if (quadlastState == 0b00 && newState == 0b10) 
        {
            quadAngle_volatile++;
        } 
        else if (quadlastState == 0b10 && newState == 0b00) 
        {
            quadAngle_volatile--;
        }
        pulseDeltaTime_volatile = micros() - lastPulseTime_volatile;
        lastPulseTime_volatile = micros();
        //quadAngle_volitile %= PULSE_PER_ROTATION;
        quadlastState = newState;
    }
}

void Controller::LogData()
{
    Serial4.print((unsigned char)_cmd_data);
    Serial4.print('\t');
    Serial4.print(state);
    Serial4.print('\t');
    Serial4.print(quadAngle);
    Serial4.print('\t');
    Serial4.print(deltaAngle);
    Serial4.println();
}

// void Controller::UpdateSpeed()
// {
//     rotationalSpeed_volatile = 
// }