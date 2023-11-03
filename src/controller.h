#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <Arduino.h>


//  Pin definitions
#define TEENSY4
#ifdef TEENSY4
    #define MOTOR_CTRL 2
    #define QUAD_A 15
    #define QUAD_B 16
    #define RELEASE 6
    #define ENGAGE_BTN 7
    #define ARMS_CLOSED 8
    #define STATUS_LED 5
#endif

#define PULSE_PER_ROTATION 600

enum State
{
    IDLE,
    SPIN_UP,
    READY,
    LAUNCH,
    SPIN_DOWN,
    ABORT
};

enum CMD
{
    _cmd_start_spin,
    _cmd_ready,
    _cmd_not_ready,
    _cmd_release,
    _cmd_abort,
    _cmd_error
};


volatile int quadlastState = 0;
volatile int quadAngle_volitile = 0;  //  Volitile value for asyncronously changing quadAngle.
void UpdateQuad();  // ISR to update the value of quadAngle

class Controller
{
private:

    // these values are arbitrary for now
    const int launchAngle = 135;
    const int launchAngleTolerance = 5;
    const int maxSpeed = 3600;
    const int launchSpeedTolerance = 20;
    const int idleSpeedTolerance = 5;

    // PID tuning parameters
    const float P = 0.5;
    const float I = 0.01;
    const float D = 0.2;


    State state;

    int quadAngle = 0;  // Angle initializes to 0; gravity pulls arm down.
    int rotationalSpeed = 0;
    int lastQuadAngle;
    uint32_t lastTime;
    uint32_t deltaTime;
    uint32_t accumulatedError = 0;

    int motorTargetSpeed = 0;
    
    bool payloadLatched = false;


    void ActuateLatch();
    void MotorSpeedController();
    
public:
    int Init();
    void Loop();

    State GetState(){return state;}

};

#endif