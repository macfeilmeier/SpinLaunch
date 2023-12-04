#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <Arduino.h>

//  Pin definitions
#define TEENSY4
#ifdef TEENSY4
    #define MOTOR_CTRL 11
    #define QUAD_A 18
    #define QUAD_B 19
    #define RELEASE_0 7
    #define RELEASE_1 8
    //#define ENGAGE_BTN 7
    //#define ARMS_CLOSED 8
    #define STATUS_LED 5
#endif

#define PULSE_PER_ROTATION 1200

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
    _cmd_error,
    _cmd_data,
    _cmd_cmd
};


static volatile int quadlastState = 0;
static volatile int quadAngle_volatile = 0;  //  Volitile value for asyncronously changing quadAngle.
static volatile uint32_t lastPulseTime_volatile = 0; // Time last pulse occured.
static volatile uint32_t pulseDeltaTime_volatile = 0;
void UpdateQuad();  // ISR to update the value of quadAngle

class Controller
{
private:

    // these values are arbitrary for now
    const int launchAngle = 450;
    const int launchAngleTolerance = 20;
    const int maxDeltaAngle = 3000;
    const int launchSpeedTolerance = 20;
    const int idleSpeedTolerance = 5;
    const int accelerationFactor = 1;
    const int maxAcceleration = 1;
    const int messageFrequency = 200; // Millisecs

    // PID tuning parameters
    const float P = 0.5;
    const float I = 0.01;
    const float D = 0.2;


    State state;

    int quadAngle = 0;  // Angle initializes to 0; gravity pulls arm down.
    float rotationalSpeed = 0;
    int lastQuadAngle;
    uint32_t lastTime;
    uint32_t deltaTime;
    uint32_t lastMessage;       
    int deltaAngle = 0;
    int accumulatedError = 0;
    int lastDutyCycle = 0;

    

    int motorTargetSpeed = 0;
    
    bool payloadLatched = false;
    bool released1 = false;


    void ActuateLatch(int latchNum);
    void MotorSpeedController();
    
public:
    int Init();
    void Loop();

    State GetState(){return state;}
    void LogData();
    //void UpdateSpeed();
};

#endif