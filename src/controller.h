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
    #define STATUS_LED 5
#endif

#define PULSE_PER_ROTATION 600

enum State
{
    IDLE,
    SPIN_UP,
    LAUNCH,
    READY,
    SPIN_DOWN
};


volatile int quadlastState = 0;
volatile int quadAngle_volitile = 0;  //  Volitile value for asyncronously changing quadAngle.
void UpdateQuad();  // ISR to update the value of quadAngle

class Controller
{
private:
    State state;

    int quadAngle = 0;  // Angle initializes to 0; gravity pulls arm down.


    
public:
    int Init();
    void Loop();

    State GetState(){return state;}

};

#endif