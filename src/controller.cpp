#include "controller.h"

int Controller::Init()
{
    pinMode(MOTOR_CTRL, OUTPUT);
    pinMode(RELEASE, OUTPUT);
    pinMode(STATUS_LED, OUTPUT);

    pinMode(QUAD_A, INPUT_PULLUP);
    pinMode(QUAD_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(QUAD_A), UpdateQuad, CHANGE);
    
    return 0;
}

void Controller::Loop()
{
    // Copy quadAngle from volitle to non-volitile.
    quadAngle = quadAngle_volitile;
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
        quadlastState = newState;
    }
}