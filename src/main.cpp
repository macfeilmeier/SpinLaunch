#include <Arduino.h>
#include "controller.h"

int lastState = 0;
volatile int encoderCount = 0;
int lastEncoderCount = 0;
//void handleEncoder();
Controller ctrl;
IntervalTimer telemeryTimer;
//IntervalTimer speedTimer;

void SendTelemetry()
{
    ctrl.LogData();
}
// void UpdateSpeed()
// {
//   ctrl.UpdateSpeed();
// }

void setup() {
  Serial.begin(115200);
  ctrl.Init();
  // pinMode(QUAD_A,INPUT_PULLUP);
  // pinMode(QUAD_B,INPUT_PULLUP);
  // pinMode(RELEASE_0, OUTPUT);

  // attachInterrupt(digitalPinToInterrupt(QUAD_A), handleEncoder, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(QUAD_B), handleEncoder, CHANGE);
  
  telemeryTimer.priority(128);
  telemeryTimer.begin(SendTelemetry, 100000);
  // speedTimer.priority(64);
  // speedTimer.begin(UpdateSpeed, 1000);
}

void loop() {
  ctrl.Loop();
  delay(100);
  //Serial4.println("Blah");
  // Serial.print(digitalRead(QUAD_A));
  // Serial.print("\t");
  // Serial.println(digitalRead(QUAD_B));
  // if(lastEncoderCount != encoderCount)
  // {
  //   lastEncoderCount = encoderCount;
  //   Serial.println(lastEncoderCount);
  // }
  // for(int i = 0; i < 255; i++)
  // {
  //   analogWrite(7, i);
  //   delay(10);
  // }
  // for(int i = 255; i >0; i--)
  // {
  //   analogWrite(7, i);
  //   delay(10);
  // }
  // analogWrite(RELEASE_0, 0);
  // delay(1000);
  // analogWrite(RELEASE_0, 128);
  // delay(1000);
  // analogWrite(RELEASE_0, 255);
  // delay(1000);
}


// void handleEncoder() {
//   int newState = digitalRead(QUAD_A) | (digitalRead(QUAD_B) << 1);
//   if (newState != lastState) {
//     if (lastState == 0b00 && newState == 0b01) {
//       encoderCount++;
//     } else if (lastState == 0b01 && newState == 0b00) {
//       encoderCount--;
//     } else if (lastState == 0b00 && newState == 0b10) {
//       encoderCount--;
//     } else if (lastState == 0b10 && newState == 0b00) {
//       encoderCount++;
//     }
//     lastState = newState;
//   }
// }