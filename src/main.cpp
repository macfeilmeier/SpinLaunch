#include <Arduino.h>
#include "controller.h"

int lastState = 0;
volatile int encoderCount = 0;
int lastEncoderCount = 0;
void handleEncoder();
int lastState = 0;
volatile int encoderCount = 0;
int lastEncoderCount = 0;
void handleEncoder();
void setup() {
  Serial.begin(115200);
  pinMode(QUAD_A,INPUT_PULLUP);
  pinMode(QUAD_B,INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(QUAD_A), handleEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(QUAD_B), handleEncoder, CHANGE);
}

void loop() {
  // Serial.print(digitalRead(QUAD_A));
  // Serial.print("\t");
  // Serial.println(digitalRead(QUAD_B));
  if(lastEncoderCount != encoderCount)
  {
    lastEncoderCount = encoderCount;
    Serial.println(lastEncoderCount);
  }
}


void handleEncoder() {
  int newState = digitalRead(QUAD_A) | (digitalRead(QUAD_B) << 1);
  if (newState != lastState) {
    if (lastState == 0b00 && newState == 0b01) {
      encoderCount++;
    } else if (lastState == 0b01 && newState == 0b00) {
      encoderCount--;
    } else if (lastState == 0b00 && newState == 0b10) {
      encoderCount--;
    } else if (lastState == 0b10 && newState == 0b00) {
      encoderCount++;
    }
    lastState = newState;
  }
}