#include <Arduino.h>

int lastState = 0;
volatile int encoderCount = 0;
int lastEncoderCount = 0;
void handleEncoder();
void setup() {
  Serial.begin(115200);
  pinMode(15,INPUT_PULLUP);
  pinMode(16,INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(15), handleEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(16), handleEncoder, CHANGE);
}

void loop() {
  // Serial.print(digitalRead(15));
  // Serial.print("\t");
  // Serial.println(digitalRead(16));
  if(lastEncoderCount != encoderCount)
  {
    lastEncoderCount = encoderCount;
    Serial.println(lastEncoderCount);
  }
}


void handleEncoder() {
  int newState = digitalRead(15) | (digitalRead(16) << 1);
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