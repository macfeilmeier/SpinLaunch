#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  pinMode(15,INPUT_PULLUP);
  pinMode(16,INPUT_PULLUP);
}

void loop() {
  Serial.print(digitalRead(15));
  Serial.print("\t");
  Serial.println(digitalRead(16));
}


