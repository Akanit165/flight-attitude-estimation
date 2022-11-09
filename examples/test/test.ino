#include "Attitude.h"
//#include "Attitude.cpp"
#include <Wire.h>

Attitude attitude;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  attitude.init();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  float currentTime;
  float previousTime = currentTime;
  currentTime = millis();
  float timeStep = (currentTime - previousTime)/1000;

  Vector data = attitude.getEuler();
  // Serial.print("Roll : ");
  // Serial.print(data.x);
  // Serial.print("  Pitch : ");
  // Serial.print(data.y);
  // Serial.print("  Yaw : ");
  // Serial.println(data.z);

  attitude.transmitData(data.z, data.y, data.x);

  delay(125);

}
