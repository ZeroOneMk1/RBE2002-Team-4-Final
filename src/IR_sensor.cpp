#include <Romi32U4.h>
#include "IR_sensor.h"

void IRsensor::Init(void)
{
    pinMode(pin_IR, INPUT);
}

float IRsensor::PrintData(void)
{
    Serial.println(ReadData());
}

float IRsensor::ReadData(void)
{
  //assignment 1.1
  //read out and calibrate your IR sensor, to convert readouts to distance in [cm]
  // 12cm = close wall = 150 Analog
  // 53cm = far wall = 100 Analog
  float analogData = analogRead(pin_IR);
  // Serial.println( analogData * (5.0/(pow(2,10))));
  float expDist = abs(analogData-6.81389);
  float distance = 4721.35/expDist;
  return distance;
}