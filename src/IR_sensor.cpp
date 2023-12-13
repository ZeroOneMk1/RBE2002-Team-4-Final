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
  float analogData = analogRead(pin_IR);
  // Serial.println( analogData * (5.0/(pow(2,10))));
  float expDist = 0.0002657 * analogData - 0.01975;
  float distance = 1.0/expDist;
  return distance;
}