#include <Romi32U4.h>
#include "IR_sensor.h"

#define INV_TO_CM 6605.37

void IRsensor::Init(void)
{
    pinMode(pin_IR, INPUT);
    Serial.begin(115200);
}

float IRsensor::PrintData(void)
{
    Serial.println(ReadData());
}

float IRsensor::ReadData(void)
{
  int ADCread = analogRead(pin_IR);

  float cm_dist = INV_TO_CM/(10+ADCread) - 6.41513;

  Serial.println(cm_dist);
  
  return cm_dist;
}