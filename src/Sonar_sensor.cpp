#include <Romi32U4.h>
#include "Sonar_sensor.h"

void SonarSensor::Init(void)
{
    pinMode(pin_TRIG,OUTPUT);
    pinMode(pin_ECHO, INPUT);   

    Serial.begin(115200);
}

float SonarSensor::PrintData(void)
{
    Serial.println(ReadData());
    return 0;
}

float SonarSensor::ReadData(void)
{
    // ! assignment 1.2
    digitalWrite(pin_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(pin_TRIG, LOW);

    float duration = pulseIn(pin_ECHO, HIGH);

    //read out and calibrate your sonar sensor, to convert readouts to distance in [cm]
    return 0.0169186*duration + 0.841603;
}