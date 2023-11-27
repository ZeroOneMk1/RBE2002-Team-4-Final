#include <Romi32U4.h>
#include "Encoders.h"
#include "Wall_following_controller.h"
#include "IR_sensor.h"
#include "Sonar_sensor.h"

IRsensor SharpIR;
SonarSensor HCSR04;
void WallFollowingController::Init(void)
{
    SharpIR.Init();
    HCSR04.Init();
}

float WallFollowingController::Process(float target_distance)
{
  float irdist = SharpIR.ReadData();
  float sondist = HCSR04.ReadData();
  float dist = 0;
  
  // dist = (irdist + sondist) /  2;

  E_distance = target_distance - irdist;

  float P = E_distance * Kp;
  float D = (E_distance - prev_e_distance) * Kd;
  prev_e_distance = E_distance;
  float speed = P + D;

  return speed;
}