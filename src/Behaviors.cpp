#include <Romi32U4.h>
#include "Behaviors.h"
#include "Speed_controller.h"
#include "Position_estimation.h"
#include "IR_sensor.h"

//sensors
Romi32U4ButtonA buttonA;

//motor-speed controller
SpeedController robot;

//IR sensor
IRsensor SharpIR;

void Behaviors::Init(void)
{
    robot.Init();
    SharpIR.Init();
}

void Behaviors::Stop(void)
{
    robot.Stop();
}

void Behaviors::Run(void)
{
    switch (robot_state)
    {
    case IDLE:
        if(buttonA.getSingleDebouncedRelease()){ 
            robot_state = DRIVE; 
            robot.Stop();             
        } 
        else { 
            robot_state = IDLE;
            robot.Stop(); 
        }   
        break;
    
    case DRIVE:
        if(buttonA.getSingleDebouncedRelease()){ 
            robot_state = IDLE; 
            robot.Stop();             
        } else {
            robot_state = DRIVE;
            Serial.print("Distance: ");
            Serial.println(SharpIR.ReadData());
        }
        break;
    }
}