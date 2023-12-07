#include <Romi32U4.h>
#include "Behaviors.h"
#include "Speed_controller.h"
#include "Position_estimation.h"
#include "apriltagdatum.h"
#include "openmv.h"
#include "Wire.h"

//sensors
Romi32U4ButtonA buttonA;

//motor-speed controller
SpeedController robot;

//Camera controller
OpenMV camera;

void Behaviors::Init(void)
{
    robot.Init();
    delay(1000);
    Wire.begin();
    Wire.setClock(100000ul);
    robot.Init();
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
        if(buttonA.getSingleDebouncedRelease()) {
            robot_state = IDLE;
            robot.Stop();
        } else {
            delay(1);   
            //    Serial.print("counts:");
            uint8_t numTags = camera.getTagCount();
            //    Serial.println(numTags);
            AprilTagDatum aprltag;
            while(numTags > 0)
            {
                camera.readTag(aprltag);
                Serial.print("id:");
                Serial.println(aprltag.id);
                numTags--;
            }
            robot_state = DRIVE;
        }
        break;
    }
}