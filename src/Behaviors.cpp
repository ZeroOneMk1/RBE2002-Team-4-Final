#include <Romi32U4.h>
#include "Behaviors.h"
#include "Speed_controller.h"
#include "Position_estimation.h"

//sensors
Romi32U4ButtonA buttonA;

//motor-speed controller
SpeedController robot;

//Flags
int target_room = -1;
int in_corridoor = 0;

void Behaviors::Init(void)
{
    robot.Init();
}

void Behaviors::Stop(void)
{
    robot.Stop();
}

int Behaviors::getTargetRoom(void)
{
    return -1; // TODO Azura will fix this.
}

int Behaviors::getInHallway(void)
{
    return 0; // TODO Caleb, make this return 1 f it's in the hallway.
}

void Behaviors::SetFlags(void)
{
    target_room = getTargetRoom();
    in_corridoor = getInHallway();
}

void Behaviors::Run(void)
{
    SetFlags();
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
        robot_state = DRIVE;
        // ! assignment
        robot.Straight(100,10); //velocity, duration
        // robot.Turn(180,0); //degrees, direction
        robot.Curved(85.7125,114.2875,10); //velocity left, velocity right, duration
        robot.Stop(); 
        robot_state = IDLE;
        break;
    }
}