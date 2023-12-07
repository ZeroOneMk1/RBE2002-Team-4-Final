#include <Romi32U4.h>
#include "Behaviors.h"
#include "Speed_controller.h"
#include "Position_estimation.h"

#define WALLDIST 10
#define THRESHOLD_HIGH 45
#define THRESHOLD_LOW 55

//sensors
Romi32U4ButtonA buttonA;

//motor-speed controller
SpeedController robot;

void Behaviors::Init(void)
{
    robot.Init();
}

void Behaviors::Stop(void)
{
    robot.Stop();
}

void Behaviors::setTargetRoom(void)
{
    target_room = -1; // TODO Azura will add this.
}

void Behaviors::setInHallway(void)
{
    in_hallway = 0; // TODO Caleb, make this return 1 f it's in the hallway.
}

void Behaviors::setConfirmDelivery(void)
{
    confirm_delivery = 0; // TODO Azura will add this.
}

void Behaviors::setWallDistance(enum DIRECTION dir)
{
    if (dir == LEFT){
        distL = 0;
    }else if(dir == RIGHT){
        distR = 0;
    }else if(dir == FRONT){
        distF = 0;
    } // TODO Whoever's first do this
}

int Behaviors::collisionDetected(void)
{
    return 1; // TODO Whoever's first do this
}

int Behaviors::getAprilTag(void)
{
    return NOTAG; // TODO CALEB, return the ENUM
}

void Behaviors::Run(void)
{
    switch (robot_state)
    {
    case IDLE:
        if(buttonA.getSingleDebouncedRelease()){ 
            robot_state = FORWARD; 
            robot.Stop();             
        } 
        else { 
            robot_state = IDLE;
            robot.Stop(); 
        }   
        break;
    
    case FORWARD:
        setWallDistance(FRONT);
        april = getAprilTag();

        if(april == HALLWAY){
            in_hallway = 1;
        }

        if(buttonA.getSingleDebouncedRelease() || april == HOME) {
            robot_state = IDLE;
            robot.Stop();
        }else if(distF < WALLDIST){
            robot_state = CHECKD_1;
            robot.Stop();
        }else if(distF < THRESHOLD_HIGH && distF > THRESHOLD_LOW){
            robot_state = CHECKD_1;
            robot.Stop();
        }else{
            robot.Run(100,100);
        }
        break;

    case CHECKD_1:
        setWallDistance(RIGHT);
        april = getAprilTag();

        if(april == target_room){
            robot_state = KNOCK;
            robot.Stop();
        }else if(april == HALLWAY){
            in_hallway = 1;
        }else if(april == HOME){
            robot.Stop();
            robot_state = IDLE;
        }else{
            robot.Curved(50, -50, 1); // TODO Calibrate point turn
            robot.Stop();
            robot_state = CHECKD_2;
        }
        break;

    case CHECKD_2:
        setWallDistance(LEFT);
        april = getAprilTag();

        if(april == target_room){
            robot_state = KNOCK;
            robot.Stop();
        }else if(april == HALLWAY){
            in_hallway = 1;
        }else if(april == HOME){
            robot.Stop();
            robot_state = IDLE;
        }else{
            if(distR > distL){
                robot.Curved(50, -50, 1); // TODO Calibrate point turn, get direction
                robot.Stop();
                robot_state = FORWARD;
            }else if(distL < distR){
                robot.Curved(-50, 50, 1); // TODO Calibrate point turn, get direction
                robot.Stop();
                robot_state = FORWARD;
            }else{
                robot_state = IDLE;
                robot.Stop();
            }
            robot.Stop();
        }
        break;

    case KNOCK:
        robot.Run(200, 200);
        if(collisionDetected()){
            robot_state = REVERSE;
            robot.Stop();
            collisions += 1;
        }
        break;
    case REVERSE:
        setWallDistance(FRONT);
        if(distF < WALLDIST){
            robot.Run(-100, -100);
        }else{
            if(collisions >= 3){
                robot_state = WAIT;
                robot.Stop();
            }else{
                robot_state = KNOCK;
                robot.Stop();
            }
        }
        break;
    case WAIT:
        setConfirmDelivery();
        if(confirm_delivery){
            target_room = -1;
            robot.Stop();
            robot_state = FORWARD;
        }
        break;
    }
}